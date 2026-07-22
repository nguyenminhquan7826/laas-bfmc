#!/usr/bin/env python3
"""Run YOLOv8 ONNX locally on the Raspberry Pi.

Data path:
  C++ camera -> 127.0.0.1:9996 -> this process
  this process -> 127.0.0.1:8888 -> C++ ObstacleMsg
  this process -> monitor PC:9998 -> annotated detection JPEG

The distance/obstacle result never depends on the Wi-Fi monitor link.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort


JPEG_MAGIC = b"LJPG"
JPEG_HEADER = struct.Struct("!4sIHHI")
MAX_DATAGRAM_SIZE = 1400
MAX_JPEG_SIZE = 4 * 1024 * 1024
REASSEMBLY_TIMEOUT_S = 0.75
SOCKET_BUFFER_SIZE = 4 * 1024 * 1024


@dataclass
class Settings:
    model_path: Path
    bind_ip: str = "127.0.0.1"
    raw_frame_port: int = 9996
    distance_ip: str = "127.0.0.1"
    distance_port: int = 8888
    monitor_ip: str = "192.168.1.253"
    monitor_port: int = 9998
    monitor_fps: float = 10.0
    monitor_jpeg_quality: int = 75

    input_size: int = 640
    confidence_threshold: float = 0.15
    nms_threshold: float = 0.70
    car_class_id: int = 0

    # These two values must be calibrated for the real OV5647 installation.
    focal_length_px: float = 250.0
    car_real_height_m: float = 0.22

    receive_timeout_s: float = 1.0
    no_frame_warning_s: float = 5.0


def env_default(name: str, default: str) -> str:
    value = os.environ.get(name)
    return value if value else default


def parse_args() -> Settings:
    default_model = Path(__file__).resolve().with_name("best.onnx")

    parser = argparse.ArgumentParser(
        description="Run ONNX obstacle detection locally on Raspberry Pi"
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(env_default("LAAS_MODEL_PATH", str(default_model))),
    )
    parser.add_argument(
        "--bind-ip",
        default=env_default("LAAS_AI_BIND_IP", "127.0.0.1"),
        help="IP used to receive raw frames from the local C++ process",
    )
    parser.add_argument("--raw-port", type=int, default=9996)
    parser.add_argument(
        "--distance-ip",
        default=env_default("LAAS_DISTANCE_IP", "127.0.0.1"),
    )
    parser.add_argument("--distance-port", type=int, default=8888)
    parser.add_argument(
        "--monitor-ip",
        default=env_default("LAAS_MONITOR_IP", "192.168.1.104"),
        help="Laptop IP; use an empty string to disable annotated streaming",
    )
    parser.add_argument("--monitor-port", type=int, default=9998)
    parser.add_argument("--monitor-fps", type=float, default=10.0)
    parser.add_argument("--monitor-quality", type=int, default=75)
    parser.add_argument(
        "--focal-length",
        type=float,
        default=float(env_default("LAAS_FOCAL_LENGTH_PX", "250.0")),
    )
    parser.add_argument(
        "--car-height",
        type=float,
        default=float(env_default("LAAS_CAR_HEIGHT_M", "0.22")),
    )
    args = parser.parse_args()

    if not (1 <= args.raw_port <= 65535):
        parser.error("--raw-port must be in [1, 65535]")
    if not (1 <= args.distance_port <= 65535):
        parser.error("--distance-port must be in [1, 65535]")
    if not (1 <= args.monitor_port <= 65535):
        parser.error("--monitor-port must be in [1, 65535]")
    if args.monitor_fps < 0.0:
        parser.error("--monitor-fps must be >= 0")
    if not (1 <= args.monitor_quality <= 100):
        parser.error("--monitor-quality must be in [1, 100]")
    if args.focal_length <= 0.0 or args.car_height <= 0.0:
        parser.error("distance calibration values must be positive")

    return Settings(
        model_path=args.model.expanduser().resolve(),
        bind_ip=args.bind_ip,
        raw_frame_port=args.raw_port,
        distance_ip=args.distance_ip,
        distance_port=args.distance_port,
        monitor_ip=args.monitor_ip.strip(),
        monitor_port=args.monitor_port,
        monitor_fps=args.monitor_fps,
        monitor_jpeg_quality=args.monitor_quality,
        focal_length_px=args.focal_length,
        car_real_height_m=args.car_height,
    )


class UdpJpegReassembler:
    """Reassemble LJPG packets; legacy one-datagram JPEGs also work."""

    def __init__(self) -> None:
        self.frames: Dict[Tuple[str, int, int], dict] = {}

    def _cleanup(self, now: float) -> None:
        stale = [
            key
            for key, state in self.frames.items()
            if now - state["updated"] > REASSEMBLY_TIMEOUT_S
        ]
        for key in stale:
            del self.frames[key]

    def add_packet(
        self, packet: bytes, address: Tuple[str, int]
    ) -> Optional[np.ndarray]:
        now = time.monotonic()
        self._cleanup(now)

        if not packet.startswith(JPEG_MAGIC):
            return cv2.imdecode(
                np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR
            )

        if len(packet) < JPEG_HEADER.size:
            return None

        magic, frame_id, chunk_index, chunk_count, total_size = (
            JPEG_HEADER.unpack_from(packet)
        )
        if magic != JPEG_MAGIC:
            return None
        if (
            chunk_count == 0
            or chunk_index >= chunk_count
            or total_size == 0
            or total_size > MAX_JPEG_SIZE
        ):
            return None

        key = (address[0], address[1], frame_id)
        state = self.frames.get(key)
        if state is None:
            state = {
                "updated": now,
                "chunk_count": chunk_count,
                "total_size": total_size,
                "chunks": {},
            }
            self.frames[key] = state

        if (
            state["chunk_count"] != chunk_count
            or state["total_size"] != total_size
        ):
            del self.frames[key]
            return None

        state["updated"] = now
        state["chunks"][chunk_index] = packet[JPEG_HEADER.size :]
        if len(state["chunks"]) != chunk_count:
            return None

        try:
            jpeg = b"".join(state["chunks"][i] for i in range(chunk_count))
        except KeyError:
            return None
        finally:
            self.frames.pop(key, None)

        if len(jpeg) != total_size:
            return None

        return cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
        )


class UdpJpegSender:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.frame_id = 0

    def send(
        self,
        frame: np.ndarray,
        destination: Tuple[str, int],
        quality: int,
    ) -> bool:
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
        )
        if not ok:
            return False

        jpeg = encoded.tobytes()
        if not jpeg or len(jpeg) > MAX_JPEG_SIZE:
            return False

        payload_size = MAX_DATAGRAM_SIZE - JPEG_HEADER.size
        chunk_count = math.ceil(len(jpeg) / payload_size)
        if chunk_count > 0xFFFF:
            return False

        self.frame_id = (self.frame_id + 1) & 0xFFFFFFFF
        for chunk_index in range(chunk_count):
            start = chunk_index * payload_size
            payload = jpeg[start : start + payload_size]
            packet = JPEG_HEADER.pack(
                JPEG_MAGIC,
                self.frame_id,
                chunk_index,
                chunk_count,
                len(jpeg),
            ) + payload
            if self.sock.sendto(packet, destination) != len(packet):
                return False
        return True


class YOLOv8ONNX:
    def __init__(self, settings: Settings) -> None:
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.intra_op_num_threads = max(1, min(4, os.cpu_count() or 1))
        options.inter_op_num_threads = 1

        self.settings = settings
        self.session = ort.InferenceSession(
            str(settings.model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name

    def preprocess(
        self, image: np.ndarray
    ) -> Tuple[np.ndarray, float, int, int]:
        height, width = image.shape[:2]
        input_size = self.settings.input_size
        scale = min(input_size / width, input_size / height)
        new_width = max(1, int(round(width * scale)))
        new_height = max(1, int(round(height * scale)))

        resized = cv2.resize(image, (new_width, new_height))
        canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        pad_x = (input_size - new_width) // 2
        pad_y = (input_size - new_height) // 2
        canvas[pad_y : pad_y + new_height, pad_x : pad_x + new_width] = resized

        tensor = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = tensor.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None, ...]
        return np.ascontiguousarray(tensor), scale, pad_x, pad_y

    def postprocess(
        self,
        output: np.ndarray,
        original_shape: Tuple[int, ...],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[dict]:
        image_height, image_width = original_shape[:2]
        predictions = output[0] if output.ndim == 3 else output
        if predictions.ndim != 2:
            raise RuntimeError(f"Unexpected YOLO output shape: {output.shape}")
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        boxes: List[List[int]] = []
        scores: List[float] = []
        class_ids: List[int] = []

        for detection in predictions:
            if detection.shape[0] < 5:
                continue
            center_x, center_y, box_width, box_height = detection[:4]
            class_scores = detection[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if class_id != self.settings.car_class_id:
                continue
            if score < self.settings.confidence_threshold:
                continue

            x1 = int((center_x - box_width / 2.0 - pad_x) / scale)
            y1 = int((center_y - box_height / 2.0 - pad_y) / scale)
            x2 = int((center_x + box_width / 2.0 - pad_x) / scale)
            y2 = int((center_y + box_height / 2.0 - pad_y) / scale)

            x1 = int(np.clip(x1, 0, image_width - 1))
            y1 = int(np.clip(y1, 0, image_height - 1))
            x2 = int(np.clip(x2, 0, image_width - 1))
            y2 = int(np.clip(y2, 0, image_height - 1))
            width = x2 - x1
            height = y2 - y1
            if width < 2 or height < 2:
                continue

            boxes.append([x1, y1, width, height])
            scores.append(score)
            class_ids.append(class_id)

        results: List[dict] = []
        if boxes:
            indices = cv2.dnn.NMSBoxes(
                boxes,
                scores,
                self.settings.confidence_threshold,
                self.settings.nms_threshold,
            )
            for index in np.asarray(indices).reshape(-1):
                i = int(index)
                x, y, width, height = boxes[i]
                results.append(
                    {
                        "bbox": (x, y, x + width, y + height),
                        "class_id": class_ids[i],
                        "score": scores[i],
                    }
                )
        return results

    def detect(self, image: np.ndarray) -> List[dict]:
        tensor, scale, pad_x, pad_y = self.preprocess(image)
        outputs = self.session.run(None, {self.input_name: tensor})
        return self.postprocess(outputs[0], image.shape, scale, pad_x, pad_y)


class FrontDistanceFilter:
    def __init__(self) -> None:
        self.alpha_near = 0.50
        self.alpha_far = 0.22
        self.max_farther_jump_m = 1.20
        self.hold_frames = 3
        self.min_valid_m = 0.05
        self.max_valid_m = 10.0
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.filtered: Optional[float] = None
        self.last_raw: Optional[float] = None
        self.miss_count = 0

    def update(self, raw_distance: Optional[float]) -> Optional[float]:
        valid = (
            raw_distance is not None
            and np.isfinite(raw_distance)
            and self.min_valid_m <= raw_distance <= self.max_valid_m
        )
        if not valid:
            self.miss_count += 1
            if self.initialized and self.miss_count <= self.hold_frames:
                return self.filtered
            return None

        raw_distance = float(raw_distance)
        if not self.initialized:
            self.initialized = True
            self.filtered = raw_distance
            self.last_raw = raw_distance
            self.miss_count = 0
            return self.filtered

        assert self.filtered is not None
        if (
            self.last_raw is not None
            and raw_distance - self.last_raw > self.max_farther_jump_m
        ):
            self.miss_count += 1
            if self.miss_count <= self.hold_frames:
                return self.filtered
            self.filtered = raw_distance
            self.last_raw = raw_distance
            self.miss_count = 0
            return self.filtered

        self.miss_count = 0
        alpha = self.alpha_near if raw_distance < self.filtered else self.alpha_far
        self.filtered = alpha * raw_distance + (1.0 - alpha) * self.filtered
        self.last_raw = raw_distance
        return self.filtered


class ObstacleEstimator:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.pixel_heights: deque = deque(maxlen=5)
        self.distance_filter = FrontDistanceFilter()
        self.out_of_lane_frames = 0

        self.min_bottom_y_ratio = 0.42
        self.lane_half_width_bottom_ratio = 0.16
        self.lane_half_width_top_ratio = 0.003
        self.out_of_lane_hold_frames = 2

    def lane_half_width(self, y: int, image_height: int, image_width: int) -> float:
        y = int(np.clip(y, 0, image_height - 1))
        ratio = y / float(max(image_height - 1, 1))
        top = self.lane_half_width_top_ratio * image_width
        bottom = self.lane_half_width_bottom_ratio * image_width
        return (1.0 - ratio) * top + ratio * bottom

    def in_ego_lane(
        self, bbox: Tuple[int, int, int, int], image_width: int, image_height: int
    ) -> bool:
        x1, _y1, x2, y2 = bbox
        if y2 < self.min_bottom_y_ratio * image_height:
            return False
        foot_x = 0.5 * (x1 + x2)
        center_x = 0.5 * image_width
        return abs(foot_x - center_x) <= self.lane_half_width(
            y2, image_height, image_width
        )

    def draw_corridor(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        y_top = int(self.min_bottom_y_ratio * height)
        y_bottom = height - 1
        center_x = width // 2
        half_top = int(self.lane_half_width(y_top, height, width))
        half_bottom = int(self.lane_half_width(y_bottom, height, width))
        points = np.array(
            [
                [center_x - half_top, y_top],
                [center_x + half_top, y_top],
                [center_x + half_bottom, y_bottom],
                [center_x - half_bottom, y_bottom],
            ],
            dtype=np.int32,
        )
        cv2.polylines(frame, [points], True, (255, 120, 0), 2)

    def process(
        self, frame: np.ndarray, detections: List[dict]
    ) -> Tuple[np.ndarray, Optional[float], float]:
        annotated = frame.copy()
        image_height, image_width = annotated.shape[:2]
        self.draw_corridor(annotated)

        best_bbox: Optional[Tuple[int, int, int, int]] = None
        best_score = 0.0
        best_height = -1

        for detection in detections:
            bbox = detection["bbox"]
            x1, y1, x2, y2 = bbox
            pixel_height = y2 - y1
            if pixel_height < 20:
                continue

            in_lane = self.in_ego_lane(bbox, image_width, image_height)
            color = (0, 255, 0) if in_lane else (0, 0, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated, ((x1 + x2) // 2, y2), 4, color, -1)
            cv2.putText(
                annotated,
                f"{'IN' if in_lane else 'OUT'} lane {detection['score']:.2f}",
                (x1, max(y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                2,
            )

            if in_lane and pixel_height > best_height:
                best_bbox = bbox
                best_score = float(detection["score"])
                best_height = pixel_height

        if best_bbox is not None:
            self.out_of_lane_frames = 0
            x1, y1, x2, y2 = best_bbox
            self.pixel_heights.append(y2 - y1)
            smooth_height = float(np.median(self.pixel_heights))
            raw_distance = (
                self.settings.focal_length_px
                * self.settings.car_real_height_m
                / smooth_height
            )
            distance = self.distance_filter.update(raw_distance)
            if distance is None:
                label = f"Obstacle raw={raw_distance:.2f}m filtered=invalid"
            else:
                label = (
                    f"Obstacle raw={raw_distance:.2f}m "
                    f"filtered={distance:.2f}m conf={best_score:.2f}"
                )
            cv2.putText(
                annotated,
                label,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
            )
            return annotated, distance, best_score

        held_distance = self.distance_filter.update(None)
        self.out_of_lane_frames += 1
        if (
            self.out_of_lane_frames <= self.out_of_lane_hold_frames
            and held_distance is not None
        ):
            distance = held_distance
        else:
            distance = None
            self.pixel_heights.clear()
            self.distance_filter.reset()

        status = "none" if distance is None else f"held {distance:.2f}m"
        cv2.putText(
            annotated,
            f"No in-lane obstacle: {status}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
        )
        return annotated, distance, 0.0


def make_receive_socket(settings: Settings) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_BUFFER_SIZE)
    sock.bind((settings.bind_ip, settings.raw_frame_port))
    sock.settimeout(settings.receive_timeout_s)
    return sock


def recv_latest_frame(
    sock: socket.socket, reassembler: UdpJpegReassembler
) -> Tuple[np.ndarray, Tuple[str, int]]:
    latest_frame: Optional[np.ndarray] = None
    latest_address: Optional[Tuple[str, int]] = None

    # Block until at least one complete frame is available.
    while True:
        packet, address = sock.recvfrom(65535)
        frame = reassembler.add_packet(packet, address)
        if frame is not None:
            latest_frame = frame
            latest_address = address
            break

    # Inference is normally slower than camera capture. Drain queued packets
    # and keep the newest complete frame so AI latency does not grow over time.
    for _ in range(4096):
        try:
            packet, address = sock.recvfrom(65535, socket.MSG_DONTWAIT)
        except BlockingIOError:
            break
        frame = reassembler.add_packet(packet, address)
        if frame is not None:
            latest_frame = frame
            latest_address = address

    assert latest_frame is not None and latest_address is not None
    return latest_frame, latest_address


def run(settings: Settings) -> int:
    if not settings.model_path.is_file():
        print(f"[AI] Model not found: {settings.model_path}")
        return 2

    try:
        detector = YOLOv8ONNX(settings)
    except Exception as exc:
        print(f"[AI] Cannot initialize ONNX model: {exc}")
        return 3

    try:
        receive_sock = make_receive_socket(settings)
    except OSError as exc:
        print(
            f"[AI] Cannot bind {settings.bind_ip}:{settings.raw_frame_port}: {exc}"
        )
        return 4

    distance_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SOCKET_BUFFER_SIZE)
    monitor_sender = UdpJpegSender(monitor_sock)
    reassembler = UdpJpegReassembler()
    estimator = ObstacleEstimator(settings)

    distance_destination = (settings.distance_ip, settings.distance_port)
    monitor_destination = (settings.monitor_ip, settings.monitor_port)
    monitor_period = (
        1.0 / settings.monitor_fps if settings.monitor_fps > 0.0 else math.inf
    )
    last_monitor_send = 0.0
    last_complete_frame = time.monotonic()
    last_warning = 0.0
    previous_processed = 0.0
    filtered_fps = 0.0

    print(f"[AI] Model: {settings.model_path}")
    print(f"[AI] Raw frame: {settings.bind_ip}:{settings.raw_frame_port}")
    print(
        f"[AI] Distance result: {settings.distance_ip}:{settings.distance_port}"
    )
    if settings.monitor_ip and settings.monitor_fps > 0.0:
        print(
            f"[AI] Detection monitor: {settings.monitor_ip}:"
            f"{settings.monitor_port} at <= {settings.monitor_fps:.1f} FPS"
        )
    else:
        print("[AI] Detection monitor disabled")
    print(
        "[AI] WARNING: focal length and real object height must be calibrated "
        "before driving the real vehicle."
    )

    try:
        while True:
            try:
                frame, _sender = recv_latest_frame(receive_sock, reassembler)
            except socket.timeout:
                now = time.monotonic()
                if (
                    now - last_complete_frame >= settings.no_frame_warning_s
                    and now - last_warning >= settings.no_frame_warning_s
                ):
                    print("[AI] No complete camera frame received")
                    last_warning = now
                continue
            except OSError as exc:
                print(f"[AI] UDP receive error: {exc}")
                continue

            last_complete_frame = time.monotonic()
            try:
                detections = detector.detect(frame)
                annotated, distance, _confidence = estimator.process(
                    frame, detections
                )
            except Exception as exc:
                # Do not send a fresh "no obstacle" packet after an inference
                # failure. The C++ obstacle timeout will stop the vehicle.
                print(f"[AI] Inference error; distance packet suppressed: {exc}")
                continue

            now = time.monotonic()
            if previous_processed > 0.0:
                instant_fps = 1.0 / max(now - previous_processed, 1e-6)
                filtered_fps = (
                    instant_fps
                    if filtered_fps == 0.0
                    else 0.90 * filtered_fps + 0.10 * instant_fps
                )
            previous_processed = now

            message = f"{distance:.3f}" if distance is not None else "-1.000"
            try:
                distance_sock.sendto(message.encode("ascii"), distance_destination)
            except OSError as exc:
                print(f"[AI] Cannot send distance: {exc}")

            cv2.putText(
                annotated,
                f"Pi inference: {filtered_fps:.1f} FPS",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 0),
                2,
            )

            if (
                settings.monitor_ip
                and settings.monitor_fps > 0.0
                and now - last_monitor_send >= monitor_period
            ):
                try:
                    monitor_sender.send(
                        annotated,
                        monitor_destination,
                        settings.monitor_jpeg_quality,
                    )
                except OSError as exc:
                    print(f"[AI] Detection monitor send error: {exc}")
                last_monitor_send = now
    except KeyboardInterrupt:
        print("\n[AI] Stopped by user")
    finally:
        receive_sock.close()
        distance_sock.close()
        monitor_sock.close()

    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
