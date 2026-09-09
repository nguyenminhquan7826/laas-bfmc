#!/usr/bin/env python3
"""YOLO11 INT8 perception process for the LAAS/BFMC Raspberry Pi 5 stack.

Data path:
  C++ camera -> 127.0.0.1:9996 -> this process
  this process -> 127.0.0.1:8888 -> C++ ObstacleMsg distance payload
  this process -> monitor PC:9998 -> annotated detection JPEG

Model contract used by the parking model:
  class 0 = parking_sign
  class 1 = vehicle
  input   = 416x416

The safety-critical distance path remains backward-compatible with the current
C++ UdpYoloInterface: only the in-lane VEHICLE distance is sent on UDP 8888.
Parking-sign detections and all vehicle boxes are retained/visualized here; the
slot association (P_B1/P_B2/P_T1/P_T2 -> FREE/OCCUPIED) should be performed in
the parking/map layer where slot geometry is known.
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
from typing import Deque, Dict, List, Optional, Sequence, Tuple

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

    # YOLO11n parking model.
    input_size: int = 416
    confidence_threshold: float = 0.25
    nms_threshold: float = 0.70
    parking_sign_class_id: int = 0
    vehicle_class_id: int = 1

    # Raspberry Pi 5 CPU budget for this AI process.
    cpu_threads: int = 2
    cpu_affinity: str = "2,3"

    # Distance calibration for the physical BFMC vehicle.
    focal_length_px: float = 250.0
    car_real_height_m: float = 0.22

    receive_timeout_s: float = 1.0
    no_frame_warning_s: float = 5.0

    perf_window: int = 100


def env_default(name: str, default: str) -> str:
    value = os.environ.get(name)
    return value if value else default


def parse_args() -> Settings:
    default_model = Path(__file__).resolve().with_name("best_int8.onnx")

    parser = argparse.ArgumentParser(
        description="Run YOLO11n INT8 parking/vehicle detection on Raspberry Pi 5"
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(env_default("LAAS_MODEL_PATH", str(default_model))),
    )
    parser.add_argument(
        "--bind-ip",
        default=env_default("LAAS_AI_BIND_IP", "127.0.0.1"),
    )
    parser.add_argument("--raw-port", type=int, default=9996)
    parser.add_argument(
        "--distance-ip",
        default=env_default("LAAS_DISTANCE_IP", "127.0.0.1"),
    )
    parser.add_argument("--distance-port", type=int, default=8888)

    parser.add_argument(
        "--monitor-ip",
        default=env_default("LAAS_MONITOR_IP", "192.168.1.253"),
        help="Laptop IP; pass an empty string to disable annotated streaming",
    )
    parser.add_argument("--monitor-port", type=int, default=9998)
    parser.add_argument("--monitor-fps", type=float, default=10.0)
    parser.add_argument("--monitor-quality", type=int, default=75)

    parser.add_argument("--input-size", type=int, default=416)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--nms", type=float, default=0.70)
    parser.add_argument("--parking-sign-class-id", type=int, default=0)
    parser.add_argument("--vehicle-class-id", type=int, default=1)

    parser.add_argument(
        "--cpu-threads",
        type=int,
        default=int(env_default("LAAS_AI_CPU_THREADS", "2")),
        help="ONNX Runtime intra-op threads (default: 2)",
    )
    parser.add_argument(
        "--cpu-affinity",
        default=env_default("LAAS_AI_CPU_AFFINITY", "2,3"),
        help="Linux CPU list, e.g. 2,3. Pass empty string to disable pinning.",
    )

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

    for name, port in (
        ("raw-port", args.raw_port),
        ("distance-port", args.distance_port),
        ("monitor-port", args.monitor_port),
    ):
        if not (1 <= port <= 65535):
            parser.error(f"--{name} must be in [1, 65535]")

    if args.monitor_fps < 0.0:
        parser.error("--monitor-fps must be >= 0")
    if not (1 <= args.monitor_quality <= 100):
        parser.error("--monitor-quality must be in [1, 100]")
    if args.input_size <= 0:
        parser.error("--input-size must be positive")
    if not (0.0 <= args.conf <= 1.0):
        parser.error("--conf must be in [0, 1]")
    if not (0.0 <= args.nms <= 1.0):
        parser.error("--nms must be in [0, 1]")
    if args.cpu_threads <= 0:
        parser.error("--cpu-threads must be positive")
    if args.focal_length <= 0.0 or args.car_height <= 0.0:
        parser.error("distance calibration values must be positive")
    if args.parking_sign_class_id < 0 or args.vehicle_class_id < 0:
        parser.error("class IDs must be non-negative")
    if args.parking_sign_class_id == args.vehicle_class_id:
        parser.error("parking-sign and vehicle class IDs must be different")

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
        input_size=args.input_size,
        confidence_threshold=args.conf,
        nms_threshold=args.nms,
        parking_sign_class_id=args.parking_sign_class_id,
        vehicle_class_id=args.vehicle_class_id,
        cpu_threads=args.cpu_threads,
        cpu_affinity=args.cpu_affinity.strip(),
        focal_length_px=args.focal_length,
        car_real_height_m=args.car_height,
    )


def parse_cpu_list(text: str) -> List[int]:
    if not text.strip():
        return []

    cpus: List[int] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        value = int(token)
        if value < 0:
            raise ValueError("CPU indices must be non-negative")
        cpus.append(value)
    return sorted(set(cpus))


def apply_cpu_affinity(settings: Settings) -> None:
    # Linux/Raspberry Pi only. Failure is non-fatal so bench runs still work
    # on other platforms.
    if not settings.cpu_affinity:
        print("[AI CPU] affinity pinning disabled")
        return

    if not hasattr(os, "sched_setaffinity"):
        print("[AI CPU] sched_setaffinity unavailable; continuing without pinning")
        return

    try:
        requested = set(parse_cpu_list(settings.cpu_affinity))
        available = set(os.sched_getaffinity(0))
        selected = requested & available
        if not selected:
            print(
                f"[AI CPU] requested affinity {sorted(requested)} unavailable; "
                f"available={sorted(available)}; continuing without pinning"
            )
            return
        os.sched_setaffinity(0, selected)
        print(f"[AI CPU] affinity={sorted(os.sched_getaffinity(0))}")
    except (OSError, ValueError) as exc:
        print(f"[AI CPU] cannot set affinity: {exc}; continuing without pinning")


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


class YOLO11ONNX:
    """YOLO11 ONNX Runtime wrapper supporting FP32 and QDQ INT8 models."""

    def __init__(self, settings: Settings) -> None:
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.intra_op_num_threads = settings.cpu_threads
        options.inter_op_num_threads = 1

        self.settings = settings
        self.session = ort.InferenceSession(
            str(settings.model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )

        input_meta = self.session.get_inputs()[0]
        self.input_name = input_meta.name
        self.input_type = input_meta.type
        self.input_shape = list(input_meta.shape)

        if len(self.input_shape) != 4:
            raise RuntimeError(f"Unexpected model input shape: {self.input_shape}")

        # Prefer the static model size. For the trained model this is 416x416.
        model_h = self.input_shape[2]
        model_w = self.input_shape[3]
        if isinstance(model_h, int) and isinstance(model_w, int):
            if model_h != model_w:
                raise RuntimeError(
                    f"Only square YOLO input is supported, got {model_w}x{model_h}"
                )
            self.input_size = int(model_h)
            if self.input_size != settings.input_size:
                print(
                    f"[AI] --input-size={settings.input_size} differs from model "
                    f"input={self.input_size}; using model input size"
                )
        else:
            self.input_size = settings.input_size

        output_meta = self.session.get_outputs()[0]
        self.output_name = output_meta.name
        self.output_shape = list(output_meta.shape)

        print(
            f"[AI MODEL] input={self.input_shape} {self.input_type} "
            f"output={self.output_shape}"
        )
        print(
            f"[AI MODEL] classes: parking_sign={settings.parking_sign_class_id}, "
            f"vehicle={settings.vehicle_class_id}"
        )
        print(
            f"[AI CPU] ORT intra={settings.cpu_threads} inter=1 "
            f"OpenCV={cv2.getNumThreads()}"
        )

    def preprocess(
        self, image: np.ndarray
    ) -> Tuple[np.ndarray, float, int, int]:
        height, width = image.shape[:2]
        input_size = self.input_size
        scale = min(input_size / width, input_size / height)
        new_width = max(1, int(round(width * scale)))
        new_height = max(1, int(round(height * scale)))

        resized = cv2.resize(
            image,
            (new_width, new_height),
            interpolation=cv2.INTER_LINEAR,
        )
        canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        pad_x = (input_size - new_width) // 2
        pad_y = (input_size - new_height) // 2
        canvas[pad_y : pad_y + new_height, pad_x : pad_x + new_width] = resized

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

        # Ultralytics QDQ INT8 ONNX normally keeps a float32 graph input.
        # Support uint8 input too in case a fully-quantized model is supplied.
        if self.input_type == "tensor(float)":
            tensor = rgb.astype(np.float32) / 255.0
        elif self.input_type == "tensor(uint8)":
            tensor = rgb.astype(np.uint8, copy=False)
        else:
            raise RuntimeError(f"Unsupported ONNX input type: {self.input_type}")

        tensor = np.transpose(tensor, (2, 0, 1))[None, ...]
        return np.ascontiguousarray(tensor), scale, pad_x, pad_y

    @staticmethod
    def _normalize_predictions(output: np.ndarray) -> np.ndarray:
        predictions = np.squeeze(output)
        if predictions.ndim != 2:
            raise RuntimeError(f"Unexpected YOLO output shape: {output.shape}")

        # Ultralytics YOLO11 detect export is commonly [4+nc, N].
        # Convert to [N, 4+nc].
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        if predictions.shape[1] < 5:
            raise RuntimeError(
                f"Unexpected YOLO prediction shape: {predictions.shape}"
            )
        return predictions

    def postprocess(
        self,
        output: np.ndarray,
        original_shape: Tuple[int, ...],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[dict]:
        """Vectorized YOLO11 postprocess with class-aware NMS."""
        image_height, image_width = original_shape[:2]
        predictions = self._normalize_predictions(output)

        class_scores = predictions[:, 4:]
        if class_scores.shape[1] <= max(
            self.settings.parking_sign_class_id,
            self.settings.vehicle_class_id,
        ):
            raise RuntimeError(
                "Model output does not contain the configured class IDs: "
                f"shape={predictions.shape}"
            )

        class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
        scores = class_scores[np.arange(len(predictions)), class_ids].astype(np.float32)

        wanted = np.logical_or(
            class_ids == self.settings.parking_sign_class_id,
            class_ids == self.settings.vehicle_class_id,
        )
        keep = np.logical_and(wanted, scores >= self.settings.confidence_threshold)

        if not np.any(keep):
            return []

        selected = predictions[keep, :4].astype(np.float32, copy=False)
        selected_scores = scores[keep]
        selected_class_ids = class_ids[keep]

        cx = selected[:, 0]
        cy = selected[:, 1]
        bw = selected[:, 2]
        bh = selected[:, 3]

        x1 = (cx - 0.5 * bw - pad_x) / scale
        y1 = (cy - 0.5 * bh - pad_y) / scale
        x2 = (cx + 0.5 * bw - pad_x) / scale
        y2 = (cy + 0.5 * bh - pad_y) / scale

        x1 = np.clip(np.rint(x1), 0, image_width - 1).astype(np.int32)
        y1 = np.clip(np.rint(y1), 0, image_height - 1).astype(np.int32)
        x2 = np.clip(np.rint(x2), 0, image_width - 1).astype(np.int32)
        y2 = np.clip(np.rint(y2), 0, image_height - 1).astype(np.int32)

        widths = x2 - x1
        heights = y2 - y1
        valid = np.logical_and(widths >= 2, heights >= 2)

        if not np.any(valid):
            return []

        x1 = x1[valid]
        y1 = y1[valid]
        x2 = x2[valid]
        y2 = y2[valid]
        widths = widths[valid]
        heights = heights[valid]
        selected_scores = selected_scores[valid]
        selected_class_ids = selected_class_ids[valid]

        results: List[dict] = []

        # NMS separately for each class so a parking sign cannot suppress a car.
        for class_id in (
            self.settings.parking_sign_class_id,
            self.settings.vehicle_class_id,
        ):
            indices_for_class = np.flatnonzero(selected_class_ids == class_id)
            if indices_for_class.size == 0:
                continue

            boxes = [
                [
                    int(x1[i]),
                    int(y1[i]),
                    int(widths[i]),
                    int(heights[i]),
                ]
                for i in indices_for_class
            ]
            class_scores_list = [
                float(selected_scores[i]) for i in indices_for_class
            ]

            nms_indices = cv2.dnn.NMSBoxes(
                boxes,
                class_scores_list,
                self.settings.confidence_threshold,
                self.settings.nms_threshold,
            )

            if len(nms_indices) == 0:
                continue

            for local_index in np.asarray(nms_indices).reshape(-1):
                source_index = int(indices_for_class[int(local_index)])
                results.append(
                    {
                        "bbox": (
                            int(x1[source_index]),
                            int(y1[source_index]),
                            int(x2[source_index]),
                            int(y2[source_index]),
                        ),
                        "class_id": int(selected_class_ids[source_index]),
                        "score": float(selected_scores[source_index]),
                    }
                )

        return results

    def detect_profiled(
        self, image: np.ndarray
    ) -> Tuple[List[dict], Dict[str, float]]:
        t0 = time.perf_counter()
        tensor, scale, pad_x, pad_y = self.preprocess(image)
        t1 = time.perf_counter()

        outputs = self.session.run(
            [self.output_name],
            {self.input_name: tensor},
        )
        t2 = time.perf_counter()

        detections = self.postprocess(
            outputs[0], image.shape, scale, pad_x, pad_y
        )
        t3 = time.perf_counter()

        return detections, {
            "preprocess_ms": (t1 - t0) * 1000.0,
            "onnx_ms": (t2 - t1) * 1000.0,
            "postprocess_ms": (t3 - t2) * 1000.0,
            "detect_total_ms": (t3 - t0) * 1000.0,
        }

    def detect(self, image: np.ndarray) -> List[dict]:
        detections, _ = self.detect_profiled(image)
        return detections


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


class SceneEstimator:
    """Interpret YOLO detections without changing the existing C++ distance API."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.pixel_heights: Deque[int] = deque(maxlen=5)
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
        self,
        bbox: Tuple[int, int, int, int],
        image_width: int,
        image_height: int,
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
        self, frame: np.ndarray, detections: Sequence[dict]
    ) -> Tuple[np.ndarray, Optional[float], float, bool, float, int]:
        annotated = frame.copy()
        image_height, image_width = annotated.shape[:2]
        self.draw_corridor(annotated)

        best_vehicle_bbox: Optional[Tuple[int, int, int, int]] = None
        best_vehicle_score = 0.0
        best_vehicle_height = -1

        parking_sign_detected = False
        parking_sign_confidence = 0.0
        vehicle_count = 0

        for detection in detections:
            class_id = int(detection["class_id"])
            score = float(detection["score"])
            x1, y1, x2, y2 = detection["bbox"]

            if class_id == self.settings.parking_sign_class_id:
                parking_sign_detected = True
                parking_sign_confidence = max(parking_sign_confidence, score)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(
                    annotated,
                    f"PARKING SIGN {score:.2f}",
                    (x1, max(y1 - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (255, 0, 255),
                    2,
                )
                continue

            if class_id != self.settings.vehicle_class_id:
                continue

            vehicle_count += 1
            pixel_height = y2 - y1
            if pixel_height < 20:
                continue

            in_lane = self.in_ego_lane(
                (x1, y1, x2, y2), image_width, image_height
            )
            color = (0, 255, 0) if in_lane else (0, 0, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated, ((x1 + x2) // 2, y2), 4, color, -1)
            cv2.putText(
                annotated,
                f"VEHICLE {'IN' if in_lane else 'OUT'} lane {score:.2f}",
                (x1, max(y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                2,
            )

            if in_lane and pixel_height > best_vehicle_height:
                best_vehicle_bbox = (x1, y1, x2, y2)
                best_vehicle_score = score
                best_vehicle_height = pixel_height

        if best_vehicle_bbox is not None:
            self.out_of_lane_frames = 0
            x1, y1, x2, y2 = best_vehicle_bbox
            self.pixel_heights.append(y2 - y1)
            smooth_height = float(np.median(self.pixel_heights))
            raw_distance = (
                self.settings.focal_length_px
                * self.settings.car_real_height_m
                / smooth_height
            )
            distance = self.distance_filter.update(raw_distance)
            if distance is None:
                distance_label = f"Obstacle raw={raw_distance:.2f}m filtered=invalid"
            else:
                distance_label = (
                    f"Obstacle raw={raw_distance:.2f}m "
                    f"filtered={distance:.2f}m conf={best_vehicle_score:.2f}"
                )
            cv2.putText(
                annotated,
                distance_label,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
            )
        else:
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
                f"No in-lane vehicle: {status}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
            )

        sign_text = (
            f"Parking sign: YES {parking_sign_confidence:.2f}"
            if parking_sign_detected
            else "Parking sign: NO"
        )
        cv2.putText(
            annotated,
            sign_text,
            (10, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 255) if parking_sign_detected else (180, 180, 180),
            2,
        )

        return (
            annotated,
            distance,
            best_vehicle_score,
            parking_sign_detected,
            parking_sign_confidence,
            vehicle_count,
        )


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

    # Always prefer the newest already-queued frame; never build an AI backlog.
    previous_timeout = sock.gettimeout()
    sock.setblocking(False)

    try:
        drain_deadline = time.monotonic() + 0.010
        for _ in range(512):
            if time.monotonic() >= drain_deadline:
                break
            try:
                packet, address = sock.recvfrom(65535)
            except BlockingIOError:
                break

            frame = reassembler.add_packet(packet, address)
            if frame is not None:
                latest_frame = frame
                latest_address = address
    finally:
        sock.settimeout(previous_timeout)

    assert latest_frame is not None and latest_address is not None
    return latest_frame, latest_address


def perf_summary(values: Deque[float]) -> Tuple[float, float, float]:
    if not values:
        return 0.0, 0.0, 0.0
    array = np.asarray(values, dtype=np.float64)
    return (
        float(np.mean(array)),
        float(np.percentile(array, 50)),
        float(np.percentile(array, 95)),
    )


def run(settings: Settings) -> int:
    if not settings.model_path.is_file():
        print(f"[AI] Model not found: {settings.model_path}")
        return 2

    apply_cpu_affinity(settings)
    cv2.setNumThreads(settings.cpu_threads)

    try:
        detector = YOLO11ONNX(settings)
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
    estimator = SceneEstimator(settings)

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

    stat_start: Optional[float] = None
    stat_last_print = time.monotonic()
    stat_frames = 0
    stat_detections = 0
    stat_vehicle_detections = 0
    stat_sign_detections = 0
    stat_tx = 0

    perf_recv: Deque[float] = deque(maxlen=settings.perf_window)
    perf_pre: Deque[float] = deque(maxlen=settings.perf_window)
    perf_onnx: Deque[float] = deque(maxlen=settings.perf_window)
    perf_post: Deque[float] = deque(maxlen=settings.perf_window)
    perf_estimator: Deque[float] = deque(maxlen=settings.perf_window)
    perf_distance_tx: Deque[float] = deque(maxlen=settings.perf_window)
    perf_monitor: Deque[float] = deque(maxlen=settings.perf_window)
    perf_cycle: Deque[float] = deque(maxlen=settings.perf_window)

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
        "[AI] NOTE: parking_sign is detected by AI, but parking-slot occupancy "
        "association still belongs to the map/parking layer."
    )
    print(
        "[AI] WARNING: focal length and real object height must be calibrated "
        "before driving the real vehicle."
    )

    try:
        while True:
            cycle_t0 = time.perf_counter()
            recv_t0 = cycle_t0

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

            recv_t1 = time.perf_counter()
            recv_ms = (recv_t1 - recv_t0) * 1000.0
            last_complete_frame = time.monotonic()

            if stat_start is None:
                stat_start = time.monotonic()

            try:
                detections, ai_perf = detector.detect_profiled(frame)

                estimator_t0 = time.perf_counter()
                (
                    annotated,
                    distance,
                    _vehicle_confidence,
                    parking_sign_detected,
                    parking_sign_confidence,
                    vehicle_count,
                ) = estimator.process(frame, detections)
                estimator_t1 = time.perf_counter()
            except Exception as exc:
                # Do not send a fresh "no obstacle" packet after an inference
                # failure. The C++ obstacle timeout will stop the vehicle.
                print(f"[AI] Inference error; distance packet suppressed: {exc}")
                continue

            stat_frames += 1
            stat_detections += len(detections)
            stat_vehicle_detections += sum(
                1
                for detection in detections
                if detection["class_id"] == settings.vehicle_class_id
            )
            stat_sign_detections += sum(
                1
                for detection in detections
                if detection["class_id"] == settings.parking_sign_class_id
            )

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
            distance_t0 = time.perf_counter()
            try:
                distance_sock.sendto(message.encode("ascii"), distance_destination)
                stat_tx += 1
            except OSError as exc:
                print(f"[AI] Cannot send distance: {exc}")
            distance_t1 = time.perf_counter()

            cv2.putText(
                annotated,
                f"Pi inference: {filtered_fps:.1f} FPS",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 0),
                2,
            )

            monitor_ms = 0.0
            if (
                settings.monitor_ip
                and settings.monitor_fps > 0.0
                and now - last_monitor_send >= monitor_period
            ):
                monitor_t0 = time.perf_counter()
                try:
                    monitor_sender.send(
                        annotated,
                        monitor_destination,
                        settings.monitor_jpeg_quality,
                    )
                except OSError as exc:
                    print(f"[AI] Detection monitor send error: {exc}")
                monitor_ms = (time.perf_counter() - monitor_t0) * 1000.0
                last_monitor_send = now

            cycle_ms = (time.perf_counter() - cycle_t0) * 1000.0
            estimator_ms = (estimator_t1 - estimator_t0) * 1000.0
            distance_tx_ms = (distance_t1 - distance_t0) * 1000.0

            perf_recv.append(recv_ms)
            perf_pre.append(ai_perf["preprocess_ms"])
            perf_onnx.append(ai_perf["onnx_ms"])
            perf_post.append(ai_perf["postprocess_ms"])
            perf_estimator.append(estimator_ms)
            perf_distance_tx.append(distance_tx_ms)
            perf_monitor.append(monitor_ms)
            perf_cycle.append(cycle_ms)

            if now - stat_last_print >= 2.0:
                elapsed = max(now - (stat_start or now), 1e-6)
                print(
                    f"[AI STAT] frames={stat_frames} "
                    f"processHz={stat_frames / elapsed:.2f} "
                    f"filteredFps={filtered_fps:.2f} "
                    f"det={stat_detections} "
                    f"vehicle={stat_vehicle_detections} "
                    f"parkingSign={stat_sign_detections} "
                    f"signNow={'YES' if parking_sign_detected else 'NO'} "
                    f"signConf={parking_sign_confidence:.2f} "
                    f"tx={stat_tx} last={message}"
                )

                recv_avg, recv_p50, recv_p95 = perf_summary(perf_recv)
                pre_avg, pre_p50, pre_p95 = perf_summary(perf_pre)
                onnx_avg, onnx_p50, onnx_p95 = perf_summary(perf_onnx)
                post_avg, post_p50, post_p95 = perf_summary(perf_post)
                est_avg, est_p50, est_p95 = perf_summary(perf_estimator)
                tx_avg, tx_p50, tx_p95 = perf_summary(perf_distance_tx)
                mon_avg, mon_p50, mon_p95 = perf_summary(perf_monitor)
                cycle_avg, cycle_p50, cycle_p95 = perf_summary(perf_cycle)

                active_avg = pre_avg + onnx_avg + post_avg + est_avg + tx_avg + mon_avg
                onnx_share = 100.0 * onnx_avg / max(cycle_avg, 1e-6)
                cycle_hz = 1000.0 / max(cycle_avg, 1e-6)

                print(
                    "[AI PERF]\n"
                    f" receive/decode : avg={recv_avg:7.2f} ms "
                    f"p50={recv_p50:7.2f} p95={recv_p95:7.2f}\n"
                    f" preprocess     : avg={pre_avg:7.2f} ms "
                    f"p50={pre_p50:7.2f} p95={pre_p95:7.2f}\n"
                    f" ONNX           : avg={onnx_avg:7.2f} ms "
                    f"p50={onnx_p50:7.2f} p95={onnx_p95:7.2f}\n"
                    f" postprocess    : avg={post_avg:7.2f} ms "
                    f"p50={post_p50:7.2f} p95={post_p95:7.2f}\n"
                    f" estimator      : avg={est_avg:7.2f} ms "
                    f"p50={est_p50:7.2f} p95={est_p95:7.2f}\n"
                    f" distance TX    : avg={tx_avg:7.2f} ms "
                    f"p50={tx_p50:7.2f} p95={tx_p95:7.2f}\n"
                    f" monitor send   : avg={mon_avg:7.2f} ms "
                    f"p50={mon_p50:7.2f} p95={mon_p95:7.2f}\n"
                    f" whole cycle    : avg={cycle_avg:7.2f} ms "
                    f"p50={cycle_p50:7.2f} p95={cycle_p95:7.2f}\n"
                    f" active(no recv): {active_avg:7.2f} ms\n"
                    f" ONNX share     : {onnx_share:7.2f}%\n"
                    f" cycle Hz       : {cycle_hz:7.2f}"
                )
                stat_last_print = now

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
