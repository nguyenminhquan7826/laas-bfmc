#!/usr/bin/env python3
"""Run YOLO ONNX on the Raspberry Pi and stream annotated JPEG frames over TCP."""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


MAGIC = b"DDBG"
HEADER = struct.Struct("!4sIIQ")  # magic, JPEG bytes, frame sequence, capture timestamp us
CLASS_NAME = "Red_Car"


class LatestFrameCamera:
    """Continuously capture frames and retain only the newest one."""

    def __init__(
        self,
        source: str,
        width: int,
        height: int,
        fps: int,
        gstreamer: bool,
        picamera2: bool,
        rpicam: bool,
    ):
        self._source = source
        self._width = width
        self._height = height
        self._fps = fps
        self._gstreamer = gstreamer
        self._use_picamera2 = picamera2
        self._use_rpicam = rpicam
        self._cap: Optional[cv2.VideoCapture] = None
        self._picam2 = None
        self._rpicam_process: Optional[subprocess.Popen] = None
        self._mjpeg_buffer = bytearray()
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._timestamp_us = 0
        self._frame_counter = 0
        self._last_returned_counter = -1
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._use_rpicam:
            executable = shutil.which("rpicam-vid")
            if executable is None:
                raise RuntimeError(
                    "rpicam-vid was not found. On Ubuntu run: "
                    "sudo apt install -y rpicam-apps"
                )

            try:
                camera_index = int(self._source)
            except ValueError as error:
                raise RuntimeError("--camera must be a numeric index with --rpicam") from error

            command = [
                executable,
                "--camera",
                str(camera_index),
                "--width",
                str(self._width),
                "--height",
                str(self._height),
                "--framerate",
                str(self._fps),
                "--timeout",
                "0",
                "--codec",
                "mjpeg",
                "--nopreview",
                "--flush",
                "--output",
                "-",
            ]
            try:
                self._rpicam_process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=None,
                    bufsize=0,
                )
            except OSError as error:
                raise RuntimeError(f"Cannot start rpicam-vid: {error}") from error

            if self._rpicam_process.stdout is None:
                self._rpicam_process.terminate()
                self._rpicam_process = None
                raise RuntimeError("rpicam-vid stdout pipe is unavailable")
        elif self._use_picamera2:
            try:
                from picamera2 import Picamera2
            except ImportError as error:
                raise RuntimeError(
                    "Picamera2 is not installed. On Raspberry Pi OS run: "
                    "sudo apt install -y python3-picamera2 --no-install-recommends"
                ) from error

            try:
                camera_index = int(self._source)
            except ValueError as error:
                raise RuntimeError("--camera must be a numeric index with --picamera2") from error

            try:
                self._picam2 = Picamera2(camera_index)
                configuration = self._picam2.create_video_configuration(
                    main={"format": "RGB888", "size": (self._width, self._height)},
                    controls={"FrameRate": float(self._fps)},
                    buffer_count=4,
                )
                self._picam2.configure(configuration)
                self._picam2.start()
            except Exception as error:
                if self._picam2 is not None:
                    self._picam2.close()
                    self._picam2 = None
                raise RuntimeError(f"Cannot start Picamera2 camera {camera_index}: {error}") from error
        elif self._gstreamer:
            self._cap = cv2.VideoCapture(self._source, cv2.CAP_GSTREAMER)
        else:
            try:
                camera_index = int(self._source)
                self._cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            except ValueError:
                self._cap = cv2.VideoCapture(self._source)

        if not self._use_picamera2 and not self._use_rpicam:
            assert self._cap is not None
            if not self._cap.isOpened():
                raise RuntimeError(f"Cannot open camera source: {self._source}")

        if not self._use_picamera2 and not self._use_rpicam and not self._gstreamer:
            assert self._cap is not None
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        consecutive_failures = 0
        while self._running:
            if self._use_rpicam:
                assert self._rpicam_process is not None
                assert self._rpicam_process.stdout is not None
                try:
                    chunk = os.read(self._rpicam_process.stdout.fileno(), 65536)
                except OSError as error:
                    print(f"\n[CAM] rpicam-vid read failed: {error}")
                    self._running = False
                    break

                if not chunk:
                    return_code = self._rpicam_process.poll()
                    if return_code is not None:
                        print(f"\n[CAM] rpicam-vid exited with code {return_code}")
                        self._running = False
                        break
                    continue

                self._mjpeg_buffer.extend(chunk)
                frame = None
                while True:
                    start = self._mjpeg_buffer.find(b"\xff\xd8")
                    if start < 0:
                        if len(self._mjpeg_buffer) > 1:
                            del self._mjpeg_buffer[:-1]
                        break
                    end = self._mjpeg_buffer.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start > 0:
                            del self._mjpeg_buffer[:start]
                        break

                    jpeg = bytes(self._mjpeg_buffer[start : end + 2])
                    del self._mjpeg_buffer[: end + 2]
                    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if decoded is not None:
                        frame = decoded

                if frame is None:
                    continue
                ok = True
                timestamp_us = time.monotonic_ns() // 1_000
            elif self._use_picamera2:
                assert self._picam2 is not None
                try:
                    with self._picam2.captured_request() as request:
                        frame = request.make_array("main")
                        metadata = request.get_metadata()
                    sensor_timestamp_ns = metadata.get("SensorTimestamp")
                    timestamp_us = (
                        int(sensor_timestamp_ns) // 1_000
                        if sensor_timestamp_ns is not None
                        else time.monotonic_ns() // 1_000
                    )
                    ok = frame is not None
                except Exception as error:
                    print(f"\n[CAM] Picamera2 capture failed: {error}")
                    ok, frame = False, None
                    timestamp_us = time.monotonic_ns() // 1_000
            else:
                assert self._cap is not None
                ok, frame = self._cap.read()
                timestamp_us = time.monotonic_ns() // 1_000

            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= 100:
                    self._running = False
                    break
                time.sleep(0.01)
                continue

            consecutive_failures = 0
            with self._lock:
                self._frame = frame
                self._timestamp_us = timestamp_us
                self._frame_counter += 1

    def read_latest(self, timeout_s: float = 2.0) -> Tuple[Optional[np.ndarray], int]:
        deadline = time.monotonic() + timeout_s
        while self._running and time.monotonic() < deadline:
            with self._lock:
                if self._frame is not None and self._frame_counter != self._last_returned_counter:
                    self._last_returned_counter = self._frame_counter
                    return self._frame.copy(), self._timestamp_us
            time.sleep(0.002)
        return None, 0

    def stop(self) -> None:
        self._running = False
        if self._rpicam_process is not None and self._rpicam_process.poll() is None:
            self._rpicam_process.terminate()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            finally:
                self._picam2.close()
                self._picam2 = None
        if self._rpicam_process is not None:
            try:
                self._rpicam_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._rpicam_process.kill()
                self._rpicam_process.wait(timeout=1.0)
            self._rpicam_process = None


def letterbox(image: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

    pad_width = size - resized_width
    pad_height = size - resized_height
    left = pad_width // 2
    right = pad_width - left
    top = pad_height // 2
    bottom = pad_height - top
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return padded, scale, left, top


def normalize_predictions(output: np.ndarray) -> np.ndarray:
    """Return predictions as [number_of_candidates, 4 + number_of_classes]."""
    predictions = np.squeeze(output)
    if predictions.ndim != 2:
        raise RuntimeError(f"Unexpected model output shape: {output.shape}")
    if predictions.shape[0] <= 16 and predictions.shape[1] > predictions.shape[0]:
        predictions = predictions.T
    if predictions.shape[1] < 5:
        raise RuntimeError(f"Unexpected prediction shape after transpose: {predictions.shape}")
    return predictions


def detect(
    net: cv2.dnn.Net,
    frame: np.ndarray,
    input_size: int,
    confidence_threshold: float,
    nms_threshold: float,
) -> Tuple[np.ndarray, int, float, float, int]:
    input_image, scale, pad_x, pad_y = letterbox(frame, input_size)
    blob = cv2.dnn.blobFromImage(
        input_image,
        scalefactor=1.0 / 255.0,
        size=(input_size, input_size),
        swapRB=True,
        crop=False,
    )

    net.setInput(blob)
    start = time.perf_counter()
    output = net.forward()
    inference_ms = (time.perf_counter() - start) * 1000.0
    predictions = normalize_predictions(output)

    class_scores = predictions[:, 4:]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = class_scores[np.arange(len(predictions)), class_ids]
    max_confidence = float(np.max(confidences)) if len(confidences) else 0.0

    frame_height, frame_width = frame.shape[:2]
    boxes = []
    scores = []
    kept_class_ids = []

    candidate_indices = np.flatnonzero(confidences >= confidence_threshold)
    for index in candidate_indices:
        cx, cy, box_width, box_height = predictions[index, :4]
        x1 = (cx - box_width / 2.0 - pad_x) / scale
        y1 = (cy - box_height / 2.0 - pad_y) / scale
        x2 = (cx + box_width / 2.0 - pad_x) / scale
        y2 = (cy + box_height / 2.0 - pad_y) / scale

        x1 = int(np.clip(round(x1), 0, frame_width - 1))
        y1 = int(np.clip(round(y1), 0, frame_height - 1))
        x2 = int(np.clip(round(x2), 0, frame_width - 1))
        y2 = int(np.clip(round(y2), 0, frame_height - 1))
        width = x2 - x1
        height = y2 - y1
        if width <= 1 or height <= 1:
            continue

        boxes.append([x1, y1, width, height])
        scores.append(float(confidences[index]))
        kept_class_ids.append(int(class_ids[index]))

    nms_indices = []
    if boxes:
        raw_indices = cv2.dnn.NMSBoxes(boxes, scores, confidence_threshold, nms_threshold)
        if len(raw_indices):
            nms_indices = np.asarray(raw_indices).reshape(-1).tolist()

    annotated = frame.copy()
    for index in nms_indices:
        x, y, width, height = boxes[index]
        score = scores[index]
        class_id = kept_class_ids[index]
        class_name = CLASS_NAME if class_id == 0 else f"class_{class_id}"
        cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.putText(
            annotated,
            f"{class_name} {score:.2f}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return annotated, len(nms_indices), max_confidence, inference_ms, len(boxes)


def connect_to_viewer(host: str, port: int) -> socket.socket:
    while True:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            print(f"[NET] Connecting to {host}:{port} ...")
            client.connect((host, port))
            print("[NET] Connected")
            return client
        except OSError as error:
            print(f"[NET] Viewer unavailable: {error}; retrying in 2 s")
            client.close()
            time.sleep(2.0)


def rotate_frame(frame: np.ndarray, angle: int) -> np.ndarray:
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="best.onnx", help="ONNX model path")
    parser.add_argument("--host", required=True, help="Laptop IPv4 address")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--camera", default="0", help="Camera index, video path, or pipeline")
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument(
        "--gstreamer", action="store_true", help="Treat --camera as GStreamer pipeline"
    )
    backend.add_argument(
        "--picamera2",
        action="store_true",
        help="Use Raspberry Pi CSI camera through libcamera/Picamera2",
    )
    backend.add_argument(
        "--rpicam",
        action="store_true",
        help="Use Raspberry Pi CSI camera through rpicam-vid MJPEG (Ubuntu-compatible)",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.10, help="Debug confidence threshold")
    parser.add_argument("--nms", type=float, default=0.45)
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)

    cv2.setNumThreads(4)
    net = cv2.dnn.readNetFromONNX(str(model_path))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    print(f"[MODEL] Loaded {model_path}")

    camera = LatestFrameCamera(
        args.camera,
        args.width,
        args.height,
        args.camera_fps,
        args.gstreamer,
        args.picamera2,
        args.rpicam,
    )
    camera.start()
    camera_backend = (
        "rpicam-vid"
        if args.rpicam
        else "Picamera2"
        if args.picamera2
        else "GStreamer"
        if args.gstreamer
        else "V4L2"
    )
    print(f"[CAM] Opened {args.camera} via {camera_backend}")

    client: Optional[socket.socket] = None
    sequence = 0
    try:
        client = connect_to_viewer(args.host, args.port)
        while True:
            frame, capture_timestamp_us = camera.read_latest()
            if frame is None:
                raise RuntimeError("Camera stopped or no new frame was received")
            frame = rotate_frame(frame, args.rotate)

            annotated, detection_count, max_confidence, inference_ms, candidate_count = detect(
                net, frame, args.input_size, args.conf, args.nms
            )
            fps = 1000.0 / inference_ms if inference_ms > 0.0 else 0.0
            cv2.putText(
                annotated,
                f"seq={sequence} det={detection_count} cand={candidate_count} max={max_confidence:.3f}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                f"infer={inference_ms:.1f} ms ({fps:.2f} FPS)",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            ok, encoded = cv2.imencode(
                ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )
            if not ok:
                print("[JPEG] Encoding failed")
                continue
            payload = encoded.tobytes()

            while True:
                if client is None:
                    client = connect_to_viewer(args.host, args.port)
                try:
                    client.sendall(HEADER.pack(MAGIC, len(payload), sequence, capture_timestamp_us))
                    client.sendall(payload)
                    break
                except OSError as error:
                    print(f"[NET] Send failed: {error}")
                    client.close()
                    client = None

            print(
                f"\r[RUN] seq={sequence} det={detection_count} max={max_confidence:.3f} "
                f"infer={inference_ms:.1f}ms jpeg={len(payload) / 1024:.1f}KiB",
                end="",
                flush=True,
            )
            sequence = (sequence + 1) & 0xFFFFFFFF
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        camera.stop()
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()