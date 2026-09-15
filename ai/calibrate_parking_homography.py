#!/usr/bin/env python3
"""Calibrate image-pixel -> vehicle-ground homography for parking occupancy.

Each --point is u_px,v_px,forward_m,left_m. Use points on the flat road plane
after CameraInterface undistortion. +forward follows the vehicle heading and
+left points to the vehicle's left side.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


def parse_point(text: str) -> Tuple[float, float, float, float]:
    try:
        values = tuple(float(value.strip()) for value in text.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid point: {text}") from exc
    if len(values) != 4 or not np.all(np.isfinite(values)):
        raise argparse.ArgumentTypeError(
            "point must be u_px,v_px,forward_m,left_m"
        )
    return values  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--point",
        action="append",
        required=True,
        type=parse_point,
        help="u_px,v_px,forward_m,left_m; provide at least four",
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="optional undistorted image used to verify source points",
    )
    parser.add_argument(
        "--preview",
        type=Path,
        default=Path("parking_homography_points.jpg"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    points: List[Tuple[float, float, float, float]] = args.point
    if len(points) < 4:
        raise SystemExit("At least four non-collinear points are required")

    image_points = np.asarray([(p[0], p[1]) for p in points], dtype=np.float64)
    ground_points = np.asarray([(p[2], p[3]) for p in points], dtype=np.float64)
    homography, inlier_mask = cv2.findHomography(
        image_points, ground_points, method=0
    )
    if homography is None or not np.all(np.isfinite(homography)):
        raise SystemExit("Homography solve failed; check point order/geometry")

    projected = cv2.perspectiveTransform(
        image_points.reshape(1, -1, 2), homography
    ).reshape(-1, 2)
    errors = np.linalg.norm(projected - ground_points, axis=1)
    coefficients = ",".join(f"{value:.12g}" for value in homography.reshape(-1))

    print(f"points={len(points)} inliers={int(np.sum(inlier_mask))}")
    print(
        f"ground_error_m: mean={float(np.mean(errors)):.6f} "
        f"max={float(np.max(errors)):.6f}"
    )
    print("Run the C++ client with:")
    print("LAAS_PARKING_PERCEPTION=1 \\")
    print(f"LAAS_PARKING_IMAGE_TO_GROUND_H='{coefficients}' \\")
    print("./build/laas_pp pp")

    if args.image is not None:
        image = cv2.imread(str(args.image))
        if image is None:
            raise SystemExit(f"Cannot read image: {args.image}")
        for index, (u, v, forward, left) in enumerate(points, start=1):
            pixel = (int(round(u)), int(round(v)))
            cv2.circle(image, pixel, 6, (0, 255, 255), -1)
            cv2.putText(
                image,
                f"{index}:F={forward:.2f},L={left:.2f}",
                (pixel[0] + 8, pixel[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if not cv2.imwrite(str(args.preview), image):
            raise SystemExit(f"Cannot write preview: {args.preview}")
        print(f"preview={args.preview}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
