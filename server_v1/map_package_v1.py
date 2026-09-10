#!/usr/bin/env python3
"""Versioned operational-map package shared by Server and Raspberry Pi."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


MAP_PACKAGE_FILES = (
    "map_v1.yaml",
    "vehicle_v1.yaml",
    "planner_v1.yaml",
    "drivable_grid_v1.json",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_manifest(root: Path) -> dict[str, Any]:
    files = []
    package_digest = hashlib.sha256()
    for name in MAP_PACKAGE_FILES:
        path = root / name
        if not path.is_file():
            raise FileNotFoundError(f"map_package_file_missing:{name}")
        size = path.stat().st_size
        digest = _sha256(path)
        files.append({"path": name, "size_bytes": size, "sha256": digest})
        package_digest.update(f"{name}\0{size}\0{digest}\n".encode("utf-8"))
    return {
        "schema_version": 1,
        "map_id": "map_v1",
        "package_version": 1,
        "package_sha256": package_digest.hexdigest(),
        "yaw_convention": "MAP_CCW_POSITIVE_SENSOR_CW_CONVERTED_AT_PI",
        "files": files,
    }


def load_and_verify_manifest(root: Path) -> dict[str, Any]:
    manifest_path = root / "map_manifest_v1.json"
    try:
        expected = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"map_manifest_unreadable:{exc}") from exc
    actual = build_manifest(root)
    if expected != actual:
        raise ValueError(
            "map_package_checksum_mismatch:"
            f"expected={expected.get('package_sha256')}:"
            f"actual={actual['package_sha256']}"
        )
    return actual

