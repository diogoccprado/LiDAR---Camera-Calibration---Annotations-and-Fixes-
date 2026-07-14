#!/usr/bin/env python3
"""Compose a CalibAnything LiDAR-to-camera transform into FAST-LIO2's IMU frame."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-lidar",
        type=Path,
        required=True,
        help="4x4 T_color_optical_lidar matrix file",
    )
    parser.add_argument(
        "--imu-camera",
        type=Path,
        required=True,
        help="4x4 T_imu_color_optical matrix file",
    )
    parser.add_argument("-o", "--output", type=Path, help="optional JSON output path")
    return parser.parse_args()


def load_matrix(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8")
    try:
        value = json.loads(text)
        if isinstance(value, dict):
            value = value.get("data", value.get("matrix"))
        matrix = np.asarray(value, dtype=float)
    except (json.JSONDecodeError, TypeError, ValueError):
        numbers = [
            float(value)
            for value in re.findall(
                r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", text
            )
        ]
        if len(numbers) == 17 and numbers[0] in (4.0, 16.0):
            numbers = numbers[1:]
        if len(numbers) != 16:
            raise ValueError(f"{path} does not contain exactly 16 matrix values")
        matrix = np.asarray(numbers, dtype=float).reshape(4, 4)
    if matrix.shape != (4, 4):
        raise ValueError(f"{path} has shape {matrix.shape}; expected 4x4")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f"{path} is not a homogeneous transform")
    return matrix


def main() -> int:
    args = parse_args()
    try:
        camera_lidar = load_matrix(args.camera_lidar)
        imu_camera = load_matrix(args.imu_camera)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

    imu_lidar = imu_camera @ camera_lidar
    result = {
        "convention": "p_imu = extrinsic_R * p_lidar + extrinsic_T",
        "T_imu_lidar": imu_lidar.tolist(),
        "extrinsic_T": imu_lidar[:3, 3].tolist(),
        "extrinsic_R": imu_lidar[:3, :3].reshape(-1).tolist(),
    }
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"Wrote {args.output}")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
