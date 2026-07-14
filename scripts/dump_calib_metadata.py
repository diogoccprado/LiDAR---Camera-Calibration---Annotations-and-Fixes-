#!/usr/bin/env python3
"""Extract camera intrinsics and PointCloud2 field metadata from a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--pointcloud-topic", required=True)
    parser.add_argument("-o", "--output", type=Path, help="write JSON to this path")
    return parser.parse_args()


def camera_info_dict(message: Any) -> dict[str, Any]:
    return {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "K": [float(value) for value in message.k],
        "D": [float(value) for value in message.d],
        "R": [float(value) for value in message.r],
        "P": [float(value) for value in message.p],
    }


def pointcloud_dict(message: Any) -> dict[str, Any]:
    return {
        "frame_id": message.header.frame_id,
        "fields": [
            {
                "name": field.name,
                "offset": int(field.offset),
                "datatype": int(field.datatype),
                "count": int(field.count),
            }
            for field in message.fields
        ],
        "width": int(message.width),
        "height": int(message.height),
        "point_step": int(message.point_step),
        "row_step": int(message.row_step),
        "is_bigendian": bool(message.is_bigendian),
        "is_dense": bool(message.is_dense),
    }


def main() -> int:
    args = parse_args()
    if not (args.bag / "metadata.yaml").exists():
        raise SystemExit(f"{args.bag} does not look like a rosbag2 directory")

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise SystemExit("Source ROS 2 before running this script.") from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    requested = {args.camera_info_topic, args.pointcloud_topic}
    missing = requested - available.keys()
    if missing:
        raise SystemExit(f"Topics not found in bag: {sorted(missing)}")
    message_types = {
        topic: get_message(available[topic])
        for topic in requested
    }

    result: dict[str, Any] = {}
    while reader.has_next() and len(result) < 2:
        topic, serialized, _ = reader.read_next()
        if topic not in requested:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == args.camera_info_topic and "camera_info" not in result:
            result["camera_info"] = camera_info_dict(message)
        elif topic == args.pointcloud_topic and "pointcloud" not in result:
            result["pointcloud"] = pointcloud_dict(message)

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
