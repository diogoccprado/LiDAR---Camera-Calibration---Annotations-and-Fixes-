#!/usr/bin/env python3
"""Export approximately synchronized Image and PointCloud2 pairs from a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory (contains metadata.yaml)")
    parser.add_argument("output", type=Path, help="output dataset directory")
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--pointcloud-topic", required=True)
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.05,
        help="maximum image/cloud timestamp difference in seconds (default: 0.05)",
    )
    parser.add_argument("--every", type=int, default=1, help="keep every Nth matched pair")
    parser.add_argument("--max-pairs", type=int, help="stop after writing this many pairs")
    parser.add_argument(
        "--timestamp-source",
        choices=("header", "bag"),
        default="header",
        help="use message header stamps or bag receive times",
    )
    parser.add_argument(
        "--allow-missing-intensity",
        action="store_true",
        help="write zero intensity when the cloud has no intensity field",
    )
    return parser.parse_args()


def message_time(message: Any, bag_time_ns: int, source: str) -> int:
    if source == "header" and hasattr(message, "header"):
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns:
            return stamp_ns
    return int(bag_time_ns)


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


def cloud_to_xyzi(message: Any, allow_missing_intensity: bool) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    field_names = {field.name for field in message.fields}
    required = {"x", "y", "z"}
    if not required.issubset(field_names):
        raise ValueError(f"PointCloud2 is missing fields: {sorted(required - field_names)}")
    has_intensity = "intensity" in field_names
    if not has_intensity and not allow_missing_intensity:
        raise ValueError(
            "PointCloud2 has no 'intensity' field; CalibAnything requires it. "
            "Use --allow-missing-intensity only if zero intensity is acceptable."
        )

    selected = ("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z")
    points = point_cloud2.read_points(message, field_names=selected, skip_nans=True)
    array = np.asarray(points)

    if array.dtype.names:
        columns = [np.asarray(array[name], dtype=np.float32) for name in selected]
        xyzi = np.column_stack(columns)
    else:
        xyzi = np.asarray(list(points) if array.ndim == 0 else array, dtype=np.float32)
        xyzi = xyzi.reshape((-1, len(selected)))

    if not has_intensity:
        xyzi = np.column_stack((xyzi, np.zeros(len(xyzi), dtype=np.float32)))
    return np.ascontiguousarray(xyzi, dtype="<f4")


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with path.open("wb") as output:
        output.write(header)
        output.write(points.tobytes(order="C"))


def main() -> int:
    args = parse_args()
    if args.every < 1:
        raise SystemExit("--every must be at least 1")
    if args.sync_tolerance < 0:
        raise SystemExit("--sync-tolerance cannot be negative")
    if not (args.bag / "metadata.yaml").exists():
        raise SystemExit(f"{args.bag} does not look like a rosbag2 directory")

    try:
        import rosbag2_py
        from cv_bridge import CvBridge
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise SystemExit(
            "Run this script in a sourced ROS 2 environment with rosbag2_py, "
            "cv_bridge, sensor_msgs_py, and OpenCV available."
        ) from error

    image_dir = args.output / "images"
    cloud_dir = args.output / "pc"
    image_dir.mkdir(parents=True, exist_ok=True)
    cloud_dir.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    requested = {
        args.image_topic,
        args.camera_info_topic,
        args.pointcloud_topic,
    }
    missing = requested - topic_types.keys()
    if missing:
        raise SystemExit(f"Topics not found in bag: {sorted(missing)}")

    message_types = {topic: get_message(topic_types[topic]) for topic in requested}
    images: deque[tuple[int, Any]] = deque()
    clouds: deque[tuple[int, Any]] = deque()
    bridge = CvBridge()
    tolerance_ns = round(args.sync_tolerance * 1_000_000_000)
    camera_info: dict[str, Any] | None = None
    cloud_metadata: dict[str, Any] | None = None
    matched = written = discarded_images = discarded_clouds = 0
    latest_sensor_time = 0
    stop = False

    def write_pair(image_message: Any, cloud_message: Any) -> None:
        nonlocal matched, written, cloud_metadata, stop
        pair_number = matched
        matched += 1
        if pair_number % args.every:
            return

        stem = f"{written:06d}"
        image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        image_path = image_dir / f"{stem}.png"
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f"Failed to write {image_path}")

        points = cloud_to_xyzi(cloud_message, args.allow_missing_intensity)
        write_binary_pcd(cloud_dir / f"{stem}.pcd", points)
        if cloud_metadata is None:
            cloud_metadata = {
                "frame_id": cloud_message.header.frame_id,
                "fields": [field.name for field in cloud_message.fields],
                "source_width": int(cloud_message.width),
                "source_height": int(cloud_message.height),
                "exported_points": int(len(points)),
            }
        written += 1
        if args.max_pairs is not None and written >= args.max_pairs:
            stop = True

    def process_ready_pairs(final: bool = False) -> None:
        nonlocal discarded_images, discarded_clouds
        while images and clouds and not stop:
            cloud_time, cloud_message = clouds[0]
            while images and images[0][0] < cloud_time - tolerance_ns:
                images.popleft()
                discarded_images += 1
            if not images:
                return
            if images[0][0] > cloud_time + tolerance_ns:
                clouds.popleft()
                discarded_clouds += 1
                continue
            if not final and latest_sensor_time <= cloud_time + tolerance_ns:
                return

            candidate_count = 0
            for image_time, _ in images:
                if image_time > cloud_time + tolerance_ns:
                    break
                candidate_count += 1
            candidates = [images[index] for index in range(candidate_count)]
            best_index = min(
                range(candidate_count),
                key=lambda index: abs(candidates[index][0] - cloud_time),
            )
            for _ in range(best_index):
                images.popleft()
                discarded_images += 1
            _, image_message = images.popleft()
            clouds.popleft()
            write_pair(image_message, cloud_message)

    while reader.has_next() and not stop:
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in requested:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == args.camera_info_topic:
            if camera_info is None:
                camera_info = camera_info_dict(message)
            continue

        timestamp = message_time(message, bag_time_ns, args.timestamp_source)
        latest_sensor_time = max(latest_sensor_time, timestamp)
        (images if topic == args.image_topic else clouds).append((timestamp, message))
        process_ready_pairs()

    if not stop:
        process_ready_pairs(final=True)

    metadata = {
        "bag": str(args.bag.resolve()),
        "topics": {
            "image": args.image_topic,
            "camera_info": args.camera_info_topic,
            "pointcloud": args.pointcloud_topic,
        },
        "sync_tolerance_seconds": args.sync_tolerance,
        "timestamp_source": args.timestamp_source,
        "matched_pairs_before_subsampling": matched,
        "written_pairs": written,
        "discarded_images": discarded_images,
        "discarded_pointclouds": discarded_clouds,
        "camera_info": camera_info,
        "pointcloud": cloud_metadata,
    }
    metadata_path = args.output / "calib_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {written} pairs to {args.output}")
    print(f"Metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
