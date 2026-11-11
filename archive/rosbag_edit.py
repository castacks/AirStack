#!/usr/bin/env python3
import os
import sys
import argparse
import shutil

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)


def normalize_topic(t: str) -> str:
    return t if t.startswith("/") else "/" + t


def main():
    parser = argparse.ArgumentParser(
        description="Rewrite a ROS 2 bag: set Odometry.header.frame_id from '' to 'world' for a given topic."
    )
    parser.add_argument("input_bag", help="Path to input bag directory (contains metadata.yaml)")
    parser.add_argument("output_bag", help="Path to output bag directory (must NOT exist; will be created)")
    parser.add_argument("--topic", default="qvio/odom", help="Odometry topic name (default: qvio/odom)")
    parser.add_argument("--new-frame", default="world", help="New frame_id when empty (default: world)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output bag if it exists")
    args = parser.parse_args()

    in_bag = args.input_bag
    out_bag = args.output_bag
    target_topic = normalize_topic(args.topic)
    new_frame = args.new_frame

    if not os.path.isdir(in_bag):
        print(f"[ERROR] input_bag is not a directory: {in_bag}", file=sys.stderr)
        sys.exit(2)

    if os.path.exists(out_bag):
        if args.overwrite:
            shutil.rmtree(out_bag)
        else:
            print(f"[ERROR] output_bag already exists: {out_bag} (use --overwrite)", file=sys.stderr)
            sys.exit(2)

    rclpy.init(args=None)

    reader = SequentialReader()
    writer = SequentialWriter()

    storage_in = StorageOptions(uri=in_bag, storage_id="sqlite3")
    storage_out = StorageOptions(uri=out_bag, storage_id="sqlite3")

    converter = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

    reader.open(storage_in, converter)
    writer.open(storage_out, converter)

    topics = reader.get_all_topics_and_types()
    type_map = {}
    for t in topics:
        type_map[t.name] = t.type
        writer.create_topic(TopicMetadata(name=t.name, type=t.type, serialization_format="cdr"))

    # 兼容 /qvio/odom 与 qvio/odom
    if target_topic not in type_map:
        alt = target_topic.lstrip("/")
        candidates = [target_topic, alt, normalize_topic(alt)]
        found = next((c for c in candidates if c in type_map), None)
        if found is None:
            print("[ERROR] Target topic not found in bag.")
            print("        Available topics:")
            for n in sorted(type_map.keys()):
                print(f"        - {n}  ({type_map[n]})")
            rclpy.shutdown()
            sys.exit(3)
        target_topic = found

    target_type = type_map[target_topic]
    MsgClass = get_message(target_type)

    changed = 0
    total_target = 0
    total_all = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        total_all += 1

        if topic == target_topic:
            total_target += 1
            msg = deserialize_message(data, MsgClass)

            msg.header.frame_id = new_frame
            changed += 1
            data = serialize_message(msg)

        writer.write(topic, data, t)

    rclpy.shutdown()

    print(f"[OK] Done. Wrote new bag: {out_bag}")
    print(f"     Target topic: {target_topic}")
    print(f"     Messages on target topic: {total_target}")
    print(f"     Changed frame_id: {changed}")
    print(f"     Total messages copied: {total_all}")


if __name__ == "__main__":
    main()
