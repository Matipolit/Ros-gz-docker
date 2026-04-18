#!/usr/bin/env python3
import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def _truncate(text: str, max_len: int) -> str:
    if len(text) <= max_len:
        return text
    return text[: max_len - 3] + "..."


def list_topics_with_one_example(input_bag_file: str, max_example_chars: int) -> None:
    bag_path = Path(input_bag_file)
    if not bag_path.exists():
        raise FileNotFoundError(f"Input bag does not exist: {input_bag_file}")

    typestore = get_typestore(Stores.ROS2_JAZZY)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections_by_topic = {}
        for connection in reader.connections:
            connections_by_topic.setdefault(connection.topic, []).append(connection)

        topics = sorted(connections_by_topic.keys())
        print(f"Found {len(topics)} topics in bag: {bag_path}")

        if not topics:
            return

        first_message_by_topic = {}
        remaining = set(topics)

        for connection, timestamp_ns, rawdata in reader.messages():
            topic = connection.topic
            if topic not in remaining:
                continue

            try:
                msg = reader.deserialize(rawdata, connection.msgtype)
                msg_preview = _truncate(repr(msg), max_example_chars)
                first_message_by_topic[topic] = {
                    "timestamp_ns": int(timestamp_ns),
                    "msgtype": connection.msgtype,
                    "preview": msg_preview,
                }
            except Exception as exc:
                first_message_by_topic[topic] = {
                    "timestamp_ns": int(timestamp_ns),
                    "msgtype": connection.msgtype,
                    "preview": f"<failed to deserialize: {exc}>",
                }

            remaining.remove(topic)
            if not remaining:
                break

        print()
        for topic in topics:
            topic_connections = connections_by_topic[topic]
            msgtypes = sorted({c.msgtype for c in topic_connections})
            print(f"Topic: {topic}")
            print(f"  Connections: {len(topic_connections)}")
            print(f"  Msg types: {msgtypes}")

            if topic in first_message_by_topic:
                sample = first_message_by_topic[topic]
                print(f"  Example timestamp [ns]: {sample['timestamp_ns']}")
                print(f"  Example message: {sample['preview']}")
            else:
                print("  Example message: <no messages found on this topic>")

            print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="List all topics in a rosbag/mcap and print one example message per topic."
    )
    parser.add_argument("input_bag_file", help="Path to rosbag/mcap file")
    parser.add_argument(
        "--max-example-chars",
        type=int,
        default=500,
        help="Maximum number of characters to print for each example message",
    )
    args = parser.parse_args()

    if args.max_example_chars <= 0:
        raise ValueError("--max-example-chars must be > 0")

    return args


if __name__ == "__main__":
    cli_args = parse_args()
    list_topics_with_one_example(
        input_bag_file=cli_args.input_bag_file,
        max_example_chars=cli_args.max_example_chars,
    )
