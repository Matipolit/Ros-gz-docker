import csv
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def extract_reference_traj(input_file_name: str, output_file_name: str):
    bag_path = Path(input_file_name)
    typestore = get_typestore(Stores.ROS2_JAZZY)
    target_parent = "root"
    target_child = "base_link"

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        transform_topic = "/tf"
        connections = [x for x in reader.connections if x.topic == transform_topic]

        with open(output_file_name, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

            last_row_data = None

            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                msg = reader.deserialize(rawdata, connection.msgtype)

                msgs_to_process = (
                    msg.transforms if hasattr(msg, "transforms") else [msg]
                )

                for m in msgs_to_process:
                    if not hasattr(m, "transform"):
                        continue

                    # Keep only robot trajectory transforms (root -> base_link).
                    parent_frame = (
                        m.header.frame_id.lstrip("/")
                        if hasattr(m, "header") and hasattr(m.header, "frame_id")
                        else ""
                    )
                    child_frame = (
                        m.child_frame_id.lstrip("/")
                        if hasattr(m, "child_frame_id")
                        else ""
                    )
                    if parent_frame != target_parent or child_frame != target_child:
                        continue

                    pos = m.transform.translation
                    ori = m.transform.rotation

                    current_row_data = (
                        pos.x,
                        pos.y,
                        pos.z,
                        ori.x,
                        ori.y,
                        ori.z,
                        ori.w,
                    )
                    if current_row_data != last_row_data:
                        writer.writerow(
                            [
                                timestamp,
                                pos.x,
                                pos.y,
                                pos.z,
                                ori.x,
                                ori.y,
                                ori.z,
                                ori.w,
                            ]
                        )
                        last_row_data = current_row_data


if __name__ == "__main__":
    input_file_name = sys.argv[1]
    output_file_name = sys.argv[2]

    extract_reference_traj(input_file_name, output_file_name)
