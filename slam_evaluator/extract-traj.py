import csv
import sys
import tempfile
from pathlib import Path

import zstandard
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def extract_reference_traj(input_file_name: str, output_file_name: str):
    bag_path = Path(input_file_name)
    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_JAZZY)

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

                # Handle tf2_msgs/TFMessage which contains an array of transforms
                msgs_to_process = (
                    msg.transforms if hasattr(msg, "transforms") else [msg]
                )

                for m in msgs_to_process:
                    # Extract the pose depending on the message type
                    # Handles nav_msgs/Odometry, geometry_msgs/PoseStamped, and geometry_msgs/TransformStamped
                    if hasattr(m, "pose") and hasattr(m.pose, "pose"):
                        pose = m.pose.pose
                    elif hasattr(m, "pose"):
                        pose = m.pose
                    elif hasattr(m, "transform"):
                        if (
                            hasattr(m, "child_frame_id")
                            and m.child_frame_id != "base_footprint"
                        ):
                            continue
                        pose = m.transform
                    else:
                        continue

                    if hasattr(pose, "position"):
                        pos = pose.position
                        ori = pose.orientation
                    else:
                        pos = pose.translation
                        ori = pose.rotation

                    # skip duplicated rows
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
