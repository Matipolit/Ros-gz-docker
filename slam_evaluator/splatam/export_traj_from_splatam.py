import csv
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation


def export_traj(bag_path, npz_path, output_csv, color_topic, depth_topic, stride=3):
    bag_path = Path(bag_path)
    typestore = get_typestore(Stores.ROS2_FOXY)

    rgb_msgs = []
    depth_msgs = []

    print(f"Reading bag: {bag_path}")
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == color_topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                    sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                else:
                    sim_time = timestamp / 1e9
                rgb_msgs.append((timestamp, sim_time))
            elif connection.topic == depth_topic:
                depth_msgs.append(timestamp)

    # Replicate synchronization logic
    synced_pairs_ts = []
    depth_idx = 0
    for rgb_ts, sim_time in rgb_msgs:
        while depth_idx < len(depth_msgs) - 1 and abs(
            depth_msgs[depth_idx + 1] - rgb_ts
        ) < abs(depth_msgs[depth_idx] - rgb_ts):
            depth_idx += 1
        if depth_idx < len(depth_msgs):
            depth_ts = depth_msgs[depth_idx]
            if abs(rgb_ts - depth_ts) < 50_000_000:
                synced_pairs_ts.append(sim_time)

    print(f"Total synced frames in bag: {len(synced_pairs_ts)}")

    # Get timestamps using stride
    used_timestamps = synced_pairs_ts[0::stride]

    # Load SplaTAM params
    params = np.load(npz_path)
    w2c_rots = params["cam_unnorm_rots"][0]
    w2c_trans = params["cam_trans"][0]

    num_frames = w2c_rots.shape[-1]
    print(
        f"Frames in SplaTAM params: {num_frames}, Expected from stride: {len(used_timestamps)}"
    )

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

        for i in range(num_frames):
            ts = used_timestamps[i]

            # Normalize W2C quaternion
            rq = w2c_rots[:, i]
            rq = rq / np.linalg.norm(rq)
            w, x, y, z = rq

            # W2C pose
            rot_w2c = Rotation.from_quat([x, y, z, w])
            t_w2c = w2c_trans[:, i]

            # Convert to C2W (optical frame pose)
            rot_c2w = rot_w2c.inv()
            t_c2w = -rot_c2w.apply(t_w2c)

            # Base to optical transform
            rot_base_opt = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5])
            t_base_opt = np.array([0.1, 0.0, 0.15])

            # Optical to base transform
            rot_opt_base = rot_base_opt.inv()
            t_opt_base = -rot_opt_base.apply(t_base_opt)

            # Calculate world to base transform
            rot_world_base = rot_c2w * rot_opt_base
            t_world_base = t_c2w + rot_c2w.apply(t_opt_base)

            # Output in (qx, qy, qz, qw)
            qx, qy, qz, qw = rot_world_base.as_quat()

            writer.writerow(
                [ts, t_world_base[0], t_world_base[1], t_world_base[2], qx, qy, qz, qw]
            )

    print(f"Exported trajectory to {output_csv}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export trajectory from SplaTAM params.npz to CSV."
    )
    parser.add_argument(
        "--bag_path", required=True, help="Path to the rosbag directory"
    )
    parser.add_argument("--npz_path", required=True, help="Path to SplaTAM params.npz")
    parser.add_argument("--output_csv", required=True, help="Path to output CSV file")
    parser.add_argument(
        "--stride", type=int, default=3, help="Frame stride used during SplaTAM mapping"
    )
    parser.add_argument(
        "--color_topic", default="/camera/color/image_raw", help="Color image topic"
    )
    parser.add_argument(
        "--depth_topic", default="/camera/depth/image_raw", help="Depth image topic"
    )
    return parser.parse_args()


if __name__ == "__main__":
    import argparse

    args = parse_args()
    export_traj(
        args.bag_path,
        args.npz_path,
        args.output_csv,
        args.color_topic,
        args.depth_topic,
        args.stride,
    )
