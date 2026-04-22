#!/usr/bin/env python3
"""
Align SLAM point clouds to Ground Truth coordinate system.

Since RTAB-Map initializes its map origin (0,0,0) at the robot's starting pose,
this script applies the initial pose from the GT trajectory to the SLAM cloud
to move it into the global GT world frame.

It also supports applying manual roll/pitch/yaw corrections to fix any residual
optical-frame vs base-link orientations (like the common 90-degree rotations).
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args():
    parser = argparse.ArgumentParser(
        description="Align SLAM point cloud to GT coordinates."
    )
    parser.add_argument(
        "input_cloud", help="Path to input SLAM point cloud (.ply or .pcd)"
    )
    parser.add_argument("output_cloud", help="Path to save aligned point cloud")
    parser.add_argument(
        "--trajectory-csv",
        required=True,
        help="Path to GT trajectory CSV (t,x,y,z,qx,qy,qz,qw)",
    )

    parser.add_argument(
        "--pre-roll",
        type=float,
        default=0.0,
        help="Additional roll (deg) applied BEFORE trajectory alignment (X-axis)",
    )
    parser.add_argument(
        "--pre-pitch",
        type=float,
        default=0.0,
        help="Additional pitch (deg) applied BEFORE trajectory alignment (Y-axis)",
    )
    parser.add_argument(
        "--pre-yaw",
        type=float,
        default=0.0,
        help="Additional yaw (deg) applied BEFORE trajectory alignment (Z-axis)",
    )
    return parser.parse_args()


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Convert a quaternion into a full 3x3 rotation matrix."""
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)

    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy * qz - qx * qw)

    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])


def euler_to_rot_matrix(roll_deg, pitch_deg, yaw_deg):
    """Convert roll, pitch, yaw (in degrees) to a 3x3 rotation matrix."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    Rx = np.array(
        [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ]
    )

    Ry = np.array(
        [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ]
    )

    Rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ]
    )

    return Rz @ Ry @ Rx


def get_initial_pose(csv_path):
    """Extract the first valid pose from the trajectory CSV."""
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip header
        for row in reader:
            if len(row) >= 8:
                try:
                    return {
                        "x": float(row[1]),
                        "y": float(row[2]),
                        "z": float(row[3]),
                        "qx": float(row[4]),
                        "qy": float(row[5]),
                        "qz": float(row[6]),
                        "qw": float(row[7]),
                    }
                except ValueError:
                    continue
    return None


def main():
    args = parse_args()

    # 1. Load the initial pose
    init_pose = get_initial_pose(args.trajectory_csv)
    if init_pose is None:
        print(f"Error: Could not read a valid initial pose from {args.trajectory_csv}")
        sys.exit(1)

    print(
        f"Loaded initial GT pose: x={init_pose['x']:.3f}, y={init_pose['y']:.3f}, z={init_pose['z']:.3f}"
    )

    # 2. Build the GT Trajectory transformation matrix
    T_traj = np.eye(4)
    T_traj[:3, :3] = quat_to_rot_matrix(
        init_pose["qx"], init_pose["qy"], init_pose["qz"], init_pose["qw"]
    )
    T_traj[0, 3] = init_pose["x"]
    T_traj[1, 3] = init_pose["y"]
    T_traj[2, 3] = init_pose["z"]

    # 3. Build the Pre-Rotation matrix (if any manual optical/base adjustments are needed)
    T_pre = np.eye(4)
    if args.pre_roll != 0.0 or args.pre_pitch != 0.0 or args.pre_yaw != 0.0:
        print(
            f"Applying pre-rotation: roll={args.pre_roll}°, pitch={args.pre_pitch}°, yaw={args.pre_yaw}°"
        )
        T_pre[:3, :3] = euler_to_rot_matrix(args.pre_roll, args.pre_pitch, args.pre_yaw)

    # 4. Final Transformation Matrix: T = T_traj * T_pre
    T_final = T_traj @ T_pre

    # 5. Load SLAM Cloud
    print(f"Loading SLAM point cloud from: {args.input_cloud}")
    pcd = o3d.io.read_point_cloud(args.input_cloud)
    if not pcd.has_points():
        print("Error: Point cloud is empty or could not be read.")
        sys.exit(1)

    # 6. Apply Transformation
    print("Aligning point cloud to GT frame...")
    pcd.transform(T_final)

    # 7. Save Aligned Cloud
    out_path = Path(args.output_cloud)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out_path), pcd)
    print(f"Saved aligned point cloud to: {out_path}")


if __name__ == "__main__":
    main()
