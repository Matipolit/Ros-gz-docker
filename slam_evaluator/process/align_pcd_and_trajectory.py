#!/usr/bin/env python3
"""Align SLAM trajectory and point cloud into the Ground Truth world frame.

Strategy: rigid SE(3) alignment anchored on the first SLAM pose.
    T_align = T_gt(t0) * T_slam(t0)^-1
Then apply T_align to every SLAM pose and to the SLAM point cloud.

This preserves SLAM's internal shape/drift (no scaling, no per-pose fitting),
it only re-expresses the output in the GT world frame so visual overlay and
metric computation become straightforward.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ]
    )


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def make_se3(t: np.ndarray, R: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def load_gt_csv(path: Path) -> list[tuple[float, np.ndarray]]:
    """Return list of (timestamp, 4x4 pose) from GT trajectory CSV."""
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)
        for r in reader:
            if len(r) < 8:
                continue
            t, x, y, z, qx, qy, qz, qw = map(float, r[:8])
            rows.append((t, make_se3(np.array([x, y, z]), quat_to_rot(qx, qy, qz, qw))))
    rows.sort(key=lambda x: x[0])
    return rows


def load_slam_poses(path: Path) -> list[tuple[float, int, np.ndarray]]:
    """Return list of (timestamp, id, 4x4 pose) from SLAM poses.txt or CSV."""
    out = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("t"):
                continue
            parts = line.split(',') if ',' in line else line.split()
            if len(parts) < 8:
                continue
            try:
                t = float(parts[0])
                x, y, z = map(float, parts[1:4])
                qx, qy, qz, qw = map(float, parts[4:8])
                pid = int(parts[8]) if len(parts) >= 9 else -1
            except ValueError:
                continue
            out.append(
                (t, pid, make_se3(np.array([x, y, z]), quat_to_rot(qx, qy, qz, qw)))
            )
    return out


def interpolate_gt_pose(t: float, gt: list[tuple[float, np.ndarray]]) -> np.ndarray:
    """Linear interp on translation, SLERP on rotation."""
    times = [row[0] for row in gt]
    import bisect

    idx = bisect.bisect_left(times, t)
    if idx <= 0:
        return gt[0][1]
    if idx >= len(gt):
        return gt[-1][1]
    t0, T0 = gt[idx - 1]
    t1, T1 = gt[idx]
    if t1 == t0:
        return T0
    a = (t - t0) / (t1 - t0)

    # Translation
    p = (1 - a) * T0[:3, 3] + a * T1[:3, 3]

    # Rotation SLERP via quaternions
    q0 = np.array(rot_to_quat(T0[:3, :3]))
    q1 = np.array(rot_to_quat(T1[:3, :3]))
    if np.dot(q0, q1) < 0:
        q1 = -q1
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if dot > 0.9995:
        q = q0 + a * (q1 - q0)
        q /= np.linalg.norm(q)
    else:
        theta = np.arccos(dot)
        q = (np.sin((1 - a) * theta) * q0 + np.sin(a * theta) * q1) / np.sin(theta)
    R = quat_to_rot(q[0], q[1], q[2], q[3])
    return make_se3(p, R)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt-trajectory", required=True, type=Path)
    ap.add_argument("--slam-poses", required=True, type=Path)
    ap.add_argument("--slam-cloud", required=False, type=Path)
    ap.add_argument("--out-poses", required=True, type=Path)
    ap.add_argument("--out-cloud", required=False, type=Path)
    ap.add_argument("--align-icp", action="store_true", help="Refine alignment using ICP")
    ap.add_argument("--gt-cloud", required=False, type=Path, help="Ground Truth parameter for ICP")
    ap.add_argument("--icp-threshold", type=float, default=0.2, help="Distance threshold for ICP refinement")
    args = ap.parse_args()

    gt = load_gt_csv(args.gt_trajectory)
    slam_poses = load_slam_poses(args.slam_poses)
    if not gt or not slam_poses:
        raise RuntimeError("Empty GT or SLAM trajectory.")

    t0, _, T_slam0 = slam_poses[0]
    T_gt0 = interpolate_gt_pose(t0, gt)

    T_align = T_gt0 @ np.linalg.inv(T_slam0)
    print(f"Anchor timestamp: {t0:.6f}")
    print(f"Initial alignment translation: {T_align[:3, 3]}")
    print(f"Initial alignment rotation:\n{T_align[:3, :3]}")

    cloud = None
    if args.align_icp:
        if not args.gt_cloud or not args.slam_cloud:
            raise RuntimeError("--gt-cloud and --slam-cloud are required for ICP alignment.")
            
        print("Loading clouds for ICP refinement...")
        cloud = o3d.io.read_point_cloud(str(args.slam_cloud))
        gt_cloud = o3d.io.read_point_cloud(str(args.gt_cloud))
        
        if len(cloud.points) == 0 or len(gt_cloud.points) == 0:
            raise RuntimeError("One of the clouds is empty. Cannot perform ICP.")
            
        print(f"Applying initial trajectory alignment to SLAM cloud before ICP...")
        cloud.transform(T_align)
        
        print(f"Running ICP (threshold={args.icp_threshold})...")
        icp_result = o3d.pipelines.registration.registration_icp(
            cloud, gt_cloud, args.icp_threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        
        T_icp = icp_result.transformation
        print(f"ICP fitness: {icp_result.fitness:.6f}, inlier_rmse: {icp_result.inlier_rmse:.6f}")
        print(f"ICP refinement translation: {T_icp[:3, 3]}")
        
        cloud.transform(T_icp)
        T_align = T_icp @ T_align

    # Transform and write poses
    args.out_poses.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out_poses, "w") as f:
        f.write("#timestamp x y z qx qy qz qw id\n")
        for t, pid, T in slam_poses:
            Ta = T_align @ T
            x, y, z = Ta[:3, 3]
            qx, qy, qz, qw = rot_to_quat(Ta[:3, :3])
            f.write(
                f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} {pid}\n"
            )
    print(f"Wrote aligned poses: {args.out_poses}")

    # Transform and write cloud
    if args.slam_cloud and args.out_cloud:
        if cloud is None:
            cloud = o3d.io.read_point_cloud(str(args.slam_cloud))
            if len(cloud.points) == 0:
                raise RuntimeError(f"Empty SLAM cloud: {args.slam_cloud}")
            cloud.transform(T_align)
        args.out_cloud.parent.mkdir(parents=True, exist_ok=True)
        if not o3d.io.write_point_cloud(str(args.out_cloud), cloud):
            raise RuntimeError(f"Failed to write: {args.out_cloud}")
        print(f"Wrote aligned cloud: {args.out_cloud}  ({len(cloud.points)} pts)")
    elif args.slam_cloud or args.out_cloud:
        print(
            "Warning: Both --slam-cloud and --out-cloud must be provided to transform the point cloud."
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
