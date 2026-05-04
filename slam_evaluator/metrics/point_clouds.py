#!/usr/bin/env python3
"""
Script for computing key 3D map quality metrics (point clouds .ply)
relative to the reference cloud (Ground Truth).
Implemented metrics:
1. Chamfer Distance (CD)
2. F-score@tau
3. Accuracy (AC) & Completeness (COM)
4. Voxelized Average Wasserstein Distance (AWD) & Spatial Consistency Score (SCS) (from MapEval 2025)
5. Hausdorff Distance (HD)
6. Absolute Mapping Error (AME)
"""

import argparse
import os
import sys
from collections import defaultdict

import numpy as np
import open3d as o3d
import pandas as pd
from scipy.linalg import sqrtm
from scipy.spatial import KDTree


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate 3D point cloud metrics for SLAM vs Ground Truth."
    )
    parser.add_argument(
        "--slam_ply",
        nargs="+",
        required=True,
        help="Path(s) to the point cloud from the SLAM algorithm (.ply)",
    )
    parser.add_argument(
        "--gt_ply",
        required=True,
        help="Path to the reference/Ground Truth point cloud (.ply)",
    )
    parser.add_argument(
        "--output_csv",
        default="point_cloud_metrics.csv",
        help="Path to the output CSV file",
    )
    parser.add_argument(
        "--tau",
        type=float,
        default=0.05,
        help="Error tolerance threshold (tau) for removing outliers during calculations, e.g., 0.05 m",
    )
    parser.add_argument(
        "--voxel_size", type=float, default=0.2, help="Voxel size for AWD/SCS metrics"
    )
    return parser.parse_args()


def compute_wasserstein_distance(mu1, cov1, mu2, cov2):
    """
    Computes the Wasserstein distance (W2) for two Gaussian distributions.
    W^2 = ||mu1 - mu2||^2 + Tr(cov1 + cov2 - 2 * sqrt(cov1^0.5 * cov2 * cov1^0.5))
    """
    diff = mu1 - mu2
    m1_norm = np.sum(diff**2)

    # Adding a small epsilon value to the diagonal for numerical stability
    eps = 1e-6
    cov1_safe = cov1 + np.eye(3) * eps
    cov2_safe = cov2 + np.eye(3) * eps

    sqrt_cov1 = sqrtm(cov1_safe)
    sqrt_cov1 = np.real(sqrt_cov1)

    cross = sqrtm(sqrt_cov1 @ cov2_safe @ sqrt_cov1)
    cross = np.real(cross)

    trace_term = np.trace(cov1_safe + cov2_safe - 2 * cross)

    w2_squared = m1_norm + max(0.0, trace_term)
    return np.sqrt(w2_squared)


def compute_mapeval_metrics(points_slam, points_gt, voxel_size):
    """
    Computes metrics from the MapEval 2025 framework: AWD and SCS.
    """
    if len(points_slam) == 0 or len(points_gt) == 0:
        return np.nan, np.nan

    # 1. Voxelization - assigning points to voxels
    voxels_slam = np.floor(points_slam / voxel_size).astype(int)
    voxels_gt = np.floor(points_gt / voxel_size).astype(int)

    dict_slam = defaultdict(list)
    dict_gt = defaultdict(list)

    for pt, v in zip(points_slam, voxels_slam):
        dict_slam[tuple(v)].append(pt)

    for pt, v in zip(points_gt, voxels_gt):
        dict_gt[tuple(v)].append(pt)

    common_voxels = set(dict_slam.keys()).intersection(set(dict_gt.keys()))

    if not common_voxels:
        return np.nan, np.nan

    w_distances = {}

    for v in common_voxels:
        pts_s = np.array(dict_slam[v])
        pts_g = np.array(dict_gt[v])

        # At least 4 points are needed to safely compute the covariance matrix in 3D space
        if len(pts_s) < 4 or len(pts_g) < 4:
            continue

        mu_s = np.mean(pts_s, axis=0)
        cov_s = np.cov(pts_s, rowvar=False)

        mu_g = np.mean(pts_g, axis=0)
        cov_g = np.cov(pts_g, rowvar=False)

        w_dist = compute_wasserstein_distance(mu_s, cov_s, mu_g, cov_g)
        w_distances[v] = w_dist

    if not w_distances:
        return np.nan, np.nan

    awd = np.mean(list(w_distances.values()))

    # 2. SCS - Spatial Consistency Score (std deviation of W2 distances in a 3x3x3 voxel neighborhood)
    local_stds = []

    for v in w_distances.keys():
        neighbors_w = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    nv = (v[0] + dx, v[1] + dy, v[2] + dz)
                    if nv in w_distances:
                        neighbors_w.append(w_distances[nv])

        # We calculate the standard deviation only if there is more than one voxel in the neighborhood
        if len(neighbors_w) > 1:
            local_stds.append(np.std(neighbors_w))

    scs = np.mean(local_stds) if local_stds else np.nan

    return awd, scs


def main():
    args = parse_args()

    print(f"Loading GT cloud from: {args.gt_ply}")
    pcd_gt = o3d.io.read_point_cloud(args.gt_ply)

    if not pcd_gt.has_points():
        print("Error: GT point cloud is empty!")
        sys.exit(1)

    points_gt = np.asarray(pcd_gt.points)
    tree_gt = KDTree(points_gt)

    results = []

    for slam_path in args.slam_ply:
        print(f"\nProcessing SLAM cloud: {slam_path}")
        pcd_slam = o3d.io.read_point_cloud(slam_path)

        if not pcd_slam.has_points():
            print(f"Warning: SLAM point cloud {slam_path} is empty! Skipping.")
            continue

        points_slam = np.asarray(pcd_slam.points)
        print(f"SLAM points: {len(points_slam)}, GT points: {len(points_gt)}")
        print(f"Computing Nearest Neighbors...")

        # KD-trees for fast NN search
        tree_slam = KDTree(points_slam)

        # Distances
        dist_slam2gt, _ = tree_gt.query(points_slam)
        dist_gt2slam, _ = tree_slam.query(points_gt)

        # --- Classic metrics computation ---
        print(f"Applying distance threshold tau = {args.tau} m")
        valid_slam2gt = dist_slam2gt[dist_slam2gt <= args.tau]
        valid_gt2slam = dist_gt2slam[dist_gt2slam <= args.tau]

        # 1. Chamfer Distance
        mean_slam2gt = np.mean(valid_slam2gt) if len(valid_slam2gt) > 0 else np.nan
        mean_gt2slam = np.mean(valid_gt2slam) if len(valid_gt2slam) > 0 else np.nan
        cd = (mean_slam2gt + mean_gt2slam) / 2.0

        # 2. F-score@tau
        precision = len(valid_slam2gt) / len(points_slam)
        recall = len(valid_gt2slam) / len(points_gt)
        if precision + recall > 0:
            f1_score = 2 * (precision * recall) / (precision + recall)
        else:
            f1_score = 0.0

        # 3. Accuracy (AC) and Completeness (COM)
        ac = mean_slam2gt
        com = mean_gt2slam

        # 6. Absolute Mapping Error (AME)
        ame = np.sqrt(np.mean(valid_slam2gt**2)) if len(valid_slam2gt) > 0 else np.nan

        # 5. Hausdorff Distance (HD)
        hd = max(np.max(dist_slam2gt), np.max(dist_gt2slam))

        # --- MapEval metrics computation (AWD, SCS) ---
        print(f"Computing Voxelized AWD and SCS with voxel_size = {args.voxel_size} m")
        awd, scs = compute_mapeval_metrics(points_slam, points_gt, args.voxel_size)

        # Reporting
        metrics = {
            "cloud": os.path.basename(slam_path),
            "chamfer_distance_m": round(cd, 5),
            "f_score_tau": round(f1_score, 5),
            "precision": round(precision, 5),
            "recall": round(recall, 5),
            "accuracy_m": round(ac, 5),
            "completeness_m": round(com, 5),
            "hausdorff_distance_m": round(hd, 5),
            "absolute_mapping_error_rmse_m": round(ame, 5),
            "awd": round(awd, 5),
            "scs": round(scs, 5),
        }

        print("--- METRICS SUMMARY ---")
        for k, v in metrics.items():
            print(f"{k}: {v}")

        results.append(metrics)

    if not results:
        print("No metrics computed.")
        return

    # Write to CSV
    df = pd.DataFrame(results)
    df.to_csv(args.output_csv, index=False)
    print(f"\nResults saved successfully to: {args.output_csv}")


if __name__ == "__main__":
    main()
