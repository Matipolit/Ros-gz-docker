#!/usr/bin/env python3
"""
Register a SLAM cloud to a Ground Truth cloud using standard Open3D pipeline:
1) Voxel downsample
2) Normal estimation
3) FPFH feature extraction
4) Global coarse alignment (RANSAC)
5) Local refinement (ICP point-to-plane)

Outputs:
- aligned source cloud
- JSON report with final transform and metrics
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Tuple

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Register SLAM point cloud to GT cloud.")
    p.add_argument("source_cloud", help="Source cloud (e.g. RTAB-Map export)")
    p.add_argument("target_cloud", help="Target cloud (e.g. Ground Truth cloud)")
    p.add_argument("output_aligned_cloud", help="Path to save aligned source cloud")
    p.add_argument("output_report_json", help="Path to save registration report JSON")

    p.add_argument(
        "--voxel-size",
        type=float,
        default=0.08,
        help="Voxel size for registration pipeline",
    )
    p.add_argument(
        "--normal-radius-mult",
        type=float,
        default=2.0,
        help="Normal radius = voxel_size * this",
    )
    p.add_argument(
        "--fpfh-radius-mult",
        type=float,
        default=5.0,
        help="FPFH radius = voxel_size * this",
    )

    p.add_argument(
        "--ransac-distance-mult",
        type=float,
        default=5.0,
        help="RANSAC correspondence distance = voxel_size * this",
    )
    p.add_argument(
        "--icp-distance-mult",
        type=float,
        default=3.0,
        help="ICP correspondence distance = voxel_size * this",
    )
    p.add_argument("--icp-max-iter", type=int, default=100, help="Max ICP iterations")

    p.add_argument(
        "--skip-global",
        action="store_true",
        help="Skip global RANSAC init and start ICP from identity/init",
    )
    p.add_argument(
        "--init-transform-json",
        default=None,
        help="Optional JSON file with 4x4 init transform under key 'transform'",
    )

    p.add_argument(
        "--crop-z-min",
        type=float,
        default=None,
        help="Optional pre-registration crop Z min",
    )
    p.add_argument(
        "--crop-z-max",
        type=float,
        default=None,
        help="Optional pre-registration crop Z max",
    )

    return p.parse_args()


def load_cloud(path: str) -> o3d.geometry.PointCloud:
    cloud = o3d.io.read_point_cloud(path)
    if not cloud.has_points():
        raise RuntimeError(f"Cloud has no points or failed to load: {path}")
    return cloud


def maybe_crop_z(
    cloud: o3d.geometry.PointCloud, zmin: float | None, zmax: float | None
) -> o3d.geometry.PointCloud:
    if zmin is None and zmax is None:
        return cloud
    min_z = -np.inf if zmin is None else zmin
    max_z = np.inf if zmax is None else zmax
    aabb = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array([-np.inf, -np.inf, min_z], dtype=np.float64),
        max_bound=np.array([np.inf, np.inf, max_z], dtype=np.float64),
    )
    return cloud.crop(aabb)


def preprocess_for_features(
    cloud: o3d.geometry.PointCloud,
    voxel_size: float,
    normal_radius_mult: float,
    fpfh_radius_mult: float,
) -> Tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    down = cloud.voxel_down_sample(voxel_size)

    normal_radius = voxel_size * normal_radius_mult
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
    )

    fpfh_radius = voxel_size * fpfh_radius_mult
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=100),
    )
    return down, fpfh


def load_init_transform(path: str | None) -> np.ndarray:
    if path is None:
        return np.eye(4)
    with open(path, "r") as f:
        data = json.load(f)
    arr = np.array(data["transform"], dtype=np.float64)
    if arr.shape != (4, 4):
        raise ValueError("init transform must be 4x4")
    return arr


def main() -> int:
    args = parse_args()

    if args.voxel_size <= 0:
        raise ValueError("--voxel-size must be > 0")

    source_full = load_cloud(args.source_cloud)
    target_full = load_cloud(args.target_cloud)

    source_reg = maybe_crop_z(source_full, args.crop_z_min, args.crop_z_max)
    target_reg = maybe_crop_z(target_full, args.crop_z_min, args.crop_z_max)

    source_down, source_fpfh = preprocess_for_features(
        source_reg, args.voxel_size, args.normal_radius_mult, args.fpfh_radius_mult
    )
    target_down, target_fpfh = preprocess_for_features(
        target_reg, args.voxel_size, args.normal_radius_mult, args.fpfh_radius_mult
    )

    init_transform = load_init_transform(args.init_transform_json)

    # Global registration
    global_result = None
    if not args.skip_global:
        ransac_dist = args.voxel_size * args.ransac_distance_mult
        global_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down,
            target_down,
            source_fpfh,
            target_fpfh,
            mutual_filter=False,
            max_correspondence_distance=ransac_dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                False
            ),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    ransac_dist
                ),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                4000000, 0.999
            ),
        )
        init = global_result.transformation @ init_transform
    else:
        init = init_transform

    # ICP refinement (point-to-plane)
    # Need normals on downsampled clouds used by ICP
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.voxel_size * args.normal_radius_mult, max_nn=30
        )
    )
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.voxel_size * args.normal_radius_mult, max_nn=30
        )
    )

    icp_dist = args.voxel_size * args.icp_distance_mult
    icp_result = o3d.pipelines.registration.registration_icp(
        source_down,
        target_down,
        max_correspondence_distance=icp_dist,
        init=init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=args.icp_max_iter
        ),
    )

    final_transform = icp_result.transformation

    # Apply transform to full source cloud
    source_aligned_full = o3d.geometry.PointCloud(source_full)
    source_aligned_full.transform(final_transform)

    out_cloud = Path(args.output_aligned_cloud)
    out_cloud.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(out_cloud), source_aligned_full):
        raise RuntimeError(f"Failed to write aligned cloud: {out_cloud}")

    report = {
        "source_cloud": args.source_cloud,
        "target_cloud": args.target_cloud,
        "output_aligned_cloud": args.output_aligned_cloud,
        "params": {
            "voxel_size": args.voxel_size,
            "normal_radius_mult": args.normal_radius_mult,
            "fpfh_radius_mult": args.fpfh_radius_mult,
            "ransac_distance_mult": args.ransac_distance_mult,
            "icp_distance_mult": args.icp_distance_mult,
            "icp_max_iter": args.icp_max_iter,
            "skip_global": args.skip_global,
            "crop_z_min": args.crop_z_min,
            "crop_z_max": args.crop_z_max,
            "init_transform_json": args.init_transform_json,
        },
        "global_registration": None
        if global_result is None
        else {
            "fitness": float(global_result.fitness),
            "inlier_rmse": float(global_result.inlier_rmse),
            "transformation": np.asarray(global_result.transformation).tolist(),
        },
        "icp_refinement": {
            "fitness": float(icp_result.fitness),
            "inlier_rmse": float(icp_result.inlier_rmse),
            "transformation": np.asarray(icp_result.transformation).tolist(),
        },
        "final_transform": np.asarray(final_transform).tolist(),
    }

    out_report = Path(args.output_report_json)
    out_report.parent.mkdir(parents=True, exist_ok=True)
    with open(out_report, "w") as f:
        json.dump(report, f, indent=2)

    print(f"Saved aligned cloud: {out_cloud}")
    print(f"Saved registration report: {out_report}")
    print(f"ICP fitness: {icp_result.fitness:.6f}")
    print(f"ICP inlier RMSE: {icp_result.inlier_rmse:.6f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
