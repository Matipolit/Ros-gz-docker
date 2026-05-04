#!/usr/bin/env python3
"""Clip a point cloud using an axis-aligned bounding box, radial distance, or trajectory tube."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Clip a .pcd file to a set size, range, or trajectory tube/bbox."
    )
    parser.add_argument("input_file", help="Input point cloud path")
    parser.add_argument("output_file", help="Output point cloud path")

    parser.add_argument(
        "--min_range",
        type=float,
        help="Keep points with distance >= min_range from origin (meters).",
    )
    parser.add_argument(
        "--max_range",
        type=float,
        help="Keep points with distance <= max_range from origin (meters).",
    )
    parser.add_argument("--crop_x_min", type=float, help="ROI min X (meters).")
    parser.add_argument("--crop_x_max", type=float, help="ROI max X (meters).")
    parser.add_argument("--crop_y_min", type=float, help="ROI min Y (meters).")
    parser.add_argument("--crop_y_max", type=float, help="ROI max Y (meters).")
    parser.add_argument("--crop_z_min", type=float, help="ROI min Z (meters).")
    parser.add_argument("--crop_z_max", type=float, help="ROI max Z (meters).")

    parser.add_argument(
        "--trajectory",
        type=Path,
        help="Path to GT trajectory CSV file for tube/bbox clipping.",
    )
    parser.add_argument(
        "--tube_radius",
        type=float,
        help="Radius (meters) around the trajectory to keep points.",
    )
    parser.add_argument(
        "--trajectory_bbox_margin",
        type=float,
        help="Margin (meters) to add to the trajectory bounding box.",
    )

    return parser.parse_args()


def filter_by_range(
    cloud: o3d.geometry.PointCloud,
    min_range: Optional[float],
    max_range: Optional[float],
) -> o3d.geometry.PointCloud:
    if min_range is None and max_range is None:
        return cloud
    points = np.asarray(cloud.points)
    if points.size == 0:
        return cloud

    dists = np.linalg.norm(points, axis=1)
    mask = np.ones(points.shape[0], dtype=bool)
    if min_range is not None:
        mask &= dists >= min_range
    if max_range is not None:
        mask &= dists <= max_range

    idx = np.nonzero(mask)[0]
    if idx.size == 0:
        return o3d.geometry.PointCloud()
    return cloud.select_by_index(idx)


def filter_by_roi(
    args: argparse.Namespace, cloud: o3d.geometry.PointCloud
) -> o3d.geometry.PointCloud:
    xmin = -np.inf if args.crop_x_min is None else args.crop_x_min
    xmax = np.inf if args.crop_x_max is None else args.crop_x_max
    ymin = -np.inf if args.crop_y_min is None else args.crop_y_min
    ymax = np.inf if args.crop_y_max is None else args.crop_y_max
    zmin = -np.inf if args.crop_z_min is None else args.crop_z_min
    zmax = np.inf if args.crop_z_max is None else args.crop_z_max

    if np.isinf([xmin, xmax, ymin, ymax, zmin, zmax]).all():
        return cloud

    aabb = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array([xmin, ymin, zmin], dtype=np.float64),
        max_bound=np.array([xmax, ymax, zmax], dtype=np.float64),
    )
    return cloud.crop(aabb)


def filter_by_trajectory(
    cloud: o3d.geometry.PointCloud,
    trajectory_path: Optional[Path],
    tube_radius: Optional[float],
    bbox_margin: Optional[float],
) -> o3d.geometry.PointCloud:
    if not trajectory_path or (tube_radius is None and bbox_margin is None):
        return cloud

    if len(cloud.points) == 0:
        return cloud

    traj_points = []
    try:
        with open(trajectory_path, "r") as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for r in reader:
                if len(r) < 8:
                    continue
                try:
                    # t, x, y, z, qx, qy, qz, qw
                    x, y, z = float(r[1]), float(r[2]), float(r[3])
                    traj_points.append([x, y, z])
                except ValueError:
                    continue
    except Exception as e:
        print(f"Error reading trajectory: {e}", file=sys.stderr)
        return cloud

    if not traj_points:
        print("Warning: Trajectory is empty, skipping filtering.", file=sys.stderr)
        return cloud

    if bbox_margin is not None:
        print(
            f"Filtering cloud to trajectory bounding box with {bbox_margin}m margin..."
        )
        points_arr = np.array(traj_points)
        min_bound = points_arr.min(axis=0) - bbox_margin
        max_bound = points_arr.max(axis=0) + bbox_margin
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        cloud = cloud.crop(aabb)

    if tube_radius is not None:
        print(
            f"Filtering cloud to {tube_radius}m tube around {len(traj_points)} trajectory points..."
        )
        if len(cloud.points) == 0:
            return cloud

        kdtree = o3d.geometry.KDTreeFlann(cloud)
        idx_to_keep = set()
        for pt in traj_points:
            k, idx, _ = kdtree.search_radius_vector_3d(pt, tube_radius)
            if k > 0:
                idx_to_keep.update(idx)

        if not idx_to_keep:
            return o3d.geometry.PointCloud()

        cloud = cloud.select_by_index(list(idx_to_keep))

    return cloud


def main() -> int:
    args = parse_args()
    input_file = Path(args.input_file)
    output_file = Path(args.output_file)

    if not input_file.is_file():
        print(f"Error: Input file does not exist: {input_file}", file=sys.stderr)
        return 1

    if (
        args.trajectory
        and args.tube_radius is None
        and args.trajectory_bbox_margin is None
    ):
        print(
            "Error: --tube_radius or --trajectory_bbox_margin must be provided if --trajectory is used.",
            file=sys.stderr,
        )
        return 1

    cloud = o3d.io.read_point_cloud(str(input_file))
    if len(cloud.points) == 0:
        print("Error: Input point cloud is empty.", file=sys.stderr)
        return 1

    print(f"Loaded point cloud with {len(cloud.points)} points.")

    cloud = filter_by_range(cloud, args.min_range, args.max_range)
    cloud = filter_by_roi(args, cloud)
    cloud = filter_by_trajectory(
        cloud, args.trajectory, args.tube_radius, args.trajectory_bbox_margin
    )

    if len(cloud.points) == 0:
        print("Error: Point cloud is empty after clipping.", file=sys.stderr)
        return 1

    output_file.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(output_file), cloud):
        print(
            f"Error: Failed to write output point cloud: {output_file}", file=sys.stderr
        )
        return 1

    print(
        f"Saved clipped point cloud with {len(cloud.points)} points to: {output_file}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
