#!/usr/bin/env python3
"""Clip a point cloud using an axis-aligned bounding box or radial distance."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Clip a .pcd file to a set size or range.")
    parser.add_argument("input_file", help="Input point cloud path")
    parser.add_argument("output_file", help="Output point cloud path")

    parser.add_argument("--min-range", type=float, help="Keep points with distance >= min-range from origin (meters).")
    parser.add_argument("--max-range", type=float, help="Keep points with distance <= max-range from origin (meters).")
    parser.add_argument("--crop-x-min", type=float, help="ROI min X (meters).")
    parser.add_argument("--crop-x-max", type=float, help="ROI max X (meters).")
    parser.add_argument("--crop-y-min", type=float, help="ROI min Y (meters).")
    parser.add_argument("--crop-y-max", type=float, help="ROI max Y (meters).")
    parser.add_argument("--crop-z-min", type=float, help="ROI min Z (meters).")
    parser.add_argument("--crop-z-max", type=float, help="ROI max Z (meters).")

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

def main() -> int:
    args = parse_args()
    input_file = Path(args.input_file)
    output_file = Path(args.output_file)

    if not input_file.is_file():
        print(f"Error: Input file does not exist: {input_file}", file=sys.stderr)
        return 1

    cloud = o3d.io.read_point_cloud(str(input_file))
    if len(cloud.points) == 0:
        print("Error: Input point cloud is empty.", file=sys.stderr)
        return 1

    print(f"Loaded point cloud with {len(cloud.points)} points.")

    cloud = filter_by_range(cloud, args.min_range, args.max_range)
    cloud = filter_by_roi(args, cloud)

    if len(cloud.points) == 0:
        print("Error: Point cloud is empty after clipping.", file=sys.stderr)
        return 1

    output_file.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(output_file), cloud):
        print(f"Error: Failed to write output point cloud: {output_file}", file=sys.stderr)
        return 1
        
    print(f"Saved clipped point cloud with {len(cloud.points)} points to: {output_file}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
