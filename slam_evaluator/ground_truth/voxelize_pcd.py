#!/usr/bin/env python3
"""Downsample a point cloud using voxelization."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import open3d as o3d

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Voxel downsample a point cloud.")
    parser.add_argument("input_file", help="Input point cloud path")
    parser.add_argument("output_file", help="Output point cloud path")
    parser.add_argument("voxel_size", type=float, help="Voxel size for downsampling (> 0)")
    return parser.parse_args()

def main() -> int:
    args = parse_args()
    input_file = Path(args.input_file)
    output_file = Path(args.output_file)

    if args.voxel_size <= 0.0:
        print("Error: voxel_size must be > 0", file=sys.stderr)
        return 1

    if not input_file.is_file():
        print(f"Error: Input file does not exist: {input_file}", file=sys.stderr)
        return 1

    cloud = o3d.io.read_point_cloud(str(input_file))
    if len(cloud.points) == 0:
        print("Error: Input point cloud is empty.", file=sys.stderr)
        return 1

    print(f"Loaded point cloud with {len(cloud.points)} points.")
    
    downsampled = cloud.voxel_down_sample(voxel_size=args.voxel_size)

    if len(downsampled.points) == 0:
        print("Error: Point cloud is empty after downsampling.", file=sys.stderr)
        return 1

    output_file.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(output_file), downsampled):
        print(f"Error: Failed to write output point cloud: {output_file}", file=sys.stderr)
        return 1
        
    print(f"Saved downsampled point cloud with {len(downsampled.points)} points to: {output_file}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
