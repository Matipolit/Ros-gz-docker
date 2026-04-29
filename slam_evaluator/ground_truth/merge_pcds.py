#!/usr/bin/env python3
"""Merge all .pcd files in a folder into a single point cloud."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import open3d as o3d

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge .pcd files from a folder into one point cloud.")
    parser.add_argument("source_folder", help="Folder containing input .pcd files")
    parser.add_argument("output_file", help="Output point cloud path")
    return parser.parse_args()

def main() -> int:
    args = parse_args()
    source_folder = Path(args.source_folder)
    output_file = Path(args.output_file)

    if not source_folder.is_dir():
        print(f"Error: Source folder does not exist or is not a directory: {source_folder}", file=sys.stderr)
        return 1

    pcd_files = sorted(source_folder.glob("*.pcd"))
    if not pcd_files:
        print(f"Error: No .pcd files found in: {source_folder}", file=sys.stderr)
        return 1

    merged = o3d.geometry.PointCloud()
    
    print(f"Merging {len(pcd_files)} files...")
    for i, pcd_path in enumerate(pcd_files):
        cloud = o3d.io.read_point_cloud(str(pcd_path))
        merged += cloud
        print(f"\rProcessed {i + 1}/{len(pcd_files)}", end="", flush=True)
    print()

    if len(merged.points) == 0:
        print("Error: Merged point cloud is empty.", file=sys.stderr)
        return 1

    output_file.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(output_file), merged):
        print(f"Error: Failed to write output point cloud: {output_file}", file=sys.stderr)
        return 1
        
    print(f"Saved merged point cloud with {len(merged.points)} points to: {output_file}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
