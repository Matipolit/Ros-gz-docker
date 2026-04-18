#!/usr/bin/env python3
"""Merge all .pcd files in a folder and voxel-downsample the result incrementally."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import open3d as o3d
import numpy as np


def print_progress(current: int, total: int) -> None:
    if total <= 0:
        return
    bar_width = 30
    filled = int(bar_width * current / total)
    bar = "#" * filled + "-" * (bar_width - filled)
    percent = (current / total) * 100.0
    sys.stdout.write(f"\rProgress: [{bar}] {current}/{total} ({percent:5.1f}%)")
    sys.stdout.flush()

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge .pcd files from a folder into one downsampled .pcd file."
    )
    parser.add_argument("source_folder", help="Folder containing input .pcd files")
    parser.add_argument("output_file", help="Output point cloud path")
    parser.add_argument("voxel_size", type=float, help="Voxel size for downsampling (> 0)")
    parser.add_argument(
        "--output-format",
        choices=["auto", "pcd", "ply"],
        default="auto",
        help="Force output format. 'auto' uses output file extension (default).",
    )
    return parser.parse_args()


def resolve_output_path(output_file: Path, output_format: str) -> Path:
    if output_format == "auto":
        return output_file

    desired_suffix = f".{output_format}"
    if output_file.suffix.lower() == desired_suffix:
        return output_file

    return output_file.with_suffix(desired_suffix)

def main() -> int:
    args = parse_args()

    source_folder = Path(args.source_folder)
    output_file = resolve_output_path(Path(args.output_file), args.output_format)
    voxel_size = args.voxel_size

    if voxel_size <= 0.0:
        raise ValueError("voxel_size must be > 0")

    if not source_folder.is_dir():
        raise FileNotFoundError(f"Source folder does not exist or is not a directory: {source_folder}")

    pcd_files = sorted(source_folder.glob("*.pcd"))
    if not pcd_files:
        raise FileNotFoundError(f"No .pcd files found in: {source_folder}")

    merged = o3d.geometry.PointCloud()
    total_input_points = 0
    valid_files_count = 0

    print(f"Starting merge of {len(pcd_files)} files...")
    print_progress(0, len(pcd_files))

    for i, pcd_path in enumerate(pcd_files):
        cloud = o3d.io.read_point_cloud(str(pcd_path))
        point_count = len(cloud.points)
        
        if point_count == 0:
            continue

        # Skip likely local-frame artifacts near origin after initial valid frames.
        centroid = cloud.get_center()
        if i > 5 and np.linalg.norm(centroid) < 0.05 and valid_files_count > 0:
            print(f"Skipping potentially corrupted local-frame file: {pcd_path.name}")
            continue

        total_input_points += point_count
        valid_files_count += 1

        cloud = cloud.voxel_down_sample(voxel_size=voxel_size)

        merged += cloud

        # Periodically downsample merged cloud to limit growth from overlapping frames.
        if valid_files_count % 20 == 0:
            merged = merged.voxel_down_sample(voxel_size=voxel_size)

        print_progress(i + 1, len(pcd_files))

    print()

    if len(merged.points) == 0:
        raise RuntimeError("All input .pcd files were empty or unreadable.")

    print("Applying final downsampling...")
    downsampled = merged.voxel_down_sample(voxel_size=voxel_size)

    output_file.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(output_file), downsampled)
    if not ok:
        raise RuntimeError(f"Failed to write output point cloud: {output_file}")

    print(f"Processed valid files: {valid_files_count}/{len(pcd_files)}")
    print(f"Total raw input points: {total_input_points}")
    print(f"Final map points: {len(downsampled.points)}")
    print(f"Saved ground truth point cloud to: {output_file}")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())