#!/usr/bin/env python3
"""Merge all .pcd files in a folder and voxel-downsample the result incrementally.

Features:
- Per-frame voxel downsampling
- Optional radial range filtering (distance from origin)
- Optional axis-aligned ROI cropping
- Optional fixed rotation alignment around Z axis
- Optional centroid guard for likely corrupted local-frame clouds
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d


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
        description="Merge .pcd files from a folder into one downsampled point cloud."
    )
    parser.add_argument("source_folder", help="Folder containing input .pcd files")
    parser.add_argument("output_file", help="Output point cloud path")
    parser.add_argument(
        "voxel_size", type=float, help="Voxel size for downsampling (> 0)"
    )

    parser.add_argument(
        "--output-format",
        choices=["auto", "pcd", "ply"],
        default="auto",
        help="Force output format. 'auto' uses output file extension (default).",
    )

    parser.add_argument(
        "--min-range",
        type=float,
        default=None,
        help="Keep points with distance >= min-range from origin (meters).",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=None,
        help="Keep points with distance <= max-range from origin (meters).",
    )

    parser.add_argument(
        "--crop-x-min", type=float, default=None, help="ROI min X (meters)."
    )
    parser.add_argument(
        "--crop-x-max", type=float, default=None, help="ROI max X (meters)."
    )
    parser.add_argument(
        "--crop-y-min", type=float, default=None, help="ROI min Y (meters)."
    )
    parser.add_argument(
        "--crop-y-max", type=float, default=None, help="ROI max Y (meters)."
    )
    parser.add_argument(
        "--crop-z-min", type=float, default=None, help="ROI min Z (meters)."
    )
    parser.add_argument(
        "--crop-z-max", type=float, default=None, help="ROI max Z (meters)."
    )

    parser.add_argument(
        "--rotate-z-deg",
        type=float,
        default=0.0,
        help="Apply fixed rotation around Z axis (degrees) to each frame before merge.",
    )

    parser.add_argument(
        "--range-mode",
        choices=["global_origin", "sensor_local"],
        default="global_origin",
        help="How to measure distance for min/max range filtering. 'sensor_local' reads PCD VIEWPOINT to filter relative to the camera.",
    )

    parser.add_argument(
        "--keyframe-poses",
        type=str,
        default=None,
        help="Path to poses.txt from SLAM. If provided, only .pcd files matching a keyframe timestamp will be processed.",
    )
    parser.add_argument(
        "--keyframe-tolerance",
        type=float,
        default=0.05,
        help="Max time difference (in seconds) to match a PCD to a keyframe (default: 0.05).",
    )

    parser.add_argument(
        "--trajectory-csv",
        type=str,
        default=None,
        help="Path to trajectory CSV (t,x,y,z...). Used in sensor_local mode to override zeroed PCD VIEWPOINTs.",
    )

    parser.add_argument(
        "--disable-centroid-guard",
        action="store_true",
        help="Disable heuristic that skips likely corrupted local-frame clouds near origin.",
    )

    return parser.parse_args()


def resolve_output_path(output_file: Path, output_format: str) -> Path:
    if output_format == "auto":
        return output_file
    desired_suffix = f".{output_format}"
    if output_file.suffix.lower() == desired_suffix:
        return output_file
    return output_file.with_suffix(desired_suffix)


def validate_args(args: argparse.Namespace) -> None:
    if args.voxel_size <= 0.0:
        raise ValueError("voxel_size must be > 0")

    if args.min_range is not None and args.min_range < 0.0:
        raise ValueError("--min-range must be >= 0")
    if args.max_range is not None and args.max_range < 0.0:
        raise ValueError("--max-range must be >= 0")
    if (
        args.min_range is not None
        and args.max_range is not None
        and args.min_range > args.max_range
    ):
        raise ValueError("--min-range cannot be greater than --max-range")

    for axis in ("x", "y", "z"):
        lo = getattr(args, f"crop_{axis}_min")
        hi = getattr(args, f"crop_{axis}_max")
        if lo is not None and hi is not None and lo > hi:
            raise ValueError(
                f"--crop-{axis}-min cannot be greater than --crop-{axis}-max"
            )


def maybe_rotate_cloud_z(
    cloud: o3d.geometry.PointCloud, angle_deg: float
) -> o3d.geometry.PointCloud:
    if abs(angle_deg) < 1e-12:
        return cloud
    angle_rad = math.radians(angle_deg)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    rot = np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    cloud = o3d.geometry.PointCloud(cloud)
    cloud.rotate(rot, center=(0.0, 0.0, 0.0))
    return cloud


def filter_by_range(
    cloud: o3d.geometry.PointCloud,
    min_range: Optional[float],
    max_range: Optional[float],
    sensor_origin: Optional[np.ndarray] = None,
) -> o3d.geometry.PointCloud:
    if min_range is None and max_range is None:
        return cloud
    points = np.asarray(cloud.points)
    if points.size == 0:
        return cloud

    if sensor_origin is not None:
        dists = np.linalg.norm(points - sensor_origin, axis=1)
    else:
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


def get_pcd_viewpoint(pcd_path: Path) -> Optional[np.ndarray]:
    try:
        with open(pcd_path, "rb") as f:
            for line in f:
                try:
                    line_str = line.decode("ascii").strip()
                except UnicodeDecodeError:
                    continue
                if line_str.startswith("DATA"):
                    break
                if line_str.startswith("VIEWPOINT"):
                    parts = line_str.split()
                    if len(parts) >= 4:
                        return np.array(
                            [float(parts[1]), float(parts[2]), float(parts[3])],
                            dtype=np.float64,
                        )
    except Exception:
        pass
    return None


def load_trajectory(csv_path: str) -> tuple[np.ndarray, np.ndarray]:
    import csv

    times = []
    positions = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        for row in reader:
            if not row:
                continue
            try:
                times.append(float(row[0]))
                positions.append([float(row[1]), float(row[2]), float(row[3])])
            except ValueError:
                pass
    if not times:
        return np.array([]), np.empty((0, 3))

    times = np.array(times)
    positions = np.array(positions)
    sort_idx = np.argsort(times)
    return times[sort_idx], positions[sort_idx]


def get_interpolated_position(
    t: float, times: np.ndarray, positions: np.ndarray
) -> Optional[np.ndarray]:
    if len(times) == 0:
        return None
    idx = np.searchsorted(times, t)
    if idx == 0:
        return positions[0]
    if idx == len(times):
        return positions[-1]

    t0, t1 = times[idx - 1], times[idx]
    p0, p1 = positions[idx - 1], positions[idx]
    if t1 == t0:
        return p0
    alpha = (t - t0) / (t1 - t0)
    return p0 + alpha * (p1 - p0)


def load_keyframe_times(path: str) -> np.ndarray:
    times = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            try:
                times.append(float(line.split()[0]))
            except ValueError:
                pass
    return np.array(sorted(times))


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
    validate_args(args)

    source_folder = Path(args.source_folder)
    output_file = resolve_output_path(Path(args.output_file), args.output_format)

    if not source_folder.is_dir():
        raise FileNotFoundError(
            f"Source folder does not exist or is not a directory: {source_folder}"
        )

    pcd_files = sorted(source_folder.glob("*.pcd"))
    if not pcd_files:
        raise FileNotFoundError(f"No .pcd files found in: {source_folder}")

    keyframe_times = None
    if getattr(args, "keyframe_poses", None):
        kf_path = Path(args.keyframe_poses)
        if kf_path.is_file():
            keyframe_times = load_keyframe_times(str(kf_path))
            print(f"Loaded {len(keyframe_times)} keyframe timestamps from {kf_path}.")
        else:
            print(f"Warning: Keyframe poses file not found: {kf_path}")

    traj_times, traj_positions = np.array([]), np.empty((0, 3))
    if args.range_mode == "sensor_local" and getattr(args, "trajectory_csv", None):
        traj_path = Path(args.trajectory_csv)
        if traj_path.is_file():
            traj_times, traj_positions = load_trajectory(str(traj_path))
            print(f"Loaded trajectory with {len(traj_times)} poses.")
        else:
            print(f"Warning: Trajectory CSV not found: {traj_path}")

    merged = o3d.geometry.PointCloud()
    total_input_points = 0
    total_after_filters_points = 0
    valid_files_count = 0
    skipped_empty = 0
    skipped_centroid_guard = 0
    skipped_after_filters = 0
    skipped_not_keyframe = 0

    print(f"Starting merge of {len(pcd_files)} files...")
    print_progress(0, len(pcd_files))

    for i, pcd_path in enumerate(pcd_files):
        if keyframe_times is not None and len(keyframe_times) > 0:
            try:
                ts = float(pcd_path.stem)
                nearest = np.min(np.abs(keyframe_times - ts))
                if nearest > args.keyframe_tolerance:
                    skipped_not_keyframe += 1
                    print_progress(i + 1, len(pcd_files))
                    continue
            except ValueError:
                pass

        cloud = o3d.io.read_point_cloud(str(pcd_path))
        point_count = len(cloud.points)

        if point_count == 0:
            skipped_empty += 1
            print_progress(i + 1, len(pcd_files))
            continue

        if not args.disable_centroid_guard:
            centroid = cloud.get_center()
            if i > 5 and np.linalg.norm(centroid) < 0.05 and valid_files_count > 0:
                skipped_centroid_guard += 1
                print_progress(i + 1, len(pcd_files))
                continue

        sensor_origin = None
        if args.range_mode == "sensor_local" and (
            args.min_range is not None or args.max_range is not None
        ):
            if len(traj_times) > 0:
                try:
                    ts = float(pcd_path.stem)
                    sensor_origin = get_interpolated_position(
                        ts, traj_times, traj_positions
                    )
                except ValueError:
                    pass

            if sensor_origin is None:
                sensor_origin = get_pcd_viewpoint(pcd_path)
                if (
                    sensor_origin is not None
                    and np.allclose(sensor_origin, 0.0)
                    and len(traj_times) > 0
                ):
                    sensor_origin = None

            if sensor_origin is None:
                sensor_origin = np.array([0.0, 0.0, 0.0])

        cloud = filter_by_range(cloud, args.min_range, args.max_range, sensor_origin)
        cloud = maybe_rotate_cloud_z(cloud, args.rotate_z_deg)
        cloud = filter_by_roi(args, cloud)

        filtered_count = len(cloud.points)
        if filtered_count == 0:
            skipped_after_filters += 1
            print_progress(i + 1, len(pcd_files))
            continue

        total_input_points += point_count
        total_after_filters_points += filtered_count
        valid_files_count += 1

        cloud = cloud.voxel_down_sample(voxel_size=args.voxel_size)
        merged += cloud

        if valid_files_count % 20 == 0:
            merged = merged.voxel_down_sample(voxel_size=args.voxel_size)

        print_progress(i + 1, len(pcd_files))

    print()

    if len(merged.points) == 0:
        raise RuntimeError("All input .pcd files were empty or removed by filters.")

    print("Applying final downsampling...")
    downsampled = merged.voxel_down_sample(voxel_size=args.voxel_size)

    output_file.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(output_file), downsampled)
    if not ok:
        raise RuntimeError(f"Failed to write output point cloud: {output_file}")

    print(f"Processed valid files: {valid_files_count}/{len(pcd_files)}")
    if keyframe_times is not None:
        print(f"Skipped not matching keyframes: {skipped_not_keyframe}")
    print(f"Skipped empty files: {skipped_empty}")
    print(f"Skipped by centroid guard: {skipped_centroid_guard}")
    print(f"Skipped by range/ROI filters: {skipped_after_filters}")
    print(f"Total raw input points (accepted files): {total_input_points}")
    print(f"Total points after range/ROI filters: {total_after_filters_points}")
    print(f"Final map points: {len(downsampled.points)}")
    print(f"Saved ground truth point cloud to: {output_file}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
