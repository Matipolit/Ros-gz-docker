import argparse
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial import cKDTree


def _validate_trajectory(traj_df: pd.DataFrame) -> np.ndarray:
    required_columns = ["x", "y", "z"]
    missing = [col for col in required_columns if col not in traj_df.columns]
    if missing:
        raise ValueError(f"Trajectory CSV is missing required columns: {missing}")

    poses = traj_df[required_columns].to_numpy(dtype=np.float64)
    if poses.size == 0:
        raise ValueError("Trajectory CSV contains no pose rows.")
    if not np.isfinite(poses).all():
        raise ValueError("Trajectory CSV contains NaN or infinite pose values.")
    return poses


def _is_collision_free_segment(
    p0: np.ndarray,
    p1: np.ndarray,
    obstacle_tree: cKDTree,
    collision_check_step: float,
    obstacle_margin: float,
) -> bool:
    segment = p1 - p0
    length = float(np.linalg.norm(segment))
    if length == 0.0:
        return True

    samples = max(2, int(np.ceil(length / collision_check_step)) + 1)
    t = np.linspace(0.0, 1.0, samples)
    points = p0[None, :] + t[:, None] * segment[None, :]
    nearest_distances, _ = obstacle_tree.query(points, k=1)
    return bool(np.all(nearest_distances > obstacle_margin))


def _sanitize_point_cloud_points(obstacles: np.ndarray) -> tuple[np.ndarray, int]:
    if obstacles.size == 0:
        return obstacles, 0

    finite_mask = np.isfinite(obstacles).all(axis=1)
    cleaned = obstacles[finite_mask]
    dropped = int(obstacles.shape[0] - cleaned.shape[0])
    return cleaned, dropped


def extract_topology(
    trajectory_csv_path: str,
    point_cloud_path: str,
    output_json_path: str,
    max_edge_distance: float | None = None,
    collision_check_step: float = 0.2,
    obstacle_margin: float = 0.0,
) -> None:
    traj_df = pd.read_csv(trajectory_csv_path)
    poses = _validate_trajectory(traj_df)

    pcd = o3d.io.read_point_cloud(point_cloud_path)
    obstacles = np.asarray(pcd.points, dtype=np.float64)
    if obstacles.size == 0:
        raise ValueError("Point cloud has no points; cannot compute topology clearance.")
    if obstacles.ndim != 2 or obstacles.shape[1] != 3:
        raise ValueError("Point cloud must be a Nx3 set of points.")
    obstacles, dropped_points = _sanitize_point_cloud_points(obstacles)
    if obstacles.shape[0] == 0:
        raise ValueError(
            "Point cloud contains only non-finite points (NaN/Inf); cannot compute topology clearance."
        )
    if dropped_points > 0:
        print(f"Warning: dropped {dropped_points} non-finite points from point cloud.")

    obstacle_tree = cKDTree(obstacles)

    distances, _ = obstacle_tree.query(poses, k=1)
    if not np.isfinite(distances).all():
        raise ValueError("Found non-finite clearance distances. Check point cloud quality.")

    pose_tree = cKDTree(poses)
    if max_edge_distance is None:
        # Adaptive default from local clearances.
        max_edge_distance = float(np.percentile(distances, 75) * 2.0)
        if max_edge_distance <= 0.0:
            max_edge_distance = 1.0

    edges = []
    num_nodes = len(poses)
    for i in range(num_nodes):
        neighbors = pose_tree.query_ball_point(poses[i], r=max_edge_distance)
        for j in neighbors:
            if j <= i:
                continue

            dist_ij = np.linalg.norm(poses[i] - poses[j])
            if dist_ij > (distances[i] + distances[j]):
                continue

            if not _is_collision_free_segment(
                poses[i],
                poses[j],
                obstacle_tree,
                collision_check_step=collision_check_step,
                obstacle_margin=obstacle_margin,
            ):
                continue

            edges.append({"source": i, "target": j, "weight": float(dist_ij)})

    topo_data = {"nodes": [], "edges": edges}

    for idx, (pos, r) in enumerate(zip(poses, distances)):
        topo_data["nodes"].append(
            {
                "id": idx,
                "x": float(pos[0]),
                "y": float(pos[1]),
                "z": float(pos[2]),
                "clearance_radius": float(r),
            }
        )

    output_path = Path(output_json_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(topo_data, f, indent=4)

    print(
        f"Generated graph: {len(poses)} nodes, {len(edges)} edges "
        f"(max_edge_distance={max_edge_distance:.3f}, step={collision_check_step:.3f}, margin={obstacle_margin:.3f})"
    )


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract topology graph from trajectory and map point cloud.")
    parser.add_argument("trajectory_csv_path", nargs="?", default="traj_gt.csv")
    parser.add_argument("point_cloud_path", nargs="?", default="map_gt.ply")
    parser.add_argument("output_json_path", nargs="?", default="topo_gt.json")
    parser.add_argument(
        "--max-edge-distance",
        type=float,
        default=None,
        help="Maximum candidate edge length in meters. If omitted, uses an adaptive value from clearances.",
    )
    parser.add_argument(
        "--collision-check-step",
        type=float,
        default=0.2,
        help="Sampling step in meters when checking segment collision against obstacles.",
    )
    parser.add_argument(
        "--obstacle-margin",
        type=float,
        default=0.0,
        help="Minimum allowed distance to obstacles in meters for collision checks.",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = _parse_args(sys.argv[1:])
    extract_topology(
        trajectory_csv_path=args.trajectory_csv_path,
        point_cloud_path=args.point_cloud_path,
        output_json_path=args.output_json_path,
        max_edge_distance=args.max_edge_distance,
        collision_check_step=args.collision_check_step,
        obstacle_margin=args.obstacle_margin,
    )