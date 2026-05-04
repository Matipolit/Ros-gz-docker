import argparse
import csv
import json
from pathlib import Path

import networkx as nx
import numpy as np
import open3d as o3d
from scipy.linalg import eigh
from scipy.spatial import cKDTree


def calculate_d_optimality(laplacian_eigenvalues):
    """
    Computes a metric related to D-optimality using the log-sum to prevent overflow.
    """
    # Filter out zero eigenvalues and numerical errors
    non_zero_eigs = [val for val in laplacian_eigenvalues if val > 1e-7]
    if not non_zero_eigs:
        return 0.0

    # Geometric mean of non-zero eigenvalues via log-sum
    log_det = np.sum(np.log(non_zero_eigs))
    d_opt = np.exp(log_det / len(non_zero_eigs))
    return d_opt


def analyze_graph(json_path, obstacle_tree=None):
    print(f"Analyzing {json_path}...")
    with open(json_path, "r") as f:
        data = json.load(f)

    G = nx.Graph()

    for n in data.get("nodes", []):
        G.add_node(n["id"], x=n.get("x", 0.0), y=n.get("y", 0.0), z=n.get("z", 0.0))

    for e in data.get("edges", []):
        from_id = e["from_id"]
        to_id = e["to_id"]

        if from_id in G.nodes and to_id in G.nodes:
            p1 = np.array(
                [G.nodes[from_id]["x"], G.nodes[from_id]["y"], G.nodes[from_id]["z"]]
            )
            p2 = np.array(
                [G.nodes[to_id]["x"], G.nodes[to_id]["y"], G.nodes[to_id]["z"]]
            )
            dist = np.linalg.norm(p1 - p2)
            G.add_edge(from_id, to_id, weight=dist)

    stats = {
        "graph": Path(json_path).name,
        "nodes": G.number_of_nodes(),
        "edges": G.number_of_edges(),
        "connected_components_betti_0": 0,
        "cycles_betti_1": 0,
        "total_skeleton_length": 0.0,
        "algebraic_connectivity": 0.0,
        "d_optimality": 0.0,
        "spanning_tree_weight": 0.0,
        "pgti_lambda": 0.0,
    }

    if stats["nodes"] == 0:
        return stats

    # Betti-0: Connected components
    betti_0 = nx.number_connected_components(G)
    stats["connected_components_betti_0"] = betti_0

    # Betti-1: Independent cycles (E - V + C)
    betti_1 = stats["edges"] - stats["nodes"] + betti_0
    stats["cycles_betti_1"] = betti_1

    # Skeleton length proxy
    stats["total_skeleton_length"] = sum(nx.get_edge_attributes(G, "weight").values())

    # Minimum Spanning Tree Weight
    if betti_0 == 1:
        mst = nx.minimum_spanning_tree(G, weight="weight")
        stats["spanning_tree_weight"] = sum(
            nx.get_edge_attributes(mst, "weight").values()
        )
    else:
        # Sum of MSTs for each component
        mst_weight = 0.0
        for comp in nx.connected_components(G):
            subgraph = G.subgraph(comp)
            mst = nx.minimum_spanning_tree(subgraph, weight="weight")
            mst_weight += sum(nx.get_edge_attributes(mst, "weight").values())
        stats["spanning_tree_weight"] = mst_weight

    # PGTI (Pose-Graph Topological Integrity) Calculation
    if obstacle_tree is not None and stats["nodes"] > 0:
        nodes_list = list(G.nodes())
        X = np.array(
            [[G.nodes[n]["x"], G.nodes[n]["y"], G.nodes[n]["z"]] for n in nodes_list]
        )

        # Get clearance radii (distances to nearest obstacles)
        R_clearance, _ = obstacle_tree.query(X, k=1)

        # Build Free-Space Graph (Gm = Gp U Gf)
        Gm = G.copy()

        # Add edges where clearance spheres overlap: ||Xi - Xj|| <= Ri + Rj
        # To avoid N^2 loop, we use KDTree of the poses
        pose_tree = cKDTree(X)
        max_r = float(np.max(R_clearance)) if X.shape[0] > 0 else 0
        search_radius = 2.0 * max_r

        for i in range(len(X)):
            neighbors = pose_tree.query_ball_point(X[i], r=search_radius)
            for j in neighbors:
                if j <= i:
                    continue
                dist_ij = np.linalg.norm(X[i] - X[j])
                if dist_ij <= (R_clearance[i] + R_clearance[j]):
                    if not Gm.has_edge(nodes_list[i], nodes_list[j]):
                        Gm.add_edge(nodes_list[i], nodes_list[j], weight=dist_ij)

        # Calculate Heat Kernel Signatures (HKS)
        try:
            # We use normalized Laplacian to make the eigenvalues comparable regardless of graph degree
            L_p = nx.normalized_laplacian_matrix(G).toarray()
            L_m = nx.normalized_laplacian_matrix(Gm).toarray()

            evals_p, evecs_p = eigh(L_p)
            evals_m, evecs_m = eigh(L_m)

            # Logarithmically spaced time scales for heat diffusion
            t_vals = np.logspace(-1, 2, 20)

            # Compute HKS matrices: shape (N_nodes, N_time_scales)
            H_p = (evecs_p**2) @ np.exp(-np.outer(evals_p, t_vals))
            H_m = (evecs_m**2) @ np.exp(-np.outer(evals_m, t_vals))

            # Max difference in thermal propagation -> Lambda
            stats["pgti_lambda"] = float(np.max(np.abs(H_m - H_p)))
        except Exception as e:
            print(f"Warning: PGTI calculation failed for {json_path}: {e}")

    # Spectral Metrics
    try:
        L_eigvals = nx.laplacian_spectrum(G)
        sorted_eigvals = np.sort(L_eigvals)

        # Algebraic Connectivity (second smallest eigenvalue)
        if len(sorted_eigvals) > 1:
            stats["algebraic_connectivity"] = sorted_eigvals[1]

        stats["d_optimality"] = calculate_d_optimality(sorted_eigvals)

    except Exception as e:
        print(f"Warning: Spectral analysis failed for {json_path}: {e}")

    return stats


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate Pose-Graph and Topological Metrics"
    )
    parser.add_argument(
        "--gt_graph",
        required=True,
        help="Ground truth graph JSON file to compare against",
    )
    parser.add_argument(
        "--slam_graphs",
        nargs="+",
        required=True,
        help="List of SLAM graph JSON files to evaluate",
    )
    parser.add_argument(
        "--gt_pcd",
        required=False,
        help="Ground truth point cloud (.ply) for PGTI calculation",
    )
    parser.add_argument(
        "--output_csv", default="graph_metrics.csv", help="Output CSV file path"
    )
    args = parser.parse_args()

    results = []
    obstacle_tree = None

    # Load point cloud for PGTI if provided
    if args.gt_pcd:
        try:
            print(f"Loading point cloud for PGTI: {args.gt_pcd}")
            pcd = o3d.io.read_point_cloud(args.gt_pcd)
            obstacles = np.asarray(pcd.points, dtype=np.float64)
            if obstacles.size > 0:
                finite_mask = np.isfinite(obstacles).all(axis=1)
                obstacles = obstacles[finite_mask]
                obstacle_tree = cKDTree(obstacles)
                print(f"Loaded {obstacles.shape[0]} valid obstacle points.")
        except Exception as e:
            print(f"Error loading point cloud {args.gt_pcd}: {e}")

    # Analyze Ground Truth Graph
    try:
        gt_stats = analyze_graph(args.gt_graph, obstacle_tree)

        gt_row = gt_stats.copy()
        gt_row["data_type"] = "Ground Truth"
        for key in [
            "betti_0_error",
            "betti_1_error",
            "connectivity_error",
            "d_opt_error",
            "skeleton_length_error",
        ]:
            gt_row[key] = 0.0
        results.append(gt_row)
    except Exception as e:
        print(f"Error analyzing ground truth graph {args.gt_graph}: {e}")
        return

    # Analyze SLAM Graphs
    for json_path in args.slam_graphs:
        try:
            slam_stats = analyze_graph(json_path, obstacle_tree)

            row = slam_stats.copy()
            row["data_type"] = "SLAM"
            row["betti_0_error"] = abs(
                slam_stats["connected_components_betti_0"]
                - gt_stats["connected_components_betti_0"]
            )
            row["betti_1_error"] = abs(
                slam_stats["cycles_betti_1"] - gt_stats["cycles_betti_1"]
            )
            row["connectivity_error"] = abs(
                slam_stats["algebraic_connectivity"]
                - gt_stats["algebraic_connectivity"]
            )
            row["d_opt_error"] = abs(
                slam_stats["d_optimality"] - gt_stats["d_optimality"]
            )
            row["skeleton_length_error"] = abs(
                slam_stats["total_skeleton_length"] - gt_stats["total_skeleton_length"]
            )

            results.append(row)
        except Exception as e:
            print(f"Error analyzing {json_path}: {e}")

    if len(results) <= 1:
        print("No SLAM graphs processed.")
        return

    output_path = Path(args.output_csv)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    headers = list(results[0].keys())
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(results)

    print(f"Evaluation complete. Results saved to {args.output_csv}")


if __name__ == "__main__":
    main()
