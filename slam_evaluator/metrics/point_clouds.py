#!/usr/bin/env python3
"""
Skrypt do wyliczania kluczowych metryk jakości mapy 3D (chmur punktów .ply)
względem chmury referencyjnej (Ground Truth).
Zaimplementowane metryki:
1. Chamfer Distance (CD)
2. F-score@tau
3. Accuracy (AC) & Completeness (COM)
4. Voxelized Average Wasserstein Distance (AWD) & Spatial Consistency Score (SCS) (z MapEval 2025)
5. Hausdorff Distance (HD)
6. Absolute Mapping Error (AME)
"""

import argparse
import sys
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial import KDTree
from scipy.linalg import sqrtm
from collections import defaultdict
import os

def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate 3D point cloud metrics for SLAM vs Ground Truth.")
    parser.add_argument("--slam_ply", nargs='+', required=True, help="Scieżka(i) do chmury punktów z algorytmu SLAM (.ply)")
    parser.add_argument("--gt_ply", required=True, help="Scieżka do chmury punktów referencyjnej/Ground Truth (.ply)")
    parser.add_argument("--out_csv", default="point_cloud_metrics.csv", help="Ścieżka do pliku wynikowego CSV")
    parser.add_argument("--tau", type=float, default=0.05, help="Próg tolerancji błędu (tau) do usuwania wartości odstających przy obliczeniach, np. 0.05 m")
    parser.add_argument("--voxel_size", type=float, default=0.2, help="Rozmiar woksela dla metryk AWD/SCS")
    return parser.parse_args()


def compute_wasserstein_distance(mu1, cov1, mu2, cov2):
    """
    Oblicza dystans Wassersteina (W2) dla dwóch rozkładów Gaussa.
    W^2 = ||mu1 - mu2||^2 + Tr(cov1 + cov2 - 2 * sqrt(cov1^0.5 * cov2 * cov1^0.5))
    """
    diff = mu1 - mu2
    m1_norm = np.sum(diff**2)
    
    # Dodanie małej wartości epsilon do przekątnej dla stabilności numerycznej
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
    Oblicza metryki z frameworku MapEval 2025: AWD oraz SCS.
    """
    if len(points_slam) == 0 or len(points_gt) == 0:
        return np.nan, np.nan

    # 1. Wokselizacja - przydzielenie punktów do wokseli
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
        
        # Do bezpiecznego wyliczenia macierzy kowariancji w przestrzeni 3D potrzeba minimum 4 punktów
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
    
    # 2. SCS - Spatial Consistency Score (odchylenie std. odległości W2 w sąsiedztwie 3x3x3 wokseli)
    local_stds = []
    
    for v in w_distances.keys():
        neighbors_w = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    nv = (v[0]+dx, v[1]+dy, v[2]+dz)
                    if nv in w_distances:
                        neighbors_w.append(w_distances[nv])
        
        # Obliczamy odchylenie standardowe tylko jeśli mamy więcej niż jeden woksel w otoczeniu
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

        # Drzewa KD do szybkiego wyszukiwania NN
        tree_slam = KDTree(points_slam)

        # Odległości
        dist_slam2gt, _ = tree_gt.query(points_slam)
        dist_gt2slam, _ = tree_slam.query(points_gt)

        # --- Obliczanie metryk klasycznych ---
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

        # 3. Accuracy (AC) i Completeness (COM)
        ac = mean_slam2gt
        com = mean_gt2slam

        # 6. Absolute Mapping Error (AME)
        ame = np.sqrt(np.mean(valid_slam2gt**2)) if len(valid_slam2gt) > 0 else np.nan

        # 5. Hausdorff Distance (HD)
        hd = max(np.max(dist_slam2gt), np.max(dist_gt2slam))

        # --- Obliczanie metryk MapEval (AWD, SCS) ---
        print(f"Computing Voxelized AWD and SCS with voxel_size = {args.voxel_size} m")
        awd, scs = compute_mapeval_metrics(points_slam, points_gt, args.voxel_size)

        # Raportowanie
        metrics = {
            "Cloud": os.path.basename(slam_path),
            "Chamfer_Distance_m": round(cd, 5),
            "F_score_tau": round(f1_score, 5),
            "Precision": round(precision, 5),
            "Recall": round(recall, 5),
            "Accuracy_m": round(ac, 5),
            "Completeness_m": round(com, 5),
            "Hausdorff_Distance_m": round(hd, 5),
            "Absolute_Mapping_Error_RMSE_m": round(ame, 5),
            "AWD": round(awd, 5),
            "SCS": round(scs, 5)
        }

        print("--- ZESTAWIENIE METRYK ---")
        for k, v in metrics.items():
            print(f"{k}: {v}")
        
        results.append(metrics)

    if not results:
        print("No metrics computed.")
        return

    # Zapis do CSV
    df = pd.DataFrame(results)
    df.to_csv(args.out_csv, index=False)
    print(f"\nWyniki zapisane pomyślnie do: {args.out_csv}")

if __name__ == "__main__":
    main()