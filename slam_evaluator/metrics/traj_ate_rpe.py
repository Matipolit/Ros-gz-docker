#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def parse_trajectory(path: Path):
    """
    Parses a trajectory file. Supports CSV with header and space-separated TXT.
    Expected columns: t x y z qx qy qz qw
    Returns:
        times: np.ndarray shape (N,)
        positions: np.ndarray shape (N, 3)
        quats: np.ndarray shape (N, 4) in (x,y,z,w)
    """
    times = []
    positions = []
    quats = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("t"):
                continue
            parts = line.split(",") if "," in line else line.split()
            if len(parts) < 8:
                continue
            try:
                t = float(parts[0])
                x, y, z = map(float, parts[1:4])
                qx, qy, qz, qw = map(float, parts[4:8])
                times.append(t)
                positions.append([x, y, z])
                quats.append([qx, qy, qz, qw])
            except ValueError:
                continue
    
    times = np.array(times)
    positions = np.array(positions)
    quats = np.array(quats)
    
    # Sort by time and remove exact duplicate timestamps
    if len(times) > 0:
        idx = np.argsort(times)
        times = times[idx]
        positions = positions[idx]
        quats = quats[idx]
        
        _, unique_indices = np.unique(times, return_index=True)
        unique_indices = np.sort(unique_indices)
        times = times[unique_indices]
        positions = positions[unique_indices]
        quats = quats[unique_indices]

    return times, positions, quats


def associate_trajectories(t_gt, p_gt, q_gt, t_est, p_est, q_est):
    """
    Interpolates GT to match the timestamps of EST.
    Returns matched arrays of equal length.
    """
    # Only keep estimated points within GT time bounds
    valid_idx = (t_est >= t_gt[0]) & (t_est <= t_gt[-1])
    t_est_valid = t_est[valid_idx]
    p_est_valid = p_est[valid_idx]
    q_est_valid = q_est[valid_idx]
    
    if len(t_est_valid) == 0:
        return np.empty((0,3)), np.empty((0,3)), np.empty((0,4)), np.empty((0,4))
        
    # Interpolate positions
    p_gt_interp = np.zeros_like(p_est_valid)
    for i in range(3):
        p_gt_interp[:, i] = interp1d(t_gt, p_gt[:, i])(t_est_valid)
        
    # Interpolate rotations (SLERP)
    slerp = Slerp(t_gt, R.from_quat(q_gt))
    q_gt_interp = slerp(t_est_valid).as_quat()
    
    return p_gt_interp, p_est_valid, q_gt_interp, q_est_valid


def align_horn(p_gt, p_est):
    """
    Aligns p_est to p_gt using Kabsch/Horn's method via scipy's align_vectors.
    Seeks R, t such that: p_gt \approx R @ p_est + t
    Returns:
        R_align: 3x3 rotation matrix
        t_align: 3x1 translation vector
        p_est_aligned: Nx3 transformed points
    """
    centroid_gt = np.mean(p_gt, axis=0)
    centroid_est = np.mean(p_est, axis=0)
    
    p_gt_centered = p_gt - centroid_gt
    p_est_centered = p_est - centroid_est
    
    R_align, _ = R.align_vectors(p_gt_centered, p_est_centered)
    t_align = centroid_gt - R_align.apply(centroid_est)
    p_est_aligned = R_align.apply(p_est) + t_align
    
    return R_align.as_matrix(), t_align, p_est_aligned


def compute_ate(p_gt, p_est_aligned):
    """
    Computes Absolute Trajectory Error (ATE).
    Reports the Root Mean Square (RMS) error of the positional distances.
    """
    errors = np.linalg.norm(p_gt - p_est_aligned, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    return rmse


def compute_rpe(p_gt, q_gt, p_est, q_est, delta=1):
    """
    Computes Relative Pose Error (RPE) translation component.
    Reports the error distribution of relative motions over 'delta' frames.
    """
    N = len(p_gt)
    errors = []
    
    for i in range(N - delta):
        # GT SE(3) poses
        r_gt_i = R.from_quat(q_gt[i]).as_matrix()
        T_gt_i = np.eye(4); T_gt_i[:3, :3] = r_gt_i; T_gt_i[:3, 3] = p_gt[i]
        
        r_gt_j = R.from_quat(q_gt[i+delta]).as_matrix()
        T_gt_j = np.eye(4); T_gt_j[:3, :3] = r_gt_j; T_gt_j[:3, 3] = p_gt[i+delta]
        
        T_gt_rel = np.linalg.inv(T_gt_i) @ T_gt_j
        
        # EST SE(3) poses
        r_est_i = R.from_quat(q_est[i]).as_matrix()
        T_est_i = np.eye(4); T_est_i[:3, :3] = r_est_i; T_est_i[:3, 3] = p_est[i]
        
        r_est_j = R.from_quat(q_est[i+delta]).as_matrix()
        T_est_j = np.eye(4); T_est_j[:3, :3] = r_est_j; T_est_j[:3, 3] = p_est[i+delta]
        
        T_est_rel = np.linalg.inv(T_est_i) @ T_est_j
        
        # Difference in relative motion
        E = np.linalg.inv(T_gt_rel) @ T_est_rel
        errors.append(np.linalg.norm(E[:3, 3]))
        
    errors = np.array(errors)
    if len(errors) == 0:
        return 0.0, 0.0, {25: 0.0, 50: 0.0, 75: 0.0, 90: 0.0, 95: 0.0, 99: 0.0}
        
    mean_err = np.mean(errors)
    median_err = np.median(errors)
    percentiles = {
        25: np.percentile(errors, 25),
        50: np.percentile(errors, 50),
        75: np.percentile(errors, 75),
        90: np.percentile(errors, 90),
        95: np.percentile(errors, 95),
        99: np.percentile(errors, 99)
    }
    return mean_err, median_err, percentiles


def main():
    parser = argparse.ArgumentParser(description="Calculate ATE and RPE between trajectories using Horn's SE(3) alignment.")
    parser.add_argument("--gt", required=True, type=Path, help="Ground truth trajectory file (CSV/TXT)")
    parser.add_argument("--est", required=True, nargs='+', type=Path, help="Estimated trajectory file(s) to compare")
    parser.add_argument("--out", required=True, type=Path, help="Output CSV file for the metrics report")
    args = parser.parse_args()

    t_gt, p_gt, q_gt = parse_trajectory(args.gt)
    if len(t_gt) == 0:
        print(f"Error: No valid data found in Ground Truth trajectory {args.gt}")
        sys.exit(1)
    
    results = []
    
    for est_path in args.est:
        t_est, p_est, q_est = parse_trajectory(est_path)
        
        if len(t_est) == 0:
            print(f"Warning: No valid data found in {est_path}")
            continue
            
        p_gt_m, p_est_m, q_gt_m, q_est_m = associate_trajectories(t_gt, p_gt, q_gt, t_est, p_est, q_est)
        
        if len(p_gt_m) < 2:
            print(f"Warning: Not enough overlapping timestamps to evaluate {est_path}")
            continue
            
        # 1. Align the compared trajectory to Ground Truth via rigid body SE(3) using Horn's method
        R_align, t_align, p_est_aligned = align_horn(p_gt_m, p_est_m)
        
        # Apply the rotational alignment to the estimated quaternions to complete the SE(3) adjustment
        q_est_aligned = (R.from_matrix(R_align) * R.from_quat(q_est_m)).as_quat()
        
        # 2. Calculate ATE (RMS of positional distances)
        ate_rmse = compute_ate(p_gt_m, p_est_aligned)
        
        # 3. Calculate RPE (mean, median, and percentiles)
        rpe_mean, rpe_median, rpe_perc = compute_rpe(p_gt_m, q_gt_m, p_est_aligned, q_est_aligned, delta=1)
        
        results.append({
            "Trajectory": est_path.name,
            "ATE_RMSE": ate_rmse,
            "RPE_Mean": rpe_mean,
            "RPE_Median": rpe_median,
            "RPE_P25": rpe_perc[25],
            "RPE_P50": rpe_perc[50],
            "RPE_P75": rpe_perc[75],
            "RPE_P90": rpe_perc[90],
            "RPE_P95": rpe_perc[95],
            "RPE_P99": rpe_perc[99]
        })
        
    if not results:
        print("No metrics could be computed across the provided estimation files.")
        return
        
    # Write the results to the specified output CSV
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
        
    print(f"Metrics successfully written to {args.out}")

if __name__ == "__main__":
    main()