import argparse
import os
import sys
import time

import numpy as np
import trimesh


def generate_reference_cloud_trimesh(
    input_mesh_path, output_ply_path, target_points=2000000
):
    print(f"Loading the mesh from file: {input_mesh_path}")

    if not os.path.exists(input_mesh_path):
        print("Error: The specified input file does not exist.")
        return

    # Loading the triangle mesh
    try:
        mesh = trimesh.load(input_mesh_path, force="mesh")
    except Exception as e:
        print(f"Error loading the mesh: {e}")
        return

    if mesh.is_empty:
        print("Error: The specified input file does not contain geometry.")
        return

    print(f"Starting surface sampling ({target_points} points)...")
    start_time = time.time()

    # Uniform surface sampling (equivalent to Uniform/Poisson Sampling)
    points, _ = trimesh.sample.sample_surface_even(mesh, target_points)

    end_time = time.time()
    print(f"Sampling finished in {end_time - start_time:.2f} seconds.")

    print(f"Saving reference point cloud to: {output_ply_path}")

    # Creating a point cloud object and saving to .ply
    point_cloud = trimesh.points.PointCloud(points)
    point_cloud.export(output_ply_path)

    print("Ground truth point cloud generated.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert mesh to point cloud using trimesh."
    )
    parser.add_argument("--input_mesh_path", required=True, help="Input mesh file path")
    parser.add_argument("--output_ply_path", required=True, help="Output PLY file path")
    parser.add_argument(
        "--target_points", type=int, default=2000000, help="Target number of points"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    generate_reference_cloud_trimesh(
        args.input_mesh_path, args.output_ply_path, target_points=args.target_points
    )
