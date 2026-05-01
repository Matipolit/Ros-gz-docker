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

    # Równomierne próbkowanie powierzchni (odpowiednik Uniform/Poisson Sampling)
    points, faces = trimesh.sample.sample_surface_even(mesh, target_points)

    end_time = time.time()
    print(f"Sampling finished in {end_time - start_time:.2f} seconds.")

    print(f"Saving reference point cloud to: {output_ply_path}")

    # Tworzenie obiektu chmury punktów i zapis do .ply
    point_cloud = trimesh.points.PointCloud(points)
    point_cloud.export(output_ply_path)

    print("Ground truth point cloud generated.")


if __name__ == "__main__":
    input_file_name = sys.argv[1]
    output_file_name = sys.argv[2]

    generate_reference_cloud_trimesh(
        input_file_name, output_file_name, target_points=2000000
    )
