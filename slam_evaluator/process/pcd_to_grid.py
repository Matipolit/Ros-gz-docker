import argparse
import numpy as np
import open3d as o3d
import cv2
import yaml
import os

def pcd_to_grid(pcd_path, output_prefix, resolution=0.05, min_z=0.1, max_z=1.5, occ_threshold=1):
    print(f"Loading {pcd_path}...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    print(f"Loaded {len(points)} points.")

    # Filter by Z (height)
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    filtered_points = points[mask]

    print(f"Points after Z filtering ({min_z} to {max_z}): {len(filtered_points)}")

    if len(filtered_points) == 0:
        print("No points left after filtering!")
        return

    # Get XY bounds
    min_x = np.min(filtered_points[:, 0])
    max_x = np.max(filtered_points[:, 0])
    min_y = np.min(filtered_points[:, 1])
    max_y = np.max(filtered_points[:, 1])

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    print(f"Grid size: {width} x {height}")

    # Create grid (0 = free, initially all unknown/free)
    # Actually ROS uses: 0 = free, 100 = occupied, -1 = unknown.
    # But in PGM, it's usually 0 to 255. 0 is black (occupied), 254 is white (free), 205 is unknown.
    # Let's initialize with 205 (unknown)
    grid = np.full((height, width), 254, dtype=np.uint8) # assume white/free where no obstacle, maybe better to trace rays for unknown? 
    # For a simple grid, let's just make everything free (254) except obstacles (0).
    
    # Calculate indices
    indices_x = np.floor((filtered_points[:, 0] - min_x) / resolution).astype(int)
    indices_y = np.floor((filtered_points[:, 1] - min_y) / resolution).astype(int)

    # Count points in each cell
    # A fast way is using np.histogram2d or np.add.at
    occupancy_counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(occupancy_counts, (indices_y, indices_x), 1)

    # Threshold
    grid[occupancy_counts >= occ_threshold] = 0 # Occupied

    # Flip Y because image origin is top-left, but map origin is bottom-left
    grid = np.flipud(grid)

    pgm_path = f"{output_prefix}.pgm"
    yaml_path = f"{output_prefix}.yaml"

    # Write PGM
    cv2.imwrite(pgm_path, grid)

    # Origin is the bottom-left corner
    # Since we flipped Y, the bottom-left cell corresponds to (min_x, min_y)
    origin_x = float(min_x)
    origin_y = float(min_y)

    yaml_data = {
        'image': os.path.basename(pgm_path),
        'resolution': float(resolution),
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    print(f"Saved {pgm_path} and {yaml_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convert PCD/PLY to 2D Occupancy Grid (PGM+YAML)")
    parser.add_argument("input", help="Input PCD or PLY file")
    parser.add_argument("output_prefix", help="Output prefix (e.g. 'map' to produce map.pgm and map.yaml)")
    parser.add_argument("--res", type=float, default=0.05, help="Resolution (m/pixel)")
    parser.add_argument("--min_z", type=float, default=0.1, help="Minimum Z for obstacle")
    parser.add_argument("--max_z", type=float, default=1.5, help="Maximum Z for obstacle")
    parser.add_argument("--occ", type=int, default=1, help="Min points per cell to be occupied")

    args = parser.parse_args()
    pcd_to_grid(args.input, args.output_prefix, args.res, args.min_z, args.max_z, args.occ)
