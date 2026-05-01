import argparse
import numpy as np
import cv2
import yaml
import os
from scipy.sparse import dok_matrix
from scipy.sparse.csgraph import dijkstra

def load_grid(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    grid_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    
    # 0 = occupied, 254 = free, 205 = unknown (standard ROS pgm)
    # We will treat occupied and unknown as obstacles for strict shortest path,
    # or just occupied as obstacle. Let's threshold.
    free_mask = grid_img > 250
    return free_mask, meta['resolution'], meta['origin']

def build_graph(free_mask):
    height, width = free_mask.shape
    nodes = height * width
    graph = dok_matrix((nodes, nodes), dtype=np.float32)
    
    # Directions: 8-connected
    dirs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    
    print("Building adjacency graph...")
    # This can be slow in pure python, but OK for small-medium grids.
    # To optimize, we could use numpy arrays and coo_matrix, but dok is simple.
    
    # Let's vectorize graph building
    y, x = np.where(free_mask)
    
    # Map 2D coords to 1D node indices
    node_ids = y * width + x
    valid_nodes = set(node_ids)
    
    rows = []
    cols = []
    data = []
    
    for dy, dx, cost in dirs:
        ny = y + dy
        nx = x + dx
        
        valid = (ny >= 0) & (ny < height) & (nx >= 0) & (nx < width)
        
        y_valid = y[valid]
        x_valid = x[valid]
        ny_valid = ny[valid]
        nx_valid = nx[valid]
        
        # Check if neighbors are free
        neighbor_free = free_mask[ny_valid, nx_valid]
        
        y_final = y_valid[neighbor_free]
        x_final = x_valid[neighbor_free]
        ny_final = ny_valid[neighbor_free]
        nx_final = nx_valid[neighbor_free]
        
        from_ids = y_final * width + x_final
        to_ids = ny_final * width + nx_final
        
        rows.extend(from_ids)
        cols.extend(to_ids)
        data.extend([cost] * len(from_ids))

    from scipy.sparse import coo_matrix
    graph = coo_matrix((data, (rows, cols)), shape=(nodes, nodes))
    return graph.tocsr(), width, height

def compute_cost_grid(graph, width, height, goal_x, goal_y):
    goal_id = goal_y * width + goal_x
    print(f"Computing shortest paths to goal ({goal_x}, {goal_y})...")
    dist_matrix = dijkstra(csgraph=graph, directed=False, indices=goal_id, return_predecessors=False)
    
    cost_grid = dist_matrix.reshape((height, width))
    cost_grid[cost_grid == np.inf] = -1.0
    return cost_grid

def evaluate_mse(grid1_yaml, grid2_yaml, goal_world_x, goal_world_y):
    # Load grids
    free1, res1, origin1 = load_grid(grid1_yaml)
    free2, res2, origin2 = load_grid(grid2_yaml)
    
    if res1 != res2:
        print("Warning: Resolutions do not match!")
        
    # Build graphs
    graph1, w1, h1 = build_graph(free1)
    graph2, w2, h2 = build_graph(free2)
    
    # Convert goal to pixel coords
    g1_x = int((goal_world_x - origin1[0]) / res1)
    g1_y = int((goal_world_y - origin1[1]) / res1)
    g1_y = h1 - 1 - g1_y # flip Y
    
    g2_x = int((goal_world_x - origin2[0]) / res2)
    g2_y = int((goal_world_y - origin2[1]) / res2)
    g2_y = h2 - 1 - g2_y # flip Y
    
    if not (0 <= g1_x < w1 and 0 <= g1_y < h1 and free1[g1_y, g1_x]):
        print("Goal is not in free space in map 1!")
        return
        
    if not (0 <= g2_x < w2 and 0 <= g2_y < h2 and free2[g2_y, g2_x]):
        print("Goal is not in free space in map 2!")
        return

    # Compute costs
    cost1 = compute_cost_grid(graph1, w1, h1, g1_x, g1_y)
    cost2 = compute_cost_grid(graph2, w2, h2, g2_x, g2_y)
    
    # We can only compute MSE on the intersecting valid regions
    # For simplicity, if they are the same size, just diff them directly
    if cost1.shape == cost2.shape:
        valid_mask = (cost1 >= 0) & (cost2 >= 0)
        diff = cost1[valid_mask] - cost2[valid_mask]
        mse = np.mean(diff ** 2)
        
        print(f"Valid overlapping area: {np.sum(valid_mask)} cells")
        print(f"Cost Grid MSE: {mse:.4f}")
    else:
        print("Maps are different sizes, need alignment before MSE.")
        # Need logic to align maps based on origins and compute intersection mask
        # This is a bit more involved, but basically crop to common world bounds
        pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Compute Cost Grid MSE between two occupancy grid maps")
    parser.add_argument("map1", help="Path to first map YAML")
    parser.add_argument("map2", help="Path to second map YAML")
    parser.add_argument("--goal_x", type=float, default=0.0, help="Goal X in world coordinates")
    parser.add_argument("--goal_y", type=float, default=0.0, help="Goal Y in world coordinates")
    
    args = parser.parse_args()
    evaluate_mse(args.map1, args.map2, args.goal_x, args.goal_y)
