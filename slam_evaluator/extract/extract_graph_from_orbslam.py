import argparse
import json

def round_pt(pt, decimals=5):
    """Round point coordinates to handle slight floating point differences"""
    return (round(pt[0], decimals), round(pt[1], decimals), round(pt[2], decimals))

def parse_orbslam_graph(input_txt, output_json):
    nodes = {}
    edges = []
    node_counter = 0

    # Helper function to get or create a node ID based on 3D coordinates
    def get_node_id(pt):
        nonlocal node_counter
        rpt = round_pt(pt)
        if rpt not in nodes:
            nodes[rpt] = {
                "id": node_counter,
                "x": pt[0],
                "y": pt[1],
                "z": pt[2]
            }
            node_counter += 1
        return nodes[rpt]["id"]

    try:
        with open(input_txt, 'r') as f:
            for line in f:
                parts = line.strip().split()
                # Each line should have at least 6 floats: x1 y1 z1 x2 y2 z2
                if len(parts) >= 6:
                    x1, y1, z1, x2, y2, z2 = map(float, parts[:6])
                    
                    id1 = get_node_id((x1, y1, z1))
                    id2 = get_node_id((x2, y2, z2))
                    
                    # Avoid self-loops if two points are nearly identical
                    if id1 != id2:
                        edges.append({
                            "from_id": id1,
                            "to_id": id2
                        })
    except FileNotFoundError:
        print(f"Error: Could not find {input_txt}")
        return

    # Format nodes for JSON output (matching RTAB-Map format for comparability)
    nodes_list = list(nodes.values())

    graph_data = {
        "nodes": nodes_list,
        "edges": edges
    }

    with open(output_json, 'w') as f:
        json.dump(graph_data, f, indent=4)
    
    print(f"Graph with {len(nodes_list)} nodes and {len(edges)} edges exported to {output_json}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_txt", required=True, help="Path to ORB-SLAM graph txt (e.g., essential_graph.txt or covisibility_graph.txt)")
    parser.add_argument("--output_json", required=True, help="Path to save the extracted graph JSON")
    args = parser.parse_args()
    
    parse_orbslam_graph(args.input_txt, args.output_json)
