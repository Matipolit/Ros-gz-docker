#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path


def align_graph(graph_path, aligned_poses_path, output_path):
    with open(graph_path, "r") as f:
        graph_data = json.load(f)

    # Load aligned poses: timestamp x y z qx qy qz qw id
    id_to_pos = {}
    with open(aligned_poses_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 9:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                pid = int(parts[8])
                id_to_pos[pid] = (x, y, z)
            except ValueError:
                continue

    if not id_to_pos:
        print("Error: No valid aligned poses found.", file=sys.stderr)
        return 1

    for node in graph_data.get("nodes", []):
        node_id = node["id"]
        if node_id in id_to_pos:
            node["x"], node["y"], node["z"] = id_to_pos[node_id]
        else:
            print(
                f"Warning: Node {node_id} not found in aligned poses.", file=sys.stderr
            )

    with open(output_path, "w") as f:
        json.dump(graph_data, f, indent=4)
    print(f"Aligned graph saved to {output_path}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Align graph JSON using aligned poses file."
    )
    parser.add_argument("graph_json", help="Input graph JSON file")
    parser.add_argument("aligned_poses", help="Aligned poses TXT file")
    parser.add_argument("output_json", help="Output aligned graph JSON file")
    args = parser.parse_args()

    sys.exit(align_graph(args.graph_json, args.aligned_poses, args.output_json))
