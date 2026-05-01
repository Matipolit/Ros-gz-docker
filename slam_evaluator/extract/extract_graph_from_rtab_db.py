import sqlite3
import argparse
import json
import struct

def extract_graph(db_path, output_path):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get nodes (poses)
    # The 'pose' column is a BLOB containing a 3x4 transformation matrix (12 floats)
    cursor.execute("SELECT id, map_id, time_enter, pose FROM Node")
    nodes = []
    for row in cursor.fetchall():
        node_id = row[0]
        map_id = row[1]
        time_enter = row[2]
        pose_blob = row[3]
        
        x, y, z = 0.0, 0.0, 0.0
        if pose_blob and len(pose_blob) == 48:  # 12 floats * 4 bytes
            # Unpack 12 little-endian floats
            pose = struct.unpack('<12f', pose_blob)
            # The matrix is row-major: r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz
            x, y, z = pose[3], pose[7], pose[11]
            
        nodes.append({
            "id": node_id,
            "map_id": map_id,
            "time": time_enter,
            "x": x,
            "y": y,
            "z": z
        })

    # Get links (edges)
    # Types: 0=Neighbor, 1=GlobalLoopClosure, 2=LocalLoopSpace, 3=LocalLoopTime, 4=UserLoop 
    cursor.execute("SELECT from_id, to_id, type FROM Link")
    links = []
    
    for row in cursor.fetchall():
        link_type_str = "Neighbor" if row[2] == 0 else "LoopClosure"
        links.append({
            "from_id": row[0],
            "to_id": row[1],
            "type_id": row[2],
            "type_name": link_type_str
        })

    conn.close()

    graph_data = {
        "nodes": nodes,
        "edges": links
    }

    with open(output_path, 'w') as f:
        json.dump(graph_data, f, indent=4)
    print(f"Graph with {len(nodes)} nodes and {len(links)} edges exported to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--db_path", required=True, help="Path to rtabmap.db")
    parser.add_argument("--output_path", required=True, help="Path to save the extracted graph JSON")
    args = parser.parse_args()
    
    extract_graph(args.db_path, args.output_path)