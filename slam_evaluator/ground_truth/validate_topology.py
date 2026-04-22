import argparse
import json
import math
from collections import deque
from pathlib import Path


def _is_finite_number(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _percentile(sorted_values: list[float], p: float) -> float:
    if not sorted_values:
        return 0.0
    idx = int(p * (len(sorted_values) - 1))
    return sorted_values[idx]


def validate_topology(path: Path, args: argparse.Namespace) -> int:
    errors: list[str] = []
    warnings: list[str] = []

    try:
        topology = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"FAIL: unable to parse JSON from {path}: {exc}")
        return 2

    if not isinstance(topology, dict):
        print("FAIL: topology root must be a JSON object.")
        return 2

    if "nodes" not in topology or "edges" not in topology:
        print("FAIL: topology JSON must contain 'nodes' and 'edges' keys.")
        return 2

    nodes = topology["nodes"]
    edges = topology["edges"]

    if not isinstance(nodes, list):
        errors.append("'nodes' must be a list.")
        nodes = []
    if not isinstance(edges, list):
        errors.append("'edges' must be a list.")
        edges = []

    node_ids: list[int] = []
    node_id_set: set[int] = set()
    for i, node in enumerate(nodes):
        if not isinstance(node, dict):
            errors.append(f"node[{i}] is not an object")
            continue

        for key in ("id", "x", "y", "z", "clearance_radius"):
            if key not in node:
                errors.append(f"node[{i}] missing key '{key}'")

        node_id = node.get("id")
        if not isinstance(node_id, int):
            errors.append(f"node[{i}] id must be an integer")
        else:
            node_ids.append(node_id)
            node_id_set.add(node_id)

        for key in ("x", "y", "z", "clearance_radius"):
            if key in node and not _is_finite_number(node[key]):
                errors.append(f"node[{i}] key '{key}' must be finite")

    if len(node_id_set) != len(node_ids):
        errors.append("node ids are not unique")

    if node_ids:
        min_id = min(node_ids)
        max_id = max(node_ids)
        if not args.allow_noncontiguous_ids:
            expected = set(range(min_id, max_id + 1))
            if node_id_set != expected:
                errors.append("node ids are not contiguous")
    else:
        min_id = None
        max_id = None

    edge_weights: list[float] = []
    invalid_edge_refs = 0
    self_loops = 0
    adjacency: dict[int, set[int]] = {node_id: set() for node_id in node_id_set}

    for i, edge in enumerate(edges):
        if not isinstance(edge, dict):
            errors.append(f"edge[{i}] is not an object")
            continue

        for key in ("source", "target", "weight"):
            if key not in edge:
                errors.append(f"edge[{i}] missing key '{key}'")

        src = edge.get("source")
        dst = edge.get("target")
        w = edge.get("weight")

        if not isinstance(src, int) or not isinstance(dst, int):
            errors.append(f"edge[{i}] source/target must be integers")
            continue

        if src not in node_id_set or dst not in node_id_set:
            invalid_edge_refs += 1
            continue

        if src == dst:
            self_loops += 1

        if not _is_finite_number(w):
            errors.append(f"edge[{i}] weight must be finite")
            continue

        wf = float(w)
        if wf < 0.0:
            errors.append(f"edge[{i}] weight must be non-negative")
            continue

        edge_weights.append(wf)
        adjacency[src].add(dst)
        adjacency[dst].add(src)

    if invalid_edge_refs:
        errors.append(f"invalid edge references: {invalid_edge_refs}")
    if self_loops and not args.allow_self_loops:
        errors.append(f"self loops found: {self_loops}")

    degrees = [len(adjacency[nid]) for nid in sorted(node_id_set)]
    isolated_nodes = sum(1 for d in degrees if d == 0)

    components = 0
    largest_component = 0
    visited: set[int] = set()
    for nid in node_id_set:
        if nid in visited:
            continue
        components += 1
        queue = deque([nid])
        visited.add(nid)
        size = 0
        while queue:
            u = queue.popleft()
            size += 1
            for v in adjacency[u]:
                if v not in visited:
                    visited.add(v)
                    queue.append(v)
        largest_component = max(largest_component, size)

    total_nodes = len(node_id_set)
    largest_component_ratio = (
        float(largest_component) / float(total_nodes) if total_nodes > 0 else 0.0
    )

    if isolated_nodes > args.max_isolated_nodes:
        errors.append(
            f"isolated nodes {isolated_nodes} exceeds allowed max {args.max_isolated_nodes}"
        )
    if largest_component_ratio < args.min_largest_component_ratio:
        errors.append(
            "largest component ratio "
            f"{largest_component_ratio:.3f} below minimum {args.min_largest_component_ratio:.3f}"
        )

    avg_degree = float(sum(degrees)) / float(len(degrees)) if degrees else 0.0
    max_degree = max(degrees) if degrees else 0
    if args.max_avg_degree is not None and avg_degree > args.max_avg_degree:
        warnings.append(
            f"average degree {avg_degree:.3f} exceeds threshold {args.max_avg_degree:.3f}"
        )
    if args.max_max_degree is not None and max_degree > args.max_max_degree:
        warnings.append(f"max degree {max_degree} exceeds threshold {args.max_max_degree}")

    if edge_weights:
        sorted_weights = sorted(edge_weights)
        p95 = _percentile(sorted_weights, 0.95)
        min_w = sorted_weights[0]
        max_w = sorted_weights[-1]
        median_w = _percentile(sorted_weights, 0.50)
    else:
        p95 = 0.0
        min_w = 0.0
        max_w = 0.0
        median_w = 0.0

    if args.max_p95_weight is not None and p95 > args.max_p95_weight:
        warnings.append(
            f"edge weight p95 {p95:.4f} exceeds threshold {args.max_p95_weight:.4f}"
        )

    if not args.quiet:
        print(f"file: {path}")
        print(f"nodes: {len(nodes)}")
        print(f"edges: {len(edges)}")
        print(f"node_id_range: {min_id}..{max_id}")
        print(f"components: {components}")
        print(f"largest_component: {largest_component} ({largest_component_ratio:.3f})")
        print(f"isolated_nodes: {isolated_nodes}")
        print(f"degree_avg_max: {avg_degree:.3f}, {max_degree}")
        print(f"weight_min_median_p95_max: {min_w:.4f}, {median_w:.4f}, {p95:.4f}, {max_w:.4f}")

    if warnings:
        for item in warnings:
            print(f"WARN: {item}")

    if errors:
        for item in errors:
            print(f"FAIL: {item}")
        return 2

    print("PASS: topology validation checks passed")
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate topology JSON output quality.")
    parser.add_argument("topology_json_path", help="Path to topology.json")
    parser.add_argument("--allow-self-loops", action="store_true")
    parser.add_argument("--allow-noncontiguous-ids", action="store_true")
    parser.add_argument("--max-isolated-nodes", type=int, default=0)
    parser.add_argument("--min-largest-component-ratio", type=float, default=1.0)
    parser.add_argument("--max-avg-degree", type=float, default=None)
    parser.add_argument("--max-max-degree", type=int, default=None)
    parser.add_argument("--max-p95-weight", type=float, default=None)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    cli_args = _parse_args()
    exit_code = validate_topology(Path(cli_args.topology_json_path), cli_args)
    raise SystemExit(exit_code)