from collections import Counter
from dataclasses import dataclass

import open3d as o3d
import numpy as np
from pathlib import Path

import argparse
import bisect

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

@dataclass
class PoseSample:
    timestamp_ns: int
    parent_frame: str
    child_frame: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class SyncStats:
    total_depth: int = 0
    synced_depth: int = 0
    dropped_depth: int = 0
    decode_errors: int = 0
    points_before_downsample: int = 0
    points_after_downsample: int = 0
    points_clipped_by_world_range: int = 0

def _ros_stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

def _message_stamp_ns(msg, fallback_timestamp_ns: int) -> int:
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return _ros_stamp_to_ns(msg.header.stamp)
    return int(fallback_timestamp_ns)

def _normalize_frame_id(frame_id: str) -> str:
    return frame_id.lstrip("/")

def _to_bytes(data_field) -> bytes:
    if isinstance(data_field, (bytes, bytearray, memoryview)):
        return bytes(data_field)
    return bytes(data_field)


def quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("Quaternion norm is zero.")
    q /= norm
    x, y, z, w = q

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )

def decode_depth_image_to_meters(msg) -> np.ndarray:
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    raw = _to_bytes(msg.data)

    if encoding == "16uc1":
        expected = height * width * 2
        if len(raw) < expected:
            raise ValueError(
                f"Depth payload too small for 16UC1: {len(raw)} < {expected}"
            )
        depth_mm = np.frombuffer(raw, dtype=np.uint16, count=height * width).reshape(
            (height, width)
        )
        return depth_mm.astype(np.float32) * 0.001

    if encoding == "32fc1":
        expected = height * width * 4
        if len(raw) < expected:
            raise ValueError(
                f"Depth payload too small for 32FC1: {len(raw)} < {expected}"
            )
        depth_m = np.frombuffer(raw, dtype=np.float32, count=height * width).reshape(
            (height, width)
        )
        return depth_m

    raise ValueError(
        f"Unsupported depth encoding: {msg.encoding}. Expected 16UC1 or 32FC1."
    )

def extract_camera_intrinsics(reader, camera_info_topic: str) -> CameraIntrinsics:
    connections = [x for x in reader.connections if x.topic == camera_info_topic]
    if not connections:
        raise RuntimeError(f"Missing camera info topic: {camera_info_topic}")

    for connection, _, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        k = msg.k
        return CameraIntrinsics(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )

    raise RuntimeError(f"No messages found on topic: {camera_info_topic}")

def extract_tf_poses(
    reader,
    tf_topic: str,
    pose_child_frame: str,
) -> tuple[list[PoseSample], Counter]:
    connections = [x for x in reader.connections if x.topic == tf_topic]
    if not connections:
        raise RuntimeError(f"Missing TF topic: {tf_topic}")

    selected: list[PoseSample] = []
    child_counts: Counter = Counter()
    wanted_child = _normalize_frame_id(pose_child_frame)

    for connection, fallback_timestamp_ns, rawdata in reader.messages(
        connections=connections
    ):
        msg = reader.deserialize(rawdata, connection.msgtype)
        transforms = msg.transforms if hasattr(msg, "transforms") else [msg]

        for t in transforms:
            child = _normalize_frame_id(t.child_frame_id)
            parent = _normalize_frame_id(t.header.frame_id)
            child_counts[child] += 1

            if child != wanted_child:
                continue

            timestamp_ns = _ros_stamp_to_ns(t.header.stamp)
            if timestamp_ns == 0:
                timestamp_ns = int(fallback_timestamp_ns)

            selected.append(
                PoseSample(
                    timestamp_ns=timestamp_ns,
                    parent_frame=parent,
                    child_frame=child,
                    x=float(t.transform.translation.x),
                    y=float(t.transform.translation.y),
                    z=float(t.transform.translation.z),
                    qx=float(t.transform.rotation.x),
                    qy=float(t.transform.rotation.y),
                    qz=float(t.transform.rotation.z),
                    qw=float(t.transform.rotation.w),
                )
            )

    selected.sort(key=lambda p: p.timestamp_ns)
    return selected, child_counts


def _build_pixel_grid(width: int, height: int, pixel_stride: int) -> tuple[np.ndarray, np.ndarray]:
    u = np.arange(0, width, pixel_stride, dtype=np.float32)
    v = np.arange(0, height, pixel_stride, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    return uu, vv


def undistort_plumb_bob(
    xd: np.ndarray,
    yd: np.ndarray,
    k1: float,
    k2: float,
    p1: float,
    p2: float,
    k3: float,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray]:
    x = xd.copy()
    y = yd.copy()

    for _ in range(iterations):
        r2 = x * x + y * y
        r4 = r2 * r2
        r6 = r4 * r2
        radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6

        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y

        x = (xd - delta_x) / radial
        y = (yd - delta_y) / radial

    return x, y


def backproject_depth_to_camera_points(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    uu: np.ndarray,
    vv: np.ndarray,
    pixel_stride: int,
    min_depth_m: float,
    max_depth_m: float,
    depth_value_mode: str,
    distortion_coeffs: tuple[float, float, float, float, float],
    undistort_iterations: int,
) -> tuple[np.ndarray, int]:
    if depth_m.shape != (intrinsics.height, intrinsics.width):
        raise ValueError(
            f"Depth image shape mismatch: {depth_m.shape} vs expected {(intrinsics.height, intrinsics.width)}"
        )

    sampled_depth = depth_m[::pixel_stride, ::pixel_stride]
    valid_mask = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= min_depth_m)
        & (sampled_depth <= max_depth_m)
    )
    valid_count = int(valid_mask.sum())
    if valid_count == 0:
        return np.empty((0, 3), dtype=np.float32), valid_count

    sampled_u = uu[valid_mask]
    sampled_v = vv[valid_mask]
    depth_values = sampled_depth[valid_mask]

    xd = (sampled_u - intrinsics.cx) / intrinsics.fx
    yd = (sampled_v - intrinsics.cy) / intrinsics.fy

    k1, k2, p1, p2, k3 = distortion_coeffs
    if any(abs(c) > 1e-12 for c in distortion_coeffs):
        dx, dy = undistort_plumb_bob(
            xd=xd,
            yd=yd,
            k1=k1,
            k2=k2,
            p1=p1,
            p2=p2,
            k3=k3,
            iterations=undistort_iterations,
        )
    else:
        dx, dy = xd, yd

    if depth_value_mode == "range":
        # Range mode treats depth as Euclidean distance from camera origin along the viewing ray.
        denom = np.sqrt(dx * dx + dy * dy + 1.0)
        z = depth_values / denom
    else:
        # Z mode treats depth as distance to camera image plane (optical-axis depth).
        z = depth_values

    x = dx * z
    y = dy * z
    points = np.column_stack((x, y, z)).astype(np.float32, copy=False)
    return points, valid_count


def voxel_downsample_points(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(pcd.points, dtype=np.float32)


def merge_pending_batches(
    accumulated_points: np.ndarray,
    pending_batches: list[np.ndarray],
    voxel_size: float,
) -> np.ndarray:
    if not pending_batches:
        return accumulated_points

    if accumulated_points.size == 0:
        merged = np.concatenate(pending_batches, axis=0)
    else:
        merged = np.concatenate([accumulated_points, *pending_batches], axis=0)
    return voxel_downsample_points(merged, voxel_size)

def find_nearest_pose(pose_timestamps_ns: list[int], depth_timestamp_ns: int) -> int:
    idx = bisect.bisect_left(pose_timestamps_ns, depth_timestamp_ns)
    if idx == 0:
        return 0
    if idx >= len(pose_timestamps_ns):
        return len(pose_timestamps_ns) - 1

    prev_idx = idx - 1
    if abs(pose_timestamps_ns[idx] - depth_timestamp_ns) < abs(
        pose_timestamps_ns[prev_idx] - depth_timestamp_ns
    ):
        return idx
    return prev_idx


def synchronize_and_build_point_cloud(
    reader,
    intrinsics: CameraIntrinsics,
    depth_topic: str,
    poses: list[PoseSample],
    max_sync_dt_ms: float,
    max_depth_frames: int,
    print_every: int,
    min_depth_m: float,
    max_depth_m: float,
    pixel_stride: int,
    voxel_size: float,
    downsample_every_frames: int,
    pose_to_camera_txyz: tuple[float, float, float],
    pose_to_camera_qxyzw: tuple[float, float, float, float],
    max_world_distance_m: float,
    depth_value_mode: str,
    distortion_coeffs: tuple[float, float, float, float, float],
    undistort_iterations: int,
) -> tuple[np.ndarray, SyncStats]:
    connections = [x for x in reader.connections if x.topic == depth_topic]
    if not connections:
        raise RuntimeError(f"Missing depth topic: {depth_topic}")

    pose_timestamps_ns = [p.timestamp_ns for p in poses]
    max_sync_dt_ns = int(max_sync_dt_ms * 1_000_000)
    uu, vv = _build_pixel_grid(intrinsics.width, intrinsics.height, pixel_stride)

    t_pose_camera = np.array(pose_to_camera_txyz, dtype=np.float64)
    qx, qy, qz, qw = pose_to_camera_qxyzw
    r_pose_camera = quaternion_to_rotation_matrix(qx, qy, qz, qw)

    accumulated_points = np.empty((0, 3), dtype=np.float32)
    pending_batches: list[np.ndarray] = []

    stats = SyncStats()
    example_rows = []

    for connection, fallback_timestamp_ns, rawdata in reader.messages(
        connections=connections
    ):
        msg = reader.deserialize(rawdata, connection.msgtype)
        depth_timestamp_ns = _message_stamp_ns(msg, fallback_timestamp_ns)
        stats.total_depth += 1

        try:
            depth_m = decode_depth_image_to_meters(msg)
        except ValueError as err:
            stats.decode_errors += 1
            if stats.decode_errors <= 3:
                print(f"[WARN] Depth decode error frame #{stats.total_depth}: {err}")
            continue

        nearest_idx = find_nearest_pose(pose_timestamps_ns, depth_timestamp_ns)
        nearest_pose = poses[nearest_idx]
        dt_ns = abs(nearest_pose.timestamp_ns - depth_timestamp_ns)

        if dt_ns > max_sync_dt_ns:
            stats.dropped_depth += 1
        else:
            stats.synced_depth += 1
            points_camera, valid_points = backproject_depth_to_camera_points(
                depth_m=depth_m,
                intrinsics=intrinsics,
                uu=uu,
                vv=vv,
                pixel_stride=pixel_stride,
                min_depth_m=min_depth_m,
                max_depth_m=max_depth_m,
                depth_value_mode=depth_value_mode,
                distortion_coeffs=distortion_coeffs,
                undistort_iterations=undistort_iterations,
            )

            if points_camera.shape[0] > 0:
                r_world_pose = quaternion_to_rotation_matrix(
                    nearest_pose.qx,
                    nearest_pose.qy,
                    nearest_pose.qz,
                    nearest_pose.qw,
                )
                t_world_pose = np.array(
                    [nearest_pose.x, nearest_pose.y, nearest_pose.z],
                    dtype=np.float64,
                )

                r_world_camera = r_world_pose @ r_pose_camera
                t_world_camera = r_world_pose @ t_pose_camera + t_world_pose

                points_world = (
                    points_camera.astype(np.float64, copy=False) @ r_world_camera.T
                ) + t_world_camera

                if max_world_distance_m > 0.0 and points_world.shape[0] > 0:
                    distances = np.linalg.norm(points_world - t_world_camera, axis=1)
                    keep_mask = distances <= max_world_distance_m
                    clipped = int((~keep_mask).sum())
                    stats.points_clipped_by_world_range += clipped
                    points_world = points_world[keep_mask]

                pending_batches.append(points_world.astype(np.float32, copy=False))
                stats.points_before_downsample += int(points_world.shape[0])

                if len(pending_batches) >= downsample_every_frames:
                    accumulated_points = merge_pending_batches(
                        accumulated_points=accumulated_points,
                        pending_batches=pending_batches,
                        voxel_size=voxel_size,
                    )
                    pending_batches.clear()

            if len(example_rows) < 5:
                example_rows.append(
                    {
                        "depth_ts": depth_timestamp_ns,
                        "pose_ts": nearest_pose.timestamp_ns,
                        "dt_ms": dt_ns / 1_000_000.0,
                        "shape": depth_m.shape,
                        "valid_depth_px": valid_points,
                        "pose_xyz": (nearest_pose.x, nearest_pose.y, nearest_pose.z),
                    }
                )

        if print_every > 0 and stats.total_depth % print_every == 0:
            print(
                f"[INFO] processed depth={stats.total_depth}, synced={stats.synced_depth}, dropped={stats.dropped_depth}, decode_errors={stats.decode_errors}, points_after_downsample={accumulated_points.shape[0]}"
            )

        if max_depth_frames > 0 and stats.total_depth >= max_depth_frames:
            break

    if pending_batches:
        accumulated_points = merge_pending_batches(
            accumulated_points=accumulated_points,
            pending_batches=pending_batches,
            voxel_size=voxel_size,
        )
        pending_batches.clear()

    stats.points_after_downsample = int(accumulated_points.shape[0])

    print("\n=== Synchronization report (stage 1) ===")
    print(f"Depth frames processed: {stats.total_depth}")
    print(f"Depth frames synced:    {stats.synced_depth}")
    print(f"Depth frames dropped:   {stats.dropped_depth}")
    print(f"Depth decode errors:    {stats.decode_errors}")
    if stats.total_depth > 0:
        print(f"Sync ratio:             {100.0 * stats.synced_depth / stats.total_depth:.2f}%")

    if example_rows:
        print("\nExample synchronized frames:")
        for i, row in enumerate(example_rows, start=1):
            print(
                f"  #{i}: depth_ts={row['depth_ts']} pose_ts={row['pose_ts']} dt={row['dt_ms']:.3f}ms shape={row['shape']} valid_depth_px={row['valid_depth_px']} pose_xyz={row['pose_xyz']}"
            )

    print("\n=== Point cloud report (stage 2) ===")
    print(f"Points before downsampling: {stats.points_before_downsample}")
    print(f"Points after downsampling:  {stats.points_after_downsample}")
    print(f"Points clipped by world range: {stats.points_clipped_by_world_range}")
    print(f"Voxel size [m]:             {voxel_size}")

    return accumulated_points, stats


def save_point_cloud(output_file_name: str, points_world: np.ndarray) -> None:
    output_path = Path(output_file_name)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_world.astype(np.float64, copy=False))

    ok = o3d.io.write_point_cloud(str(output_path), cloud)
    if not ok:
        raise RuntimeError(f"Failed to write point cloud to: {output_file_name}")

def recording_to_point_cloud(
    input_file_name: str,
    output_file_name: str,
    depth_topic: str,
    camera_info_topic: str,
    tf_topic: str,
    pose_child_frame: str,
    max_sync_dt_ms: float,
    max_depth_frames: int,
    print_every: int,
    min_depth_m: float,
    max_depth_m: float,
    pixel_stride: int,
    voxel_size: float,
    downsample_every_frames: int,
    pose_to_camera_txyz: tuple[float, float, float],
    pose_to_camera_qxyzw: tuple[float, float, float, float],
    max_world_distance_m: float,
    depth_value_mode: str,
    distortion_coeffs: tuple[float, float, float, float, float],
    undistort_iterations: int,
) -> None:
    bag_path = Path(input_file_name)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        topics = sorted({c.topic for c in reader.connections})
        print(f"Topics in the bag ({len(topics)}): {topics}")

        intrinsics = extract_camera_intrinsics(reader, camera_info_topic)
        print(
            f"Camera intrinsics from {camera_info_topic}: fx={intrinsics.fx:.3f} fy={intrinsics.fy:.3f} cx={intrinsics.cx:.3f} cy={intrinsics.cy:.3f} size={intrinsics.width}x{intrinsics.height}"
        )

        poses, child_counts = extract_tf_poses(reader, tf_topic, pose_child_frame)
        if not poses:
            top_children = child_counts.most_common(10)
            raise RuntimeError(
                f"No TF poses found for child frame '{pose_child_frame}' on {tf_topic}. Available child frames (top 10): {top_children}"
            )

        print(f"Selected TF child frame: {pose_child_frame}")
        print(f"Pose samples loaded: {len(poses)}")
        first_pose = poses[0]
        print(
            f"First pose: t={first_pose.timestamp_ns} parent={first_pose.parent_frame} child={first_pose.child_frame} xyz=({first_pose.x:.3f}, {first_pose.y:.3f}, {first_pose.z:.3f})"
        )

        print(
            "Pose-to-camera extrinsic used for stage 2: "
            f"t={pose_to_camera_txyz}, q={pose_to_camera_qxyzw}"
        )
        print(f"Depth value mode: {depth_value_mode}")
        print(
            "Depth undistortion (plumb_bob D=[k1,k2,p1,p2,k3]): "
            f"{distortion_coeffs}, iterations={undistort_iterations}"
        )

        points_world, stats = synchronize_and_build_point_cloud(
            reader=reader,
            intrinsics=intrinsics,
            depth_topic=depth_topic,
            poses=poses,
            max_sync_dt_ms=max_sync_dt_ms,
            max_depth_frames=max_depth_frames,
            print_every=print_every,
            min_depth_m=min_depth_m,
            max_depth_m=max_depth_m,
            pixel_stride=pixel_stride,
            voxel_size=voxel_size,
            downsample_every_frames=downsample_every_frames,
            pose_to_camera_txyz=pose_to_camera_txyz,
            pose_to_camera_qxyzw=pose_to_camera_qxyzw,
            max_world_distance_m=max_world_distance_m,
            depth_value_mode=depth_value_mode,
            distortion_coeffs=distortion_coeffs,
            undistort_iterations=undistort_iterations,
        )

    save_point_cloud(output_file_name, points_world)
    print(
        f"\nStage 2 complete. Saved point cloud: {output_file_name} ({stats.points_after_downsample} points)"
    )

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read depth+TF from rosbag and generate GT point cloud via back-projection."
    )
    parser.add_argument("input_bag_file", help="Path to input rosbag/mcap file")
    parser.add_argument("output_pcd_file", help="Output point cloud path (.pcd or .ply)")
    parser.add_argument(
        "--depth-topic",
        default="/camera/depth/image_raw",
        help="Depth image topic",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera_info",
        help="CameraInfo topic",
    )
    parser.add_argument(
        "--tf-topic",
        default="/tf",
        help="TF topic used for dynamic poses",
    )
    parser.add_argument(
        "--pose-child-frame",
        default="base_footprint",
        help="Child frame in TF used as pose source (example: base_footprint, base_link)",
    )
    parser.add_argument(
        "--max-sync-dt-ms",
        type=float,
        default=25.0,
        help="Maximum time difference depth-pose for synchronization in milliseconds",
    )
    parser.add_argument(
        "--max-depth-frames",
        type=int,
        default=0,
        help="Optional processing limit for depth frames; 0 means all",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=200,
        help="Progress print frequency in number of depth frames",
    )
    parser.add_argument(
        "--min-depth-m",
        type=float,
        default=0.05,
        help="Minimum valid depth in meters",
    )
    parser.add_argument(
        "--max-depth-m",
        type=float,
        default=10.0,
        help="Maximum valid depth in meters",
    )
    parser.add_argument(
        "--max-world-distance-m",
        type=float,
        default=12.0,
        help="Optional max distance from camera origin for transformed points; <=0 disables",
    )
    parser.add_argument(
        "--depth-value-mode",
        choices=["auto", "z", "range"],
        default="auto",
        help="Interpretation of depth values: z=optical-axis depth, range=ray distance, auto=range for 32FC1 and z for 16UC1",
    )
    parser.add_argument(
        "--distortion-coeffs",
        nargs=5,
        type=float,
        metavar=("K1", "K2", "P1", "P2", "K3"),
        default=[0.0, 0.0, 0.0, 0.0, 0.0],
        help="Optional plumb_bob distortion coefficients D=[k1,k2,p1,p2,k3] for undistorting pixel rays",
    )
    parser.add_argument(
        "--undistort-iterations",
        type=int,
        default=5,
        help="Number of fixed-point iterations for plumb_bob undistortion",
    )
    parser.add_argument(
        "--use-leo-sim-distortion",
        action="store_true",
        help="Use Leo simulation camera distortion from URDF: D=[-0.279817,0.060321,0.000310,0.0,0.000487]",
    )
    parser.add_argument(
        "--pixel-stride",
        type=int,
        default=1,
        help="Depth pixel stride for back-projection (1 keeps full resolution)",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="Voxel size in meters for map downsampling",
    )
    parser.add_argument(
        "--downsample-every-frames",
        type=int,
        default=20,
        help="Run voxel downsampling after this many synchronized frames",
    )
    parser.add_argument(
        "--pose-to-camera-txyz",
        nargs=3,
        type=float,
        metavar=("TX", "TY", "TZ"),
        default=[0.0971, 0.0, 0.15513],
        help="Static translation from pose_child_frame to camera_optical_frame [m]",
    )
    parser.add_argument(
        "--pose-to-camera-qxyzw",
        nargs=4,
        type=float,
        metavar=("QX", "QY", "QZ", "QW"),
        default=[0.54951639, -0.54951639, 0.44500757, -0.44500757],
        help="Static rotation from pose_child_frame to camera_optical_frame as quaternion",
    )
    args = parser.parse_args()

    if args.pixel_stride <= 0:
        raise ValueError("--pixel-stride must be > 0")
    if args.voxel_size < 0.0:
        raise ValueError("--voxel-size must be >= 0")
    if args.min_depth_m < 0.0 or args.max_depth_m <= args.min_depth_m:
        raise ValueError("Depth range must satisfy 0 <= min_depth_m < max_depth_m")
    if args.max_world_distance_m < 0.0:
        raise ValueError("--max-world-distance-m must be >= 0")
    if args.downsample_every_frames <= 0:
        raise ValueError("--downsample-every-frames must be > 0")
    if args.undistort_iterations <= 0:
        raise ValueError("--undistort-iterations must be > 0")

    if args.use_leo_sim_distortion:
        args.distortion_coeffs = [-0.279817, 0.060321, 0.000310, 0.0, 0.000487]

    return args


def resolve_depth_value_mode(depth_value_mode: str) -> str:
    if depth_value_mode == "auto":
        # Simulators commonly publish 32FC1 as ray distance; this default avoids corner stretching artifacts.
        return "range"
    return depth_value_mode

if __name__ == "__main__":
    args = parse_args()

    recording_to_point_cloud(
        input_file_name=args.input_bag_file,
        output_file_name=args.output_pcd_file,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        tf_topic=args.tf_topic,
        pose_child_frame=args.pose_child_frame,
        max_sync_dt_ms=args.max_sync_dt_ms,
        max_depth_frames=args.max_depth_frames,
        print_every=args.print_every,
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
        pixel_stride=args.pixel_stride,
        voxel_size=args.voxel_size,
        downsample_every_frames=args.downsample_every_frames,
        pose_to_camera_txyz=tuple(args.pose_to_camera_txyz),
        pose_to_camera_qxyzw=tuple(args.pose_to_camera_qxyzw),
        max_world_distance_m=args.max_world_distance_m,
        depth_value_mode=resolve_depth_value_mode(args.depth_value_mode),
        distortion_coeffs=tuple(args.distortion_coeffs),
        undistort_iterations=args.undistort_iterations,
    )
