import argparse
import os
import subprocess
import shutil

import numpy as np
from PIL import Image
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


SPLATAM_PROFILES = {
    "fast": {
        "stride": 5,
        "map_every": 5,
        "keyframe_every": 5,
        "mapping_window_size": 24,
        "tracking_iters": 15,
        "mapping_iters": 20,
        "cam_unnorm_rots_lr": 0.001,
        "cam_trans_lr": 0.004,
    },
    "balanced": {
        "stride": 3,
        "map_every": 3,
        "keyframe_every": 3,
        "mapping_window_size": 24,
        "tracking_iters": 25,
        "mapping_iters": 25,
        "cam_unnorm_rots_lr": 0.001,
        "cam_trans_lr": 0.004,
    },
    "robust": {
        "stride": 2,
        "map_every": 2,
        "keyframe_every": 2,
        "mapping_window_size": 32,
        "tracking_iters": 40,
        "mapping_iters": 40,
        "cam_unnorm_rots_lr": 0.0008,
        "cam_trans_lr": 0.003,
    },
}


def create_splatam_dataset(bag_path, output_dir, color_topic, depth_topic):
    """Extracts synchronized color and depth images to a realsense-like dataset for SplaTAM."""
    rgb_dir = os.path.join(output_dir, "rgb")
    depth_dir = os.path.join(output_dir, "depth")
    poses_dir = os.path.join(output_dir, "poses")

    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(poses_dir, exist_ok=True)

    typestore = get_typestore(Stores.ROS2_FOXY)

    rgb_msgs = []
    depth_msgs = []

    print(f"Reading bag: {bag_path}")
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == color_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                rgb_msgs.append((timestamp, msg))
            elif connection.topic == depth_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                depth_msgs.append((timestamp, msg))

    print(f"Found {len(rgb_msgs)} RGB messages and {len(depth_msgs)} depth messages.")

    # Simple synchronization based on nearest timestamp
    synced_pairs = []
    if len(rgb_msgs) == 0 or len(depth_msgs) == 0:
        print("No images found for the given topics.")
        return 0

    depth_idx = 0
    for rgb_ts, rgb_msg in rgb_msgs:
        while depth_idx < len(depth_msgs) - 1 and abs(
            depth_msgs[depth_idx + 1][0] - rgb_ts
        ) < abs(depth_msgs[depth_idx][0] - rgb_ts):
            depth_idx += 1

        depth_ts, depth_msg = depth_msgs[depth_idx]

        # Check if they are somewhat synchronized (e.g. within 50ms)
        if abs(rgb_ts - depth_ts) < 50_000_000:  # nanoseconds
            synced_pairs.append((rgb_msg, depth_msg))

    print(f"Extracted {len(synced_pairs)} synchronized frames.")

    dummy_pose = np.eye(4)
    for i, (rgb_msg, depth_msg) in enumerate(synced_pairs):
        # Convert RGB
        rgb_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(
            rgb_msg.height, rgb_msg.width, -1
        )
        if rgb_arr.shape[2] == 4:  # BGRA or RGBA
            rgb_arr = rgb_arr[:, :, :3]
        if rgb_msg.encoding == "bgr8":
            rgb_arr = rgb_arr[:, :, ::-1]  # BGR to RGB
        Image.fromarray(rgb_arr).save(os.path.join(rgb_dir, f"{i:05d}.png"))

        # Convert Depth
        if depth_msg.encoding == "16UC1":
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width
            )
        elif depth_msg.encoding == "32FC1":
            depth_arr = (
                np.frombuffer(depth_msg.data, dtype=np.float32)
                .reshape(depth_msg.height, depth_msg.width)
                .copy()
            )
            depth_arr[np.isnan(depth_arr)] = 0
            depth_arr[np.isinf(depth_arr)] = 0
            depth_arr = (depth_arr * 1000).astype(
                np.uint16
            )  # Convert meters to millimeters
        else:
            print(f"Warning: unsupported depth encoding {depth_msg.encoding}")
            continue

        # SplaTAM expects a scale to convert to meters, 1000 means it expects millimeters in the png.
        Image.fromarray(depth_arr).save(os.path.join(depth_dir, f"{i:05d}.png"))

        # SplaTAM realsense dataset expects dummy poses even if tracking.use_gt_poses=False
        np.save(os.path.join(poses_dir, f"{i:05d}.npy"), dummy_pose)

    return len(synced_pairs)


def generate_splatam_config(dataset_dir, output_config_path, num_frames, slam_params):
    """Generates a config file pointing to the extracted dataset."""
    config_content = f"""
import os
from os.path import join as p_join

primary_device = "cuda:0"
seed = 0

base_dir = "{os.path.dirname(dataset_dir)}"
scene_name = "{os.path.basename(dataset_dir)}"
num_frames = {num_frames}
depth_scale = 1000.0 # Depth scale to convert PNG values to meters
overwrite = False

# Assuming camera parameters from your isaac_camera.yaml
full_res_width = 640
full_res_height = 480
downscale_factor = 1.0
densify_downscale_factor = 1.0

# Calculate camera intrinsics for SplaTAM
fx = 480.0
fy = 480.0
cx = 320.0
cy = 240.0

map_every = {slam_params["map_every"]}
keyframe_every = {slam_params["keyframe_every"]}
mapping_window_size = {slam_params["mapping_window_size"]}
tracking_iters = {slam_params["tracking_iters"]}
mapping_iters = {slam_params["mapping_iters"]}

tracking_cam_rot_lr = {slam_params["cam_unnorm_rots_lr"]}
tracking_cam_trans_lr = {slam_params["cam_trans_lr"]}

config = dict(
    workdir=p_join(base_dir, f"{{scene_name}}_output"),
    run_name="SplaTAM_Rosbag",
    overwrite=overwrite,
    depth_scale=depth_scale,
    num_frames=num_frames,
    seed=seed,
    primary_device=primary_device,
    map_every=map_every,
    keyframe_every=keyframe_every,
    mapping_window_size=mapping_window_size,
    report_global_progress_every=100,
    eval_every=1,
    scene_radius_depth_ratio=3,
    mean_sq_dist_method="projective",
    gaussian_distribution="isotropic",
    report_iter_progress=True,
    load_checkpoint=False,
    checkpoint_time_idx=0,
    save_checkpoints=False,
    checkpoint_interval=100,
    use_wandb=False,

    # Camera params must be at the root of the config dict for GradSLAMDataset
    camera_params=dict(
        image_height=full_res_height,
        image_width=full_res_width,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        png_depth_scale=depth_scale,
    ),

    data=dict(
        dataset_name="realsense",
        basedir=base_dir,
        sequence=scene_name,
        desired_image_height=int(full_res_height//downscale_factor),
        desired_image_width=int(full_res_width//downscale_factor),
        densification_image_height=int(full_res_height//densify_downscale_factor),
        densification_image_width=int(full_res_width//densify_downscale_factor),
        start=0,
        end=-1,
        stride={slam_params["stride"]},
        # Let SplaTAM infer frame count after applying start/end/stride.
        num_frames=-1,
    ),    tracking=dict(
        use_gt_poses=False,
        forward_prop=True,
        visualize_tracking_loss=False,
        num_iters=tracking_iters,
        use_sil_for_loss=True,
        sil_thres=0.99,
        use_l1=True,
        use_depth_loss_thres=True,
        depth_loss_thres=20000,
        ignore_outlier_depth_loss=False,
        use_uncertainty_for_loss_mask=False,
        use_uncertainty_for_loss=False,
        use_chamfer=False,
        loss_weights=dict(
            im=0.5,
            depth=1.0,
        ),
        lrs=dict(
            means3D=0.0,
            rgb_colors=0.0,
            unnorm_rotations=0.0,
            logit_opacities=0.0,
            log_scales=0.0,
            cam_unnorm_rots=tracking_cam_rot_lr,
            cam_trans=tracking_cam_trans_lr,
        ),
    ),
    mapping=dict(
        num_iters=mapping_iters,
        add_new_gaussians=True,
        sil_thres=0.5,
        use_l1=True,
        ignore_outlier_depth_loss=False,
        use_sil_for_loss=False,
        use_uncertainty_for_loss_mask=False,
        use_uncertainty_for_loss=False,
        use_chamfer=False,
        loss_weights=dict(
            im=0.5,
            depth=1.0,
        ),
        lrs=dict(
            means3D=0.0001,
            rgb_colors=0.0025,
            unnorm_rotations=0.001,
            logit_opacities=0.05,
            log_scales=0.001,
            cam_unnorm_rots=0.0000,
            cam_trans=0.0000,
        ),
        prune_gaussians=True,
        pruning_dict=dict(
            start_after=0,
            remove_big_after=0,
            stop_after=20,
            prune_every=20,
            removal_opacity_threshold=0.005,
            final_removal_opacity_threshold=0.005,
            reset_opacities=False,
            reset_opacities_every=500,
        ),
        use_gaussian_splatting_densification=False,
        densify_dict=dict(
            start_after=500,
            remove_big_after=3000,
            stop_after=5000,
            densify_every=100,
            grad_thresh=0.0002,
            num_to_split_into=2,
            removal_opacity_threshold=0.005,
            final_removal_opacity_threshold=0.005,
            reset_opacities_every=3000,
        ),
    ),
    viz=dict(
        render_mode='color',
        offset_first_viz_cam=True,
        show_sil=False,
        visualize_cams=True,
        viz_w=600, viz_h=340,
        viz_near=0.01, viz_far=100.0,
        view_scale=2,
        viz_fps=5,
        enter_interactive_post_online=False,
    ),
)
"""
    with open(output_config_path, "w") as f:
        f.write(config_content)
    print(f"Config generated at: {output_config_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Extract Rosbag for SplaTAM and run it."
    )
    parser.add_argument("bag", help="Path to the rosbag directory")
    parser.add_argument(
        "--color_topic", default="/camera/color/image_raw", help="Color image topic"
    )
    parser.add_argument(
        "--depth_topic", default="/camera/depth/image_raw", help="Depth image topic"
    )
    parser.add_argument(
        "--output_dir",
        default="/data/splatam_dataset",
        help="Output directory for the extracted images",
    )
    parser.add_argument(
        "--run", action="store_true", help="Automatically run SplaTAM after extraction"
    )
    parser.add_argument(
        "--delete",
        action="store_true",
        help="Delete the previous extracted dataset and SplaTAM output for this bag before rerunning.",
    )
    parser.add_argument(
        "--profile",
        choices=["fast", "balanced", "robust"],
        default="balanced",
        help="SplaTAM tuning preset (balanced is safer for low-texture scenes).",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=None,
        help="Override frame stride in generated config.",
    )
    parser.add_argument(
        "--map_every",
        type=int,
        default=None,
        help="Override mapping frequency (map every N frames).",
    )
    parser.add_argument(
        "--keyframe_every",
        type=int,
        default=None,
        help="Override keyframe insertion frequency (every N frames).",
    )
    parser.add_argument(
        "--tracking_iters",
        type=int,
        default=None,
        help="Override tracking iterations per frame.",
    )
    parser.add_argument(
        "--mapping_iters",
        type=int,
        default=None,
        help="Override mapping iterations per mapping step.",
    )
    parser.add_argument(
        "--mapping_window_size",
        type=int,
        default=None,
        help="Override mapping window size.",
    )
    parser.add_argument(
        "--cam_rot_lr",
        type=float,
        default=None,
        help="Override tracking camera rotation learning rate.",
    )
    parser.add_argument(
        "--cam_trans_lr",
        type=float,
        default=None,
        help="Override tracking camera translation learning rate.",
    )

    args = parser.parse_args()

    slam_params = dict(SPLATAM_PROFILES[args.profile])
    overrides = {
        "stride": args.stride,
        "map_every": args.map_every,
        "keyframe_every": args.keyframe_every,
        "tracking_iters": args.tracking_iters,
        "mapping_iters": args.mapping_iters,
        "mapping_window_size": args.mapping_window_size,
        "cam_unnorm_rots_lr": args.cam_rot_lr,
        "cam_trans_lr": args.cam_trans_lr,
    }
    for key, value in overrides.items():
        if value is not None:
            slam_params[key] = value

    int_keys = [
        "stride",
        "map_every",
        "keyframe_every",
        "tracking_iters",
        "mapping_iters",
        "mapping_window_size",
    ]
    float_keys = ["cam_unnorm_rots_lr", "cam_trans_lr"]
    for key in int_keys:
        if slam_params[key] <= 0:
            parser.error(f"{key} must be > 0, got {slam_params[key]}.")
    for key in float_keys:
        if slam_params[key] <= 0:
            parser.error(f"{key} must be > 0, got {slam_params[key]}.")

    bag_name = os.path.basename(os.path.normpath(args.bag))
    dataset_dir = os.path.join(args.output_dir, bag_name)
    output_run_dir = os.path.join(
        os.path.dirname(dataset_dir),
        f"{os.path.basename(dataset_dir)}_output",
        "SplaTAM_Rosbag",
    )

    if args.delete:
        for path in [dataset_dir, output_run_dir]:
            if os.path.exists(path):
                print(f"Deleting previous output: {path}")
                shutil.rmtree(path)

    print(f"Extracting rosbag {args.bag} to {dataset_dir} ...")
    num_frames = create_splatam_dataset(
        args.bag, dataset_dir, args.color_topic, args.depth_topic
    )

    if num_frames == 0:
        print("Error: No frames were extracted.")
        return

    print(
        "Using SplaTAM settings: "
        f"profile={args.profile}, stride={slam_params['stride']}, "
        f"map_every={slam_params['map_every']}, keyframe_every={slam_params['keyframe_every']}, "
        f"tracking_iters={slam_params['tracking_iters']}, mapping_iters={slam_params['mapping_iters']}, "
        f"mapping_window_size={slam_params['mapping_window_size']}"
    )

    config_path = os.path.join(dataset_dir, "splatam_config.py")
    generate_splatam_config(dataset_dir, config_path, num_frames, slam_params)
    expected_ply_path = os.path.join(output_run_dir, "splat.ply")

    print("\nDataset extraction complete.")

    if args.run:
        print("\nStarting SplaTAM...")
        splatam_dir = "/opt/SplaTAM"
        splatam_script = os.path.join(splatam_dir, "scripts/splatam.py")
        venv_python = "/opt/venv/bin/python3"

        if not os.path.exists(venv_python):
            # Fallback for old local mounting setup
            splatam_dir = "/workspace/slam_evaluator/SplaTAM"
            splatam_script = os.path.join(splatam_dir, "scripts/splatam.py")
            venv_python = os.path.join(splatam_dir, ".venv/bin/python3")

        if not os.path.exists(venv_python):
            print(f"Error: SplaTAM venv python not found at {venv_python}")
            print(
                "Please ensure you have built the SplaTAM image with the updated Dockerfile."
            )
            return

        # SplaTAM's gradslam dataset loader expects camera_params.
        # Since it dynamically reads from a yaml file we don't have, we inject it before running.
        patch_cmd = f"""
cat << 'EOF' > /tmp/patch_realsense.py
from datasets.gradslam_datasets.realsense import RealsenseDataset

original_init = RealsenseDataset.__init__

def patched_init(self, config_dict, basedir, sequence, **kwargs):
    if "camera_params" not in config_dict:
        config_dict["camera_params"] = dict(
            image_height=480,
            image_width=640,
            fx=480.0,
            fy=480.0,
            cx=320.0,
            cy=240.0,
            png_depth_scale=1000.0,
        )
    original_init(self, config_dict, basedir, sequence, **kwargs)

RealsenseDataset.__init__ = patched_init

import glob
import os
from natsort import natsorted

def patched_get_filepaths(self):
    color_paths = natsorted(glob.glob(os.path.join(self.input_folder, "rgb", "*.png")))
    if not color_paths:
        color_paths = natsorted(glob.glob(os.path.join(self.input_folder, "rgb", "*.jpg")))
    depth_paths = natsorted(glob.glob(os.path.join(self.input_folder, "depth", "*.png")))
    embedding_paths = None
    if getattr(self, "load_embeddings", False):
        embedding_paths = natsorted(glob.glob(os.path.join(self.input_folder, self.embedding_dir, "*.pt")))
    return color_paths, depth_paths, embedding_paths

RealsenseDataset.get_filepaths = patched_get_filepaths
EOF
"""
        patch_result = subprocess.run(patch_cmd, shell=True)
        if patch_result.returncode != 0:
            print(
                f"Error: failed to prepare Realsense patch (exit {patch_result.returncode})."
            )
            return

        # We run splatam with the python venv, appending our patch at startup
        run_cmd = f'cd {splatam_dir} && PYTHONPATH=.:/tmp {venv_python} -c \'import patch_realsense; import runpy; runpy.run_path("{splatam_script}", run_name="__main__")\' {config_path}'
        print(f"Executing: {run_cmd}")
        run_result = subprocess.run(run_cmd, shell=True)
        if run_result.returncode != 0:
            print(
                f"Error: SplaTAM failed with exit code {run_result.returncode}. Skipping export."
            )
            return

        export_script = os.path.join(splatam_dir, "scripts/export_ply.py")
        export_cmd = f"cd {splatam_dir} && {venv_python} {export_script} {config_path}"
        print(f"Executing export: {export_cmd}")
        export_result = subprocess.run(export_cmd, shell=True)
        if export_result.returncode != 0:
            print(f"Error: export_ply failed with exit code {export_result.returncode}.")
            return

        print(
            "\nSplaTAM processing finished! 3D Map (.ply) and Trajectory should be in the output folder."
        )
        print(f"Expected PLY path: {expected_ply_path}")
    else:
        print(f"\nTo run SplaTAM manually, use:")
        print(f"python /opt/SplaTAM/scripts/splatam.py {config_path}")
        print(f"Expected output run directory: {output_run_dir}")


if __name__ == "__main__":
    main()
