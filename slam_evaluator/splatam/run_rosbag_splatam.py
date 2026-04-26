import argparse
import os
import shutil
import numpy as np
from PIL import Image
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

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
        while depth_idx < len(depth_msgs) - 1 and abs(depth_msgs[depth_idx+1][0] - rgb_ts) < abs(depth_msgs[depth_idx][0] - rgb_ts):
            depth_idx += 1
        
        depth_ts, depth_msg = depth_msgs[depth_idx]
        
        # Check if they are somewhat synchronized (e.g. within 50ms)
        if abs(rgb_ts - depth_ts) < 50_000_000: # nanoseconds
            synced_pairs.append((rgb_msg, depth_msg))

    print(f"Extracted {len(synced_pairs)} synchronized frames.")
    
    dummy_pose = np.eye(4)
    for i, (rgb_msg, depth_msg) in enumerate(synced_pairs):
        # Convert RGB
        rgb_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
        if rgb_arr.shape[2] == 4: # BGRA or RGBA
           rgb_arr = rgb_arr[:, :, :3]
        if rgb_msg.encoding == 'bgr8':
           rgb_arr = rgb_arr[:, :, ::-1] # BGR to RGB
        Image.fromarray(rgb_arr).save(os.path.join(rgb_dir, f"{i:05d}.jpg"))

        # Convert Depth
        if depth_msg.encoding == '16UC1':
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
        elif depth_msg.encoding == '32FC1':
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width).copy()
            depth_arr[np.isnan(depth_arr)] = 0
            depth_arr[np.isinf(depth_arr)] = 0
            depth_arr = (depth_arr * 1000).astype(np.uint16) # Convert meters to millimeters
        else:
            print(f"Warning: unsupported depth encoding {depth_msg.encoding}")
            continue
            
        # SplaTAM expects a scale to convert to meters, 1000 means it expects millimeters in the png.
        Image.fromarray(depth_arr).save(os.path.join(depth_dir, f"{i:05d}.png"))
        
        # SplaTAM realsense dataset expects dummy poses even if tracking.use_gt_poses=False
        np.save(os.path.join(poses_dir, f"{i:05d}.npy"), dummy_pose)

    return len(synced_pairs)

def generate_splatam_config(dataset_dir, output_config_path, num_frames):
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

map_every = 1
keyframe_every = 5
mapping_window_size = 24
tracking_iters = 40
mapping_iters = 60

config = dict(
    workdir=f"./{{base_dir}}/{{scene_name}}_output",
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
    
    # Overriding intrinsic matrix for gradslam dataset loader since we don't have an intrinsics.txt file
    camera_params=dict(
        image_height=full_res_height,
        image_width=full_res_width,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
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
        stride=1,
        num_frames=num_frames,
    ),
    tracking=dict(
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
            cam_unnorm_rots=0.001,
            cam_trans=0.004,
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
    parser = argparse.ArgumentParser(description="Extract Rosbag for SplaTAM and run it.")
    parser.add_argument("bag", help="Path to the rosbag directory")
    parser.add_argument("--color_topic", default="/camera/color/image_raw", help="Color image topic")
    parser.add_argument("--depth_topic", default="/camera/depth/image_raw", help="Depth image topic")
    parser.add_argument("--output_dir", default="/data/splatam_dataset", help="Output directory for the extracted images")
    parser.add_argument("--run", action="store_true", help="Automatically run SplaTAM after extraction")
    
    args = parser.parse_args()
    
    bag_name = os.path.basename(os.path.normpath(args.bag))
    dataset_dir = os.path.join(args.output_dir, bag_name)
    
    print(f"Extracting rosbag {args.bag} to {dataset_dir} ...")
    num_frames = create_splatam_dataset(args.bag, dataset_dir, args.color_topic, args.depth_topic)
    
    if num_frames == 0:
        print("Error: No frames were extracted.")
        return
        
    config_path = os.path.join(dataset_dir, "splatam_config.py")
    generate_splatam_config(dataset_dir, config_path, num_frames)
    
    print("\nDataset extraction complete.")
    
    if args.run:
        print("\nStarting SplaTAM...")
        splatam_script = "/workspace/slam_evaluator/SplaTAM/scripts/splatam.py"
        venv_python = "/workspace/slam_evaluator/SplaTAM/.venv/bin/python3"
        
        if not os.path.exists(venv_python):
            print(f"Error: SplaTAM venv python not found at {venv_python}")
            print("Did you finish installing SplaTAM dependencies? Run `./run-slam-evaluator.sh bootstrap-splatam` then inside the container run:")
            print("python3 -m venv /workspace/slam_evaluator/SplaTAM/.venv")
            print("source /workspace/slam_evaluator/SplaTAM/.venv/bin/activate")
            print("pip install -r /workspace/slam_evaluator/SplaTAM/venv_requirements.txt")
            return
            
        # The realsense dataset loader in gradslam attempts to load intrinsics.txt which we don't have.
        # We need a quick patch to the dataset loader to use our config intrinsics if the file is missing.
        patch_cmd = f"""
cat << 'EOF' > /tmp/patch_realsense.py
import os
import torch
from datasets.gradslam_datasets.realsense import RealsenseDataset

original_get_intrinsics = RealsenseDataset.get_intrinsics

def patched_get_intrinsics(self):
    intrinsic_path = os.path.join(self.input_folder, "intrinsics.txt")
    if not os.path.exists(intrinsic_path) and "camera_params" in self.config_dict:
        # Fallback to config provided intrinsics
        params = self.config_dict["camera_params"]
        intrinsics = torch.eye(3)
        intrinsics[0, 0] = params["fx"]
        intrinsics[1, 1] = params["fy"]
        intrinsics[0, 2] = params["cx"]
        intrinsics[1, 2] = params["cy"]
        return intrinsics.float()
    return original_get_intrinsics(self)

RealsenseDataset.get_intrinsics = patched_get_intrinsics
EOF
"""
        os.system(patch_cmd)
        
        # We run splatam with the python venv, appending our patch at startup
        run_cmd = f"cd /workspace/slam_evaluator/SplaTAM && PYTHONPATH=.:/tmp/patch_realsense.py {venv_python} -c 'import patch_realsense; import runpy; runpy.run_path(\"{splatam_script}\", run_name=\"__main__\")' {config_path}"
        print(f"Executing: {run_cmd}")
        os.system(run_cmd)
        
        export_script = "/workspace/slam_evaluator/SplaTAM/scripts/export_ply.py"
        export_cmd = f"cd /workspace/slam_evaluator/SplaTAM && {venv_python} {export_script} {config_path}"
        print(f"Executing export: {export_cmd}")
        os.system(export_cmd)
        
        print("\nSplaTAM processing finished! 3D Map (.ply) and Trajectory should be in the output folder.")
    else:
        print(f"\nTo run SplaTAM manually, use:")
        print(f"source /workspace/slam_evaluator/SplaTAM/.venv/bin/activate")
        print(f"python /workspace/slam_evaluator/SplaTAM/scripts/splatam.py {config_path}")

if __name__ == "__main__":
    main()
