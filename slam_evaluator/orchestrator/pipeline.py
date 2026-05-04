import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

SLAM_RUNTIME_CONTAINER_NAME = "slam-runtime-container"
SLAM_EVALUATOR_CONTAINER_NAME = "slam-evaluator-container"


def check_disk_space():
    total, used, free = shutil.disk_usage("/")
    free_gb = free // (2**30)
    if free_gb < 20:
        raise RuntimeError(
            f"Critically low disk space: {free_gb} GB! Stopping the batch."
        )


# DOCKER MANAGEMENT


def setup_environment_vars():
    """Sets up environment variables necessary for docker-compose"""
    env = os.environ.copy()
    env["HOST_UID"] = str(os.getuid())
    env["HOST_GID"] = str(os.getgid())

    # make sure required directories are present
    data_dir = env.get("HOST_DATA_DIR", os.path.abspath("./data"))
    directories = [
        f"{data_dir}/rosbags",
        f"{data_dir}/reports",
        f"{data_dir}/rtabmap",
        f"{data_dir}/slam_ws",
        f"{data_dir}/isaac-sim-cache/ov-data",
        f"{data_dir}/isaac-sim-cache/ov-cache",
        f"{data_dir}/isaac-sim-cache/warp",
        f"{data_dir}/isaac-sim-cache/nvidia-omniverse",
    ]
    for d in directories:
        os.makedirs(d, exist_ok=True)

    return env


def start_dockers(env):
    print("Starting the dockers for benchmarking...")
    # xhost for GUI
    subprocess.run("xhost +local:root > /dev/null 2>&1", shell=True)

    # -d runs it in the background
    subprocess.run(["docker", "compose", "up", "-d"], env=env, check=True)

    time.sleep(5)


def stop_dockers(env):
    print("Stopping the dockers...")
    subprocess.run(["docker", "compose", "down"], env=env, check=True)


def run_docker_command(container_name: str, command: str, env: dict, description: str):
    print(f"{description} in {container_name}...")
    cmd = ["docker", "exec", container_name, "bash", "-c", command]
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)

    if result.returncode != 0:
        print(f"Error during {description}. STDERR:\n{result.stderr}")
    else:
        print(f"{description} complete.")


def generate_ground_truth(rosbag_name: str, env: dict):
    """Generates ground truth representations - map, trajectory, topology"""
    report_dir = f"/data/reports/{rosbag_name}/gt"

    print("Generating ground truth...")

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"mkdir -p {report_dir}",
        env,
        "Creating ground truth report directory",
    )

    if os.path.exists("data/working_pcd_files"):
        shutil.rmtree("data/working_pcd_files")

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"source /opt/ros/jazzy/setup.bash && ros2 launch /slam_evaluator/launch_scripts/gt_pcd.py bag_path:=/home/ubuntu/shared/rosbags/{rosbag_name}",
        env,
        "Generating ground truth PCD files",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/merge_pcds.py --source_folder /data/working_pcd_files --output_file {report_dir}/point_cloud_gt.ply",
        env,
        "Merging PCD files into ground truth point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/voxelize_pcd.py --input_file {report_dir}/point_cloud_gt.ply --output_file {report_dir}/point_cloud_voxelized_gt.ply --voxel_size 0.05",
        env,
        "Voxelizing ground truth point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 extract/extract_trajectory_from_rosbag.py --bag_path /data/rosbags/{rosbag_name} --output_csv {report_dir}/traj_gt.csv",
        env,
        "Extracting ground truth trajectory",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/clip_pcd.py {report_dir}/point_cloud_voxelized_gt.ply {report_dir}/point_cloud_vox_clip.ply --trajectory {report_dir}/traj_gt.csv --tube_radius 5.0",
        env,
        "Clipping ground truth point cloud around trajectory",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 extract/extract_graph_from_gt.py {report_dir}/traj_gt.csv {report_dir}/point_cloud_vox_clip.ply {report_dir}/graph_gt.json --max_edge_distance 1.0 --collision_check_step 0.1 --obstacle_margin 0.1 --min_node_distance 0.1",
        env,
        "Extracting ground truth graph",
    )


def run_rtab_map(rosbag_name: str, env: dict):
    report_dir = f"/data/reports/{rosbag_name}/rtab"
    gt_dir = f"/data/reports/{rosbag_name}/gt"

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"mkdir -p {report_dir} && rm -f /root/.ros/rtabmap.db",
        env,
        "Cleaning old RTAB-Map DB and preparing directories",
    )

    print("Starting RTAB-Map node in background...")
    rtab_launch_cmd = (
        "source /opt/ros/jazzy/setup.bash && ros2 launch rtabmap_launch rtabmap.launch.py "
        'rtabmap_args:="--delete_db_on_start --RGBD/LinearUpdate 0.1 --RGBD/AngularUpdate 0.1 --Grid/FromDepth true --Grid/3D true --Grid/RangeMax 8.0 --Grid/CellSize 0.05" '
        "frame_id:=base_link map_frame_id:=root rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info approx_sync:=true use_sim_time:=true rtabmap_viz:=false"
    )

    rtab_process = subprocess.Popen(
        ["docker", "exec", SLAM_RUNTIME_CONTAINER_NAME, "bash", "-c", rtab_launch_cmd],
        env=env,
        stdout=subprocess.DEVNULL,  # ignore console logs to not flood the output
        stderr=subprocess.DEVNULL,
    )

    time.sleep(
        5
    )  # wait a bit for RTAB-Map to initialize and start subscribing to topics

    # play the rosbag, blocking until it's done (rosbag play will exit when done, so we wait for it)
    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"source /opt/ros/jazzy/setup.bash && ros2 bag play /data/rosbags/{rosbag_name} --clock",
        env,
        "Playing rosbag",
    )

    # SIGINT to RTAB-Map to trigger database save and proper shutdown
    print("Sending SIGINT to RTAB-Map to save database...")
    subprocess.run(
        ["docker", "exec", SLAM_RUNTIME_CONTAINER_NAME, "pkill", "-SIGINT", "rtabmap"],
        env=env,
    )

    rtab_process.wait()
    print("RTAB-Map stopped.")

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"rtabmap-export --poses --output_dir {report_dir} /root/.ros/rtabmap.db",
        env,
        "Extracting RTAB-Map trajectory",
    )

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"rtabmap-export --cloud --cloud_max_depth 8.0 --cloud_min_depth 0.3 --cloud_voxel 0.05 --cloud_decimation 1 --output_dir {report_dir} /root/.ros/rtabmap.db",
        env,
        "Extracting RTAB-Map point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/align_pcd_and_trajectory.py --gt_trajectory {gt_dir}/traj_gt.csv --slam_poses {report_dir}/poses.txt --slam_cloud {report_dir}/cloud.ply --out_poses {report_dir}/poses_aligned.txt --out_cloud {report_dir}/cloud_aligned.ply",
        env,
        "Aligning RTAB-Map output to ground truth",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/voxelize_pcd.py --input_file {report_dir}/cloud_aligned.ply --output_file {report_dir}/cloud_voxelized.ply --voxel_size 0.05",
        env,
        "Voxelizing RTAB-Map point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/clip_pcd.py {report_dir}/cloud_voxelized.ply {report_dir}/cloud_vox_clip.ply --trajectory {gt_dir}/traj_gt.csv --tube_radius 5.0",
        env,
        "Clipping RTAB-Map point cloud around ground truth trajectory",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 extract/extract_graph_from_rtab_db.py --db_path /data/rtabmap/rtabmap.db --output_path {report_dir}/graph_rtab.json",
        env,
        "Extracting RTAB-Map graph",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/align_graph.py {report_dir}/graph_rtab.json {report_dir}/poses_aligned.txt {report_dir}/graph_rtab_aligned.json",
        env,
        "Aligning RTAB-Map graph to ground truth",
    )


def run_orb_slam(rosbag_name: str, env: dict):
    report_dir = f"/data/reports/{rosbag_name}/orb_slam"
    gt_dir = f"/data/reports/{rosbag_name}/gt"

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"mkdir -p {report_dir} && rm -f {report_dir}/FrameTrajectoryTUM.txt {report_dir}/orbslam3_final_map.pcd {report_dir}/covisibility_graph.txt",
        env,
        "Cleaning old ORB-SLAM3 data and preparing directory",
    )

    print("Starting ORB-SLAM3 node in background...")
    # Note: we cd to report_dir so the output files are saved there.
    orb_launch_cmd = (
        "source /opt/ros/jazzy/setup.bash && source /opt/slam_ws/install/setup.bash && "
        f"cd {report_dir} && "
        "ros2 run orbslam3 rgbd /opt/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /opt/slam_ws/isaac_camera.yaml "
        "--ros-args "
        "-r /camera/rgb:=/camera/color/image_raw "
        "-r /camera/depth:=/camera/depth/image_raw "
        "-r /camera/camera_info:=/camera/camera_info"
    )

    orb_process = subprocess.Popen(
        ["docker", "exec", SLAM_RUNTIME_CONTAINER_NAME, "bash", "-c", orb_launch_cmd],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    time.sleep(15)  # Give ORB-SLAM3 time to load vocabulary and initialize

    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        f"source /opt/ros/jazzy/setup.bash && ros2 bag play /data/rosbags/{rosbag_name} --clock",
        env,
        "Playing rosbag",
    )

    print("Sending SIGINT to ORB-SLAM3 to save data...")
    subprocess.run(
        [
            "docker",
            "exec",
            SLAM_RUNTIME_CONTAINER_NAME,
            "pkill",
            "-SIGINT",
            "-f",
            "orbslam3",
        ],
        env=env,
    )

    orb_process.wait()
    print("ORB-SLAM3 stopped.")

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/align_pcd_and_trajectory.py "
        f"--gt_trajectory {gt_dir}/traj_gt.csv "
        f"--slam_poses {report_dir}/FrameTrajectoryTUM.txt "
        f"--slam_cloud {report_dir}/orbslam3_final_map.pcd "
        f"--out_poses {report_dir}/poses_aligned.txt "
        f"--out_cloud {report_dir}/cloud_aligned.ply",
        env,
        "Aligning ORB-SLAM3 output to ground truth",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/voxelize_pcd.py --input_file {report_dir}/cloud_aligned.ply --output_file {report_dir}/cloud_voxelized.ply --voxel_size 0.05",
        env,
        "Voxelizing ORB-SLAM3 point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/clip_pcd.py {report_dir}/cloud_voxelized.ply {report_dir}/cloud_vox_clip.ply --trajectory {gt_dir}/traj_gt.csv --tube_radius 5.0",
        env,
        "Clipping ORB-SLAM3 point cloud around ground truth trajectory",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 extract/extract_graph_from_orbslam.py --input_txt {report_dir}/covisibility_graph.txt --output_json {report_dir}/graph_orbslam.json",
        env,
        "Extracting ORB-SLAM3 graph",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/align_graph.py {report_dir}/graph_orbslam.json {report_dir}/poses_aligned.txt {report_dir}/graph_orbslam_aligned.json",
        env,
        "Aligning ORB-SLAM3 graph to ground truth",
    )


def run_splatam(rosbag_name: str, env: dict):
    report_dir = f"/data/reports/{rosbag_name}/splatam"
    gt_dir = f"/data/reports/{rosbag_name}/gt"

    # SplaTAM script handles dataset extraction and mapping
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"mkdir -p {report_dir} && "
        f"python3 splatam/run_rosbag_splatam.py --bag_path /data/rosbags/{rosbag_name} --output_dir /data/splatam_dataset --run --delete --profile balanced",
        env,
        "Running SplaTAM extraction and mapping",
    )

    splatam_output_dir = f"/data/splatam_dataset/{rosbag_name}_output/SplaTAM_Rosbag"

    # Export trajectory from SplaTAM's internal params.npz
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 splatam/export_traj_from_splatam.py --bag_path /data/rosbags/{rosbag_name} --npz_path {splatam_output_dir}/params.npz --output_csv {report_dir}/traj_splatam.csv --stride 3",
        env,
        "Exporting SplaTAM trajectory",
    )

    # Align SplaTAM output to ground truth
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/align_pcd_and_trajectory.py "
        f"--gt_trajectory {gt_dir}/traj_gt.csv "
        f"--slam_poses {report_dir}/traj_splatam.csv "
        f"--slam_cloud {splatam_output_dir}/splat.ply "
        f"--out_poses {report_dir}/poses_aligned.txt "
        f"--out_cloud {report_dir}/cloud_aligned.ply",
        env,
        "Aligning SplaTAM output to ground truth",
    )

    # Voxelize and clip point cloud
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/voxelize_pcd.py --input_file {report_dir}/cloud_aligned.ply --output_file {report_dir}/cloud_voxelized.ply --voxel_size 0.05",
        env,
        "Voxelizing SplaTAM point cloud",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/clip_pcd.py {report_dir}/cloud_voxelized.ply {report_dir}/cloud_vox_clip.ply --trajectory {gt_dir}/traj_gt.csv --tube_radius 5.0",
        env,
        "Clipping SplaTAM point cloud",
    )

    # For graph extraction, we use the generic extractor on the aligned trajectory and point cloud
    # We first need to convert the space-separated aligned poses to a compatible CSV format
    convert_cmd = (
        f"echo 't,x,y,z,qx,qy,qz,qw,id' > {report_dir}/poses_aligned.csv && "
        f"tail -n +2 {report_dir}/poses_aligned.txt | tr ' ' ',' >> {report_dir}/poses_aligned.csv"
    )
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        convert_cmd,
        env,
        "Preparing aligned poses CSV for graph extraction",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 extract/extract_graph_from_gt.py {report_dir}/poses_aligned.csv {report_dir}/cloud_vox_clip.ply {report_dir}/graph_splatam.json --max_edge_distance 1.0 --collision_check_step 0.1 --obstacle_margin 0.1 --min_node_distance 0.1",
        env,
        "Extracting SplaTAM graph (topology)",
    )


def evaluate_metrics(rosbag_name: str, env: dict):
    gt_dir = f"/data/reports/{rosbag_name}/gt"
    rtab_dir = f"/data/reports/{rosbag_name}/rtab"
    orb_dir = f"/data/reports/{rosbag_name}/orb_slam"
    splatam_dir = f"/data/reports/{rosbag_name}/splatam"

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 metrics/traj.py --gt_trajectory {gt_dir}/traj_gt.csv --est_trajectories {rtab_dir}/poses_aligned.txt {orb_dir}/poses_aligned.txt {splatam_dir}/poses_aligned.txt --output_csv /data/reports/{rosbag_name}/traj_metrics.csv",
        env,
        "Measuring trajectory metrics (ATE, RPE)",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 metrics/point_clouds.py --gt_ply {gt_dir}/point_cloud_voxelized_gt.ply --slam_ply {rtab_dir}/cloud_vox_clip.ply {orb_dir}/cloud_vox_clip.ply {splatam_dir}/cloud_vox_clip.ply --output_csv /data/reports/{rosbag_name}/point_cloud_metrics.csv --tau 0.05 --voxel_size 0.05",
        env,
        "Measuring point cloud metrics (Chamfer Distance, F-score, Accuracy/Completeness, AWD/SCS, Hausdorff Distance, AME)",
    )

    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 metrics/graph_metrics.py --gt_graph {gt_dir}/graph_gt.json --slam_graphs {rtab_dir}/graph_rtab_aligned.json {orb_dir}/graph_orbslam_aligned.json {splatam_dir}/graph_splatam.json --gt_pcd {gt_dir}/point_cloud_voxelized_gt.ply --output_csv /data/reports/{rosbag_name}/graph_metrics.csv",
        env,
        "Measuring graph metrics (Betti numbers error, connectivity error, d_opt error, skeleton length error, PGTI)",
    )


ROSBAG_WORLD_MAPPING = {
    "map_1": "1-warehouse/magazyn.usd",
}


def get_world_for_bag(rosbag_name: str) -> str:
    """Maps rosbag prefix to a USD world file"""
    for prefix, world in ROSBAG_WORLD_MAPPING.items():
        if rosbag_name.startswith(prefix):
            return world
    return "1-warehouse/magazyn.usd"  # Default fallback


def run_nav2_evaluation(rosbag_name: str, slam_type: str, env: dict):
    """Runs a live Nav2 navigation test on the SLAM-generated map using Isaac Sim"""
    report_dir = f"/data/reports/{rosbag_name}/{slam_type}"
    map_yaml = f"{report_dir}/map_2d.yaml"
    metrics_csv = f"{report_dir}/nav2_metrics.csv"
    world_usd = get_world_for_bag(rosbag_name)

    # 1. Convert PCD to 2D Grid
    pcd_file = f"{report_dir}/cloud_vox_clip.ply"
    run_docker_command(
        SLAM_EVALUATOR_CONTAINER_NAME,
        f"python3 process/pcd_to_grid.py --input {pcd_file} --output_prefix {report_dir}/map_2d",
        env,
        f"Converting {slam_type} PCD to 2D grid",
    )

    # 2. Start Headless Isaac Sim in simulation container
    isaac_cmd = (
        f"/opt/isaacsim/_build/linux-x86_64/release/isaac-sim.sh "
        f"/workspace/slam_evaluator/orchestrator/isaac_headless.py --usd_path {world_usd}"
    )
    print(f"Starting headless Isaac Sim for {slam_type} evaluation...")
    isaac_process = subprocess.Popen(
        ["docker", "exec", "ros-jazzy-container", "bash", "-c", isaac_cmd],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Give Isaac Sim time to initialize
    time.sleep(30)

    # 3. Start Nav2 Bringup in background in runtime container
    nav2_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"ros2 launch nav2_bringup bringup_launch.py map:={map_yaml} use_sim_time:=True"
    )
    print(f"Starting Nav2 bringup for {slam_type}...")
    nav2_process = subprocess.Popen(
        ["docker", "exec", SLAM_RUNTIME_CONTAINER_NAME, "bash", "-c", nav2_cmd],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Wait for Nav2 stack to come up
    time.sleep(20)

    # 4. Run evaluate_nav2.py in runtime container
    eval_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"python3 /workspace/slam_evaluator/metrics/evaluate_nav2.py --output_csv {metrics_csv}"
    )
    run_docker_command(
        SLAM_RUNTIME_CONTAINER_NAME,
        eval_cmd,
        env,
        f"Running Nav2 evaluation for {slam_type}",
    )

    # 5. Cleanup
    print(f"Stopping Nav2 and Isaac Sim for {slam_type}...")
    subprocess.run(
        ["docker", "exec", SLAM_RUNTIME_CONTAINER_NAME, "pkill", "-f", "nav2"], env=env
    )
    subprocess.run(
        ["docker", "exec", "ros-jazzy-container", "pkill", "-f", "isaac-sim"], env=env
    )

    nav2_process.wait()
    isaac_process.wait()


def main():
    parser = argparse.ArgumentParser(
        description="Run the full evaluation pipeline for provided rosbag(s)"
    )

    rosbags = os.listdir("data/rosbags")
    print(rosbags)
    num_rosbags = len(rosbags)

    env = setup_environment_vars()

    for i, rosbag in enumerate(rosbags):
        print(f"ROSBAG {i + 1}/{num_rosbags}: {rosbag}")

        start_dockers(env)
        check_disk_space()

        generate_ground_truth(rosbag, env)
        run_rtab_map(rosbag, env)
        run_orb_slam(rosbag, env)
        run_splatam(rosbag, env)

        evaluate_metrics(rosbag, env)

        # New: Nav2 utility benchmarking
        print(f"Starting Nav2 utility benchmarking for {rosbag}...")
        for slam in ["rtab", "orb_slam", "splatam"]:
            run_nav2_evaluation(rosbag, slam, env)

        stop_dockers(env)


if __name__ == "__main__":
    main()
