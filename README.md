# SLAM benchmarking environment

Docker split in this repository:

- simulation container: Isaac + ROS/GZ (`Dockerfile`)
- slam runtime container: `rtabmap_ros` + ORB-SLAM3 ROS 2 wrapper (`Dockerfile.slam_runtime`)
- evaluator container: SplaTAM + trajectory/topology scripts (`Dockerfile.slam_evaluator`)

This split avoids dependency conflicts and keeps rebuild/debug cycles short.

## Build images

```bash
./build-isaac-sim-runner.sh
./build-slam-runtime.sh
./build-slam-evaluator.sh
```

## Simulation

To start with an Nvidia GPU:

```bash
./run-nvidia.sh
```

To start Isaac Sim inside the container:

```bash
isaac-sim
```

To control the robot, join the container from a new terminal and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Run this transform in another terminal:

```bash
ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.15 -0.5 0.5 -0.5 0.5 base_link true_optical_frame
```

To record a run, join the container from a new terminal and run:

```bash
ros2 bag record -o /home/ubuntu/shared/rosbags/max_x_run_y_description \
/clock /tf /tf_static \
/camera/color/image_raw /camera/depth/image_raw /camera/camera_info \
/imu/data \
/cmd_vel
```

## Generating Ground Truth

### Why frame consistency matters

To compare Ground Truth (GT) map vs RTAB-Map map objectively, both maps must be generated in a consistent global frame and with comparable filtering assumptions.

If GT is exported in a camera-local or optical frame while RTAB-Map cloud is in `map`, you can observe:
- apparent 90° rotation differences,
- misleading alignment offsets.

If GT keeps all depth points without range/ROI filtering, it will usually cover a much larger area than RTAB-Map and bias quality metrics.

### Generating a reference point cloud

*Inside the simulation container*

Create a PointCloud2 topic:

```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/camera_info \
  -r points:=/camera/points
```

In another shell in the same container, export incoming `/camera/points` messages to individual `.pcd` files in a fixed global frame:

```bash
mkdir -p /home/ubuntu/shared/reports/pcd_files

ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/pcd_files/ \
  -p fixed_frame:=root \
  -p use_sim_time:=true \
  -r input:=/camera/points
```

Play a recorded rosbag to register the individual point clouds:

```bash
ros2 bag play /home/ubuntu/shared/rosbags/map_x_run_y \
  --qos-profile-overrides-path /home/ubuntu/shared/qos.yaml --clock

```

After playback finishes, stop nodes. The registered point clouds will be in `data/reports/pcd_files/` on the host.

*Inside the evaluation container*

Extract the trajectory - important for helping the pcd merge script

```bash
./run-slam-evaluator.sh python3 ground_truth/extract_trajectory.py \
  /data/rosbags/map_x_run_y \
  /data/reports/map_x_run_y/ground_truth/traj.csv
```

Run the merge script with filtering so GT and RTAB-Map represent comparable observable space.
*(Note: If you want to use `--keyframe-poses` for fair evaluation, run the SLAM algorithm first to generate the trajectory/poses, then come back and run this merge script.)*

```bash
./run-slam-evaluator.sh python3 ground_truth/merge_downsample_pcd.py \
  /data/reports/working_pcd_files \
  /data/reports/map_x_run_y/ground_truth/merged.ply \
  0.05 \
  --range-mode sensor_local \
  --trajectory-csv /data/reports/map_x_run_y/ground_truth/traj.csv \
  --min-range 0.3 \
  --max-range 8.0 \
  --crop-z-min -1.0 \
  --crop-z-max 2.5 \
  --keyframe-poses /data/reports/map_x_run_y/rtab/rtabmap_poses.txt \
  --keyframe-tolerance 0.1

```

Optional useful arguments:
- `--keyframe-poses <path>`: limits merged GT frames to those matching timestamps in the provided poses file. Prevents GT from containing dense viewpoints that SLAM never registered. Please note that this may result in GT having some features under-represented, but that's fine.
- `--keyframe-tolerance <sec>`: maximum time difference to match a GT `.pcd` to a keyframe (default `0.05`).
- `--range-mode sensor_local`: calculates range distance from the original camera center (using the `.pcd` VIEWPOINT header) rather than the global origin. Highly recommended when exporting with a global `fixed_frame`.
- `--min-range <m>`: remove too-near points.
- `--max-range <m>`: remove far points not reliably reconstructed by SLAM.
- `--crop-x-min/--crop-x-max --crop-y-min/--crop-y-max --crop-z-min/--crop-z-max`: keep a shared ROI.
- `--rotate-z-deg 90`: only for legacy correction if historical exports used an inconsistent frame convention.
- `--disable-centroid-guard`: disables heuristic skipping of likely corrupted near-origin local-frame clouds.

Recommended practice for fair comparison:
1. Same global frame for both clouds (`root` preferred).
2. Same or very similar spatial filtering assumptions (range/ROI).
3. Subsample GT using SLAM keyframe timestamps so both maps are built from the same observable views.
4. Same final voxel size before metric computation.

The primary merge script arguments are:
- `input_dir`: directory containing individual `.pcd` files
- `output_file`: output merged point cloud (`.ply` or `.pcd`)
- `voxel_size`: downsampling voxel size (e.g. `0.05` for 5 cm)



### Generating a reference trajectory

*Inside the evaluation container*

Run the `extract_trajectory` script:

```bash
./run-slam-evaluator.sh extract_trajectory.py \
  /data/rosbags/max_x_run_y \
  /data/reports/map_x_run_y/ground_truth/traj.csv
```

### Generating a reference topology

*Inside the evaluation container*

Run the `extract_topology` script:

```bash
./run-slam-evaluator.sh extract_topology.py \
  /data/rosbags/max_x_run_y \
  /data/reports/map_x_run_y/ground_truth/topology.json
```

## Running SLAM algorithms

*Inside the SLAM runtime container*

### ORB-SLAM3

Inside the container bootstrap the ORB-SLAM3 wrapper once:

```bash
bootstrap-orbslam3-wrapper
```

Then build the wrapper workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/slam_ws
colcon build --symlink-install
```

### RTAB-Map

Run RTAB-Map with explicit map-generation parameters for reproducibility and easier GT matching:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0 --Grid/FromDepth true --Grid/3D true --Grid/RangeMax 8.0 --Grid/CellSize 0.05" \
  frame_id:=base_link \
  map_frame_id:=root \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/camera_info \
  approx_sync:=true \
  use_sim_time:=true \
  rtabmap_viz:=false
```

Then start the runtime container if needed:

```bash
./run-slam-runtime.sh
```

Open a new terminal and play the rosbag:

```bash
ros2 bag play /home/ubuntu/shared/rosbags/map_x_run_y_description
```

After rosbag playback, close RTAB-Map. The resulting DB will be in:

- `/root/.ros/rtabmap.db`

To extract trajectory and point cloud:

```bash
mkdir -p /data/reports/map_x_run_y/rtab

rtabmap-export --poses \
  --output_dir /data/reports/map_x_run_y/rtab \
  /root/.ros/rtabmap.db

rtabmap-export --cloud \
    --cloud_max_depth 8.0 \
    --cloud_min_depth 0.3 \
    --cloud_voxel 0.05 \
    --cloud_decimation 1 \
    --output_dir /data/reports/map_x_run_y/rtab \
    /root/.ros/rtabmap.db
```

RTAB-Map treats the first pose as being in 0,0,0 and builds the map relative to that.
To align the RTAB-Map point cloud to Ground Truth, use the script:


```bash
./run-slam-evaluator.sh python3 evaluate/align_opus.py \
  --gt-trajectory /data/reports/map_x_run_y/ground_truth/traj.csv \
  --rtab-poses    /data/reports/map_x_run_y/rtab/rtabmap_poses.txt \
  --rtab-cloud    /data/reports/map_x_run_y/rtab/rtabmap_cloud.ply \
  --out-poses     /data/reports/map_x_run_y/rtab/rtabmap_poses_aligned.txt \
  --out-cloud     /data/reports/map_x_run_y/rtab/rtabmap_cloud_aligned.ply
```

## Evaluating SLAM algorithms

```bash
./run-slam-evaluator.sh
```

Inside the evaluator container bootstrap research dependencies once:

```bash
bootstrap-splatam
python3 -m venv /opt/research/SplaTAM/.venv
source /opt/research/SplaTAM/.venv/bin/activate
pip install -r /opt/research/SplaTAM/venv_requirements.txt
```

Evaluator image intentionally does not include ROS packages. It is Python-only for
SplaTAM and analysis tools.

## ROS networking notes

All three containers should use the same:

- `ROS_DOMAIN_ID` (should be 0 - `export ROS_DOMAIN_ID=0`)
- `RMW_IMPLEMENTATION`

The runtime launcher uses `--network host` for simpler DDS discovery on Linux.

## Optional: override ORB-SLAM3 wrapper repository/branch

Defaults:

- repo: `https://github.com/Mechazo11/ros2_orb_slam3.git`
- branch: `jazzy`

Override in runtime container:

```bash
export ORB_SLAM3_WRAPPER_REPO=https://github.com/<owner>/<repo>.git
export ORB_SLAM3_WRAPPER_BRANCH=<branch>
bootstrap-orbslam3-wrapper
```
