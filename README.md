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

To record a run, join the container from a new terminal and run:

```bash
ros2 bag record -o /home/ubuntu/shared/rosbags/max_x_run_y_description \
/clock /tf /tf_static \
/camera/color/image_raw /camera/depth/image_raw /camera/camera_info \
/imu/data \
/cmd_vel
```

## Generating Ground Truth

### Generating a reference point cloud

*Inside the simulation container*

Create a PointCloud2 topic:

```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/camera_info \
  -r points:=/camera/points
```

In another shell in the same container, export incoming `/camera/points` messages to individual `.pcd` files:

```bash
mkdir -p /home/ubuntu/shared/reports/pcd_files

ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/pcd_files/ \
  -p use_sim_time:=true \
  -r input:=/camera/points
```


Play a recorded rosbag to register the individual point clouds:

```bashbash
ros2 bag play /home/ubuntu/shared/rosbags/max_x_run_y_description
```

After it finishes playing, you can stop all the ros nodes and the registered point clouds will be in `data/reports/pcd_files/` on the host.

*Inside the evaluation container*

Run the merge script:

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/pcd_files \
  /data/reports/map_x_run_y/ground_truth/map_merged_downsampled.ply \
  0.05
```

The arguments for the script are as follows:
- `input_dir`: directory containing the individual `.pcd` files
- `output_file`: path to the output merged point cloud (can be `.ply` or `.pcd`)
- `voxel_size`: voxel size for downsampling (e.g. 0.05 for 5cm)

### Generating a reference trajectory

*Inside the evaluation container*

Run the extract_trajectory script:

```bash
./run-slam-evaluator.sh extract_trajectory.py \
  /data/rosbags/max_x_run_y \
  /data/reports/map_x_run_y/ground_truth/trajectory.csv
```

### Generating a reference topology

*Inside the evaluation container*

Run the extract_topology script:

```bash
./run-slam-evaluator.sh extract_topology.py \
  /data/rosbags/max_x_run_y \
  /data/reports/map_x_run_y/ground_truth/topology.json
```

## Running SLAM algorithms 

*Inside the SLAM runtime container*

### Orb-SLAM3


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

Listen to appropriate topics with RTAB-Map:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  frame_id:=base_link \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/camera_info \
  approx_sync:=true \
  use_sim_time:=true \
  rtabmap_viz:=false

```bash
./run-slam-runtime.sh
```

Open a new terminal and play the rosbag:

```bash
ros2 bag play /home/ubuntu/shared/rosbags/map_x_run_y_description
```

After the rosbag finishes, you can close RTAB-Map, and the resulting db will be in `/root/.ros/rtabmap.db`

To extract the trajectory and point cloud:

```bash
mkdir -p /data/reports/map_x_run_y/rtab-map

rtabmap-export --poses \
  --output_dir /data/reports/map_x_run_y/rtab-map \
  /root/.ros/rtabmap.db

rtabmap-export --cloud \
  --output_dir /data/reports/map_x_run_y/rtab-map \
  /root/.ros/rtabmap.db
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
