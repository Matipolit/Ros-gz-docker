# Point Cloud Extraction Workflow (ROS Bag -> PCD/PLY)

This document describes the full process used in this repository to generate a final merged point cloud map (`.pcd` or `.ply`) from recorded ROS data.

## 1) Data Gathering Environment

### 1.1 Build the simulator image

```bash
./build-isaac-sim-runner.sh
```

The simulator image is based on [Dockerfile](../Dockerfile) and includes the required ROS packages for this workflow:
- `ros-jazzy-image-pipeline` (provides `depth_image_proc`)
- `ros-jazzy-pcl-ros` (provides `pcl_ros`)

### 1.2 Run simulator container

```bash
./run-nvidia.sh
```

Important mount from [run-nvidia.sh](../run-nvidia.sh):
- host `data/` is mounted to container path `/home/ubuntu/shared`

So files written under `/home/ubuntu/shared/reports/...` inside the simulator container appear in host `data/reports/...`.

### 1.3 Join the running simulator container (optional extra shell)

```bash
./join-simulator-container.sh
```

## 2) Create PointCloud2 Topic from Depth

Inside the simulator container, convert depth image + camera info into a point cloud topic:

```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/camera_info \
  -r points:=/camera/points
```

Notes:
- If your camera publishes to different topic names, update remaps accordingly.
- Keep this node running while exporting frames.

## 3) Export Per-Frame PCD Files

In another shell in the same container, export incoming `/camera/points` messages to individual `.pcd` files:

```bash
mkdir -p /home/ubuntu/shared/reports/pcd_files

ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/pcd_files/ \
  -p use_sim_time:=true \
  -r input:=/camera/points
```

Notes:
- `prefix` must point to an existing writable directory.
- Use an absolute path to avoid confusion.
- If you set `fixed_frame`, make sure this frame exists in TF.

Stop recording when enough frames are collected (`Ctrl+C`).

## 4) Merge and Downsample in Evaluator Container

### 4.1 Build evaluator image

```bash
./build-slam-evaluator.sh
```

### 4.2 Run merge script via evaluator container

Use [run-slam-evaluator.sh](../run-slam-evaluator.sh). It mounts host `data/` as `/data` in the container.

Example (output `.ply`):

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/pcd_files \
  /data/reports/map_merged_downsampled.ply \
  0.05
```

Example (output `.pcd`):

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/pcd_files \
  /data/reports/map_merged_downsampled.pcd \
  0.05
```

Optional format override:

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/pcd_files \
  /data/reports/map_merged_anyname \
  0.05 \
  --output-format ply
```

Script used: [merge_downsample_pcd.py](merge_downsample_pcd.py)

## 5) What the Merge Script Does

For each input `.pcd` file:
1. Reads cloud from disk.
2. Skips empty clouds.
3. Applies optional centroid-based guard for likely bad local-frame clouds.
4. Downsamples each frame with the chosen voxel size.
5. Adds to merged cloud.
6. Periodically re-downsamples merged cloud (every 20 valid files).
7. Shows progress bar.

After all files:
1. Applies final voxel downsampling.
2. Writes output (`.pcd` or `.ply`).
3. Prints summary statistics.

## 6) Common Issues and Fixes

### 6.1 `Package 'depth_image_proc' not found`
Install `ros-jazzy-image-pipeline` in simulator image.

### 6.2 `Package 'pcl_ros' not found`
Install `ros-jazzy-pcl-ros` in simulator image.

### 6.3 `Could not open file for writing`
- Create target directory first (`mkdir -p ...`).
- Use absolute paths in `prefix`.
- Ensure mounted path is writable.

### 6.4 Wrong path in evaluator container
`run-slam-evaluator.sh` uses `/data` for host `data/` mount.
Use `/data/reports/...` paths, not relative paths like `../reports/...`.

### 6.5 `fixed_frame ... does not exist`
The TF frame name is wrong or unavailable. Remove `fixed_frame` or use a valid frame.

## 7) Minimal End-to-End Command Set

Inside simulator stack:

```bash
./run-nvidia.sh
# new shell
./join-simulator-container.sh

ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/camera_info \
  -r points:=/camera/points

mkdir -p /home/ubuntu/shared/reports/pcd_files
ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/pcd_files/ \
  -p use_sim_time:=true \
  -r input:=/camera/points
```

Then merge from host:

```bash
./build-slam-evaluator.sh
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/pcd_files \
  /data/reports/map_merged_downsampled.ply \
  0.05
```
