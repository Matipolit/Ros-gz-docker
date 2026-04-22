# Point Cloud Extraction Workflow (ROS Bag -> PCD/PLY)

This document describes the process used in this repository to generate a merged point cloud map (`.pcd` or `.ply`) from recorded ROS data in a way that is more directly comparable with RTAB-Map outputs.

## 1) Data Gathering Environment

### 1.1 Build the simulator image

```bash
./build-isaac-sim-runner.sh
```

The simulator image is based on [Dockerfile](../Dockerfile) and includes required ROS packages:
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

---

## 2) Create PointCloud2 Topic from Depth

Inside the simulator container, convert depth image + camera info into a point cloud topic:

```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r /camera/depth/camera_info:=/camera/camera_info \
  -r points:=/camera/points \
  -p use_approximate_sync:=true \
  -p use_sim_time:=true
```

Notes:
- If your camera publishes to different topic names, update remaps accordingly.
- Keep this node running while exporting frames.

---

## 3) Export Per-Frame PCD Files in a Global Frame (Critical)

In another shell in the same container, export incoming `/camera/points` to `.pcd` files and force transform into a **global frame**:

```bash
ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/working_pcd_files/ \
  -p use_sim_time:=true \
  -p fixed_frame:=root \
  -r input:=/camera/points
```

Finally, play the rosbag. **Important:** Add a delay to give TF listeners time to initialize before the data stream starts.

```bash
ros2 bag play /home/ubuntu/shared/rosbags/max_x_run_y_description \
  --clock \
  --qos-profile-overrides-path /home/ubuntu/shared/qos.yaml \
  --delay 3
```

Stop recording when enough frames are collected (`Ctrl+C`).

---

## 4) Merge and Downsample in Evaluator Container

### 4.1 Build evaluator image

```bash
./build-slam-evaluator.sh
```

### 4.2 Run merge script via evaluator container

Use [run-slam-evaluator.sh](../run-slam-evaluator.sh). It mounts host `data/` as `/data` in the container.

Basic example (output `.ply`):

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/working_pcd_files \
  /data/reports/map_merged_downsampled.ply \
  0.05
```

### 4.3 Recommended objective filtering options

To improve fairness against RTAB-Map cloud, use:
- same voxel resolution (`voxel_size`)
- depth/range gating (`--min-range`, `--max-range`)
- optional ROI cropping (`--crop-*`)
- optional fixed Z rotation only for legacy datasets (`--rotate-z-deg`)

Example:

```bash
./run-slam-evaluator.sh python3 ground_truth/merge_downsample_pcd.py \
  /data/reports/working_pcd_files \
  /data/reports/map_merged_downsampled_filtered.ply \
  0.05 \
  --range-mode sensor_local \
  --min-range 0.3 \
  --max-range 8.0 \
  --crop-z-min -1.0 \
  --crop-z-max 2.5
```

**Note on `--range-mode sensor_local`:**
When exporting with `fixed_frame:=root`, the points in `.pcd` are saved in global coordinates. However, `pcl_ros` saves the original sensor origin into the PCD `VIEWPOINT` header. The `--range-mode sensor_local` argument reads this viewpoint to correctly filter depth range relative to the moving camera, just like RTAB-Map does, rather than filtering distance from the static global origin `(0,0,0)`.

Optional format override:

```bash
./run-slam-evaluator.sh merge_downsample_pcd.py \
  /data/reports/working_pcd_files \
  /data/reports/map_merged_anyname \
  0.05 \
  --output-format ply
```

Script used: [merge_downsample_pcd.py](merge_downsample_pcd.py)

---

## 5) What the Merge Script Does

For each input `.pcd` file:
1. Reads cloud from disk.
2. Skips empty clouds.
3. Optionally applies centroid-based guard for likely bad local-frame artifacts.
4. Optionally rotates cloud around Z (`--rotate-z-deg`).
5. Optionally filters by radial distance (`--min-range`, `--max-range`). If `--range-mode sensor_local` is used, distance is calculated from the PCD `VIEWPOINT` (sensor origin), otherwise from the global origin `(0,0,0)`.
6. Optionally crops with axis-aligned ROI (`--crop-x/y/z-min/max`).
7. Downsamples each valid frame with selected voxel size.
8. Adds to merged cloud.
9. Periodically re-downsamples merged cloud (every 20 valid files).
10. Shows progress bar.

After all files:
1. Applies final voxel downsampling.
2. Writes output (`.pcd` or `.ply`).
3. Prints summary statistics (accepted/skipped files and point counts).

---

## 6) Practical Guidance for Fair GT vs RTAB-Map Comparison

1. Use the same global frame for both clouds (`map` preferred).
2. Use comparable effective range limits in GT and RTAB-Map.
3. Use the same final voxel size for both exported maps.
4. Compare over common observed region (avoid evaluating huge GT-only regions).
5. Treat `--rotate-z-deg` as a migration workaround, not the default method.

---

## 7) Common Issues and Fixes

### 7.1 `Package 'depth_image_proc' not found`
Install `ros-jazzy-image-pipeline` in simulator image.

### 7.2 `Package 'pcl_ros' not found`
Install `ros-jazzy-pcl-ros` in simulator image.

### 7.3 `Could not open file for writing`
- Create target directory first (`mkdir -p ...`).
- Use absolute paths in `prefix`.
- Ensure mounted path is writable.

### 7.4 Wrong path in evaluator container
`run-slam-evaluator.sh` uses `/data` for host `data/` mount.
Use `/data/reports/...` paths, not relative paths like `../reports/...`.

### 7.5 `fixed_frame ... does not exist`
The TF frame name is wrong/unavailable for the played bag.
- Verify `/tf` and `/tf_static` are being played.
- Ensure chosen fixed frame exists in TF graph during export.
- Use a valid global frame consistently in all later processing.

### 7.6 All points disappear after filtering
Your filtering may be too aggressive.
- Relax `--min-range` / increase `--max-range`.
- Widen ROI bounds.
- Test first without crop arguments, then tighten progressively.

### 7.7 `skip transform: ... source_frame does not exist`
If **all** your `.pcd` files are skipping transforms and saving as invalid local frames, the TF tree (`/tf` or `/tf_static`) is not reaching the node in time.
- Start the `pointcloud_to_pcd` node *first*, wait a couple of seconds, and *then* run `ros2 bag play`.
- Always use `--delay 3` (or more) when playing the bag to ensure discovery completes before messages are sent.
- Ensure `--clock` is used in `ros2 bag play` so `use_sim_time:=true` works correctly.

---

## 8) Minimal End-to-End Command Set

Inside simulator stack:

```bash
./run-nvidia.sh
# new shell
./join-simulator-container.sh

ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/camera_info \
  -r points:=/camera/points

mkdir -p /home/ubuntu/shared/reports/working_pcd_files
ros2 run pcl_ros pointcloud_to_pcd --ros-args \
  -p prefix:=/home/ubuntu/shared/reports/working_pcd_files/ \
  -p use_sim_time:=true \
  -p fixed_frame:=root \
  -r input:=/camera/points
```

Play rosbag in another shell (with `--delay 3`), wait for completion, then merge from host:

```bash
./build-slam-evaluator.sh
./run-slam-evaluator.sh python3 ground_truth/merge_downsample_pcd.py \
  /data/reports/working_pcd_files \
  /data/reports/map_merged_downsampled.ply \
  0.05 \
  --range-mode sensor_local \
  --min-range 0.3 \
  --max-range 8.0
```
