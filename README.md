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

## Start simulation

To start with an Nvidia GPU:

```bash
./run-nvidia.sh
```

## Start SLAM runtime container

```bash
./run-slam-runtime.sh
```

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

`rtabmap_ros` is installed from apt (`ros-jazzy-rtabmap-ros`) and can be launched directly with `ros2 launch ...`.

## Start evaluator container

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
