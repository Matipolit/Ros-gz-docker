from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        "bag_path", description="Absolute path to the rosbag to play"
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/ubuntu/shared/working_pcd_files/",
        description="Absolute path to save the generated PCD files",
    )

    qos_path_arg = DeclareLaunchArgument(
        "qos_path",
        default_value="/home/ubuntu/shared/qos.yaml",
        description="Absolute path to the QoS profile overrides",
    )

    # 2. Create LaunchConfiguration variables
    bag_path = LaunchConfiguration("bag_path")
    output_dir = LaunchConfiguration("output_dir")
    qos_path = LaunchConfiguration("qos_path")

    # 3. Define the nodes and processes
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--qos-profile-overrides-path",
            qos_path,
            "--clock",
        ],
        output="screen",
    )

    depth_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("image_rect", "/camera/depth/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("points", "/camera/points"),
        ],
    )

    pcd_export_node = Node(
        package="pcl_ros",
        executable="pointcloud_to_pcd",
        parameters=[
            {"prefix": output_dir, "fixed_frame": "root", "use_sim_time": True}
        ],
        remappings=[("input", "/camera/points")],
    )

    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bag_play,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription(
        [
            bag_path_arg,
            output_dir_arg,
            qos_path_arg,
            depth_node,
            pcd_export_node,
            bag_play,
            shutdown_handler,
        ]
    )
