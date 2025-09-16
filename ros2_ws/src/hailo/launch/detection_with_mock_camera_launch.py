from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "hailo"
    package_dir = get_package_share_directory(package_name)

    # Declare launch arguments
    sim_arg = DeclareLaunchArgument(
        "sim", default_value="True", description="Use YOLOv8 simulation mode (recommended with mock camera)"
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="15.0", description="Image publish rate in Hz"
    )

    # Mock camera node
    mock_camera_node = Node(
        package=package_name,
        executable="mock_camera",
        name="mock_camera_node",
        parameters=[
            {
                "publish_rate": LaunchConfiguration("publish_rate"),
                "frame_id": "camera_frame",
            }
        ],
        output="screen",
    )

    # Detection node
    detection_node = Node(
        package=package_name,
        executable="hailo_detection",
        name="hailo_detection_node",
        parameters=[
            {
                "sim": LaunchConfiguration("sim"),
                "yolo_model": "yolov8m.pt",
                "score_threshold": 0.5,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sim_arg,
            publish_rate_arg,
            mock_camera_node,
            detection_node,
        ]
    )
