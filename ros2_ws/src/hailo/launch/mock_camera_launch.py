from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "hailo"

    # Declare launch arguments
    image_folder_arg = DeclareLaunchArgument(
        "image_folder", default_value="config", description="Folder containing images to cycle through"
    )

    cycle_images_arg = DeclareLaunchArgument(
        "cycle_images", default_value="True", description="Whether to cycle through images"
    )

    cycle_interval_arg = DeclareLaunchArgument(
        "cycle_interval", default_value="1.0", description="Seconds between image changes when cycling"
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="10.0", description="Image publish rate in Hz"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="camera_frame", description="Frame ID for the camera image"
    )

    jpeg_quality_arg = DeclareLaunchArgument(
        "jpeg_quality", default_value="95", description="JPEG compression quality (0-100)"
    )

    # Create the mock camera node
    mock_camera_node = Node(
        package=package_name,
        executable="mock_camera",
        name="mock_camera_node",
        parameters=[
            {
                "image_folder": LaunchConfiguration("image_folder"),
                "cycle_images": LaunchConfiguration("cycle_images"),
                "cycle_interval": LaunchConfiguration("cycle_interval"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "frame_id": LaunchConfiguration("frame_id"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            image_folder_arg,
            cycle_images_arg,
            cycle_interval_arg,
            publish_rate_arg,
            frame_id_arg,
            jpeg_quality_arg,
            mock_camera_node,
        ]
    )
