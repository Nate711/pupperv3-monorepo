from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "hailo"
    package_dir = get_package_share_directory(package_name)

    # Declare launch arguments
    sim_arg = DeclareLaunchArgument(
        "sim", default_value="False", description="Use YOLOv8 simulation mode instead of Hailo"
    )

    model_path_arg = DeclareLaunchArgument(
        "model_name", default_value="yolov8m.hef", description="Name of the Hailo HEF model file"
    )

    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model", default_value="yolov8m.pt", description="YOLOv8 model name for simulation mode"
    )

    labels_path_arg = DeclareLaunchArgument(
        "labels_path",
        default_value=os.path.join(package_dir, "config", "coco.txt"),
        description="Path to the labels file",
    )

    score_threshold_arg = DeclareLaunchArgument(
        "score_threshold", default_value="0.5", description="Detection score threshold"
    )

    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value="/camera/image_raw", description="Camera image topic to subscribe to"
    )

    # Create the node
    hailo_detection_node = Node(
        package=package_name,
        executable="hailo_detection",
        name="hailo_detection_node",
        parameters=[
            {
                "sim": LaunchConfiguration("sim"),
                "model_name": LaunchConfiguration("model_name"),
                "yolo_model": LaunchConfiguration("yolo_model"),
                "labels_path": LaunchConfiguration("labels_path"),
                "score_threshold": LaunchConfiguration("score_threshold"),
            }
        ],
        remappings=[
            ("/camera/image_raw", LaunchConfiguration("camera_topic")),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sim_arg,
            model_path_arg,
            yolo_model_arg,
            labels_path_arg,
            score_threshold_arg,
            camera_topic_arg,
            hailo_detection_node,
        ]
    )
