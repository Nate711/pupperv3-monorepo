from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Define the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory("pupper_v3_description"),
        "description",
        "urdf",
        "pupper_v3.edited.fixed.urdf",
    )
    with open(urdf_file_path, "r") as infp:
        robot_desc = infp.read()

    # Rviz node
    rviz_config_file_path = os.path.join(
        get_package_share_directory("pupper_v3_description"),
        "description",
        "pupper_v3.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # arguments=["-d", rviz_config_file_path],
        output="screen",
    )

    # # Robot state publisher node
    params = {"robot_description": robot_desc}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[params],
        # parameters=[{"robot_description": Command(["xacro ", urdf_file_path])}],
    )

    # Joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
        ]
    )
