from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare launch arguments
    declare_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="False",
        description=(
            "Run `ros2 launch animation_controller animation_playback.launch.py sim:=True` to run the robot "
            "in the Mujoco simulator, otherwise the default value of False will run the real robot."
        ),
    )

    declare_csv_file_arg = DeclareLaunchArgument(
        name="csv_file",
        default_value="",
        description="Path to the CSV file containing animation keyframes",
    )

    declare_frame_rate_arg = DeclareLaunchArgument(
        name="frame_rate",
        default_value="30.0",
        description="Animation playback frame rate in Hz",
    )

    declare_loop_arg = DeclareLaunchArgument(
        name="loop",
        default_value="False",
        description="Whether to loop the animation continuously",
    )

    declare_auto_start_arg = DeclareLaunchArgument(
        name="auto_start",
        default_value="False", 
        description="Whether to start the animation automatically when the controller is activated",
    )

    # Construct the path to the URDF file using IfElseSubstitution
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("pupper_v3_description"),
            "description",
            IfElseSubstitution(
                condition=PythonExpression(LaunchConfiguration("sim")),
                if_value=TextSubstitution(text="pupper_v3_mujoco.urdf.xacro"),
                else_value=TextSubstitution(text="pupper_v3.urdf.xacro"),
            ),
        ]
    )

    # Create the robot_description using xacro
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Animation controller parameters
    animation_config_file = PathJoinSubstitution(
        [FindPackageShare("animation_controller"), "launch", "animation_config.yaml"]
    )

    # Controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[animation_config_file],
        output="both",
    )

    # Animation controller spawner
    animation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "animation_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # IMU sensor broadcaster spawner
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    # Foxglove bridge for visualization
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    # Put them all together
    nodes = [
        robot_state_publisher,
        control_node,
        animation_controller_spawner,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        foxglove_bridge,
    ]

    return LaunchDescription([
        declare_sim_arg,
        declare_csv_file_arg,
        declare_frame_rate_arg,
        declare_loop_arg,
        declare_auto_start_arg,
        *nodes
    ])