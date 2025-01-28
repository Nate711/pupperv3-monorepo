from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pupper_v3_description"),
                    "description",
                    "pupper_v3.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("neural_controller"),
                "launch",
                "config.yaml",
            ]
        ),
        allow_substs=True,
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[robot_controllers],
        output="both",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[robot_controllers],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    three_legged_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller_three_legged",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

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

    joy_util_node = Node(
        package="joy_utils",
        executable="estop_controller",
        parameters=[robot_controllers],
        output="both",
    )

    nodes = [
        robot_state_publisher,
        # imu_sensor_broadcaster_spawner,
        control_node,
        robot_controller_spawner,
        three_legged_robot_controller_spawner,
        joint_state_broadcaster_spawner,
        joy_node,
        teleop_twist_joy_node,
        joy_util_node,
    ]

    return LaunchDescription(nodes)
