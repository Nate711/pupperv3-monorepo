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
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    #
    # 1. Declare a boolean launch argument 'sim' that controls whether to run the robot in the simulator or not
    #
    declare_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="False",
        description=(
            "Run `ros2 launch neural_controller launch.py sim:=True` to run the robot "
            "in the Mujoco simulator, otherwise the default value of False will run the real robot."
        ),
    )

    declare_teleop_arg = DeclareLaunchArgument(
        name="teleop",
        default_value="True",
        description=(
            "Run `ros2 launch neural_controller launch.py teleop:=True` to enable teleop, "
            "otherwise the default value of False will not run teleop."
        ),
    )

    declare_bag_recorder_arg = DeclareLaunchArgument(
        name="bag_recorder",
        default_value="True",
        description=(
            "Run `ros2 launch neural_controller launch.py bag_recorder:=True` to enable bag recording, "
            "otherwise the default value of False will not run bag recorder."
        ),
    )

    #
    # 2. Construct the path to the URDF file using IfElseSubstitution

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

    #
    # 3. Create the robot_description using xacro
    #
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file])
    robot_description = {"robot_description": robot_description_content}

    #
    # 4. Robot State Publisher
    #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    #
    # 5. Common controller parameters
    #
    node_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "config.yaml"]),
        allow_substs=True,
    )

    #
    # 6. Nodes from your original launch files
    #
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     parameters=[node_parameters],
    #     output="both",
    # )
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        parameters=[node_parameters],
        output="both",
        name="joy_linux_node",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[node_parameters],
        output="both",
        condition=IfCondition(LaunchConfiguration("teleop")),
        remappings=[("cmd_vel", "teleop_cmd_vel")],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[node_parameters],
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
            "--inactive",
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

    # Forward command controllers for animation system (inactive by default)
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    forward_kp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kp_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    forward_kd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kd_controller",
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

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    joy_util_node = Node(
        package="joy_utils",
        executable="estop_controller",
        parameters=[node_parameters],
        output="both",
        name="joy_util_node",
    )

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="both",
        parameters=[node_parameters],
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    cmd_vel_mux_node = Node(
        package="cmd_vel_mux",
        executable="cmd_vel_mux_node",
        parameters=[node_parameters],
        output="both",
    )

    bag_recorder_node = Node(
        package="bag_recorder",
        executable="bag_recorder_node",
        name="bag_recorder",
        output="both",
        parameters=[
            {
                "record_start_button": 4,  # L1 button
                "record_stop_button": 5,  # R1 button
                "bag_output_dir": "~/bags",
                "start_button_hold_duration": 5.0,  # 5 seconds
            }
        ],
        condition=IfCondition(LaunchConfiguration("bag_recorder")),
    )

    imu_to_tf_node = Node(
        package="imu_to_tf",
        executable="imu_to_tf_node",
        output="both",
    )

    animation_controller_py_node = Node(
        package="animation_controller_py",
        executable="animation_controller_py",
        name="animation_controller_py",
        parameters=[node_parameters],
        output="both",
    )

    #
    # 7. Put them all together
    #
    nodes = [
        robot_state_publisher,
        control_node,
        robot_controller_spawner,
        three_legged_robot_controller_spawner,
        # Forward command controllers for animation
        forward_position_controller_spawner,
        forward_kp_controller_spawner,
        forward_kd_controller_spawner,
        joint_state_broadcaster_spawner,
        # Comment/uncomment as needed:
        imu_sensor_broadcaster_spawner,
        foxglove_bridge,
        joy_util_node,
        # joy_node,
        joy_linux_node,
        teleop_twist_joy_node,
        camera_node,
        cmd_vel_mux_node,
        bag_recorder_node,
        imu_to_tf_node,
        animation_controller_py_node,
    ]

    #
    # 8. Return the LaunchDescription with the declared arg + all nodes
    #
    return LaunchDescription([declare_sim_arg, declare_teleop_arg, declare_bag_recorder_arg, *nodes])
