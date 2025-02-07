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
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
    )
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
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("neural_controller"), "launch", "config.yaml"]
        ),
        allow_substs=True,
    )

    #
    # 6. Nodes from your original launch files
    #
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
        condition=IfCondition(LaunchConfiguration("teleop")),
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
        parameters=[robot_controllers],
        output="both",
    )

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        output="both",
        parameters=[{"format": "RGB888", "width": 1400, "height": 1050}],
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    #
    # 7. Put them all together
    #
    nodes = [
        robot_state_publisher,
        control_node,
        robot_controller_spawner,
        three_legged_robot_controller_spawner,
        joint_state_broadcaster_spawner,
        # Comment/uncomment as needed:
        # imu_sensor_broadcaster_spawner,
        foxglove_bridge,
        joy_util_node,
        joy_node,
        teleop_twist_joy_node,
        camera_node,
    ]

    #
    # 8. Return the LaunchDescription with the declared arg + all nodes
    #
    return LaunchDescription([declare_sim_arg, declare_teleop_arg, *nodes])
