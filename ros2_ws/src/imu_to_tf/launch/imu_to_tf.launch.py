from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_frame',
            default_value='map',
            description='World frame for TF transform'
        ),
        
        DeclareLaunchArgument(
            'body_frame', 
            default_value='base_link',
            description='Body frame for TF transform'
        ),
        
        Node(
            package='imu_to_tf',
            executable='imu_to_tf_node',
            name='imu_to_tf',
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'body_frame': LaunchConfiguration('body_frame'),
            }]
        )
    ])