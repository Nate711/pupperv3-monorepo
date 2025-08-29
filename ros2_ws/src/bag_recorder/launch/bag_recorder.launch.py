from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_start_button',
            default_value='6',
            description='Button index to start recording (default: L1 button)'
        ),
        
        DeclareLaunchArgument(
            'record_stop_button', 
            default_value='7',
            description='Button index to stop recording (default: R1 button)'
        ),
        
        DeclareLaunchArgument(
            'bag_output_dir',
            default_value='~/bags',
            description='Directory to save bag files'
        ),
        
        Node(
            package='bag_recorder',
            executable='bag_recorder_node',
            name='bag_recorder',
            output='screen',
            parameters=[{
                'record_start_button': LaunchConfiguration('record_start_button'),
                'record_stop_button': LaunchConfiguration('record_stop_button'),
                'bag_output_dir': LaunchConfiguration('bag_output_dir'),
            }]
        )
    ])