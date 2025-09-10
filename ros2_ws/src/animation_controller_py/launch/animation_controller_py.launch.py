from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='animation_controller_py',
            executable='animation_controller_py',
            name='animation_controller_py',
            parameters=[{
                'frame_rate': 30.0,
                'loop_animation': False,
                'default_animation': '',
                'init_duration': 2.0,
                'joint_names': [
                    'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
                    'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
                    'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
                    'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
                ],
                'kps': [5.0] * 12,
                'kds': [0.25] * 12,
                'init_kps': [7.5] * 12,
                'init_kds': [0.25] * 12,
            }],
            output='screen'
        )
    ])