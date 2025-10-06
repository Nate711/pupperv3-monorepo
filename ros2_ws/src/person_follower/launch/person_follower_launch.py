#!/usr/bin/env python3
"""
Launch file for the person follower node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'kp_angular',
            default_value='1.0',
            description='Proportional gain for angular velocity control'
        ),
        DeclareLaunchArgument(
            'kp_linear',
            default_value='0.5',
            description='Proportional gain for linear velocity control'
        ),
        DeclareLaunchArgument(
            'target_bbox_area',
            default_value='0.3',
            description='Target bounding box area ratio (0-1)'
        ),
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.5',
            description='Maximum linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='1.0',
            description='Maximum angular velocity in rad/s'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Control loop frequency in Hz'
        ),
        DeclareLaunchArgument(
            'angular_deadzone',
            default_value='0.05',
            description='Deadzone for angular error (0-1)'
        ),
        DeclareLaunchArgument(
            'linear_deadzone',
            default_value='0.05',
            description='Deadzone for linear error (0-1)'
        ),
        DeclareLaunchArgument(
            'detection_timeout',
            default_value='2.0',
            description='Timeout for detection messages in seconds'
        ),

        # Person follower node
        Node(
            package='person_follower',
            executable='person_follower_node',
            name='person_follower_node',
            output='screen',
            parameters=[{
                'kp_angular': LaunchConfiguration('kp_angular'),
                'kp_linear': LaunchConfiguration('kp_linear'),
                'target_bbox_area': LaunchConfiguration('target_bbox_area'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'angular_deadzone': LaunchConfiguration('angular_deadzone'),
                'linear_deadzone': LaunchConfiguration('linear_deadzone'),
                'detection_timeout': LaunchConfiguration('detection_timeout'),
            }]
        ),
    ])