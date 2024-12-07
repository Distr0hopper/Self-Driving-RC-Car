#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'linear_speed',
            default_value='1.0',
            description='Maximum linear speed (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.1',
            description='Maximum angular speed (rad/s)'
        ),
        
        # Launch the joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Launch the Xbox controller node
        Node(
            package='remote_control',
            executable='xbox_control',
            name='xbox_control',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'deadzone': 0.1,
                'turbo_multiplier': 2.0,
                'update_rate': 50.0
            }],
            output='screen'
        ),
        
        # Launch the visualization
        Node(
            package='gui',
            executable='control_visual',
            name='control_visual',
            output='screen'
        )
    ])