# launch/test_control.launch.py:
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kimono',
            executable='xbox_control',
            name='xbox_control'
        ),
        Node(
            package='kimono',
            executable='gui',
            name='gui'
        ),
        Node(
            package='kimono',
            executable='motor_control',
            name='motor_control'
        ),
    ])