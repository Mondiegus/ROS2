from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise36_pkg',
            executable='main_exec',
            output='screen'),
    ]) 