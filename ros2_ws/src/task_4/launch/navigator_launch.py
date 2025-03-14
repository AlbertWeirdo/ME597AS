from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='task_4',
            executable='Navigation',        
            name='Navigation',
        ),
])