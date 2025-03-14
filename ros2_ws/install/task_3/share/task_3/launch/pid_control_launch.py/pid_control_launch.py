from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='task_3',
            executable='pid_speed_controller',        # make sure to change the node name
            name='pid_speed_controller',
        ),
])