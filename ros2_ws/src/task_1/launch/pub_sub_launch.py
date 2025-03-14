from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='task_1',
            executable='talker',    # should be the assigned node name in the package "task1"
            name='talker',          # namespace: can assign any names to this node (can be different from the package's) and when run "ros2 node list", it will show the namespace's
        ),
        Node(
            package='task_1',
            executable='listener',
            name='listener',
        )
])