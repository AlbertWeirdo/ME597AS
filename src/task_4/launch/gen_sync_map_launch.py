from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot4_navigation'),
                    'launch',
                    'slam.launch.py'
                ])
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot4_viz'),
                    'launch',
                    'view_robot.launch.py'
                ])
            ),
        ]
    )
