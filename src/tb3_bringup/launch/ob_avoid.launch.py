"""
Launch file for avoiding obstacles.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Function to generate launch description.
    """

    return LaunchDescription([
        # Open turtlebot3 world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
            )
        ),

        # Open obstacle avoidance node
        Node(
            name='obstacle_avoidance_node',
            package='tb3_navigation',
            executable='obstacle_avoidance_node'
        ),
    ])