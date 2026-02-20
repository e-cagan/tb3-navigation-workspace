"""
Launch file for implementing SLAM to localize and map.
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
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_house.launch.py')
            )
        ),

        # Open slam toolbox launch file (use simulation time)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Open rviz node to visualize
        Node(
            name='rviz_node',
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}]
        ),

        # Open teleop node to control tb3
        Node(
            name='teleop_twist_keyboard_node',
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
        ),
    ])