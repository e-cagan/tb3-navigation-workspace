"""
Navigation launch file.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    A function that generates launch description.
    """
    
    return LaunchDescription([
        # Open turtlebot3 world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
            )
        ),

        # Open nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'map': os.path.join(get_package_share_directory('tb3_bringup'), 'maps', 'tb3_world_map.yaml'),
                'params_file': os.path.join(get_package_share_directory('tb3_bringup'), 'config', 'nav2_params.yaml'),
                'autostart': 'true',
            }.items()
        ),    

        # Open rviz node to visualize and give goals manually for now with presaved setup file
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])