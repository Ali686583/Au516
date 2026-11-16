"""Système complet sur le robot : matériel + SLAM Toolbox + navigation (+ RViz en option)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('au516_nav')
    lc = LaunchConfiguration

    def inc(name, **kw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)), **kw
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument('slam', default_value='true'),
            DeclareLaunchArgument('navigate', default_value='true'),
            DeclareLaunchArgument(
                'autostart', default_value='false'
            ),  # sur le robot : démarrage manuel
            DeclareLaunchArgument('rviz', default_value='false'),  # RViz plutôt sur le PC
            DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
            inc('robot.launch.py', launch_arguments={'lidar_port': lc('lidar_port')}.items()),
            inc('slam.launch.py', condition=IfCondition(lc('slam'))),
            inc(
                'navigation.launch.py',
                condition=IfCondition(lc('navigate')),
                launch_arguments={'autostart': lc('autostart'), 'use_map': lc('slam')}.items(),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(share, 'rviz', 'au516_nav.rviz')],
                condition=IfCondition(lc('rviz')),
            ),
        ]
    )
