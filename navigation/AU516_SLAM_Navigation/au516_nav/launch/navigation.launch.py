"""Navigation autonome : local_navigator (seul éditeur de /cmd_vel)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('au516_nav')
    return LaunchDescription(
        [
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('use_map', default_value='true'),
            Node(
                package='au516_nav',
                executable='local_navigator',
                name='local_navigator',
                output='screen',
                parameters=[
                    os.path.join(share, 'config', 'nav.yaml'),
                    {
                        'autostart': LaunchConfiguration('autostart'),
                        'use_map': LaunchConfiguration('use_map'),
                    },
                ],
            ),
        ]
    )
