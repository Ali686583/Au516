"""Simulation sans robot : sim_2d + TF laser + SLAM Toolbox + navigation + RViz.

ros2 launch au516_nav sim.launch.py scenario:=D_zigzag
"""

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
            DeclareLaunchArgument('scenario', default_value='ring'),
            DeclareLaunchArgument('slam', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            Node(
                package='au516_nav',
                executable='sim_2d',
                name='sim_2d',
                output='screen',
                parameters=[{'scenario': lc('scenario')}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser',
                arguments=[
                    '--x',
                    '0.22',
                    '--y',
                    '0',
                    '--z',
                    '0.10',
                    '--frame-id',
                    'base_link',
                    '--child-frame-id',
                    'laser',
                ],
            ),
            inc('slam.launch.py', condition=IfCondition(lc('slam'))),
            inc(
                'navigation.launch.py',
                launch_arguments={'autostart': 'true', 'use_map': lc('slam')}.items(),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(share, 'rviz', 'au516_nav.rviz')],
                condition=IfCondition(lc('rviz')),
            ),
        ]
    )
