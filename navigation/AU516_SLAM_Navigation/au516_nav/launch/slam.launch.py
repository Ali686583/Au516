"""SLAM Toolbox (online async) : /scan + TF odom -> base_link  ==>  /map + TF map -> odom."""

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
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'slam_params', default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')
            ),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    LaunchConfiguration('slam_params'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            ),
        ]
    )
