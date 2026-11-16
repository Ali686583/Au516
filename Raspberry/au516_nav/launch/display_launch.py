"""
Display the SLAM map and the robot position in RViz2.

Run it on the PC (not on the Pi), connected to the robot WiFi, with the same
ROS_DOMAIN_ID as the Pi, while slam_launch.py runs on the Pi:
    ros2 launch au516_nav display_launch.py

Without building au516_nav on the PC, the same view is obtained with:
    rviz2 -d slam.rviz
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory("au516_nav"), "config", "slam.rviz"
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )
    ])
