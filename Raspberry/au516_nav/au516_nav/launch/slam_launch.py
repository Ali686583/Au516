
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_dir = get_package_share_directory("slam_toolbox")
    au516_dir = get_package_share_directory("au516_nav")

    slam_launch = os.path.join(
        slam_dir, "launch", "online_async_launch.py"
    )

    slam_config = os.path.join(
        au516_dir, "config", "slam.yaml"
    )

    return LaunchDescription([

        Node(
            package="au516_nav",
            executable="encoder_reader",
            output="screen"
        ),

        Node(
            package="au516_nav",
            executable="odometry_node",
            output="screen"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam_params_file": slam_config,
                "use_sim_time": "false"
            }.items()
        )
    ])
