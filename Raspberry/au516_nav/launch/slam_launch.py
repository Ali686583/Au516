
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
            executable="serial_bridge",
            output="screen"
        ),

        # Lidar position on the robot: base_link -> laser
        # (x y z yaw pitch roll, in m and rad; measure it on the car)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser"
            ]
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
