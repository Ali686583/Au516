
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    # The Pico (FTDI cable) and the LiDAR are both /dev/ttyUSBx and their order
    # can change at boot: use the stable /dev/serial/by-id/... name of the LiDAR
    # (ls -l /dev/serial/by-id/) and pass it with lidar_port:=...
    lidar_port = LaunchConfiguration("lidar_port")

    return LaunchDescription([

        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="Serial port of the RPLIDAR C1"
        ),

        # RPLIDAR C1 driver -> /scan (frame "laser")
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 460800,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }]
        ),

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
