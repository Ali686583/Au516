
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    # ==========================================
    # 1. Package directories
    # ==========================================

    slam_dir = get_package_share_directory(
        "slam_toolbox"
    )

    au516_dir = get_package_share_directory(
        "au516_nav"
    )

    slam_launch = os.path.join(
        slam_dir,
        "launch",
        "online_async_launch.py"
    )

    slam_config = os.path.join(
        au516_dir,
        "config",
        "slam.yaml"
    )

    # ==========================================
    # 2. RPLIDAR C1 configuration
    # ==========================================

    # Stable USB identifier of the LiDAR.
    # The Pico FTDI cable has a different ID.
    DEFAULT_LIDAR_PORT = (
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102N_USB_to_UART_"
        "Bridge_Controller_"
        "b0b32cfd4170f01181c594301045c30f"
        "-if00-port0"
    )

    lidar_port = LaunchConfiguration("lidar_port")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")

    return LaunchDescription([

        # ======================================
        # 3. Launch arguments
        # ======================================

        DeclareLaunchArgument(
            "lidar_port",
            default_value=DEFAULT_LIDAR_PORT,
            description="Serial port of the RPLIDAR C1"
        ),

        DeclareLaunchArgument(
            "lidar_scan_mode",
            default_value="DenseBoost",
            description="LiDAR scan mode: DenseBoost or Standard"
        ),

        # ======================================
        # 4. RPLIDAR C1 driver
        # Publishes /scan
        # ======================================

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
                "scan_mode": lidar_scan_mode,
            }]
        ),

        # ======================================
        # 5. Pico communication
        # Reads encoders and sends commands
        # ======================================

        Node(
            package="au516_nav",
            executable="serial_bridge",
            output="screen"
        ),

        # ======================================
        # 6. LiDAR static transform
        # base_link -> laser
        # ======================================

        # IMPORTANT:
        # These values are placeholders.
        # Replace them with measured LiDAR
        # position and orientation.

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--yaw", "0.0",
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "laser"
            ],
            output="screen"
        ),

        # ======================================
        # 7. Odometry
        # Publishes /odom and odom -> base_link
        # ======================================

        Node(
            package="au516_nav",
            executable="odometry_node",
            output="screen"
        ),

        # ======================================
        # 8. SLAM Toolbox
        # Mapping and localization correction
        # ======================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                slam_launch
            ),
            launch_arguments={
                "slam_params_file": slam_config,
                "use_sim_time": "false"
            }.items()
        )

    ])
