"""Matériel : RPLIDAR C1 (/scan), TF statique base_link -> laser, pont série Pico (/odom + TF)."""

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
    args = [
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'laser_x', default_value='0.22'
        ),  # mêmes valeurs que vehicle.laser_*
        DeclareLaunchArgument('laser_z', default_value='0.10'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='0.0'
        ),  # 3.14159 si LiDAR monté à l'envers
        DeclareLaunchArgument('dry_run', default_value='false'),
    ]
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'
            )
        ),
        launch_arguments={'serial_port': lc('lidar_port'), 'frame_id': 'laser'}.items(),
        condition=IfCondition(lc('lidar')),
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x',
            lc('laser_x'),
            '--y',
            '0',
            '--z',
            lc('laser_z'),
            '--yaw',
            lc('laser_yaw'),
            '--frame-id',
            'base_link',
            '--child-frame-id',
            'laser',
        ],
    )
    bridge = Node(
        package='au516_nav',
        executable='pico_bridge',
        name='pico_bridge',
        output='screen',
        parameters=[os.path.join(share, 'config', 'pico_bridge.yaml'), {'dry_run': lc('dry_run')}],
    )
    return LaunchDescription(args + [lidar, static_tf, bridge])
