"""
Single launch file of the AU516 robot: sensors, Pico, odometry, SLAM, autonomous navigation.

On the Pi:
    ros2 launch au516_nav robot_launch.py      # the navigation waits for /nav/enable
    ros2 run au516_nav move                    # 2nd terminal: keyboard, "n" = autonomous
Without the robot (simulated LiDAR and Pico, same nodes as on the robot):
    ros2 launch au516_nav robot_launch.py sim:=true autostart:=true scenario:=ring

Data flow (topics):
    RPLIDAR C1 --/scan--> slam_toolbox --/map, TF map->odom--+
         |                                                   v
         +------------------/scan--------------------> local_navigator --/cmd_vel--+
    Pico <--serial--> serial_bridge --/encoders, /steering_pwm--> odometry_node     |
                          ^             (/odom, TF odom->base_link)                 |
                          +--/cmd_drive_mux-- drive_mux <--/cmd_drive-- move        |
                                                  ^------------------------------- +
slam_launch.py and display_launch.py are unchanged and still work on their own.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Stable USB names (same as slam_launch.py and serial_bridge.py)
DEFAULT_LIDAR_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102N_USB_to_UART_'
    'Bridge_Controller_'
    'b0b32cfd4170f01181c594301045c30f'
    '-if00-port0'
)
DEFAULT_PICO_PORT = '/dev/serial/by-id/usb-FTDI_TTL232R_FTBG33U7-if00-port0'
FAKE_PICO_PORT = '/tmp/au516_fake_pico'  # created by sim_2d when sim:=true

ARGUMENTS = [
    ('sim', 'false', 'true: simulated robot (sim_2d emulates the LiDAR and the Pico)'),
    ('scenario', 'ring', 'sim_2d world: ring, A_straight, B_left, D_zigzag, G_dead_end...'),
    ('lidar_port', DEFAULT_LIDAR_PORT, 'Serial port of the RPLIDAR C1'),
    ('lidar_scan_mode', 'DenseBoost', 'LiDAR scan mode: DenseBoost or Standard'),
    ('pico_port', DEFAULT_PICO_PORT, 'Serial port of the Pico (FTDI cable)'),
    ('laser_x', '0.22', 'LiDAR position in base_link (m from the rear axle), TO MEASURE'),
    ('laser_y', '0.0', 'LiDAR lateral offset (m)'),
    ('laser_z', '0.10', 'LiDAR height (m)'),
    ('laser_yaw', '0.0', 'LiDAR yaw (rad), 3.14159 if mounted backwards'),
    ('slam', 'true', 'Run slam_toolbox (map + pose correction)'),
    ('navigation', 'true', 'Run local_navigator and drive_mux'),
    ('autostart', 'false', 'Navigation drives right away (false: wait for /nav/enable)'),
    ('v_max', '0.6', 'Max speed of the navigation (m/s), the car tops at ~0.81'),
    ('web_viewer', 'true', 'Lidar_map web page on http://<pi>:<web_port>'),
    ('web_port', '8000', 'Port of the web viewer'),
    ('rviz', 'false', 'Open RViz with the navigation view (needs a screen)'),
]


def is_true(text):
    return text.strip().lower() in ('true', '1', 'yes', 'on')


def build(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}
    share = get_package_share_directory('au516_nav')
    sim = is_true(arg['sim'])
    slam = is_true(arg['slam'])
    navigation = is_true(arg['navigation'])

    # One set of LiDAR values for the TF (slam_toolbox) and for the navigation geometry
    laser = {k: float(arg['laser_' + k]) for k in ('x', 'y', 'z', 'yaw')}
    vehicle = {
        'vehicle.laser_x': laser['x'],
        'vehicle.laser_y': laser['y'],
        'vehicle.laser_yaw': laser['yaw'],
    }
    actions = []

    # 1. Sensors: real RPLIDAR, or the simulator (LiDAR + Pico on a fake serial port)
    if sim:
        actions.append(Node(
            package='au516_nav',
            executable='sim_2d',
            name='sim_2d',
            output='screen',
            parameters=[{
                'scenario': arg['scenario'],
                'emulate_pico': True,
                'pico_link': FAKE_PICO_PORT,
                **vehicle,
            }],
        ))
        pico_port = FAKE_PICO_PORT
    else:
        actions.append(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'channel_type': 'serial',
                'serial_port': arg['lidar_port'],
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': arg['lidar_scan_mode'],
            }],
        ))
        pico_port = arg['pico_port']

    # 2. TF base_link -> laser
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', str(laser['x']), '--y', str(laser['y']), '--z', str(laser['z']),
            '--yaw', str(laser['yaw']), '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
        output='screen',
    ))

    # 3. Pico link. With the navigation, its input comes from drive_mux only.
    #    respawn: if the port is missing (cable, simulator not ready) it retries every 2 s.
    serial_bridge = Node(
        package='au516_nav',
        executable='serial_bridge',
        output='screen',
        parameters=[{'port': pico_port}],
        remappings=[('/cmd_drive', '/cmd_drive_mux')] if navigation else [],
        respawn=True,
        respawn_delay=2.0,
    )
    actions.append(TimerAction(period=2.0, actions=[serial_bridge]) if sim else serial_bridge)

    # 4. Odometry: /odom and TF odom -> base_link
    actions.append(Node(
        package='au516_nav',
        executable='odometry_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    ))

    # 5. SLAM Toolbox, same include and configuration as slam_launch.py
    if slam:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
            )),
            launch_arguments={
                'slam_params_file': os.path.join(share, 'config', 'slam.yaml'),
                'use_sim_time': 'false',
                # The included file has its own 'autostart' argument (lifecycle of slam_toolbox):
                # without this line it would inherit the navigation one and never activate.
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ))

    # 6. Autonomous navigation and command arbitration
    if navigation:
        actions.append(Node(
            package='au516_nav',
            executable='local_navigator',
            name='local_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                os.path.join(share, 'config', 'nav.yaml'),
                {
                    'autostart': is_true(arg['autostart']),
                    'use_map': slam,
                    'nav.v_max': float(arg['v_max']),
                    **vehicle,
                },
            ],
        ))
        actions.append(Node(
            package='au516_nav',
            executable='drive_mux',
            name='drive_mux',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'wheelbase': 0.165, 'output_topic': '/cmd_drive_mux'}],
        ))
    else:
        actions.append(LogInfo(msg='navigation:=false -> move drives serial_bridge directly'))

    # 7. Visualisation
    if is_true(arg['web_viewer']):
        actions.append(Node(
            package='au516_nav',
            executable='lidar_map',
            name='lidar_map',
            output='screen',
            parameters=[{'port': int(arg['web_port'])}],
        ))
    if is_true(arg['rviz']):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'config', 'navigation.rviz')],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default, description=text)
            for name, default, text in ARGUMENTS
        ]
        + [OpaqueFunction(function=build)]
    )
