from setuptools import find_packages, setup

package_name = 'au516_nav'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/au516_nav/launch', ['launch/slam_launch.py', 'launch/display_launch.py',
                                    'launch/robot_launch.py']),
        ('share/au516_nav/config', ['config/slam.yaml', 'config/slam.rviz',
                                    'config/nav.yaml', 'config/navigation.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='AU516 robot: Pico link, odometry, SLAM and autonomous LiDAR navigation',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_viewer = au516_nav.scan_viewer:main',
            'conversion_2D = au516_nav.conversion_2D:main',
            'serial_bridge = au516_nav.serial_bridge:main',
            'odometry_node = au516_nav.odometry_node:main',
            'move = au516_nav.move:main',
            'lidar_map = au516_nav.Lidar_map:main',
            'local_navigator = au516_nav.local_navigator:main',
            'drive_mux = au516_nav.drive_mux:main',
            'sim_2d = au516_nav.sim_2d:main',
        ],
    },
)
