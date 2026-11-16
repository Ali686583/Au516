from glob import glob
from setuptools import find_packages, setup

package_name = 'au516_nav'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='Navigation autonome AU516 : SLAM Toolbox + passages LiDAR + contrôle Ackermann',
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
            'encoder_reader = au516_nav.encoder_reader:main',
            'pico_bridge = au516_nav.pico_bridge:main',
            'local_navigator = au516_nav.local_navigator:main',
            'sim_2d = au516_nav.sim_2d:main',
        ],
    },
)
