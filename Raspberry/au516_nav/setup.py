from setuptools import find_packages, setup

package_name = 'au516_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='This package contains navigation-related nodes for the AU516 robot',
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
        ],
    },
)
