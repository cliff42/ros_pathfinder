from setuptools import find_packages, setup

package_name = 'ros_pathfinder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cliff42',
    maintainer_email='chris.cliff@shaw.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "talker = ros_pathfinder.imu_publisher:main",
            "listener = ros_pathfinder.test_subscriber:main",
            "motor_controller = ros_pathfinder.motor_controller:main",
            "teleop_node = ros_pathfinder.teleop:main",
            "odom_node = ros_pathfinder.odometry_publisher:main",
            "lidar_static_tf = ros_pathfinder.lidar_static_transform:main",
            "occupancy = ros_pathfinder.occupancy:main",
            "localization = ros_pathfinder.localization:main"
        ],
    },
)
