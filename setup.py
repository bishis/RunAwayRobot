from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Mazher',
    maintainer_email='your_email@example.com',
    description='Wall following robot with SLAM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = motor_controller.wall_follower:main',
            'hardware_controller = motor_controller.hardware_controller:main',
            'navigation_controller = motor_controller.navigation_controller:main',
            'nav2_hardware_bridge = motor_controller.nav2_hardware_bridge:main',
            'exploration_controller = motor_controller.exploration_controller:main',
            'robot_simulator = motor_controller.robot_simulator:main',
        ],
    },
)

