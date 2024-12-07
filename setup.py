from setuptools import setup
import os
from glob import glob

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = motor_controller.wall_follower:main',
        ],
    },
)

