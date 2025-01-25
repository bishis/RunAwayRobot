from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    return LaunchDescription([
        # Navigation controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
                'linear_threshold': 0.01,
                'angular_threshold': 0.02,
            }],
            output='screen'
        ),
        
        # Robot simulator instead of hardware controller
        Node(
            package='motor_controller',
            executable='robot_simulator',
            name='robot_simulator',
            parameters=[{
                'wheel_separation': 0.24,
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
            }],
            output='screen'
        ),

        # Include navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
            ])
        ),
    ]) 