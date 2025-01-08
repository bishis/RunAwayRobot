from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the keyboard_control argument
    keyboard_control_arg = DeclareLaunchArgument(
        'keyboard_control',
        default_value='false',
        description='Enable keyboard (WASD) control'
    )
    
    pkg_dir = get_package_share_directory('motor_controller')
    
    # Create the launch description
    ld = LaunchDescription([
        # Add the keyboard control argument
        keyboard_control_arg,
        
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Include the working RPLIDAR launch configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rplidar_ros'),
                           'launch', 'rplidar_a1_launch.py')
            ])
        ),

        # Simple hardware interface for motors
        Node(
            package='motor_controller',
            executable='hardware_controller',
            name='hardware_controller',
            parameters=[{
                'left_pin': 18,
                'right_pin': 12
            }]
        ),

        # Basic transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # Keyboard control node (conditionally launched)
        Node(
            package='motor_controller',
            executable='keyboard_control',
            name='keyboard_control',
            condition=IfCondition(LaunchConfiguration('keyboard_control'))
        )
    ])
    
    return ld 