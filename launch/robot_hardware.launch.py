import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    return LaunchDescription([
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

        # Hardware controller for motors
        Node(
            package='motor_controller',
            executable='hardware_controller',
            name='hardware_controller',
            parameters=[{
                'speed_channel_pin': 12,     # GPIO12 for forward/reverse
                'turn_channel_pin': 13,      # GPIO13 for left/right turning
                'max_linear_speed': 0.1,     # m/s
                'max_angular_speed': 1.0,    # rad/s
            }],
            output='screen'
        ),

        # Basic transform publisher for lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # Add Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'frame_rate': 30.0,
                'camera_frame_id': 'camera_link',
                'output_encoding': 'rgb8'
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),  # Change output topic
            ]
        ),

        # Add static camera transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[{'use_sim_time': False}]  # Make sure this is false
        ),

        # Add image compression node
        Node(
            package='image_transport',
            executable='republish',
            name='image_compress',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image_raw'),
                ('out/compressed', '/camera/image_raw/compressed'),
            ]
        ),

        # Add robot visualizer
        Node(
            package='motor_controller',
            executable='robot_visualizer',
            name='robot_visualizer',
            output='screen'
        ),
    ])
