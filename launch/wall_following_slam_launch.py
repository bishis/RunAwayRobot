import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the Cartographer config file
    cartographer_config_dir = os.path.join(
        get_package_share_directory('motor_controller'),
        'config'
    )
    configuration_basename = 'cartographer_config.lua'

    return LaunchDescription([
        # Static Transform Publisher for map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Static Transform Publisher for base_link->laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # Launch Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'publish_frame_projected_2d': True,
                'num_laser_scans': 1,
                'num_multi_echo_laser_scans': 0,
                'num_subdivisions_per_laser_scan': 1,
                'num_point_clouds': 0,
                'publish_tracked_pose': True,
                'publish_tracked_scan': True,
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                ('imu', '')  # Explicitly disable IMU
            ]
        ),

        # Add a delay before starting the occupancy grid node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'resolution': 0.05,
                        'publish_period_sec': 1.0,
                        'track_unknown_space': True,
                        'publish_full_map': True,
                    }],
                    remappings=[
                        ('map', '/map')
                    ]
                ),
            ]
        ),

        # Launch RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # Launch Wall Follower Node
        Node(
            package='motor_controller',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        )
    ])
