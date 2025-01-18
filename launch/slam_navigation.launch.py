import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')
    
    return LaunchDescription([
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # RF2O Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }]
        ),

        # Transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'resolution': 0.05,
                'map_update_interval': 5.0,
                'max_laser_range': 12.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'scan_topic': '/scan',
                'mode': 'mapping',
                'debug_logging': True,
                'throttle_scans': 1,
                'transform_publish_period': 0.05,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_time_interval': 0.5,
                'enable_interactive_mode': False,
                'transform_timeout': 0.2,
                'publish_period': 5.0
            }]
        ),

        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
                'map': ''
            }.items()
        ),

        # Navigation Controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'use_sim_time': False,
                'waypoint_threshold': 0.2,
                'leg_length': 0.5,
                'safety_radius': 0.25,
                'num_waypoints': 8,
                'robot.radius': 0.17
            }]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')]
        )
    ]) 