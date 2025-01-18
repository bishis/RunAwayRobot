import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

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

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': "false",
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'resolution': 0.05,
                'map_update_interval': 5.0,
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'update_timing': True,
                'publish_period': 0.5,
                'max_update_rate': 10.0,
                'enable_interactive_mode': "false",
                'transform_publish_period': 0.05,
                'use_pose_graph_backend': "true",
                'scan_topic': 'scan',
                'mode': 'mapping'  # Change to 'localization' if you have a map
            }]
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
                'map': os.path.join(pkg_dir, 'maps', 'map.yaml')
            }.items()
        ),

        # Navigation Controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'use_sim_time': False,
                'robot.radius': 0.17
            }]
        ),

        # Twist Mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(pkg_dir, 'config', 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', 'cmd_vel')]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')]
        )
    ]) 