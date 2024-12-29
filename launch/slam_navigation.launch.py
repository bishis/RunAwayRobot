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
        
        # Odometry processing
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
                'freq': 20.0
            }]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'),
                           'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam.yaml')
            }.items()
        ),

        # Navigation Controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'use_sim_time': False,
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'leg_length': 2.0
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