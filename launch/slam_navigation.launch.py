import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )

    return LaunchDescription([
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Launch arguments
        declare_use_sim_time_cmd,
        declare_params_file_cmd,

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

        # SLAM Toolbox in online async mode
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'),
                           'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'slam_params_file': str(os.path.join(pkg_dir, 'config', 'slam.yaml'))
            }.items()
        ),

        # Nav2 Navigation Stack (without map server since SLAM provides the map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'params_file': str(params_file),
                'use_composition': 'False'
            }.items()
        ),

        # Collision Monitor
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'cmd_vel_in_topic': 'cmd_vel',
                'cmd_vel_out_topic': 'cmd_vel_smoothed',
                'transform_tolerance': 0.5,
                'source_timeout': 1.0,
                'stop_on_collision': True,
                'observation_sources': 'scan',
                'polygons': [],
                'scan': {
                    'type': 'scan',
                    'topic': '/scan',
                    'min_height': 0.1,
                    'max_height': 0.5,
                    'clearing': True,
                    'marking': True,
                    'data_type': 'LaserScan'
                }
            }]
        ),

        # Nav2 Hardware Bridge
        Node(
            package='motor_controller',
            executable='nav2_hardware_bridge',
            name='nav2_hardware_bridge',
            parameters=[{
                'max_linear_speed': 1.0,
                'max_angular_speed': 1.0
            }]
        ),

        # RViz2 for visualization with Nav2 config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2_view.rviz')]
        )
    ]) 