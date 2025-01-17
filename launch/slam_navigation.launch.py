import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_dir, 'maps', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'))
    
    return LaunchDescription([
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
            
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
            description='Full path to map yaml file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'),
                           'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam.yaml')
            }.items()
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_yaml_file
            }.items()
        ),

        # Navigation Controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'use_sim_time': use_sim_time,
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