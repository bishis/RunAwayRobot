import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Map file to load'
    )
    
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        
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

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'),
                           'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_params.yaml')
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

        # Twist Mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(pkg_dir, 'config', 'twist_mux.yaml')],
            remappings=[
                ('/cmd_vel_out', '/cmd_vel')
            ]
        ),

        # Navigation Controller
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'use_sim_time': False,
                'waypoint_threshold': 0.2,
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