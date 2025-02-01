import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction

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

    # SLAM Toolbox lifecycle manager
    slam_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['slam_toolbox']}]
    )

    # Include the navigation launch file with a delay
    navigation_launch = TimerAction(
        period=3.0,  # 10 second delay to ensure SLAM is running
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_dir, 'launch', 'navigation.launch.py'])
                )
            )
        ]
    )

    # Add image flipper node
    image_flipper_node = Node(
        package='motor_controller',
        executable='image_flipper',
        name='image_flipper',
        output='screen',
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('image_raw_flipped', '/camera/image_raw_flipped')
        ]
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

        # SLAM Toolbox first
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

        # SLAM lifecycle manager
        slam_lifecycle_manager_cmd,
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2_view.rviz')]
        ),

        # Navigation with delay
        navigation_launch,
        # Person Detector
        Node(
            package='motor_controller',
            executable='person_detector',
            name='person_detector',
            output='screen'
        ),

        # Add image flipper
        image_flipper_node
    ])
