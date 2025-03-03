import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    
    # Launch configuration variables
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

    # First: Launch SLAM Toolbox
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'),
                       'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': os.path.join(pkg_dir, 'config', 'slam.yaml')
        }.items()
    )

    # Second: Launch SLAM lifecycle manager
    slam_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )

    # Third: Launch RF2O Odometry after SLAM is ready
    rf2o_cmd = TimerAction(
        period=2.0,
        actions=[
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
            )
        ]
    )

    # Fourth: Launch Navigation stack after SLAM and odometry are ready
    navigation_cmd = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_dir, 'launch', 'navigation.launch.py'])
                )
            )
        ]
    )

    # Fifth: Launch RViz2 after navigation is ready
    rviz_cmd = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2_view.rviz')]
            )
        ]
    )

    # Finally: Launch camera-related nodes last
    camera_nodes_cmd = TimerAction(
        period=8.0,
        actions=[
            # Image flipper
            Node(
                package='motor_controller',
                executable='image_flipper',
                name='image_flipper',
                output='screen',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('image_raw_flipped', '/camera/image_raw_flipped')
                ]
            ),
            # Person Detector
            Node(
                package='motor_controller',
                executable='person_detector',
                name='person_detector',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments must come first
        declare_use_sim_time_cmd,
        declare_params_file_cmd,

        # Environment variables
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Launch nodes in order with delays
        slam_toolbox_cmd,
        slam_lifecycle_manager_cmd,
        rf2o_cmd,
        navigation_cmd,
        rviz_cmd,
        camera_nodes_cmd
    ])
