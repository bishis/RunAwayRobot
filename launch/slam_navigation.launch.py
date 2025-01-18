import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': 'false',
        'autostart': 'true'
    }
    
    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

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

        # Nav2 Core
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]}
            ]
        ),

        # Add delay before starting navigation controller
        TimerAction(
            period=15.0,  # Increased delay to ensure Nav2 is fully initialized
            actions=[
                Node(
                    package='motor_controller',
                    executable='navigation_controller',
                    name='navigation_controller',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'robot.radius': 0.17,
                        'robot.safety_margin': 0.10
                    }]
                )
            ]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')]
        )
    ]) 