import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    # Merge your nav2_params.yaml with any overrides you like
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
        # Environment variables for networking, etc.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # 1) Laser Odometry
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
            }],
            output='screen'
        ),

        # 2) SLAM Toolbox for live map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam.yaml')
            }.items()
        ),

        # 3) Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params]
        ),

        # 3) Nav2 Core Nodes (lifecycle-managed)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel')]  # If needed
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

        # 4) Lifecycle Manager: *all* nodes to manage
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                    # If using a 'recoveries_server', list it here too
                ]
            }]
        ),

        # 5) Delay start of your navigation_controller so Nav2 is ACTIVE
        TimerAction(
            period=30.0,  # seconds
            actions=[
                Node(
                    package='motor_controller',
                    executable='navigation_controller',
                    name='navigation_controller',
                    output='screen'
                )
            ]
        ),

        # 6) RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')],
            output='screen'
        ),

        # 7) Static transform (base_link -> laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # 8) Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
    ])
