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

        # Nav2 Core (in this specific order)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params,
                {
                    'use_sim_time': False,
                    'global_frame': 'map',
                    'robot_base_frame': 'base_link',
                    'transform_tolerance': 0.1,
                    'default_nav_to_pose_bt_xml': os.path.join(
                        get_package_share_directory('nav2_bt_navigator'),
                        'behavior_trees',
                        'navigate_w_replanning_and_recovery.xml'
                    ),
                    'plugin_lib_names': [
                        'nav2_compute_path_to_pose_action_bt_node',
                        'nav2_follow_path_action_bt_node',
                        'nav2_back_up_action_bt_node',
                        'nav2_spin_action_bt_node',
                        'nav2_wait_action_bt_node',
                        'nav2_clear_costmap_service_bt_node',
                        'nav2_is_stuck_condition_bt_node',
                        'nav2_goal_reached_condition_bt_node',
                        'nav2_goal_updated_condition_bt_node',
                        'nav2_initial_pose_received_condition_bt_node',
                        'nav2_reinitialize_global_localization_service_bt_node',
                        'nav2_rate_controller_bt_node',
                        'nav2_distance_controller_bt_node',
                        'nav2_speed_controller_bt_node',
                        'nav2_recovery_node_bt_node',
                        'nav2_pipeline_sequence_bt_node',
                        'nav2_round_robin_node_bt_node',
                        'nav2_transform_available_condition_bt_node',
                        'nav2_time_expired_condition_bt_node',
                        'nav2_distance_traveled_condition_bt_node',
                        'nav2_single_trigger_bt_node'
                    ]
                }]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
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

        # Start lifecycle manager first
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['bt_navigator'],  # Start with just bt_navigator
                'activate_lifecycle_nodes': True,
                'manage_lifecycle_nodes': True,
                'attempt_respawn_on_failure': True,
                'attempt_respawn_max_tries': 3,
                'bond_disable_heartbeat_timeout': True
            }]
        ),

        # Second lifecycle manager for other nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server'
                ],
                'activate_lifecycle_nodes': True,
                'manage_lifecycle_nodes': True
            }]
        ),

        # Add delay before starting navigation controller
        TimerAction(
            period=40.0,  # Increased delay to ensure Nav2 is fully initialized
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
        ),

        # Add static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # Add robot state publisher if not already present
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ]) 