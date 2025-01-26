from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # Include Gazebo simulation launch
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'gazebo_simulation.launch.py')
        ])
    )

    # SLAM toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_laser_range': 5.0,
            'resolution': 0.05,
            'map_update_interval': 1.0,
            'max_update_rate': 10.0
        }]
    )

    # Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Lifecycle manager for Nav2
    lifecycle_nodes = ['controller_server',
                      'planner_server',
                      'recoveries_server',
                      'bt_navigator',
                      'waypoint_follower']

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                   {'autostart': True},
                   {'node_names': lifecycle_nodes}]
    )

    # Wall follower node
    wall_follower = Node(
        package='motor_controller',
        executable='wall_follower',
        name='wall_follower',
        output='screen'
    )

    # Exploration controller
    exploration_controller = Node(
        package='motor_controller',
        executable='exploration_controller',
        name='exploration_controller',
        output='screen'
    )

    # Nav2 hardware bridge
    nav2_hardware_bridge = Node(
        package='motor_controller',
        executable='nav2_hardware_bridge',
        name='nav2_hardware_bridge',
        output='screen'
    )

    # RViz
    rviz_config = os.path.join(pkg_dir, 'config', 'mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Simulation
        gazebo_simulation,
        
        # SLAM and Navigation
        slam_toolbox,
        nav2_launch,
        lifecycle_manager,
        
        # Custom controllers
        wall_follower,
        exploration_controller,
        nav2_hardware_bridge,
        
        # Visualization
        rviz
    ]) 