from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('motor_controller').find('motor_controller')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Launch configuration variables specific to simulation
    lifecycle_nodes = [
        'bt_navigator',
        'collision_monitor',
        'planner_server',
        'controller_server',
        'behavior_server',
        'local_costmap',
        'global_costmap'
    ]
    
    # Define separate lifecycle nodes for SLAM
    slam_lifecycle_nodes = ['slam_toolbox']

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_share, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [pkg_share, 'config', 'slam.yaml']),
        description='Full path to the ROS2 parameters file for SLAM')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
        
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # SLAM node
    start_async_slam_toolbox_node_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[configured_slam_params],
        remappings=[
            ('/map', '/map'),
        ])

    # Nodes launching commands
    start_controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params])

    start_planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[configured_params])

    start_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[configured_params])

    start_collision_monitor_cmd = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        output='screen',
        parameters=[configured_params])

    # Costmap nodes
    start_local_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[configured_params])

    start_global_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[configured_params])

    start_recoveries_server_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params])

    # Navigation lifecycle manager
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes}])
                    
    # SLAM lifecycle manager - separate from navigation lifecycle manager
    start_slam_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': slam_lifecycle_nodes,
                    # Add a longer bond timeout for slam_toolbox
                    'bond_timeout': 8.0}])

    # Add this as a proper Node action in the LaunchDescription
    start_navigation_controller_cmd = Node(
        package='motor_controller',
        executable='navigation_controller',
        name='navigation_controller',
        parameters=[{
            'max_linear_speed': 0.07,      
            'max_angular_speed': 0.9,     
            'min_rotation_speed': 0.8,
            'goal_timeout': 30.0,
            'robot_radius': 0.16,
            'safety_margin': 0.3,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('wheel_speeds', '/wheel_speeds'),
            ('map', '/map'),
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Start SLAM toolbox first
    ld.add_action(start_async_slam_toolbox_node_cmd)
    
    # Start SLAM lifecycle manager to activate SLAM
    ld.add_action(start_slam_lifecycle_manager_cmd)
    
    # Add delay before starting navigation components
    ld.add_action(TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            # Add the actions to launch all of the navigation nodes
            start_controller_server_cmd,
            start_planner_server_cmd,
            start_bt_navigator_cmd,
            start_collision_monitor_cmd,
            start_recoveries_server_cmd,
            start_local_costmap_cmd,
            start_global_costmap_cmd,
            start_lifecycle_manager_cmd
        ]
    ))

    # Add the navigation controller last with a delay
    ld.add_action(TimerAction(
        period=4.0,  # 4 second delay to ensure nav system is ready
        actions=[start_navigation_controller_cmd]
    ))

    return ld 