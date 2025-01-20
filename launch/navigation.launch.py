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
    
    # Launch configuration variables specific to simulation
    lifecycle_nodes = [
        'local_costmap',  # Start costmaps first
        'global_costmap',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'collision_monitor'
    ]

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

    start_behavior_server_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params])

    # Wrap node launches in TimerAction for sequential startup
    start_local_costmap_with_delay = TimerAction(
        period=2.0,  # 2 second delay before starting
        actions=[start_local_costmap_cmd]
    )

    start_global_costmap_with_delay = TimerAction(
        period=3.0,  # 3 second delay
        actions=[start_global_costmap_cmd]
    )

    start_controller_with_delay = TimerAction(
        period=4.0,
        actions=[start_controller_server_cmd]
    )

    start_planner_with_delay = TimerAction(
        period=5.0,
        actions=[start_planner_server_cmd]
    )

    start_behavior_with_delay = TimerAction(
        period=6.0,
        actions=[start_behavior_server_cmd]
    )

    start_bt_with_delay = TimerAction(
        period=7.0,
        actions=[start_bt_navigator_cmd]
    )

    start_collision_with_delay = TimerAction(
        period=8.0,
        actions=[start_collision_monitor_cmd]
    )

    start_lifecycle_manager_with_delay = TimerAction(
        period=10.0,  # Give plenty of time for other nodes to start
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                        'bond_timeout': 10.0,          # Increased timeout
                        'attempt_respawn': True,
                        'bond_respawn_max_duration': 15.0,
                        'bond_respawn_interval': 1.0}])]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the actions with delays
    ld.add_action(start_local_costmap_with_delay)
    ld.add_action(start_global_costmap_with_delay)
    ld.add_action(start_controller_with_delay)
    ld.add_action(start_planner_with_delay)
    ld.add_action(start_behavior_with_delay)
    ld.add_action(start_bt_with_delay)
    ld.add_action(start_collision_with_delay)
    ld.add_action(start_lifecycle_manager_with_delay)

    return ld 