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
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'collision_monitor',
        'global_costmap',
        'local_costmap'
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

    # Costmap nodes - adjust local costmap for transform issues
    start_local_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ])

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

    # Adjust lifecycle manager with increased timeout
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 10.0}])  # Increased bond timeout

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
    ld.add_action(declare_autostart_cmd)

    # Add the actions to launch all of the navigation nodes with proper sequencing
    # First launch costmaps
    ld.add_action(start_global_costmap_cmd)
    ld.add_action(start_local_costmap_cmd)
    
    # Then add a delay before starting other components
    ld.add_action(TimerAction(
        period=2.0,  # 2-second delay
        actions=[
            start_controller_server_cmd,
            start_planner_server_cmd,
            start_recoveries_server_cmd,
            start_bt_navigator_cmd,
            start_collision_monitor_cmd
        ]
    ))
    
    # Add a further delay before starting the lifecycle manager
    ld.add_action(TimerAction(
        period=5.0,  # 5-second delay
        actions=[start_lifecycle_manager_cmd]
    ))

    # Add the navigation controller last
    ld.add_action(TimerAction(
        period=8.0,  # 8-second delay
        actions=[start_navigation_controller_cmd]
    ))

    return ld 