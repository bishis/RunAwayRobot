import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('motor_controller')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    
    # Get Cartographer configuration
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    configuration_basename = 'cartographer.lua'

    return LaunchDescription([
        # 1. Launch RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
            )
        ),

        # 2. Launch RF2O Laser Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'freq': 20.0
            }]
        ),

        # 3. Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # 4. Launch Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # 5. Launch Cartographer Occupancy Grid
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        # 6. Launch Robot Controller
        Node(
            package='motor_controller',
            executable='wall_follower',
            name='mobile_robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'turn_speed': 1.0,
                'linear_speed': 0.3,
            }]
        )
    ])
