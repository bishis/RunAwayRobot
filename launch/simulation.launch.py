from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        
        # Robot simulator with LIDAR
        Node(
            package='motor_controller',
            executable='robot_simulator',
            name='robot_simulator',
            parameters=[{
                'wheel_separation': 0.24,
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
                'laser_range': 5.0,        # 5m range
                'laser_samples': 360,      # 1-degree resolution
                'laser_noise': 0.01,       # 1cm of noise
            }],
            output='screen'
        ),

        # Static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'resolution': 0.05,
                'map_update_interval': 1.0,
                'max_laser_range': 5.0,
                'max_update_rate': 10.0
            }]
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'mapping.rviz')],
        ),
    ]) 