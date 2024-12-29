import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    # Add arguments for ROS_DOMAIN_ID
    domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS_DOMAIN_ID to use for communication'
    )

    return LaunchDescription([
        domain_id_arg,
        
        # Launch SLAM node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_dir, 'config', 'slam.yaml')],
            output='screen'
        ),
        
        # Launch Navigation Controller (processes LIDAR data and sends movement commands)
        Node(
            package='motor_controller',
            executable='navigation_node',  # We'll create this
            name='navigation_controller',
            parameters=[{
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'leg_length': 2.0
            }],
            output='screen'
        ),
        
        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')]
        )
    ]) 