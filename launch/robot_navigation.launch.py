from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'max_laser_range': 20.0,
                'resolution': 0.05
            }]
        ),

        # Navigation Stack
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='nav2_stack',
            parameters=[{
                'use_sim_time': False,
                'bt_navigator_plugins': ["nav2_navigate_to_pose_bt"],
                'global_costmap.robot_radius': 0.17,
                'local_costmap.robot_radius': 0.17
            }]
        ),

        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'initial_pose.x': 0.0,
                'initial_pose.y': 0.0,
                'initial_pose.z': 0.0,
                'scan_topic': 'scan'
            }]
        )
    ]) 