import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('motor_controller')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    turtlebot4_dir = get_package_share_directory('turtlebot4_navigation')
    
    return LaunchDescription([
        # 1. Launch RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
            )
        ),

        # 2. Wait then launch RF2O Odometry
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    output='screen',
                    parameters=[{
                        'laser_scan_topic': '/scan',
                        'odom_topic': '/odom_rf2o',
                        'publish_tf': True,
                        'base_frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'freq': 20.0
                    }]
                )
            ]
        ),

        # 3. Wait then launch transforms
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, 'launch', 'transforms.launch.py')
                    )
                )
            ]
        ),

        # 4. Wait then launch TurtleBot4 SLAM
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot4_dir, 'launch', 'slam.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'sync': 'true'
                    }.items()
                )
            ]
        ),

        # 5. Wait then launch RViz2
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')]
                )
            ]
        ),

        # 6. Wait then launch Robot Controller
        TimerAction(
            period=10.0,
            actions=[
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
            ]
        )
    ])
