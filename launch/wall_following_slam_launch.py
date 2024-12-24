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
    
    # Get SLAM params path
    slam_params_path = os.path.join(pkg_dir, 'config', 'slam.yaml')
    
    return LaunchDescription([
        # 1. Launch RPLIDAR (wait 2 seconds)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
            )
        ),

        # 2. Launch RF2O Odometry (wait 2 seconds)
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, 'launch', 'odometry.launch.py')
                    )
                )
            ]
        ),

        # 3. Launch Static Transforms (wait 2 seconds)
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

        # 4. Launch SLAM (wait 2 seconds)
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot4_dir, 'launch', 'slam.launch.py')
                    ),
                    launch_arguments={
                        'params_file': slam_params_path
                    }.items()
                )
            ]
        ),

        # 5. Launch Robot Visualization (wait 2 seconds)
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot4_dir, 'launch', 'view_robot.launch.py')
                    )
                )
            ]
        ),

        # 6. Launch Robot Controller (wait 10 seconds)
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
