import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('motor_controller')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    turtlebot4_dir = get_package_share_directory('turtlebot4_navigation')
    
    # Declare the role argument
    declare_role_arg = DeclareLaunchArgument(
        'role',
        default_value='pi',
        description='Role of this machine (pi or mac)'
    )
    
    role = LaunchConfiguration('role')
    
    return LaunchDescription([
        declare_role_arg,

        # === Raspberry Pi Nodes ===
        # 1. Launch RPLIDAR (Pi only)
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            condition=IfCondition(f"{role} == 'pi'")
        ),

        # 2. Launch Robot Controller (Pi only)
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
            }],
            condition=IfCondition(f"{role} == 'pi'")
        ),

        # === Mac Nodes ===
        # 3. Launch RF2O Odometry (Mac only)
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
                'init_pose_from_topic': '',
                'freq': 10.0,
                'verbose': False
            }],
            condition=IfCondition(f"{role} == 'mac'")
        ),

        # 4. Launch transforms (Mac only)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(f"{role} == 'mac'")
        ),

        # 5. Launch SLAM (Mac only)
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtlebot4_dir, 'launch', 'slam.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'sync': 'true',
                        'slam_params_file': os.path.join(pkg_dir, 'config', 'slam.yaml')
                    }.items()
                )
            ],
            condition=IfCondition(f"{role} == 'mac'")
        ),

        # 6. Launch RViz2 (Mac only)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_view.rviz')],
                    condition=IfCondition(f"{role} == 'mac'")
                )
            ]
        )
    ]) 