import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Static Transform Publisher for map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Static Transform Publisher for base_link->laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # Launch RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # Launch RTAB-Map SLAM
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                
                # Frame settings
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                
                # Disable odometry as we don't have wheel encoders
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan': True,
                'subscribe_scan_cloud': False,
                'subscribe_odom_info': False,
                
                # SLAM settings for LIDAR-only operation
                'Reg/Strategy': '1',           # ICP
                'Reg/Force3DoF': 'true',      # 2D SLAM
                'GridGlobal/MinSize': '20',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/AngularUpdate': '0.1',   # Update map when robot rotates by 0.1 rad
                'RGBD/LinearUpdate': '0.1',    # Update map when robot moves by 0.1 m
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Grid/FromDepth': 'false',
                'Grid/RayTracing': 'true',
                'GridGlobal/OccupancyThr': '0.65',
                'GridGlobal/ProbHit': '0.7',
                'GridGlobal/ProbMiss': '0.4',
                
                # ICP parameters
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                
                # Loop closure
                'RGBD/LoopClosureReextractFeatures': 'true',
                'Optimizer/Strategy': '1',     # TORO
                'Optimizer/Iterations': '100',
                'Optimizer/Epsilon': '0.001',
            }]
        ),

        # Launch Mobile Robot Controller Node
        Node(
            package='motor_controller',
            executable='wall_follower',
            name='mobile_robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'turn_speed': 1.0,
                'linear_speed': 0.3,
            }]
        ),
    ])
