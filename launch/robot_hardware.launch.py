from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set ROS_DOMAIN_ID
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        
        # Set ROS_LOCALHOST_ONLY=0 to allow network communication
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        
        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # RF2O Odometry node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }]
        ),

        # Motor Controller node (simplified version that only handles motor commands)
        Node(
            package='motor_controller',
            executable='hardware_controller',  # We'll create this
            name='hardware_controller',
            parameters=[{
                'left_pin': 18,
                'right_pin': 12
            }]
        ),

        # Basic transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        )
    ]) 