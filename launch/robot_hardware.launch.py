from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Add arguments for ROS_DOMAIN_ID to ensure both machines communicate
    domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS_DOMAIN_ID to use for communication'
    )

    return LaunchDescription([
        domain_id_arg,
        
        # Launch RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),
        
        # Launch Motor Controller node (simplified for hardware control only)
        Node(
            package='motor_controller',
            executable='robot_hardware_node',  # We'll create this
            name='robot_hardware',
            parameters=[{
                'left_motor_pin': 18,
                'right_motor_pin': 12,
            }],
            output='screen'
        ),
        
        # Launch RF2O Odometry
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
                'freq': 10.0
            }]
        ),
    ]) 