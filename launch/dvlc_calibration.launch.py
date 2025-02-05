from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # LiDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[{
                'frame_id': 'laser',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'scan_mode': 'Standard',
            }],
        ),

        # DVLC calibration node
        Node(
            package='direct_visual_lidar_calibration',
            executable='dvlc_calibration_node',
            name='dvlc_calibration',
            parameters=[{
                'camera_topic': '/camera/image_raw_flipped',
                'camera_info_topic': '/camera/camera_info',
                'lidar_topic': '/scan',
                'camera_frame': 'camera_link',
                'lidar_frame': 'laser',
                'target_frame': 'base_link',
                'max_iterations': 100,
                'min_inliers': 10,
                'ransac_iterations': 100,
                'ransac_threshold': 0.02,
                'publish_debug_image': True
            }],
            output='screen'
        ),
    ]) 