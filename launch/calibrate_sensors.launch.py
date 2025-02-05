from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link',
                'video_device': '/dev/video0',
                'pixel_format': 'YUYV',
                'publish_raw': True,
                'camera_info_url': 'package://motor_controller/config/camera_info.yaml',
            }],
        ),

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

        # Calibration node
        Node(
            package='lidar_camera_calibration',
            executable='calibrate_sensors',
            name='calibrate_sensors',
            parameters=[{
                'camera_topic': '/camera/image_raw',
                'lidar_topic': '/scan',
                'camera_info_topic': '/camera/camera_info',
                'checkerboard_squares_x': 8,
                'checkerboard_squares_y': 6,
                'square_size': 0.025,
                'max_range': 4.0,
                'min_range': 0.1,
            }],
            output='screen'
        ),
    ]) 