from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Camera parameters
        DeclareLaunchArgument(
            'camera1_index',
            default_value='0',
            description='Index for camera 1'
        ),
        DeclareLaunchArgument(
            'camera2_index',
            default_value='1',
            description='Index for camera 2'
        ),
        
        # Camera controller node
        Node(
            package='motor_controller',
            executable='camera_controller',
            name='dual_camera_controller',
            output='screen',
            parameters=[{
                'camera1_index': LaunchConfiguration('camera1_index'),
                'camera2_index': LaunchConfiguration('camera2_index'),
                'frame_rate': 30,
                'image_width': 640,
                'image_height': 480
            }]
        ),
        
        # TF2 transforms for cameras
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera1_broadcaster',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera1_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera2_broadcaster',
            arguments=['-0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera2_link']
        )
    ]) 