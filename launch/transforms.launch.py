from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First publish map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Then odom->base_link is published by RF2O
        
        # Finally base_link->laser is published by the robot controller
    ]) 