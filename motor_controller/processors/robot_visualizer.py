#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # Robot dimensions (in meters)
        self.robot_length = 0.29  # 29cm
        self.robot_width = 0.34   # 34cm
        
        # Create publisher for robot footprint visualization
        self.footprint_pub = self.create_publisher(
            MarkerArray, 
            'robot_footprint', 
            10
        )
        
        # Create timer to publish footprint
        self.create_timer(0.1, self.publish_footprint)  # 10Hz update rate

    def publish_footprint(self):
        """Publish robot footprint visualization"""
        marker_array = MarkerArray()
        
        # Create marker for robot footprint
        marker = Marker()
        marker.header.frame_id = 'base_link'  # Attach to robot base
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_footprint'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.02  # Line width
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow, semi-transparent
        
        # Create points for rectangle corners
        half_length = self.robot_length / 2.0
        half_width = self.robot_width / 2.0
        
        points = [
            Point(x=half_length, y=half_width, z=0.0),    # Front right
            Point(x=half_length, y=-half_width, z=0.0),   # Front left
            Point(x=-half_length, y=-half_width, z=0.0),  # Back left
            Point(x=-half_length, y=half_width, z=0.0),   # Back right
            Point(x=half_length, y=half_width, z=0.0),    # Back to front right to close
        ]
        
        marker.points = points
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        
        marker_array.markers.append(marker)
        self.footprint_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 