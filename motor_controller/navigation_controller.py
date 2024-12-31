#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
from enum import Enum

class RobotState(Enum):
    ROTATING = 1
    MOVING = 2
    STOPPED = 3

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('waypoint_threshold', 0.3)
        self.declare_parameter('leg_length', 2.0)
        self.declare_parameter('safety_radius', 0.5)
        
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
        self.safety_radius = self.get_parameter('safety_radius').value
        
        # Robot state
        self.state = RobotState.STOPPED
        self.current_pose = None
        self.latest_scan = None  # Add LIDAR data storage
        self.current_waypoint_index = 0
        self.waypoints = self.generate_waypoints()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)  # Add LIDAR subscription
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        self.publish_waypoints()
        
        self.get_logger().info('Navigation controller initialized')
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')

    def generate_waypoints(self):
        """Generate square wave waypoints."""
        points = []
        x, y = 0.0, 0.0
        
        # Generate square wave pattern
        for i in range(4):
            point = Point()
            if i % 2 == 0:
                y += self.leg_length
            else:
                x += self.leg_length
            point.x, point.y = x, y
            points.append(point)
            self.get_logger().info(f'Waypoint {i}: ({point.x}, {point.y})')
        
        return points

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
        marker_array = MarkerArray()
        
        # All waypoints
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = 'map'
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
        waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.points = self.waypoints
        marker_array.markers.append(waypoint_marker)
        
        # Current target
        if self.current_waypoint_index < len(self.waypoints):
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'current_target'
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position = self.waypoints[self.current_waypoint_index]
            target_marker.pose.orientation.w = 1.0
            target_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(target_marker)
        
        self.waypoint_pub.publish(marker_array)

    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg

    def check_obstacles(self):
        """Check for obstacles in front of the robot."""
        if not self.latest_scan:
            return False
        
        # Check front sector (150° to 210°)
        start_idx = int(150 * len(self.latest_scan.ranges) / 360)
        end_idx = int(210 * len(self.latest_scan.ranges) / 360)
        
        for i in range(start_idx, end_idx):
            if i < len(self.latest_scan.ranges):
                range_val = self.latest_scan.ranges[i]
                if 0.1 < range_val < self.safety_radius:
                    return True
        return False

    def control_loop(self):
        if not self.current_pose:
            self.get_logger().warn('Waiting for pose data...')
            return

        self.publish_waypoints()

        # Check for obstacles first
        if self.check_obstacles():
            self.get_logger().warn('Obstacle detected! Stopping.')
            self.stop_robot()
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            return

        current_target = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance and angle to target
        dx = current_target.x - self.current_pose.position.x
        dy = current_target.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_diff = target_angle - current_angle
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi

        # Debug info
        self.get_logger().info(
            f'State: {self.state.name}, Target: {self.current_waypoint_index}, '
            f'Distance: {distance:.2f}, Angle diff: {math.degrees(angle_diff):.1f}°'
        )

        # Create command message
        cmd = Twist()

        # State machine
        if distance < self.waypoint_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            self.state = RobotState.ROTATING
            self.stop_robot()
            return

        # If angle is too large, rotate in place
        if abs(angle_diff) > math.radians(20):
            self.state = RobotState.ROTATING
            cmd.angular.z = -2.0 if angle_diff > 0 else 2.0  # Full speed rotation
            self.get_logger().info(f'Rotating with angular.z = {cmd.angular.z}')
        else:
            # Move forward
            self.state = RobotState.MOVING
            cmd.linear.x = 1.0  # Full speed forward
            self.get_logger().info(f'Moving forward with linear.x = {cmd.linear.x}')

        # Publish command
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Published command - linear.x: {cmd.linear.x}, angular.z: {cmd.angular.z}')

    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Stopping robot')

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 