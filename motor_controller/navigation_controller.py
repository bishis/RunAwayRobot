#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
from enum import Enum
import numpy as np

class RobotState(Enum):
    INITIALIZING = 1
    NAVIGATING = 2
    STOPPED = 3
    WAITING = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot.radius', 0.17)
        self.declare_parameter('robot.safety_margin', 0.10)
        self.declare_parameter('waypoint_threshold', 0.2)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        
        # Robot state
        self.state = RobotState.INITIALIZING
        self.current_pose = None
        self.latest_scan = None
        self.current_waypoint_index = 0
        self.waypoints = []
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')

    def cmd_vel_callback(self, msg):
        """Convert Nav2 commands to binary commands for the Pi."""
        try:
            # Create binary command
            cmd = Twist()
            # Convert linear velocity to binary
            cmd.linear.x = 1.0 if msg.linear.x > 0.1 else (-1.0 if msg.linear.x < -0.1 else 0.0)
            # Convert angular velocity to binary
            cmd.angular.z = 1.0 if msg.angular.z > 0.1 else (-1.0 if msg.angular.z < -0.1 else 0.0)
            
            # Publish binary command
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().debug(
                f'Binary command: linear={cmd.linear.x:.1f}, angular={cmd.angular.z:.1f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting velocity command: {str(e)}')

    def navigate_to_pose(self, pose):
        """Send navigation goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal to Nav2')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        
        send_goal_future.add_done_callback(self.navigation_response_callback)
        return True

    def navigation_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.state = RobotState.STOPPED
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
            self.current_waypoint_index += 1
            self.state = RobotState.WAITING
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.state = RobotState.STOPPED

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def control_loop(self):
        """Main control loop using Nav2 for navigation."""
        if not self.current_pose:
            return

        if self.state == RobotState.INITIALIZING:
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.state = RobotState.WAITING
                self.get_logger().info('Nav2 server ready')
            return

        if self.state == RobotState.WAITING:
            if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Generating new waypoints')
                self.waypoints = self.generate_waypoints()
                self.current_waypoint_index = 0
                if not self.waypoints:
                    self.get_logger().warn('Failed to generate waypoints')
                    return

            # Create pose goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position = self.waypoints[self.current_waypoint_index]
            
            # Calculate orientation towards next waypoint or final orientation
            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_point = self.waypoints[self.current_waypoint_index + 1]
                dx = next_point.x - goal_pose.pose.position.x
                dy = next_point.y - goal_pose.pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
                
            # Convert yaw to quaternion
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

            if self.navigate_to_pose(goal_pose):
                self.state = RobotState.NAVIGATING
            else:
                self.get_logger().error('Failed to start navigation')

        self.publish_waypoints()

    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose

    def generate_waypoints(self):
        """Generate exploration waypoints using a simple frontier-based approach."""
        if not self.current_pose:
            self.get_logger().warn('No current pose available')
            return []

        waypoints = []
        # Create a test pattern of waypoints in a square pattern
        # This is a simple example - you might want to implement frontier-based exploration
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        step_size = 1.0  # 1 meter between waypoints

        # Generate points in a square pattern
        patterns = [
            (step_size, 0),      # Right
            (0, step_size),      # Up
            (-step_size, 0),     # Left
            (0, -step_size)      # Down
        ]

        for dx, dy in patterns:
            point = Point()
            point.x = current_x + dx
            point.y = current_y + dy
            point.z = 0.0
            waypoints.append(point)

        if waypoints:
            self.get_logger().info(f'Generated {len(waypoints)} waypoints')
        else:
            self.get_logger().warn('Failed to generate waypoints')

        return waypoints

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
        if not self.waypoints:
            return

        marker_array = MarkerArray()
        
        # All waypoints as red spheres
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = 'map'
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.POINTS
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale.x = 0.1
        waypoint_marker.scale.y = 0.1
        waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        for point in self.waypoints:
            waypoint_marker.points.append(point)
        
        marker_array.markers.append(waypoint_marker)
        
        # Current target as green sphere
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
            target_marker.scale.x = 0.2
            target_marker.scale.y = 0.2
            target_marker.scale.z = 0.2
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(target_marker)
        
        self.waypoint_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 