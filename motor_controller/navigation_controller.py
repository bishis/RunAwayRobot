#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from enum import Enum
import math
import numpy as np

class RobotState(Enum):
    INITIALIZING = 1
    NAVIGATING = 2
    STOPPED = 3
    WAITING_FOR_NAV2 = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Robot parameters
        self.declare_parameter('robot.radius', 0.17)
        self.declare_parameter('robot.safety_margin', 0.10)
        self.declare_parameter('waypoint_threshold', 0.2)
        self.declare_parameter('leg_length', 0.5)
        self.declare_parameter('num_waypoints', 8)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        self.safety_radius = self.robot_radius + self.safety_margin
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        
        # Robot state
        self.state = RobotState.INITIALIZING
        self.current_pose = None
        self.latest_scan = None
        self.current_waypoint_index = 0
        self.waypoints = []
        self.initial_pose_received = False
        self.navigation_succeeded = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        # Map data
        self.map_data = None
        self.map_info = None
        
        self.get_logger().info('Navigation controller initialized')

    def send_velocity_command(self, linear_x, angular_z):
        """Send binary velocity command to the robot."""
        cmd = Twist()
        # Convert to binary values (-1, 0, 1)
        cmd.linear.x = 1.0 if linear_x > 0.1 else (-1.0 if linear_x < -0.1 else 0.0)
        cmd.angular.z = 1.0 if angular_z > 0.1 else (-1.0 if angular_z < -0.1 else 0.0)
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f'Sent command - linear: {cmd.linear.x}, angular: {cmd.angular.z}')

    def navigate_to_pose(self, x, y, theta):
        """Send navigation goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert theta to quaternion (simplified, only yaw)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        
        # Send goal
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_response_callback)
        
        self.state = RobotState.NAVIGATING
        return True

    def navigation_response_callback(self, future):
        """Handle the response from Nav2 goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = RobotState.STOPPED
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle the result of navigation."""
        status = future.result().status
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation succeeded')
            self.navigation_succeeded = True
            self.current_waypoint_index += 1
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.navigation_succeeded = False
        
        self.state = RobotState.STOPPED

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def map_callback(self, msg):
        """Process incoming map data."""
        try:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.get_logger().debug('Map data updated')
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {str(e)}')

    def scan_callback(self, msg):
        """Store latest scan data."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        if self.state == RobotState.INITIALIZING:
            self.state = RobotState.STOPPED

    def control_loop(self):
        """Main control loop."""
        if not self.current_pose:
            return

        # Publish waypoints for visualization
        self.publish_waypoints()

        # Check if we need to generate new waypoints
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Generating new set of waypoints')
            self.waypoints = self.generate_waypoints()
            self.current_waypoint_index = 0
            if not self.waypoints:
                self.get_logger().warn('Failed to generate new waypoints')
                return
            return

        # Only send new navigation goal if we're not already navigating
        if self.state == RobotState.STOPPED:
            current_target = self.waypoints[self.current_waypoint_index]
            # Calculate target orientation (face the direction of movement)
            dx = current_target.x - self.current_pose.position.x
            dy = current_target.y - self.current_pose.position.y
            target_angle = math.atan2(dy, dx)
            
            self.navigate_to_pose(current_target.x, current_target.y, target_angle)

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
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
        waypoint_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Add all waypoints
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
            target_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(target_marker)
        
        self.waypoint_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints')

    def generate_waypoints(self, preferred_angle=None):
        """Generate waypoints in a pattern around the robot."""
        if not self.current_pose or not self.map_data is not None:
            return []

        waypoints = []
        current_pos = self.current_pose.position
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # If preferred angle is not specified, use current orientation
        if preferred_angle is None:
            preferred_angle = current_yaw

        # Generate waypoints in a spiral pattern
        radius = self.leg_length
        angle = preferred_angle
        for i in range(self.num_waypoints):
            # Calculate next point
            x = current_pos.x + radius * math.cos(angle)
            y = current_pos.y + radius * math.sin(angle)

            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            waypoints.append(point)

            # Increase radius and angle for spiral pattern
            radius += self.leg_length * 0.5
            angle += math.pi / 2

        return waypoints

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
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 