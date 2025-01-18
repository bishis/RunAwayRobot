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

class RobotState(Enum):
    INITIALIZING = 1
    NAVIGATING = 2
    STOPPED = 3
    ROTATING = 4

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
        self.state = RobotState.STOPPED
        self.current_pose = None
        self.latest_scan = None
        self.current_waypoint_index = 0
        self.waypoints = []
        self.initial_pose_received = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        # Map data
        self.map_data = None
        self.map_info = None
        
        # Path planning
        self.path_planner = PathPlanner(
            safety_radius=self.safety_radius,
            num_samples=20,
            step_size=0.3
        )
        
        # Waypoint generation
        self.waypoint_generator = WaypointGenerator(
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
            num_waypoints=self.num_waypoints
        )
        
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
        )
        self.state = RobotState.NAVIGATING
        return True

    def cmd_vel_callback(self, msg):
        """Convert Nav2 velocity commands to binary commands."""
        try:
            # Convert Nav2's continuous velocities to binary commands
            self.send_velocity_command(msg.linear.x, msg.angular.z)
        except Exception as e:
            self.get_logger().error(f'Error converting velocity command: {str(e)}')

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def map_callback(self, msg):
        """Process incoming map data."""
        try:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.path_planner.update_map(self.map_data, self.map_info)
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

        # Emergency stop check using LIDAR
        if self.check_emergency_stop():
            self.send_velocity_command(0.0, 0.0)
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

        current_target = self.waypoints[self.current_waypoint_index]
        
        # Check if path is blocked
        is_blocked = self.check_path_to_target(current_target)
        
        if is_blocked:
            self.get_logger().info('Obstacle detected, finding alternative path')
            alternative_point = self.path_planner.find_alternative_path(
                self.current_pose.position,
                current_target
            )
            
            if alternative_point:
                self.get_logger().info(f'Found alternative path through ({alternative_point.x:.2f}, {alternative_point.y:.2f})')
                dx = alternative_point.x - self.current_pose.position.x
                dy = alternative_point.y - self.current_pose.position.y
                target_angle = math.atan2(dy, dx)
                current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
                angle_diff = target_angle - current_angle
                
                while angle_diff > math.pi: angle_diff -= 2*math.pi
                while angle_diff < -math.pi: angle_diff += 2*math.pi
                
                if abs(angle_diff) > math.radians(20):
                    self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
                else:
                    self.send_velocity_command(1.0, 0.0)
            else:
                self.get_logger().warn('No alternative path found, backing up')
                self.send_velocity_command(-1.0, 0.0)
            return

        # Normal waypoint navigation
        dx = current_target.x - self.current_pose.position.x
        dy = current_target.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_diff = target_angle - current_angle
        
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi

        if distance < self.waypoint_threshold:
            self.current_waypoint_index += 1
            self.stop_robot()
        elif abs(angle_diff) > math.radians(20):
            self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
        else:
            self.send_velocity_command(1.0, 0.0)

    def check_emergency_stop(self):
        """Check if emergency stop is needed based on LIDAR data."""
        if not self.latest_scan:
            return False

        # Check front sector (150° to 210°)
        start_idx = int(150 * len(self.latest_scan.ranges) / 360)
        end_idx = int(210 * len(self.latest_scan.ranges) / 360)
        
        min_distance = float('inf')
        for i in range(start_idx, end_idx):
            if i < len(self.latest_scan.ranges):
                range_val = self.latest_scan.ranges[i]
                if 0.1 < range_val < min_distance:
                    min_distance = range_val

        return min_distance < (self.robot_radius + self.safety_margin)

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

            # Check if point is valid
            if self.is_valid_point(x, y):
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                waypoints.append(point)

            # Increase radius and angle for spiral pattern
            radius += self.leg_length * 0.5
            angle += math.pi / 2

        return waypoints

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