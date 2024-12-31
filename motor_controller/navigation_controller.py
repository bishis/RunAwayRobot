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
from .processors.path_planner import MonteCarloPathPlanner

class RobotState(Enum):
    ROTATING = 1
    MOVING = 2
    STOPPED = 3
    BACKING = 4  # New state for backing away from obstacles

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
        self.latest_scan = None
        self.current_waypoint_index = 0
        self.waypoints = []  # Initialize empty, will generate after getting initial pose
        self.initial_pose_received = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        # Add path planner
        self.path_planner = MonteCarloPathPlanner()
        self.current_path = []
        
        # Add subscriber for map
        self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        self.get_logger().info('Navigation controller initialized')

    def generate_waypoints(self):
        """Generate square wave waypoints starting from current position and orientation."""
        if not self.current_pose:
            return []
        
        points = []
        # Start from current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Get current orientation
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Calculate forward and right vectors based on current orientation
        forward_x = math.cos(current_yaw)
        forward_y = math.sin(current_yaw)
        right_x = math.cos(current_yaw + math.pi/2)
        right_y = math.sin(current_yaw + math.pi/2)
        
        # Generate square wave pattern relative to robot's orientation
        for i in range(4):
            point = Point()
            if i % 2 == 0:
                # Move in robot's forward direction
                x += forward_x * self.leg_length
                y += forward_y * self.leg_length
            else:
                # Move in robot's right direction
                x += right_x * self.leg_length
                y += right_y * self.leg_length
            point.x, point.y = x, y
            points.append(point)
            self.get_logger().info(f'Waypoint {i}: ({point.x:.2f}, {point.y:.2f})')
        
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
            return False, None
        
        # Check front sector (150° to 210°)
        start_idx = int(150 * len(self.latest_scan.ranges) / 360)
        end_idx = int(210 * len(self.latest_scan.ranges) / 360)
        
        min_distance = float('inf')
        for i in range(start_idx, end_idx):
            if i < len(self.latest_scan.ranges):
                range_val = self.latest_scan.ranges[i]
                if 0.1 < range_val < min_distance:
                    min_distance = range_val
        
        return min_distance < self.safety_radius, min_distance

    def control_loop(self):
        if not self.current_pose:
            self.get_logger().warn('Waiting for pose data...')
            return

        self.publish_waypoints()

        # If no waypoints yet, return
        if not self.waypoints:
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

        # Check for obstacles
        is_blocked, obstacle_distance = self.check_obstacles()
        
        # Create command message
        cmd = Twist()

        # State machine
        if self.state == RobotState.BACKING:
            # Continue backing up until we have enough space
            if not is_blocked or (obstacle_distance and obstacle_distance > self.safety_radius * 2):
                self.state = RobotState.ROTATING
                clear_path = self.find_clear_path()
                if clear_path is not None:
                    cmd.angular.z = 1.0 if clear_path > 0 else -1.0
                else:
                    cmd.angular.z = 1.0  # Default to left if no clear path
            else:
                # Keep backing up
                cmd.linear.x = -1.0
        
        elif is_blocked:
            # Start backing up if too close to obstacle
            self.state = RobotState.BACKING
            cmd.linear.x = -1.0
            self.get_logger().info('Obstacle detected, backing up')
        
        elif distance < self.waypoint_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            self.state = RobotState.ROTATING
            self.stop_robot()
            return
        
        elif abs(angle_diff) > math.radians(20):
            self.state = RobotState.ROTATING
            cmd.angular.z = 1.0 if angle_diff > 0 else -1.0
        
        else:
            # Move forward
            self.state = RobotState.MOVING
            cmd.linear.x = 1.0

        # Publish command
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(
            f'Command sent - State: {self.state.name}, '
            f'Linear: {cmd.linear.x}, Angular: {cmd.angular.z}'
        )

        # When blocked, find new path
        if is_blocked:
            self.get_logger().info('Obstacle detected, planning new path')
            new_path = self.find_path_to_target()
            
            if new_path:
                self.current_path = new_path
                self.get_logger().info(f'Found new path with {len(new_path)} points')
                # Start following new path
                self.state = RobotState.MOVING
            else:
                # If no path found, back up and rotate
                self.state = RobotState.BACKING
                cmd.linear.x = -1.0
                self.get_logger().warn('No valid path found, backing up')

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
        
        # Generate waypoints once we get initial pose
        if not self.initial_pose_received:
            self.initial_pose_received = True
            self.waypoints = self.generate_waypoints()
            self.get_logger().info(f'Generated {len(self.waypoints)} waypoints from initial position')
            self.publish_waypoints()

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def determine_movement(self, sector_data):
        if not sector_data:
            return (0.0, 0.0)
        
        self.get_logger().info(f"State: {self.state.name}, Sector distances: {sector_data}")
        
        # Check if path is blocked
        front_blocked = sector_data['front'] < self.safety_radius * 1.5
        left_blocked = sector_data['front_left'] < self.safety_radius
        right_blocked = sector_data['front_right'] < self.safety_radius
        
        if self.state == RobotState.FORWARD:
            if front_blocked or left_blocked or right_blocked:
                self.state = RobotState.ROTATING
                self.rotation_direction = self.find_best_rotation(sector_data)
                self.get_logger().info(f"Obstacle detected, starting rotation {'right' if self.rotation_direction < 0 else 'left'}")
                # Use full rotation speed
                return (0.0, 1.0 if self.rotation_direction > 0 else -1.0)
            # Use full forward speed when clear
            return (1.0, 0.0)
        
        elif self.state == RobotState.ROTATING:
            # Keep rotating until we find a clear path
            if not front_blocked and not left_blocked and not right_blocked:
                self.state = RobotState.FORWARD
                self.get_logger().info("Clear path found, moving forward")
                # Use full forward speed when clear
                return (1.0, 0.0)
            # Use full rotation speed
            return (0.0, 1.0 if self.rotation_direction > 0 else -1.0)
        
        return (0.0, 0.0)  # Default stop

    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to the robot."""
        cmd = Twist()
        # Convert to binary values here
        cmd.linear.x = 1.0 if linear_x > 0.5 else (-1.0 if linear_x < -0.5 else 0.0)
        cmd.angular.z = 1.0 if angular_z > 0.5 else (-1.0 if angular_z < -0.5 else 0.0)
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info(
            f"Sending command - linear: {cmd.linear.x}, angular: {cmd.angular.z}"
        )

    def find_clear_path(self):
        """Find a clear direction to rotate towards."""
        if not self.latest_scan:
            return None
        
        # Check 180-degree arc in front for clear paths
        scan_step = len(self.latest_scan.ranges) // 360
        start_angle = 90  # Start checking from -90 degrees
        end_angle = 270   # to +90 degrees
        
        max_gap = 0
        best_angle = None
        
        for angle in range(start_angle, end_angle):
            idx = angle * scan_step
            if idx < len(self.latest_scan.ranges):
                if self.latest_scan.ranges[idx] > self.safety_radius:
                    gap_size = self.measure_gap(angle)
                    if gap_size > max_gap:
                        max_gap = gap_size
                        best_angle = angle - 180  # Convert to robot-relative angle
        
        return best_angle if max_gap > 30 else None  # Return None if no good gaps found

    def measure_gap(self, center_angle):
        """Measure the size of a gap centered at the given angle."""
        scan_step = len(self.latest_scan.ranges) // 360
        gap_size = 0
        
        # Check outward from center angle
        for offset in range(0, 30):  # Check up to 30 degrees each direction
            left_idx = ((center_angle + offset) % 360) * scan_step
            right_idx = ((center_angle - offset) % 360) * scan_step
            
            if (left_idx < len(self.latest_scan.ranges) and 
                right_idx < len(self.latest_scan.ranges)):
                if (self.latest_scan.ranges[left_idx] > self.safety_radius and 
                    self.latest_scan.ranges[right_idx] > self.safety_radius):
                    gap_size = offset * 2
                else:
                    break
                
        return gap_size

    def map_callback(self, msg):
        """Handle map updates."""
        self.path_planner.set_map(
            msg.data,
            msg.info.resolution,
            msg.info.origin,
            msg.info.width,
            msg.info.height
        )

    def find_path_to_target(self):
        """Find path to current waypoint."""
        if not self.current_pose or self.current_waypoint_index >= len(self.waypoints):
            return None
            
        start = Point(
            x=self.current_pose.position.x,
            y=self.current_pose.position.y,
            z=0.0
        )
        goal = self.waypoints[self.current_waypoint_index]
        
        return self.path_planner.find_path(start, goal)

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