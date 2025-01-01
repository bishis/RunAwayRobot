#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
from enum import Enum
import random
import numpy as np

class RobotState(Enum):
    ROTATING = 1
    MOVING = 2
    STOPPED = 3
    BACKING = 4  # New state for backing away from obstacles

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters - reduced distances for tighter pattern
        self.declare_parameter('waypoint_threshold', 0.2)
        self.declare_parameter('leg_length', 0.5)        # Reduced from 2.0 to 0.5 meters
        self.declare_parameter('safety_radius', 0.25)
        self.declare_parameter('num_waypoints', 8)       # Number of waypoints to generate
        
        # Get parameters
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        
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
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')
        
        # Add map subscriber
        self.map_data = None
        self.map_info = None
        
        # Monte Carlo parameters
        self.num_particles = 100
        self.max_iterations = 50
        self.step_size = 0.5  # meters

    def generate_waypoints(self):
        """Generate square wave waypoints starting from current position and orientation."""
        if not self.current_pose or not self.map_info:
            return []
        
        points = []
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Get current orientation
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Calculate forward and right vectors based on current orientation
        forward_x = math.cos(current_yaw)
        forward_y = math.sin(current_yaw)
        right_x = math.cos(current_yaw + math.pi/2)
        right_y = math.sin(current_yaw + math.pi/2)
        
        # Map boundaries with safety margin
        safety_margin = 0.3
        map_min_x = self.map_info.origin.position.x + safety_margin
        map_min_y = self.map_info.origin.position.y + safety_margin
        map_max_x = map_min_x + (self.map_info.width * self.map_info.resolution) - safety_margin
        map_max_y = map_min_y + (self.map_info.height * self.map_info.resolution) - safety_margin
        
        # Generate waypoints in a tighter pattern
        for i in range(self.num_waypoints):
            point = Point()
            
            # Calculate new position
            if i % 2 == 0:
                # Move forward
                new_x = x + forward_x * self.leg_length
                new_y = y + forward_y * self.leg_length
            else:
                # Alternate between left and right
                direction = 1 if (i // 2) % 2 == 0 else -1
                new_x = x + (right_x * self.leg_length * direction)
                new_y = y + (right_y * self.leg_length * direction)
            
            # Clamp to map boundaries
            new_x = max(map_min_x, min(map_max_x, new_x))
            new_y = max(map_min_y, min(map_max_y, new_y))
            
            # Only add point if it's in free space
            if self.is_valid_point(new_x, new_y):
                point.x = new_x
                point.y = new_y
                point.z = 0.0
                points.append(point)
                self.get_logger().info(
                    f'Waypoint {len(points)-1}: ({point.x:.2f}, {point.y:.2f})'
                )
                # Update current position for next waypoint
                x, y = new_x, new_y
            else:
                self.get_logger().warn(f'Skipping invalid waypoint at ({new_x:.2f}, {new_y:.2f})')
        
        return points

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
        if not self.waypoints:  # Debug print
            self.get_logger().warn('No waypoints to publish')
            return
        
        marker_array = MarkerArray()
        
        # All waypoints as red spheres
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = 'map'  # Make sure frame is correct
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.POINTS  # Changed to POINTS for better visibility
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)  # Smaller points
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
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints')  # Debug print

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
        """Main control loop."""
        if not self.current_pose:
            self.get_logger().warn('No pose data')
            return
        if not self.latest_scan:
            self.get_logger().warn('No LIDAR data')
            return
        if not self.map_data is not None:
            self.get_logger().warn('No map data')
            return

        # Always try to publish waypoints
        self.publish_waypoints()

        # Check if we have waypoints
        if not self.waypoints:
            self.get_logger().warn('No waypoints available')
            if self.current_pose and self.map_info:
                self.waypoints = self.generate_waypoints()
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached')
            self.stop_robot()
            return

        # Get current target
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

        # Debug output
        self.get_logger().info(
            f'Navigation status:\n'
            f'  Current pos: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})\n'
            f'  Target: ({current_target.x:.2f}, {current_target.y:.2f})\n'
            f'  Distance: {distance:.2f}m\n'
            f'  Angle diff: {math.degrees(angle_diff):.1f}°'
        )

        # Decision making with binary commands
        if distance < self.waypoint_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            self.stop_robot()
        elif abs(angle_diff) > math.radians(20):
            # Turn to face target - full turn speed
            self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
        else:
            # Move forward with minimal correction
            if abs(angle_diff) > math.radians(10):
                # Need some correction - turn in place
                self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
            else:
                # Straight ahead
                self.send_velocity_command(1.0, 0.0)

    def stop_robot(self):
        """Stop the robot using binary commands."""
        self.send_velocity_command(0.0, 0.0)

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        
        # Generate waypoints once we get initial pose
        if not self.initial_pose_received and self.map_info is not None:
            self.initial_pose_received = True
            self.waypoints = self.generate_waypoints()
            self.get_logger().info(
                f'Generated {len(self.waypoints)} waypoints from initial position: '
                f'({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})'
            )

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
        """Send velocity command to the robot with binary values."""
        cmd = Twist()
        
        # Convert to binary values (-1, 0, 1)
        cmd.linear.x = 1.0 if linear_x > 0.1 else (-1.0 if linear_x < -0.1 else 0.0)
        cmd.angular.z = 1.0 if angular_z > 0.1 else (-1.0 if angular_z < -0.1 else 0.0)
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(
            f"Binary command sent - linear: {cmd.linear.x}, angular: {cmd.angular.z}"
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
        """Process incoming map data."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates."""
        if not self.map_info:
            return None, None
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return mx, my
        return None, None
        
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates."""
        if not self.map_info:
            return None, None
        x = mx * self.map_info.resolution + self.map_info.origin.position.x
        y = my * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def is_valid_point(self, x, y):
        """Check if point is valid and free in map."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return False
        return self.map_data[my, mx] == 0  # 0 indicates free space

    def find_path_monte_carlo(self, start_x, start_y, goal_x, goal_y):
        """Use Monte Carlo to find a clear path to goal."""
        if not self.map_data is not None:
            return None

        best_path = None
        best_cost = float('inf')
        
        for _ in range(self.max_iterations):
            # Generate random path
            current_x, current_y = start_x, start_y
            path = [(current_x, current_y)]
            blocked = False
            
            while math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2) > self.step_size:
                # Generate random step towards goal with bias
                dx = goal_x - current_x
                dy = goal_y - current_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Add random variation
                angle = math.atan2(dy, dx) + random.uniform(-math.pi/4, math.pi/4)
                step_x = self.step_size * math.cos(angle)
                step_y = self.step_size * math.sin(angle)
                
                new_x = current_x + step_x
                new_y = current_y + step_y
                
                # Check if new position is valid
                if not self.is_valid_point(new_x, new_y):
                    blocked = True
                    break
                
                current_x, current_y = new_x, new_y
                path.append((current_x, current_y))
            
            if not blocked:
                # Calculate path cost (length + clearance from obstacles)
                cost = 0
                for i in range(len(path)-1):
                    x1, y1 = path[i]
                    x2, y2 = path[i+1]
                    cost += math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    
                    # Add penalty for proximity to obstacles
                    mx, my = self.world_to_map(x2, y2)
                    if mx is not None and my is not None:
                        for dx in [-1, 0, 1]:
                            for dy in [-1, 0, 1]:
                                if (0 <= mx+dx < self.map_info.width and 
                                    0 <= my+dy < self.map_info.height):
                                    if self.map_data[my+dy, mx+dx] > 50:
                                        cost += 1.0
                
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
        
        return best_path

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