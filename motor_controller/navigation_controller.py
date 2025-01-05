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
from .processors.path_planner import PathPlanner

class RobotState(Enum):
    ROTATING = 1
    MOVING = 2
    STOPPED = 3
    BACKING = 4  # New state for backing away from obstacles

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Robot physical parameters
        self.declare_parameter('robot.radius', 0.17)  # 17cm robot radius
        self.declare_parameter('robot.safety_margin', 0.10)  # 10cm additional safety margin
        
        # Navigation parameters
        self.declare_parameter('waypoint_threshold', 0.2)
        self.declare_parameter('leg_length', 0.5)
        self.declare_parameter('num_waypoints', 8)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        self.safety_radius = self.robot_radius + self.safety_margin  # Total safety distance
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
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
        
        # Add path planner
        self.path_planner = PathPlanner(
            safety_radius=self.safety_radius,
            num_samples=20,
            step_size=0.3
        )
        
        # Add loop closure parameters
        self.visited_positions = []
        self.loop_closure_threshold = 0.5  # meters
        self.min_travel_distance = 2.0  # minimum distance before considering loop closure
        self.last_position = None
        self.distance_traveled = 0.0

    def generate_waypoints(self, preferred_angle=None):
        """Generate square wave waypoints starting from current position and orientation."""
        if not self.current_pose or not self.map_info:
            return []
        
        points = []
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Use preferred angle if provided, otherwise use current orientation
        if preferred_angle is not None:
            current_yaw = preferred_angle
        else:
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            pattern_rotation = random.uniform(-math.pi/4, math.pi/4)
            current_yaw += pattern_rotation
        
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
        attempts = 0
        max_attempts = 20  # Prevent infinite loops
        
        while len(points) < self.num_waypoints and attempts < max_attempts:
            point = Point()
            
            # Calculate new position
            if len(points) % 2 == 0:
                # Move forward
                new_x = x + forward_x * self.leg_length
                new_y = y + forward_y * self.leg_length
            else:
                # Alternate between left and right
                direction = 1 if (len(points) // 2) % 2 == 0 else -1
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
                attempts += 1
                self.get_logger().warn(f'Invalid waypoint at ({new_x:.2f}, {new_y:.2f}), attempt {attempts}')
                # Try a different direction if point is invalid
                current_yaw += math.pi/4  # Rotate 45 degrees
                forward_x = math.cos(current_yaw)
                forward_y = math.sin(current_yaw)
                right_x = math.cos(current_yaw + math.pi/2)
                right_y = math.sin(current_yaw + math.pi/2)
        
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
        if not self.current_pose or not self.latest_scan or not self.map_data is not None:
            return

        self.publish_waypoints()

        # Check if we need to generate new waypoints
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Generating new set of waypoints')
            # Save the last position as the new starting point
            self.waypoints = self.generate_waypoints()
            self.current_waypoint_index = 0
            if not self.waypoints:
                self.get_logger().warn('Failed to generate new waypoints')
                return
            self.get_logger().info(f'Generated {len(self.waypoints)} new waypoints')
            return

        current_target = self.waypoints[self.current_waypoint_index]
        
        # Check for obstacles in path
        is_blocked = self.check_path_to_target(current_target)
        
        if is_blocked:
            self.get_logger().info('Obstacle detected, finding alternative path')
            # Update path planner's map
            self.path_planner.update_map(self.map_data, self.map_info)
            
            # Find alternative path
            alternative_point = self.path_planner.find_alternative_path(
                self.current_pose.position,
                current_target
            )
            
            if alternative_point:
                self.get_logger().info(
                    f'Found alternative path through ({alternative_point.x:.2f}, '
                    f'{alternative_point.y:.2f})'
                )
                # Calculate angle to alternative point
                dx = alternative_point.x - self.current_pose.position.x
                dy = alternative_point.y - self.current_pose.position.y
                target_angle = math.atan2(dy, dx)
                current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
                angle_diff = target_angle - current_angle
                
                # Normalize angle
                while angle_diff > math.pi: angle_diff -= 2*math.pi
                while angle_diff < -math.pi: angle_diff += 2*math.pi
                
                if abs(angle_diff) > math.radians(20):
                    # Turn towards alternative path
                    self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
                else:
                    # Move towards alternative path
                    self.send_velocity_command(1.0, 0.0)
            else:
                # No alternative found, back up and try again
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
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi

        if distance < self.waypoint_threshold:
            self.current_waypoint_index += 1
            self.stop_robot()
        elif abs(angle_diff) > math.radians(20):
            self.send_velocity_command(0.0, 1.0 if angle_diff > 0 else -1.0)
        else:
            self.send_velocity_command(1.0, 0.0)

    def stop_robot(self):
        """Stop the robot using binary commands."""
        self.send_velocity_command(0.0, 0.0)

    def odom_callback(self, msg):
        """Handle odometry updates with loop closure detection."""
        current_pos = msg.pose.pose.position
        
        # Update distance traveled
        if self.last_position:
            dx = current_pos.x - self.last_position.x
            dy = current_pos.y - self.last_position.y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        
        self.last_position = current_pos
        self.current_pose = msg.pose.pose
        
        # Check for loop closure if we've traveled minimum distance
        if self.distance_traveled > self.min_travel_distance:
            if self.check_loop_closure(current_pos):
                self.handle_loop_closure()
                self.distance_traveled = 0.0  # Reset distance after loop closure
        
        # Store position for future loop closure detection
        if not self.visited_positions or self.distance_from_last_stored(current_pos) > 0.5:
            self.visited_positions.append(current_pos)
        
        # Generate initial waypoints if needed
        if not self.initial_pose_received and self.map_info is not None:
            self.initial_pose_received = True
            self.waypoints = self.generate_waypoints()

    def check_loop_closure(self, current_pos):
        """Check if we've returned to a previously visited location."""
        if len(self.visited_positions) < 10:  # Need minimum number of positions
            return False
        
        # Skip recent positions to ensure we've moved away
        for old_pos in self.visited_positions[:-10]:
            dx = current_pos.x - old_pos.x
            dy = current_pos.y - old_pos.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < self.loop_closure_threshold:
                self.get_logger().info(
                    f'Loop closure detected! Distance to previous position: {distance:.2f}m'
                )
                return True
        
        return False

    def handle_loop_closure(self):
        """Handle loop closure by adjusting navigation strategy."""
        self.get_logger().info('Handling loop closure')
        
        # Generate new waypoints in unexplored directions
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Find unexplored directions by checking visited positions
        explored_angles = []
        for pos in self.visited_positions:
            dx = pos.x - self.current_pose.position.x
            dy = pos.y - self.current_pose.position.y
            if math.sqrt(dx*dx + dy*dy) > 1.0:  # Only consider points > 1m away
                angle = math.atan2(dy, dx)
                explored_angles.append(angle)
        
        # Find the largest unexplored angle gap
        if explored_angles:
            explored_angles.sort()
            largest_gap = 0
            best_angle = 0
            
            # Add the first angle again to check gap between last and first
            explored_angles.append(explored_angles[0] + 2*math.pi)
            
            for i in range(len(explored_angles)-1):
                gap = explored_angles[i+1] - explored_angles[i]
                if gap > largest_gap:
                    largest_gap = gap
                    best_angle = explored_angles[i] + gap/2
            
            # Generate new waypoints in the unexplored direction
            self.generate_waypoints(preferred_angle=best_angle)
        else:
            # If no explored angles, generate waypoints normally
            self.generate_waypoints()

    def distance_from_last_stored(self, current_pos):
        """Calculate distance from last stored position."""
        if not self.visited_positions:
            return float('inf')
        
        last_pos = self.visited_positions[-1]
        dx = current_pos.x - last_pos.x
        dy = current_pos.y - last_pos.y
        return math.sqrt(dx*dx + dy*dy)

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
        """Find a clear direction considering robot size."""
        if not self.latest_scan:
            return None
        
        scan_step = len(self.latest_scan.ranges) // 360
        best_gap = 0
        best_angle = None
        min_gap_width = int(math.degrees(math.asin(self.robot_radius / self.safety_radius)))
        
        # Check full 360 degrees
        for angle in range(360):
            if self.latest_scan.ranges[angle * scan_step] > (self.robot_radius + self.safety_margin):
                gap_width = self.measure_gap(angle)
                if gap_width > best_gap and gap_width >= min_gap_width:
                    best_gap = gap_width
                    best_angle = angle - 180  # Convert to robot-relative angle
        
        return best_angle if best_gap >= min_gap_width else None

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
        """Check if point is valid considering robot size."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return False
        
        # Check area that covers entire robot plus safety margin
        check_radius = int((self.robot_radius + self.safety_margin) / self.map_info.resolution)
        
        # Check circular area around point
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                # Only check points within circular radius
                if dx*dx + dy*dy <= check_radius*check_radius:
                    check_x = mx + dx
                    check_y = my + dy
                    if (0 <= check_x < self.map_info.width and 
                        0 <= check_y < self.map_info.height):
                        if self.map_data[check_y, check_x] > 50:  # Occupied
                            return False
        return True

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

    def check_path_to_target(self, target):
        """Check if there's a clear path to the target considering robot size."""
        if not self.latest_scan:
            return False
        
        # Calculate angle to target
        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Convert target angle to LIDAR frame
        angle_diff = target_angle - current_angle
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        
        # Check wider arc to account for robot width
        arc_width = 45  # Increased from 30 to 45 degrees
        angle_deg = math.degrees(angle_diff) + 180
        scan_idx = int(angle_deg * len(self.latest_scan.ranges) / 360)
        
        # Calculate indices for wider check
        start_idx = max(0, scan_idx - arc_width)
        end_idx = min(len(self.latest_scan.ranges), scan_idx + arc_width)
        
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        min_clearance = self.robot_radius + self.safety_margin
        
        # Check each point in the arc
        for i in range(start_idx, end_idx):
            if i < len(self.latest_scan.ranges):
                range_val = self.latest_scan.ranges[i]
                if 0.1 < range_val < min(distance_to_target, min_clearance):
                    self.get_logger().info(f'Obstacle detected at {range_val}m, need {min_clearance}m clearance')
                    return True
        
        return False

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