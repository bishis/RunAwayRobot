#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoiderNode(Node):
    """Standalone node for testing advanced obstacle avoidance maneuvers."""
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.28)
        self.declare_parameter('danger_distance', 0.20)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_avoidance_time', 2.0)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_avoidance_time = self.get_parameter('min_avoidance_time').value
        
        # Robot footprint dimensions
        self.robot_length = 0.28  # meters
        self.robot_width = 0.22   # meters
        self.safety_boundary = 0.04  # additional margin
        
        # State tracking for avoidance
        self.is_avoiding = False
        self.avoidance_start_time = None
        self.current_escape_mode = None  # e.g. "front", "left", "right", "back"
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Timer for normal forward motion (when not avoiding)
        self.create_timer(0.1, self.move_forward)
        
        self.get_logger().info('Advanced obstacle avoider node started')
    
    def get_obstacle_direction(self, collision_angles) -> str:
        """Convert collision angles to cardinal directions relative to robot."""
        directions = []
        for angle in collision_angles:
            deg = np.degrees(angle) % 360
            if 315 <= deg or deg < 45:
                directions.append("FRONT")
            elif 45 <= deg < 135:
                directions.append("RIGHT")
            elif 135 <= deg < 225:
                directions.append("BACK")
            elif 225 <= deg < 315:
                directions.append("LEFT")
        return ", ".join(sorted(set(directions)))
    
    def check_footprint_collision(self, ranges, angles) -> bool:
        """Check if any obstacles are within the robot footprint (plus safety margin)."""
        # Convert polar coordinates to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # Define half dimensions including safety boundary
        half_length = (self.robot_length / 2 + self.safety_boundary)
        half_width  = (self.robot_width / 2 + self.safety_boundary)
        
        # Create rectangular mask
        rect_mask = (np.abs(x) <= half_length) & (np.abs(y) <= half_width)
        
        # Additionally check corners (for better safety)
        corner_checks = []
        for cx, cy in [(half_length, half_width),
                       (half_length, -half_width),
                       (-half_length, half_width),
                       (-half_length, -half_width)]:
            distances = np.sqrt((x - cx)**2 + (y - cy)**2)
            corner_checks.append(distances < self.safety_boundary)
        corner_mask = np.any(corner_checks, axis=0)
        
        collision_mask = rect_mask | corner_mask
        
        if np.any(collision_mask):
            coll_angles = angles[collision_mask]
            coll_ranges = ranges[collision_mask]
            directions = self.get_obstacle_direction(coll_angles)
            min_range = np.min(coll_ranges)
            min_angle = np.degrees(coll_angles[np.argmin(coll_ranges)])
            self.get_logger().warn(
                f'Collision detected from: {directions}. '
                f'Closest: {min_range:.2f} m at {min_angle:.1f}°'
            )
        return np.any(collision_mask)
    
    def choose_escape_command(self, ranges, angles):
        """
        Evaluate scan data in four zones and choose a blended command.
        Zones:
          - Front: angles between -30° and +30° ([-pi/6, pi/6])
          - Left: angles between 30° and 90° ([pi/6, pi/2])
          - Right: angles between -90° and -30° ([-pi/2, -pi/6])
          - Back: angles outside ±90°
        Returns a Twist command and the chosen zone as a string.
        """
        # Define masks for each zone
        front_mask = (angles >= -np.pi/6) & (angles <= np.pi/6)
        left_mask  = (angles > np.pi/6) & (angles <= np.pi/2)
        right_mask = (angles >= -np.pi/2) & (angles < -np.pi/6)
        back_mask  = (np.abs(angles) > np.pi/2)
        
        # Get minimum distance (or a conservative clearance) for each zone
        front_clear = np.min(ranges[front_mask]) if np.any(front_mask) else np.inf
        left_clear  = np.min(ranges[left_mask])  if np.any(left_mask)  else np.inf
        right_clear = np.min(ranges[right_mask]) if np.any(right_mask) else np.inf
        back_clear  = np.min(ranges[back_mask])  if np.any(back_mask)  else np.inf
        
        self.get_logger().debug(
            f"Clearances - Front: {front_clear:.2f}, Left: {left_clear:.2f}, "
            f"Right: {right_clear:.2f}, Back: {back_clear:.2f}"
        )
        
        clearances = {
            "front": front_clear,
            "left": left_clear,
            "right": right_clear,
            "back": back_clear
        }
        # Choose zone with maximum clearance
        best_zone = max(clearances, key=clearances.get)
        
        # Generate a Twist command based on the chosen escape zone
        cmd = Twist()
        if best_zone == "front":
            # Even if front is best, slow forward motion (with zero turn)
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = 0.0
        elif best_zone == "left":
            # Turn left moderately while moving forward slowly
            cmd.linear.x = self.max_linear_speed * 0.3
            cmd.angular.z = self.max_angular_speed * 0.7
        elif best_zone == "right":
            # Turn right moderately while moving forward slowly
            cmd.linear.x = self.max_linear_speed * 0.3
            cmd.angular.z = -self.max_angular_speed * 0.7
        elif best_zone == "back":
            # Reverse gently if back has the most clearance
            cmd.linear.x = -self.max_linear_speed * 0.5
            cmd.angular.z = 0.0
        
        return cmd, best_zone
    
    def move_forward(self):
        """Publish constant forward command if not in avoidance mode."""
        if not self.is_avoiding:
            cmd = Twist()
            cmd.linear.x = self.max_linear_speed
            self.publish_cmd(cmd)
    
    def scan_callback(self, scan: LaserScan):
        """Process incoming scan data to determine and execute avoidance maneuvers."""
        try:
            # Convert scan ranges to numpy array; replace inf with max range.
            ranges = np.array(scan.ranges)
            ranges[np.isinf(ranges)] = scan.range_max
            
            # Generate corresponding angles.
            angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
            
            # Check for collision within the robot footprint.
            collision_imminent = self.check_footprint_collision(ranges, angles)
            
            if collision_imminent:
                # Enter or continue avoidance mode.
                if not self.is_avoiding:
                    self.is_avoiding = True
                    self.avoidance_start_time = self.get_clock().now()
                    self.get_logger().info("Entering avoidance mode.")
                
                # Choose a more advanced escape command based on four regions.
                cmd, chosen_zone = self.choose_escape_command(ranges, angles)
                self.current_escape_mode = chosen_zone
                self.get_logger().info(f"Escape maneuver: {chosen_zone}")
                self.publish_cmd(cmd)
            else:
                # If currently in avoidance mode, check if sufficient time has passed.
                if self.is_avoiding:
                    time_avoiding = (self.get_clock().now() - self.avoidance_start_time).nanoseconds / 1e9
                    if time_avoiding > self.min_avoidance_time:
                        self.get_logger().info("Exiting avoidance mode - area clear")
                        self.is_avoiding = False
                        self.current_escape_mode = None
                    else:
                        # Continue current avoidance maneuver until minimum time elapses.
                        cmd, _ = self.choose_escape_command(ranges, angles)
                        self.publish_cmd(cmd)
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')
    
    def publish_cmd(self, cmd: Twist):
        """Publish velocity commands to both cmd_vel and wheel_speeds topics."""
        self.cmd_vel_pub.publish(cmd)
        self.wheel_speeds_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
