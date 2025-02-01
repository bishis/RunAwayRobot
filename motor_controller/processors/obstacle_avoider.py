#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoiderNode(Node):
    """Standalone node for testing forward/backward-only obstacle avoidance."""
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.28)
        self.declare_parameter('danger_distance', 0.20)
        self.declare_parameter('max_linear_speed', 0.07)
        # Although angular speed is declared, we won't use turning commands.
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_avoidance_time', 2.0)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_avoidance_time = self.get_parameter('min_avoidance_time').value
        
        # Robot footprint dimensions (meters)
        self.robot_length = 0.28  
        self.robot_width = 0.22   
        self.safety_boundary = 0.04  # extra margin
        
        # State tracking for avoidance
        self.is_avoiding = False
        self.avoidance_start_time = None
        self.current_escape_mode = None  # "forward" or "backward"
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Timer for constant forward motion when not avoiding
        self.create_timer(0.1, self.move_forward)
        
        self.get_logger().info('Forward/backward obstacle avoider node started')
    
    def get_obstacle_direction(self, collision_angles) -> str:
        """Convert collision angles (in radians) to 'FRONT' or 'BACK'."""
        directions = []
        for angle in collision_angles:
            # Normalize angle to [0, 360)
            deg = np.degrees(angle) % 360
            if deg < 90 or deg >= 270:
                directions.append("FRONT")
            else:
                directions.append("BACK")
        return ", ".join(sorted(set(directions)))
    
    def check_footprint_collision(self, ranges, angles) -> bool:
        """Check if any obstacles are within the robot footprint plus safety margin."""
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        half_length = (self.robot_length / 2 + self.safety_boundary)
        half_width  = (self.robot_width / 2 + self.safety_boundary)
        
        rect_mask = (np.abs(x) <= half_length) & (np.abs(y) <= half_width)
        
        # Check corners: compute if any point is too close to a corner
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
        Evaluate scan data in two zones (front and back) and choose a command.
        Front zone: angles between -30° and +30° ([-pi/6, pi/6])
        Back zone: angles between 150° and 210° ([5pi/6, 7pi/6]) or equivalently,
                   angles outside the front zone (considering symmetry).
        Returns a Twist command (only linear.x set) and the chosen zone as a string.
        """
        # Front zone: within ±30° of 0
        front_mask = (angles >= -np.pi/6) & (angles <= np.pi/6)
        # Back zone: within ±30° of π (or -π)
        # We'll consider angles where absolute difference from π is less than π/6.
        back_mask = (np.abs(np.abs(angles) - np.pi) < np.pi/6)
        
        front_clear = np.min(ranges[front_mask]) if np.any(front_mask) else np.inf
        back_clear = np.min(ranges[back_mask]) if np.any(back_mask) else np.inf
        
        self.get_logger().debug(
            f"Clearances - Front: {front_clear:.2f}, Back: {back_clear:.2f}"
        )
        
        cmd = Twist()
        # Choose the direction with the greater clearance.
        if front_clear >= back_clear and front_clear > self.safety_distance:
            # Front has more clearance and is safe.
            cmd.linear.x = self.max_linear_speed * 0.7
            escape_mode = "forward"
        elif back_clear > self.safety_distance:
            # Back is clearer; reverse.
            cmd.linear.x = -self.max_linear_speed * 0.7
            escape_mode = "backward"
        else:
            # Both directions are too close: stop.
            cmd.linear.x = 0.0
            escape_mode = "stop"
        
        # No angular component because turning is not available.
        cmd.angular.z = 0.0
        return cmd, escape_mode
    
    def move_forward(self):
        """Publish constant forward command if not in avoidance mode."""
        if not self.is_avoiding:
            cmd = Twist()
            cmd.linear.x = self.max_linear_speed
            self.publish_cmd(cmd)
    
    def scan_callback(self, scan: LaserScan):
        """Process incoming scan data to determine and execute avoidance maneuvers."""
        try:
            ranges = np.array(scan.ranges)
            ranges[np.isinf(ranges)] = scan.range_max
            angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
            
            collision_imminent = self.check_footprint_collision(ranges, angles)
            
            if collision_imminent:
                if not self.is_avoiding:
                    self.is_avoiding = True
                    self.avoidance_start_time = self.get_clock().now()
                    self.get_logger().info("Entering avoidance mode.")
                
                cmd, chosen_zone = self.choose_escape_command(ranges, angles)
                self.current_escape_mode = chosen_zone
                self.get_logger().info(f"Escape maneuver: {chosen_zone}")
                self.publish_cmd(cmd)
            else:
                if self.is_avoiding:
                    time_avoiding = (self.get_clock().now() - self.avoidance_start_time).nanoseconds / 1e9
                    if time_avoiding > self.min_avoidance_time:
                        self.get_logger().info("Exiting avoidance mode - area clear")
                        self.is_avoiding = False
                        self.current_escape_mode = None
                    else:
                        cmd, _ = self.choose_escape_command(ranges, angles)
                        self.publish_cmd(cmd)
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')
    
    def publish_cmd(self, cmd: Twist):
        """Publish velocity commands to both topics."""
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
