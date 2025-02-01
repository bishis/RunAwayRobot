#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoiderNode(Node):
    """Standalone node for testing obstacle avoidance"""
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.4)
        self.declare_parameter('danger_distance', 0.3)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Robot footprint dimensions
        self.robot_length = 0.29  # 29cm
        self.robot_width = 0.34   # 34cm
        self.safety_boundary = 0.1  # 10cm extra safety margin
        
        # State tracking
        self.is_avoiding = False
        self.avoidance_start_time = None
        self.min_avoidance_time = 2.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Create timer for constant forward motion (for testing)
        self.create_timer(0.1, self.move_forward)
        
        self.get_logger().info('Obstacle avoider node started')

    def get_obstacle_direction(self, collision_angles) -> str:
        """Convert collision angles to cardinal directions relative to robot"""
        directions = []
        for angle in collision_angles:
            # Convert angle to degrees for easier comparison
            deg = np.degrees(angle) % 360
            
            if 315 <= deg or deg < 45:
                directions.append("FRONT")
            elif 45 <= deg < 135:
                directions.append("RIGHT")
            elif 135 <= deg < 225:
                directions.append("BACK")
            elif 225 <= deg < 315:
                directions.append("LEFT")
                
        return ", ".join(sorted(set(directions)))  # Remove duplicates

    def check_footprint_collision(self, ranges, angles) -> bool:
        """Check if any obstacles are within robot footprint + safety boundary in all directions"""
        # Convert ranges and angles to x,y coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # Define robot boundaries including safety margin
        half_length = (self.robot_length/2 + self.safety_boundary)
        half_width = (self.robot_width/2 + self.safety_boundary)
        
        # Create a rectangular mask for the robot footprint
        points_in_footprint = (np.abs(x) <= half_length) & (np.abs(y) <= half_width)
        
        # Add extra checks for corners using distance calculation
        corner_points = []
        for corner_x, corner_y in [
            (half_length, half_width),    # Front right
            (half_length, -half_width),   # Front left
            (-half_length, half_width),   # Back right
            (-half_length, -half_width),  # Back left
        ]:
            distances = np.sqrt((x - corner_x)**2 + (y - corner_y)**2)
            corner_points.append(distances < self.safety_boundary)
        
        # Combine rectangular footprint with corner checks
        collision_mask = points_in_footprint | np.any(corner_points, axis=0)
        
        # Debug output with directions
        if np.any(collision_mask):
            collision_angles = angles[collision_mask]
            collision_ranges = ranges[collision_mask]
            directions = self.get_obstacle_direction(collision_angles)
            
            min_range = np.min(collision_ranges)
            min_angle_idx = np.argmin(collision_ranges)
            min_angle_deg = np.degrees(collision_angles[min_angle_idx])
            
            self.get_logger().warn(
                f'Obstacle detected from: {directions}\n'
                f'Closest point: {min_range:.2f}m at {min_angle_deg:.1f}°'
            )
        
        return np.any(collision_mask)

    def move_forward(self):
        """Generate constant forward motion for testing"""
        cmd = Twist()
        cmd.linear.x = self.max_linear_speed
        self.process_velocity(cmd)

    def scan_callback(self, scan: LaserScan):
        """Handle incoming laser scan data"""
        try:
            # Convert scan to numpy array, handling inf values
            ranges = np.array(scan.ranges)
            ranges[np.isinf(ranges)] = scan.range_max
            
            # Get angles for each range measurement
            angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
            
            # Check for collision with robot footprint
            collision_imminent = self.check_footprint_collision(ranges, angles)
            
            if collision_imminent:
                if not self.is_avoiding:
                    self.is_avoiding = True
                    self.avoidance_start_time = self.get_clock().now()
                    self.get_logger().warn('Obstacle detected within robot footprint - backing up')
                
                # Back up
                cmd = Twist()
                cmd.linear.x = -self.max_linear_speed * 0.7
                self.publish_cmd(cmd)
                
            # Check if we should continue avoidance
            elif self.is_avoiding:
                time_avoiding = (self.get_clock().now() - self.avoidance_start_time).nanoseconds / 1e9
                
                # Exit avoidance if we've backed up enough and no collision imminent
                if time_avoiding > self.min_avoidance_time and not collision_imminent:
                    self.get_logger().info('Exiting avoidance mode - area clear')
                    self.is_avoiding = False
                else:
                    # Continue backing up if still in avoidance
                    cmd = Twist()
                    cmd.linear.x = -self.max_linear_speed * 0.7
                    self.publish_cmd(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')

    def process_velocity(self, cmd: Twist):
        """Process and publish velocity commands if not avoiding"""
        if not self.is_avoiding:
            self.publish_cmd(cmd)

    def publish_cmd(self, cmd: Twist):
        """Publish velocity commands to both topics"""
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