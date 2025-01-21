#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from .controllers.motor_controller import MotorController


class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )
        
        # Subscribe to navigation commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Robot parameters
        self.track_width = 0.3  # Distance between tracks in meters
        self.max_linear_speed = 0.1  # Max linear speed from nav2 params
        self.max_angular_speed = 0.5  # Max angular speed from nav2 params
        
        # Motor control parameters
        self.control_period = 0.1  # Increased to 100ms for more stable control
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(self.control_period, self.motor_control_timer)
        
        # Store current motor states
        self.left_speed = 0
        self.right_speed = 0
        self.target_left = 0
        self.target_right = 0
        
        # Movement thresholds - adjusted for binary control
        self.min_speed_threshold = 0.15    # Increased to reduce rapid switching
        self.speed_increment = 0.05        # Smaller increments for smoother transitions
        self.forward_boost = 1.0           # Removed boost as it causes instability
        
        # Movement state tracking with hysteresis
        self.moving_forward = False
        self.rotation_only = False
        self.state_change_threshold = 0.2   # Prevent rapid state changes
        self.last_state_change = self.get_clock().now()
        self.min_state_time = 0.3          # Minimum time before state change
        
        # Obstacle detection parameters
        self.min_obstacle_distance = 0.3  # Meters
        self.last_obstacle_warning = self.get_clock().now()
        self.warning_interval = 1.0  # Seconds between warnings
        
        # Add timing diagnostics
        self.last_control_time = self.get_clock().now()
        self.cmd_vel_count = 0
        self.create_timer(1.0, self.print_diagnostics)  # Print stats every second
        
        self.get_logger().info('Hardware Controller initialized')
    
    def motor_control_timer(self):
        """Timer callback for smooth motor control"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # Stop if no recent commands
        if dt > 0.5:
            self.target_left = 0
            self.target_right = 0
            self.moving_forward = False
            self.rotation_only = False
            self.motors.set_speeds(0, 0)
            return
        
        # Check if enough time has passed for state change
        state_dt = (now - self.last_state_change).nanoseconds / 1e9
        can_change_state = state_dt > self.min_state_time
        
        # Detect movement type with hysteresis
        if can_change_state:
            new_moving_forward = (abs(self.target_left - self.target_right) < self.state_change_threshold and 
                                abs(self.target_left) > self.min_speed_threshold)
            new_rotation_only = (abs(self.target_left + self.target_right) < self.state_change_threshold and 
                               abs(self.target_left) > self.min_speed_threshold)
            
            if new_moving_forward != self.moving_forward or new_rotation_only != self.rotation_only:
                self.last_state_change = now
                self.moving_forward = new_moving_forward
                self.rotation_only = new_rotation_only
        
        # Determine motor outputs based on movement type
        if self.moving_forward:
            # Use same direction for both motors in forward motion
            direction = 1 if (self.target_left + self.target_right) > 0 else -1
            left_out = right_out = direction
        elif self.rotation_only:
            # Pure rotation
            left_out = 1 if self.target_left > 0 else -1
            right_out = -left_out
        else:
            # Mixed motion - use thresholds to determine outputs
            left_out = 1 if self.target_left > self.min_speed_threshold else (-1 if self.target_left < -self.min_speed_threshold else 0)
            right_out = 1 if self.target_right > self.min_speed_threshold else (-1 if self.target_right < -self.min_speed_threshold else 0)
        
        self.motors.set_speeds(left_out, right_out)
        
        # Debug logging
        if self.moving_forward:
            self.get_logger().debug(f'Forward motion: L={left_out} R={right_out}')
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan data and check for obstacles"""
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if ranges:
            min_distance = min(ranges)
            
            now = self.get_clock().now()
            if (min_distance < self.min_obstacle_distance and 
                (now - self.last_obstacle_warning).nanoseconds / 1e9 >= self.warning_interval):
                self.get_logger().warn(f'Obstacle detected! Distance: {min_distance:.2f}m')
                self.last_obstacle_warning = now
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel into motor control parameters"""
        self.cmd_vel_count += 1
        self.last_control_time = self.get_clock().now()
        
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        self.get_logger().debug(
            f'CMD_VEL received: linear={linear_x:.3f} angular={angular_z:.3f}'
        )
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize speeds
        self.target_left = self.normalize_speed(left_speed)
        self.target_right = self.normalize_speed(right_speed)
        
        self.last_cmd_time = self.get_clock().now()
    
    def normalize_speed(self, speed: float) -> float:
        """Normalize speed to -1 to 1 range"""
        max_speed = self.max_linear_speed + (self.max_angular_speed * self.track_width / 2.0)
        normalized = speed / max_speed
        return max(min(normalized, 1.0), -1.0)

    def print_diagnostics(self):
        """Print diagnostic information"""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        
        self.get_logger().info(
            f'Diagnostics:\n'
            f'  CMD_VEL frequency: {self.cmd_vel_count} Hz\n'
            f'  Current speeds: L={self.left_speed:.2f} R={self.right_speed:.2f}\n'
            f'  Target speeds: L={self.target_left:.2f} R={self.target_right:.2f}\n'
            f'  Moving forward: {self.moving_forward}\n'
            f'  Time since last cmd: {dt:.3f}s'
        )
        self.cmd_vel_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
