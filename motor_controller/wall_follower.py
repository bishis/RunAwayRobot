#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
import time

from .controllers.motor_controller import MotorController
from .processors.lidar_processor import LidarProcessor

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize components
        self.motors = MotorController(
            left_pin=18,
            right_pin=12
        )
        self.lidar = LidarProcessor()
        
        # Movement state
        self.state = 'FORWARD'  # States: FORWARD, TURNING
        self.turn_direction = 'RIGHT'  # Alternates between LEFT and RIGHT
        self.turn_start_time = None
        self.forward_start_time = None
        self.turn_duration = 2.0  # Time to turn (seconds)
        self.forward_duration = 3.0  # Time to move forward (seconds)
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Add initial delay before starting
        time.sleep(2.0)
        
        # Create timer for movement control
        self.create_timer(0.1, self.move_robot)
        
        # Start moving forward
        self.start_forward()
        
    def start_forward(self):
        """Start moving forward."""
        self.state = 'FORWARD'
        self.forward_start_time = time.time()
        self.motors.set_speeds(0.4, 0.4)  # Move forward at 40% speed
        self.get_logger().info('Moving forward')
        
    def start_turning(self):
        """Start turning."""
        self.state = 'TURNING'
        self.turn_start_time = time.time()
        
        # Set turn direction and speeds
        if self.turn_direction == 'RIGHT':
            self.motors.set_speeds(0.4, -0.4)  # Turn right
            self.turn_direction = 'LEFT'  # Next time turn left
        else:
            self.motors.set_speeds(-0.4, 0.4)  # Turn left
            self.turn_direction = 'RIGHT'  # Next time turn right
            
        self.get_logger().info(f'Turning {self.turn_direction}')
        
    def move_robot(self):
        """Control robot movement pattern."""
        current_time = time.time()
        
        if self.state == 'FORWARD':
            # Check if we've been moving forward long enough
            if current_time - self.forward_start_time >= self.forward_duration:
                self.start_turning()
                
        elif self.state == 'TURNING':
            # Check if we've been turning long enough
            if current_time - self.turn_start_time >= self.turn_duration:
                self.start_forward()

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        # Get minimum distance in front sector
        front_ranges = msg.ranges[350:10]  # Front sector
        min_distance = min([r for r in front_ranges if r > 0.1], default=100)
        
        if min_distance < 0.3:  # If obstacle closer than 30cm
            # Stop and start turning
            self.motors.stop()
            time.sleep(0.1)
            self.start_turning()

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Stopping motors")
        robot.motors.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
