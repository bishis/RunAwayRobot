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
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Add initial delay before starting
        time.sleep(2.0)
        
        # Create timer for movement control
        self.create_timer(0.1, self.move_robot)
        
        # Movement state
        self.is_turning = False
        self.turn_start_time = None
        
    def move_robot(self):
        """Simple movement pattern: move forward until obstacle, then turn."""
        if self.is_turning:
            if time.time() - self.turn_start_time > 1.0:  # Turn for 1 second
                self.is_turning = False
                self.motors.set_speeds(0.7, 0.7)  # Go forward
            return

        # Default behavior: move forward
        self.motors.set_speeds(0.7, 0.7)

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        # Get minimum distance in front sector
        front_ranges = msg.ranges[350:10]  # Front sector
        min_distance = min([r for r in front_ranges if r > 0.1], default=100)
        
        if min_distance < 0.5:  # If obstacle closer than 50cm
            # Stop and start turning
            self.motors.stop()
            time.sleep(0.1)
            self.motors.set_speeds(0.6, -0.6)  # Turn right
            self.is_turning = True
            self.turn_start_time = time.time()

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
