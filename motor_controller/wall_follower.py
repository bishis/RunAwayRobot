#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Quaternion
import tf2_ros
import math
import time
import numpy as np
from enum import Enum

from .controllers.motor_controller import MotorController
from .controllers.navigation_controller import NavigationController
from .processors.lidar_processor import LidarProcessor

class RobotState(Enum):
    EXPLORING = 1    # Moving to frontier
    ROTATING = 2     # Turning to new direction
    REVERSING = 3    # Backing away from obstacle
    SCANNING = 4     # Scanning area for mapping

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize controllers
        self.motors = MotorController(left_pin=18, right_pin=12)
        self.navigator = NavigationController()
        
        # Robot state
        self.state = RobotState.EXPLORING
        self.current_pose = None
        self.last_scan = None
        self.current_target = None
        self.scan_start_time = None
        self.scan_duration = 2.0  # seconds to scan area
        
        # Motion parameters
        self.rotation_speed = 0.8  # Reduced rotation speed
        self.forward_speed = 1.0
        self.min_turn_angle = math.pi/6  # 30 degrees threshold for turning
        
        # Safety parameters
        self.safety_distance = 0.5    # Increased from 0.4
        self.critical_distance = 0.3   # Increased from 0.25
        self.side_safety_distance = 0.4  # For side obstacle checking
        
        # Setup ROS subscriptions
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.odom_callback,
            10
        )
        
        # Create control timer
        self.create_timer(0.05, self.control_loop)
        
        # Initialize LidarProcessor
        self.lidar_processor = LidarProcessor()
        
    def control_loop(self):
        """Main control loop for exploration."""
        if self.current_pose is None or self.last_scan is None:
            return

        # Process LIDAR data
        sector_data = self.lidar_processor.process_scan(self.last_scan.ranges)
        if not sector_data:
            self.motors.stop()
            return

        # Get navigation command
        command = self.lidar_processor.get_navigation_command(sector_data)
        
        # Execute command
        if command == 'forward':
            self.motors.forward(speed=0.8)
        elif command == 'reverse':
            self.motors.backward()
            time.sleep(0.3)  # Brief reverse
            # Turn randomly after reversing
            if np.random.choice([True, False]):
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            time.sleep(0.3)
        elif command == 'turn_left':
            self.motors.turn_left()
        elif command == 'turn_right':
            self.motors.turn_right()
        elif command == 'stop':
            self.motors.stop()

        # Small delay for stability
        time.sleep(0.02)

    def check_obstacles(self):
        """Check for obstacles and handle avoidance."""
        if self.last_scan is None:
            return False

        # Get distances in each direction
        front_ranges = self.last_scan.ranges[350:] + self.last_scan.ranges[:10]
        left_ranges = self.last_scan.ranges[20:60]
        right_ranges = self.last_scan.ranges[300:340]
        
        # Get minimum valid distances
        front_dist = min([r for r in front_ranges if 0.1 < r < 5.0], default=5.0)
        left_dist = min([r for r in left_ranges if 0.1 < r < 5.0], default=5.0)
        right_dist = min([r for r in right_ranges if 0.1 < r < 5.0], default=5.0)

        # Handle obstacles
        if front_dist < self.critical_distance:
            self.get_logger().warn(f"Obstacle ahead: {front_dist:.2f}m")
            self.motors.stop()
            self.motors.backward()
            time.sleep(1.0)  # Back up for 1 second
            
            # Turn away from closest obstacle
            if left_dist > right_dist:
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            time.sleep(0.5)  # Turn for 0.5 seconds
            return True
            
        elif front_dist < self.safety_distance:
            # Turn away from closest obstacle
            if left_dist > right_dist:
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            return True
            
        return False

    def handle_obstacle(self):
        """Handle immediate obstacle."""
        self.motors.stop()
        self.motors.backward()
        time.sleep(1.0)
        
        # Get latest distances
        left_ranges = self.last_scan.ranges[20:60]
        right_ranges = self.last_scan.ranges[300:340]
        left_dist = min([r for r in left_ranges if 0.1 < r < 5.0], default=5.0)
        right_dist = min([r for r in right_ranges if 0.1 < r < 5.0], default=5.0)
        
        # Turn in direction with more space
        if left_dist > right_dist:
            self.motors.turn_left()
        else:
            self.motors.turn_right()
        time.sleep(0.5)

    def find_next_target(self):
        """Find next exploration target."""
        target = self.navigator.find_best_frontier()
        if target:
            self.current_target = target
            self.get_logger().info(f"New target: {target}")
            self.state = RobotState.EXPLORING

    def lidar_callback(self, msg):
        """Process LIDAR data."""
        self.last_scan = msg

    def map_callback(self, msg):
        """Process map updates."""
        self.navigator.update_map(msg)

    def odom_callback(self, msg):
        """Process odometry data."""
        self.current_pose = msg.pose.pose
        self.navigator.update_pose(msg.pose.pose)

    def get_yaw_from_quaternion(self, q: Quaternion) -> float:
        """Extract yaw from quaternion."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Stopping robot")
        robot.motors.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
