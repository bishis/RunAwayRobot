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
        self.state = RobotState.SCANNING
        self.current_pose = None
        self.last_scan = None
        self.current_target = None
        self.scan_start_time = None
        self.scan_duration = 4.0  # seconds to scan area
        
        # Safety parameters
        self.safety_distance = 0.4
        self.critical_distance = 0.25
        
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
        
    def control_loop(self):
        """Main control loop for exploration."""
        if self.current_pose is None or self.last_scan is None:
            self.get_logger().warn("Waiting for sensor data...")
            return

        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Check for obstacles
        if self.check_obstacles():
            return  # Let obstacle avoidance take over
            
        # State machine
        if self.state == RobotState.SCANNING:
            if self.scan_start_time is None:
                self.scan_start_time = time.time()
                self.motors.move(0, self.motors.max_angular_speed * 0.5)  # Start rotating
            elif time.time() - self.scan_start_time >= self.scan_duration:
                self.state = RobotState.EXPLORING
                self.scan_start_time = None
                self.find_next_target()
            
        elif self.state == RobotState.EXPLORING:
            if self.current_target is None:
                self.find_next_target()
                if self.current_target is None:
                    self.state = RobotState.SCANNING
                    return
                    
            # Check if we've reached the target
            dx = self.current_target[0] - current_x
            dy = self.current_target[1] - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < self.navigator.goal_tolerance:
                self.get_logger().info("Reached target, scanning area")
                self.current_target = None
                self.state = RobotState.SCANNING
                return
                
            # Move towards target
            self.motors.move_to_pose(
                current_x, current_y, current_yaw,
                self.current_target[0], self.current_target[1]
            )

    def check_obstacles(self):
        """Check for obstacles and handle avoidance."""
        if self.last_scan is None:
            return False
            
        # Check front sector for obstacles
        front_ranges = self.last_scan.ranges[350:] + self.last_scan.ranges[:10]
        valid_ranges = [r for r in front_ranges if 0.1 < r < 5.0]
        
        if not valid_ranges:
            return False
            
        min_distance = min(valid_ranges)
        
        if min_distance < self.critical_distance:
            self.get_logger().warn(f"Critical obstacle at {min_distance:.2f}m!")
            self.handle_obstacle()
            return True
            
        elif min_distance < self.safety_distance:
            self.get_logger().info(f"Obstacle detected at {min_distance:.2f}m")
            self.find_alternative_path()
            return True
            
        return False

    def handle_obstacle(self):
        """Handle immediate obstacle."""
        self.motors.stop()
        self.state = RobotState.REVERSING
        self.motors.set_speeds(-1.0, -1.0)
        time.sleep(1.0)
        self.find_alternative_path()

    def find_alternative_path(self):
        """Find alternative path to target."""
        if self.current_target is None:
            self.state = RobotState.SCANNING
            return
            
        # Check left and right for clear path
        left_ranges = self.last_scan.ranges[60:120]
        right_ranges = self.last_scan.ranges[240:300]
        
        left_clear = any(r > self.safety_distance for r in left_ranges)
        right_clear = any(r > self.safety_distance for r in right_ranges)
        
        if left_clear and not right_clear:
            self.motors.set_speeds(-1.0, 1.0)  # Turn left
        elif right_clear and not left_clear:
            self.motors.set_speeds(1.0, -1.0)  # Turn right
        else:
            # Choose direction based on target position
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            target_angle = math.atan2(
                self.current_target[1] - self.current_pose.position.y,
                self.current_target[0] - self.current_pose.position.x
            )
            
            angle_diff = target_angle - current_yaw
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            
            if angle_diff > 0:
                self.motors.set_speeds(-1.0, 1.0)  # Turn left
            else:
                self.motors.set_speeds(1.0, -1.0)  # Turn right

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
