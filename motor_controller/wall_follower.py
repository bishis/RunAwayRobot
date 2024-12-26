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
        
    def control_loop(self):
        """Main control loop for exploration."""
        if self.current_pose is None or self.last_scan is None:
            self.get_logger().warn("Waiting for sensor data...")
            return

        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Always check obstacles first
        if self.check_obstacles():
            return

        # State machine
        if self.state == RobotState.SCANNING:
            if self.scan_start_time is None:
                self.scan_start_time = time.time()
                self.motors.move(0, self.rotation_speed)
            elif time.time() - self.scan_start_time >= self.scan_duration:
                self.state = RobotState.EXPLORING
                self.scan_start_time = None
                self.find_next_target()
            
        elif self.state == RobotState.EXPLORING:
            if self.current_target is None:
                self.find_next_target()
                if self.current_target is None:
                    # If no target found, do a quick scan
                    self.state = RobotState.SCANNING
                    return

            # Calculate angle to target
            dx = self.current_target[0] - current_x
            dy = self.current_target[1] - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - current_yaw
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            # Log navigation info
            self.get_logger().info(
                f"Distance to target: {distance:.2f}m, "
                f"Angle diff: {math.degrees(angle_diff):.1f}°"
            )
            
            if distance < self.navigator.goal_tolerance:
                self.get_logger().info("Reached target")
                self.current_target = None
                self.state = RobotState.SCANNING
                return

            # Decide movement based on angle difference
            if abs(angle_diff) > self.min_turn_angle:
                # Need to rotate first
                turn_speed = self.rotation_speed if angle_diff > 0 else -self.rotation_speed
                self.motors.move(0.0, turn_speed)
            else:
                # Move forward with slight turning correction
                forward_speed = min(self.forward_speed, distance)
                turn_correction = angle_diff  # Proportional correction
                self.motors.move(forward_speed, turn_correction)

    def check_obstacles(self):
        """Check for obstacles and handle avoidance."""
        if self.last_scan is None:
            return False

        # Process LIDAR data for different sectors
        front_ranges = self.last_scan.ranges[350:] + self.last_scan.ranges[:10]
        left_ranges = self.last_scan.ranges[20:60]
        right_ranges = self.last_scan.ranges[300:340]
        
        # Get minimum distances for each sector
        front_dist = min([r for r in front_ranges if 0.1 < r < 5.0], default=5.0)
        left_dist = min([r for r in left_ranges if 0.1 < r < 5.0], default=5.0)
        right_dist = min([r for r in right_ranges if 0.1 < r < 5.0], default=5.0)

        # Log distances
        self.get_logger().debug(
            f"Distances - Front: {front_dist:.2f}m, "
            f"Left: {left_dist:.2f}m, "
            f"Right: {right_dist:.2f}m"
        )

        if front_dist < self.critical_distance:
            self.get_logger().warn(f"Critical obstacle at {front_dist:.2f}m!")
            self.handle_obstacle()
            return True
            
        elif front_dist < self.safety_distance:
            self.get_logger().info(f"Obstacle ahead at {front_dist:.2f}m")
            self.find_alternative_path(left_dist, right_dist)
            return True
            
        # Handle side obstacles with gentle correction
        elif left_dist < self.side_safety_distance:
            self.motors.move(0.3, -0.5)  # Slow forward with right turn
            return True
        elif right_dist < self.side_safety_distance:
            self.motors.move(0.3, 0.5)   # Slow forward with left turn
            return True
            
        return False

    def handle_obstacle(self):
        """Handle immediate obstacle."""
        self.motors.stop()
        self.state = RobotState.REVERSING
        self.motors.set_speeds(-1.0, -1.0)
        time.sleep(1.0)
        self.find_alternative_path()

    def find_alternative_path(self, left_dist, right_dist):
        """Find alternative path to target."""
        if left_dist > right_dist + 0.2:  # Significantly more space on left
            self.motors.move(0.2, self.rotation_speed)  # Forward + left turn
        elif right_dist > left_dist + 0.2:  # Significantly more space on right
            self.motors.move(0.2, -self.rotation_speed)  # Forward + right turn
        else:
            # If similar space on both sides, choose based on target
            if self.current_target is not None:
                target_angle = math.atan2(
                    self.current_target[1] - self.current_pose.position.y,
                    self.current_target[0] - self.current_pose.position.x
                )
                current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
                angle_diff = target_angle - current_yaw
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi
                
                if angle_diff > 0:
                    self.motors.move(0.2, self.rotation_speed)
                else:
                    self.motors.move(0.2, -self.rotation_speed)
            else:
                # No target, reverse and rotate
                self.motors.move(-0.5, self.rotation_speed)

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
