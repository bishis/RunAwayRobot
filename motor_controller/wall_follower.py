#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
import time

from .controllers.motor_controller import MotorController
from .controllers.navigation_controller import NavigationController
from .models.robot_state import RobotState
from .processors.lidar_processor import LidarProcessor

class MobileRobotController(Node):
    """Main ROS2 node for autonomous mobile robot control."""
    
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize components with just PWM pins
        self.motors = MotorController(
            left_pin=18,    # GPIO18 for left motor
            right_pin=12    # GPIO12 for right motor
        )
        self.state = RobotState()
        self.lidar = LidarProcessor()
        self.navigator = NavigationController()
        
        # Initialize ROS2 components
        self.setup_ros_components()
        
        # Remove the test timer
        # self.create_timer(5.0, self.test_motors)
        
    def setup_ros_components(self):
        """Setup ROS2 publishers, subscribers, and transforms."""
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Setup TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.last_time = self.get_clock().now()

    def lidar_callback(self, msg):
        """Handle incoming LIDAR data."""
        sector_data = self.lidar.process_scan(msg.ranges)
        if not sector_data:
            self.get_logger().warn("No valid LIDAR data")
            self.motors.stop()
            return
            
        # Log distances for debugging
        self.get_logger().info(
            f"Distances - Front: {sector_data['front']['min_distance']:.2f}m, "
            f"Left: {sector_data['left']['min_distance']:.2f}m, "
            f"Right: {sector_data['right']['min_distance']:.2f}m"
        )
        
        # Get navigation command
        command = self.lidar.get_navigation_command(sector_data)
        self.execute_command(command)
        
    def execute_command(self, command):
        """Execute navigation command."""
        if command == 'stop':
            self.get_logger().warn("Emergency stop!")
            self.motors.stop()
            self.state.update_velocity(0.0, 0.0)
            
        elif command == 'reverse':
            self.get_logger().info("Reversing")
            self.motors.set_speeds(-0.7, -0.7)
            self.state.update_velocity(-0.3, 0.0)
            
        elif command == 'turn_left':
            self.get_logger().info("Turning left")
            self.motors.set_speeds(-1, 1)
            self.state.update_velocity(0.0, 1.0)
            
        elif command == 'turn_right':
            self.get_logger().info("Turning right")
            self.motors.set_speeds(1, -1)
            self.state.update_velocity(0.0, -1.0)
            
        elif command == 'forward':
            self.get_logger().info("Moving forward")
            self.motors.set_speeds(1, 1)
            self.state.update_velocity(0.3, 0.0)

        # Publish the transform from base_link to laser
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'laser'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.18  # Height of the LIDAR from base
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')

    def map_callback(self, msg):
        """Process incoming map data."""
        # Implementation remains similar but moved to separate method for clarity
        pass

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
