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
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Setup TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create timer for odometry publishing
        self.create_timer(0.01, self.publish_odom)
        
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
        WHEEL_BASE = 0.2  # Distance between wheels in meters
        
        if command == 'stop':
            self.get_logger().warn("Emergency stop!")
            self.motors.stop()
            self.state.update_velocity(0.0, 0.0)
            
        elif command == 'reverse':
            self.get_logger().info("Reversing")
            self.motors.set_speeds(-0.7, -0.7)
            self.state.update_velocity(-0.3, 0.0)  # Linear velocity only
            
        elif command == 'turn_left':
            self.get_logger().info("Turning left")
            self.motors.set_speeds(-1, 1)
            self.state.update_velocity(0.0, 1.0)  # Angular velocity only
            
        elif command == 'turn_right':
            self.get_logger().info("Turning right")
            self.motors.set_speeds(1, -1)
            self.state.update_velocity(0.0, -1.0)  # Angular velocity only
            
        elif command == 'forward':
            self.get_logger().info("Moving forward")
            self.motors.set_speeds(1, 1)
            self.state.update_velocity(0.3, 0.0)  # Linear velocity only

    def publish_odom(self):
        """Publish odometry data."""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9

            # Update robot's position based on velocities
            if self.state.linear_vel != 0.0 or self.state.angular_vel != 0.0:
                # Calculate distance traveled
                delta_x = self.state.linear_vel * math.cos(self.state.theta) * dt
                delta_y = self.state.linear_vel * math.sin(self.state.theta) * dt
                delta_th = self.state.angular_vel * dt

                # Update position
                self.state.x += delta_x
                self.state.y += delta_y
                self.state.theta += delta_th
                self.state.theta = math.atan2(math.sin(self.state.theta), 
                                            math.cos(self.state.theta))

            # Create and publish transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(self.state.x)
            t.transform.translation.y = float(self.state.y)
            t.transform.translation.z = 0.0
            
            # Create quaternion from yaw
            q = Quaternion()
            q.z = float(math.sin(self.state.theta / 2.0))
            q.w = float(math.cos(self.state.theta / 2.0))
            t.transform.rotation = q
            
            self.tf_broadcaster.sendTransform(t)
            
            # Create and publish odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Set position
            odom.pose.pose.position.x = float(self.state.x)
            odom.pose.pose.position.y = float(self.state.y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q
            
            # Set velocity
            odom.twist.twist.linear.x = float(self.state.linear_vel)
            odom.twist.twist.angular.z = float(self.state.angular_vel)
            
            # Add covariance
            pose_covariance = [0.001] * 36  # Small covariance for better SLAM
            twist_covariance = [0.001] * 36
            
            # Set main diagonal elements
            pose_covariance[0] = 0.001   # x
            pose_covariance[7] = 0.001   # y
            pose_covariance[14] = 0.001  # z
            pose_covariance[21] = 0.001  # rotation about X axis
            pose_covariance[28] = 0.001  # rotation about Y axis
            pose_covariance[35] = 0.001  # rotation about Z axis
            
            odom.pose.covariance = pose_covariance
            odom.twist.covariance = twist_covariance
            
            self.odom_pub.publish(odom)
            
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {str(e)}')

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
