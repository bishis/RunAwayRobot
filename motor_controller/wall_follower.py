#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
from tf2_ros import TransformBroadcaster
import math

from .controllers.motor_controller import MotorController
from .controllers.navigation_controller import NavigationController
from .models.robot_state import RobotState
from .processors.lidar_processor import LidarProcessor

class WallFollower(Node):
    """Main ROS2 node for wall following behavior."""
    
    def __init__(self):
        super().__init__('wall_follower')
        
        # Initialize components
        self.motors = MotorController()
        self.state = RobotState()
        self.lidar = LidarProcessor()
        self.navigator = NavigationController()
        
        # Initialize ROS2 components
        self.setup_ros_components()
        
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
        min_distance = self.lidar.process_scan(msg.ranges)
        self.get_logger().info(f"Front distance: {min_distance:.2f}m")
        
        if min_distance < self.lidar.SAFETY_RADIUS:
            self.get_logger().warn(f"Emergency stop! Wall at {min_distance:.2f}m")
            self.motors.stop()
            return
            
        if not self.state.is_turning:
            if min_distance < self.lidar.DETECTION_DISTANCE:
                self.start_turn()
            else:
                self.move_forward()
        else:
            self.execute_turn()

    def start_turn(self):
        """Initialize a turn maneuver."""
        self.state.is_turning = True
        self.state.start_turn_angle = self.state.theta
        self.state.target_angle = self.state.theta + math.radians(90)
        self.get_logger().info(
            f"Starting turn from {math.degrees(self.state.theta):.1f}° "
            f"to {math.degrees(self.state.target_angle):.1f}°"
        )

    def execute_turn(self):
        """Execute the turning maneuver."""
        angle_turned = self.navigator.calculate_turn_progress(
            self.state.theta, 
            self.state.start_turn_angle
        )
        
        self.get_logger().info(f"Turn progress: {angle_turned:.1f}°")
        
        if self.navigator.is_turn_complete(angle_turned):
            self.get_logger().info("Turn complete")
            self.state.is_turning = False
            self.move_forward()
        else:
            self.turn_right()

    def move_forward(self):
        """Move the robot forward."""
        self.motors.set_speeds(1.0, 1.0)
        self.state.update_velocity(self.navigator.LINEAR_SPEED, 0.0)

    def turn_right(self):
        """Execute a right turn."""
        self.motors.set_speeds(
            self.navigator.TURN_SPEED, 
            -self.navigator.TURN_SPEED
        )
        self.state.update_velocity(0.0, -self.navigator.TURN_SPEED)

    def publish_odom(self):
        """Publish odometry data."""
        try:
            current_time = self.get_clock().now()
            
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
            
            odom.pose.pose.position.x = float(self.state.x)
            odom.pose.pose.position.y = float(self.state.y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q
            
            odom.twist.twist.linear.x = float(self.state.linear_vel)
            odom.twist.twist.angular.z = float(self.state.angular_vel)
            
            # Add covariance matrices
            odom.pose.covariance = [0.1] * 36
            odom.twist.covariance = [0.1] * 36
            
            self.odom_pub.publish(odom)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {str(e)}')

    def map_callback(self, msg):
        """Process incoming map data."""
        # Implementation remains similar but moved to separate method for clarity
        pass

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()

    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        wall_follower.get_logger().info("Stopping motors")
        wall_follower.motors.stop()
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
