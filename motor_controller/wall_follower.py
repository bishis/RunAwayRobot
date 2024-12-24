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
from .models.robot_state import RobotState
from .models.mapping_state import MapperState, MappingState
from .processors.lidar_processor import LidarProcessor

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize components
        self.motors = MotorController(
            left_pin=18,
            right_pin=12
        )
        self.state = RobotState()
        self.mapper_state = MapperState()
        self.lidar = LidarProcessor()
        
        # Variables for mapping
        self.last_odom = None
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        
        # Setup ROS components
        self.setup_ros_components()
        
        # Create timer for mapping control
        self.create_timer(0.1, self.mapping_control_loop)
        
    def setup_ros_components(self):
        """Setup ROS2 publishers, subscribers, and transforms."""
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        
        # Setup TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        """Process odometry data for mapping."""
        if self.last_odom is None:
            self.last_odom = msg
            return

        # Calculate distance traveled
        dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        self.distance_traveled += distance

        # Calculate rotation
        current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        last_yaw = self.get_yaw_from_quaternion(self.last_odom.pose.pose.orientation)
        angle_diff = current_yaw - last_yaw
        
        # Normalize angle difference
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        self.angle_turned += angle_diff
        self.last_odom = msg

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def mapping_control_loop(self):
        """Execute the square wave mapping pattern."""
        if self.last_odom is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # Publish transform
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'laser'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.18
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')

        # Debug output
        self.get_logger().info(
            f"State: {self.mapper_state.current_state}, "
            f"Distance: {self.distance_traveled:.2f}m, "
            f"Angle: {math.degrees(self.angle_turned):.2f}°"
        )

        # Adjust speeds for better control
        if self.mapper_state.current_state == MappingState.FORWARD:
            if self.distance_traveled >= self.mapper_state.segment_length:
                self.get_logger().info("Segment complete, turning...")
                self.distance_traveled = 0
                self.angle_turned = 0
                if self.mapper_state.direction > 0:
                    self.mapper_state.current_state = MappingState.TURN_RIGHT
                else:
                    self.mapper_state.current_state = MappingState.TURN_LEFT
                self.motors.stop()
            else:
                self.motors.set_speeds(0.3, 0.3)  # Reduced speed for better control

        elif self.mapper_state.current_state in [MappingState.TURN_LEFT, MappingState.TURN_RIGHT]:
            target_angle = math.radians(self.mapper_state.turn_angle)
            if abs(self.angle_turned) >= target_angle:
                self.get_logger().info("Turn complete, moving forward...")
                self.distance_traveled = 0
                self.angle_turned = 0
                self.mapper_state.current_state = MappingState.FORWARD
                self.mapper_state.direction *= -1
                self.motors.stop()
            else:
                if self.mapper_state.current_state == MappingState.TURN_LEFT:
                    self.motors.set_speeds(-0.3, 0.3)  # Reduced turn speed
                else:
                    self.motors.set_speeds(0.3, -0.3)  # Reduced turn speed

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        sector_data = self.lidar.process_scan(msg.ranges)
        if not sector_data:
            self.get_logger().warn("No valid LIDAR data")
            self.motors.stop()
            return

        # Emergency stop if obstacles are too close
        min_distance = min(
            sector_data['front']['min_distance'],
            sector_data['left']['min_distance'],
            sector_data['right']['min_distance']
        )
        
        if min_distance < 0.3:  # 30cm safety margin
            self.get_logger().warn("Obstacle too close, stopping!")
            self.motors.stop()
            self.mapper_state.current_state = MappingState.STOP

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
