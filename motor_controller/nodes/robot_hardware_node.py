#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ..controllers.motor_controller import MotorController
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class RobotHardwareNode(Node):
    def __init__(self):
        super().__init__('robot_hardware')
        
        # Define QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Add heartbeat publisher with reliable QoS
        self.heartbeat_pub = self.create_publisher(
            String,
            'robot_heartbeat',
            qos_profile
        )
        self.create_timer(1.0, self.publish_heartbeat)
        
        # Add command echo publisher with reliable QoS
        self.cmd_echo_pub = self.create_publisher(
            String,
            'cmd_echo',
            qos_profile
        )
        
        # Initialize motor controller
        self.declare_parameter('left_motor_pin', 18)
        self.declare_parameter('right_motor_pin', 12)
        
        left_pin = self.get_parameter('left_motor_pin').value
        right_pin = self.get_parameter('right_motor_pin').value
        
        self.motors = MotorController(left_pin=left_pin, right_pin=right_pin)
        
        # Subscribe to movement commands with reliable QoS
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.get_logger().info('Robot hardware node initialized')
        
    def publish_heartbeat(self):
        msg = String()
        msg.data = f"Robot Hardware Online - {self.get_clock().now().to_msg()}"
        self.heartbeat_pub.publish(msg)
        self.get_logger().info("Published heartbeat")
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        # Echo received command
        echo_msg = String()
        echo_msg.data = f"Received cmd_vel - linear: {msg.linear.x:.2f}, angular: {msg.angular.z:.2f}"
        self.cmd_echo_pub.publish(echo_msg)
        self.get_logger().info(echo_msg.data)
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert velocity commands to motor actions
        if abs(angular_z) > 0.1:  # Turning
            if angular_z > 0:
                self.motors.turn_left()
            else:
                self.motors.turn_right()
        elif abs(linear_x) > 0.1:  # Moving forward/backward
            if linear_x > 0:
                self.motors.forward(speed=abs(linear_x))
            else:
                self.motors.backward()
        else:
            self.motors.stop()

def main(args=None):
    rclpy.init(args=args)
    node = RobotHardwareNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motors.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 