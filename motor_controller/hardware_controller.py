#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    """Controls robot hardware using binary speed values"""
    
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Declare parameters
        self.declare_parameter('left_dir_pin', 27)
        self.declare_parameter('left_pwm_pin', 18)
        self.declare_parameter('right_dir_pin', 17)
        self.declare_parameter('right_pwm_pin', 4)
        self.declare_parameter('pwm_frequency', 1000)
        
        # Get parameters
        left_dir_pin = self.get_parameter('left_dir_pin').value
        left_pwm_pin = self.get_parameter('left_pwm_pin').value
        right_dir_pin = self.get_parameter('right_dir_pin').value
        right_pwm_pin = self.get_parameter('right_pwm_pin').value
        pwm_frequency = self.get_parameter('pwm_frequency').value
        
        # Create motor controller with parameters
        self.motor_controller = MotorController(
            left_dir_pin=left_dir_pin,
            left_pwm_pin=left_pwm_pin,
            right_dir_pin=right_dir_pin,
            right_pwm_pin=right_pwm_pin,
            pwm_frequency=pwm_frequency
        )
        
        # Create subscriber for wheel speeds
        self.wheel_speeds_sub = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        # Create publisher for actual speeds (for debugging)
        self.actual_speeds_pub = self.create_publisher(
            Twist,
            'actual_speeds',
            10
        )
        
        self.get_logger().info('Hardware controller initialized')

    def wheel_speeds_callback(self, msg: Twist):
        try:
            # Get commanded speeds
            linear_x = msg.linear.x
            linear_x = linear_x * 0.6
            angular_z = msg.angular.z
            angular_z = angular_z * 5
            # Send commands to motor controller and get actual speeds
            left_speed, right_speed, left_pwm, right_pwm = self.motor_controller.set_speeds(linear_x, angular_z)
            
            # Publish actual speeds for debugging
            actual = Twist()
            actual.linear.x = linear_x
            actual.angular.z = angular_z
            self.actual_speeds_pub.publish(actual)
            
            # Debug logging with normalized speeds and PWM values
            self.get_logger().info(
                f'Speeds (-1 to 1) - Left: {left_speed:6.3f}, Right: {right_speed:6.3f} | ' +
                f'PWM% - Left: {left_pwm*100:3.0f}%, Right: {right_pwm*100:3.0f}% | ' +
                f'Input - Linear: {linear_x:6.3f}, Angular: {angular_z:6.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in wheel speeds callback: {str(e)}')
            self.motor_controller.stop_motors()

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'motor_controller'):
            self.motor_controller.stop_motors()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure motors are stopped
        if hasattr(node, 'motor_controller'):
            node.motor_controller.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()