#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading
import os

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.running = True
        
        # Movement parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        
        # Warning flag
        self.warning_shown = False
        
        # Check if running in terminal
        if sys.stdin.isatty():
            # Start keyboard reading thread
            self.keyboard_thread = threading.Thread(target=self._read_keyboard)
            self.keyboard_thread.start()
            self.get_logger().info('Keyboard controller started. Use WASD to move, Q to quit')
        else:
            # If not running in terminal, use alternative input method
            self.timer = self.create_timer(0.1, self._check_input)
            self.get_logger().info('Keyboard controller started in non-terminal mode.')
            self.get_logger().info('Please run: ros2 run motor_controller keyboard_control')
            
    def _check_input(self):
        """Alternative input method when not running in terminal"""
        if not self.warning_shown:
            self.get_logger().warn(
                'For keyboard control, please run directly with: '
                'ros2 run motor_controller keyboard_control'
            )
            self.warning_shown = True
        
    def _read_keyboard(self):
        """Read keyboard input in terminal mode"""
        try:
            # Save terminal settings
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            
            # Set terminal to raw mode
            tty.setraw(fd)
            
            while self.running:
                # Check if key is available
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    if key.lower() == 'q':
                        self.running = False
                        break
                        
                    msg = Twist()
                    
                    # Set velocities based on key
                    if key.lower() == 'w':
                        msg.linear.x = self.linear_speed
                    elif key.lower() == 's':
                        msg.linear.x = -self.linear_speed
                    elif key.lower() == 'a':
                        msg.angular.z = self.angular_speed
                    elif key.lower() == 'd':
                        msg.angular.z = -self.angular_speed
                        
                    self.publisher.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading keyboard: {str(e)}')
        finally:
            # Restore terminal settings
            if 'old_settings' in locals():
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
    def __del__(self):
        self.running = False
        if hasattr(self, 'keyboard_thread'):
            self.keyboard_thread.join()

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 