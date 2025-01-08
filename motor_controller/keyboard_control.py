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
        self.linear_speed = 1  # Reduced speed for safety
        self.angular_speed = 1  # Reduced speed for safety
        
        # Warning flag
        self.warning_shown = False
        
        self.get_logger().info('Initializing keyboard controller...')
        
        try:
            # Check if running in terminal
            if sys.stdin.isatty():
                self.get_logger().info('Running in terminal mode')
                # Start keyboard reading thread
                self.keyboard_thread = threading.Thread(target=self._read_keyboard)
                self.keyboard_thread.start()
                self.get_logger().info('Keyboard controller started. Use WASD to move, Q to quit')
            else:
                self.get_logger().warn('Not running in terminal mode')
                # If not running in terminal, use alternative input method
                self.timer = self.create_timer(0.1, self._check_input)
                self.get_logger().info('Started in non-terminal mode.')
                self.get_logger().info('Please run in a terminal with: ros2 run motor_controller keyboard_control')
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            
    def _check_input(self):
        """Alternative input method when not running in terminal"""
        if not self.warning_shown:
            self.get_logger().warn(
                'For keyboard control, please run directly in a terminal with: '
                'ros2 run motor_controller keyboard_control'
            )
            self.warning_shown = True
        
    def _read_keyboard(self):
        """Read keyboard input in terminal mode"""
        self.get_logger().info('Starting keyboard reading thread')
        
        try:
            # Save terminal settings
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            
            # Set terminal to raw mode
            tty.setraw(fd)
            
            self.get_logger().info('Terminal configured for raw input')
            self.get_logger().info('Controls: W=forward, S=backward, A=left, D=right, Q=quit')
            
            while self.running:
                # Check if key is available
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    if key.lower() == 'q':
                        self.get_logger().info('Quit command received')
                        self.running = False
                        break
                        
                    msg = Twist()
                    
                    # Set velocities based on key
                    if key.lower() == 'w':
                        msg.linear.x = self.linear_speed
                        self.get_logger().debug('Forward')
                    elif key.lower() == 's':
                        msg.linear.x = -self.linear_speed
                        self.get_logger().debug('Backward')
                    elif key.lower() == 'a':
                        msg.angular.z = self.angular_speed
                        self.get_logger().debug('Left')
                    elif key.lower() == 'd':
                        msg.angular.z = -self.angular_speed
                        self.get_logger().debug('Right')
                        
                    self.publisher.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading keyboard: {str(e)}')
        finally:
            # Restore terminal settings
            if 'old_settings' in locals():
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    self.get_logger().info('Terminal settings restored')
                except Exception as e:
                    self.get_logger().error(f'Error restoring terminal settings: {str(e)}')
            
    def __del__(self):
        self.get_logger().info('Shutting down keyboard controller')
        self.running = False
        if hasattr(self, 'keyboard_thread'):
            try:
                self.keyboard_thread.join(timeout=1.0)
                self.get_logger().info('Keyboard thread joined successfully')
            except Exception as e:
                self.get_logger().error(f'Error joining keyboard thread: {str(e)}')

def main(args=None):
    try:
        rclpy.init(args=args)
        print("ROS 2 initialized")  # Debug print
        
        controller = KeyboardController()
        print("Controller created")  # Debug print
        
        try:
            print("Starting spin")  # Debug print
            rclpy.spin(controller)
        except KeyboardInterrupt:
            print("Keyboard interrupt received")  # Debug print
        finally:
            print("Cleaning up")  # Debug print
            controller.running = False
            controller.destroy_node()
            rclpy.shutdown()
            print("Cleanup complete")  # Debug print
            
    except Exception as e:
        print(f"Error in main: {str(e)}")  # Debug print
        sys.exit(1)

if __name__ == '__main__':
    main() 