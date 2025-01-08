#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
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
        self.linear_speed = 1.0  # Make sure these are floats
        self.angular_speed = 1.0
        
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
                self.get_logger().info('Please run in a terminal with: ros2 run motor_controller keyboard_control')
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            
    def create_twist_message(self, linear_x=0.0, angular_z=0.0):
        """Create a properly formatted Twist message"""
        msg = Twist()
        
        # Create and set linear vector
        msg.linear = Vector3()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Create and set angular vector
        msg.angular = Vector3()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        
        return msg
        
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
                    
                    # Initialize velocities
                    linear_x = 0.0
                    angular_z = 0.0
                    
                    # Set velocities based on key
                    if key.lower() == 'w':
                        linear_x = self.linear_speed
                        self.get_logger().info('Forward')
                    elif key.lower() == 's':
                        linear_x = -self.linear_speed
                        self.get_logger().info('Backward')
                    elif key.lower() == 'a':
                        angular_z = self.angular_speed
                        self.get_logger().info('Left')
                    elif key.lower() == 'd':
                        angular_z = -self.angular_speed
                        self.get_logger().info('Right')
                    
                    # Create and publish message
                    msg = self.create_twist_message(linear_x, angular_z)
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