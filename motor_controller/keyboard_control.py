#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import tty
import termios
import select
import threading
import os
import time
from .controllers.motor_controller import MotorController

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.running = True
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=18,
            right_pin=12
        )
        
        # Track current key state
        self.current_key = None
        
        self.get_logger().info('Initializing keyboard controller...')
        
        try:
            # Check if running in terminal
            if sys.stdin.isatty():
                self.get_logger().info('Running in terminal mode')
                # Start keyboard reading thread
                self.keyboard_thread = threading.Thread(target=self._read_keyboard)
                self.keyboard_thread.start()
                
                # Start motor control thread
                self.motor_thread = threading.Thread(target=self._motor_control_loop)
                self.motor_thread.start()
                
                self.get_logger().info('Keyboard controller started. Hold WASD to move, Q to quit')
            else:
                self.get_logger().warn('Not running in terminal mode')
                self.get_logger().info('Please run in a terminal with: ros2 run motor_controller keyboard_control')
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            
    def _motor_control_loop(self):
        """Continuously control motors based on current key"""
        while self.running:
            if self.current_key:
                if self.current_key == 'w':
                    self.motors.forward()
                elif self.current_key == 's':
                    self.motors.backward()
                elif self.current_key == 'a':
                    self.motors.turn_left()
                elif self.current_key == 'd':
                    self.motors.turn_right()
            else:
                self.motors.stop()
            
            # Small delay to prevent CPU overuse
            time.sleep(0.01)
            
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
            self.get_logger().info('Controls: Hold WASD to move, Q to quit')
            
            while self.running:
                # Check if key is available
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    if key.lower() == 'q':
                        self.get_logger().info('Quit command received')
                        self.running = False
                        self.current_key = None
                        self.motors.stop()
                        break
                    
                    # Update current key
                    if key.lower() in ['w', 'a', 's', 'd']:
                        self.current_key = key.lower()
                        self.get_logger().info(f'Key pressed: {self.current_key}')
                    else:
                        self.current_key = None
                else:
                    # No key being pressed
                    self.current_key = None
                    
        except Exception as e:
            self.get_logger().error(f'Error reading keyboard: {str(e)}')
            self.motors.stop()
        finally:
            # Restore terminal settings
            if 'old_settings' in locals():
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    self.get_logger().info('Terminal settings restored')
                except Exception as e:
                    self.get_logger().error(f'Error restoring terminal settings: {str(e)}')
            self.motors.stop()
            
    def __del__(self):
        self.get_logger().info('Shutting down keyboard controller')
        self.running = False
        self.current_key = None
        if hasattr(self, 'motors'):
            self.motors.stop()
        if hasattr(self, 'keyboard_thread'):
            try:
                self.keyboard_thread.join(timeout=1.0)
                self.get_logger().info('Keyboard thread joined successfully')
            except Exception as e:
                self.get_logger().error(f'Error joining keyboard thread: {str(e)}')
        if hasattr(self, 'motor_thread'):
            try:
                self.motor_thread.join(timeout=1.0)
                self.get_logger().info('Motor thread joined successfully')
            except Exception as e:
                self.get_logger().error(f'Error joining motor thread: {str(e)}')

def main(args=None):
    try:
        rclpy.init(args=args)
        print("ROS 2 initialized")
        
        controller = KeyboardController()
        print("Controller created")
        
        try:
            print("Starting spin")
            rclpy.spin(controller)
        except KeyboardInterrupt:
            print("Keyboard interrupt received")
        finally:
            print("Cleaning up")
            controller.running = False
            if hasattr(controller, 'motors'):
                controller.motors.stop()
            controller.destroy_node()
            rclpy.shutdown()
            print("Cleanup complete")
            
    except Exception as e:
        print(f"Error in main: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main() 