#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.running = True
        
        # Movement parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        
        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self._read_keyboard)
        self.keyboard_thread.start()
        
        self.get_logger().info('Keyboard controller started. Use WASD to move, Q to quit')
        
    def _read_keyboard(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
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
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
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