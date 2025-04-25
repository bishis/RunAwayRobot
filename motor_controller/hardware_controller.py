#!/usr/bin/env python3

import rclpy
import os
import glob
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController
from .controllers.display_controller import DisplayController
from std_msgs.msg import String
from sensor_msgs.msg import Image
from .utils.soundbytes import get_jingle
import random
from ament_index_python.packages import get_package_share_directory

class HardwareController(Node):
    """Controls robot hardware including motors, display and buzzer"""
    
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Declare motor parameters
        self.declare_parameter('left_dir_pin', 27)
        self.declare_parameter('left_pwm_pin', 18)
        self.declare_parameter('right_dir_pin', 17)
        self.declare_parameter('right_pwm_pin', 4)
        self.declare_parameter('pwm_frequency', 1000)
        
        # Declare display and buzzer parameters
        self.declare_parameter('buzzer_pin', 21)
        self.declare_parameter('oled_address', '0x3C')
        self.declare_parameter('oled_width', 128)
        self.declare_parameter('oled_height', 64)
        
        # Get motor parameters
        left_dir_pin = self.get_parameter('left_dir_pin').value
        left_pwm_pin = self.get_parameter('left_pwm_pin').value
        right_dir_pin = self.get_parameter('right_dir_pin').value
        right_pwm_pin = self.get_parameter('right_pwm_pin').value
        pwm_frequency = self.get_parameter('pwm_frequency').value
        
        # Get display parameters
        buzzer_pin = self.get_parameter('buzzer_pin').value
        
        # Create motor controller with parameters
        self.motor_controller = MotorController(
            left_dir_pin=left_dir_pin,
            left_pwm_pin=left_pwm_pin,
            right_dir_pin=right_dir_pin,
            right_pwm_pin=right_pwm_pin,
            pwm_frequency=pwm_frequency
        )
        
        self.display_controller = DisplayController(self)

        self.previous_image = None
        
        # Find the images folder - update to use the package directory
        self.pkg_dir = get_package_share_directory('motor_controller')
        self.images_dir = os.path.join(self.pkg_dir, 'images')
        
        # Create a map of available images
        self.image_map = self._get_available_images()
        
        self.wheel_speeds_sub = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        self.actual_speeds_pub = self.create_publisher(
            Twist,
            'actual_speeds',
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )
        
        self.alert_sub = self.create_subscription(
            String,
            'sound_alert',
            self.alert_callback,
            10
        )
        
        # Display startup message
        self.show_status_image("robot", fallback_text="Robot Ready")
        sound = get_jingle('startup')
        self.display_controller.sound_buzzer(sound)
        
        self.get_logger().info('Hardware controller initialized with display/buzzer support')

    def _get_available_images(self):
        """Build a dictionary mapping status names to image files"""
        image_map = {}
        
        try:
            if os.path.exists(self.images_dir):
                # Find all image files in the images directory
                image_files = glob.glob(os.path.join(self.images_dir, "*.png"))
                image_files.extend(glob.glob(os.path.join(self.images_dir, "*.jpg")))
                image_files.extend(glob.glob(os.path.join(self.images_dir, "*.bmp")))
                image_files.extend(glob.glob(os.path.join(self.images_dir, "*.jpeg")))
                image_files.extend(glob.glob(os.path.join(self.images_dir, "*.gif")))
                
                # Create mapping from base name to file path
                for img_path in image_files:
                    basename = os.path.basename(img_path)
                    name, _ = os.path.splitext(basename)
                    image_map[name.lower()] = img_path
                
                self.get_logger().info(f'Found {len(image_map)} images in {self.images_dir}')
            else:
                self.get_logger().warn(f'Images directory not found: {self.images_dir}')
        except Exception as e:
            self.get_logger().error(f'Error loading images: {str(e)}')
        
        return image_map

    def show_status_image(self, status_name, fallback_text=None):
        """Show an image for the given status or fallback to text"""
        status_name = status_name.lower().strip()
        
        # Check if have an exact match
        if status_name in self.image_map:
            if status_name.endswith('.gif'):
                self.display_controller.show_gif(self.image_map[status_name])
            else:
                self.get_logger().info(f'Showing image for status: {status_name}')
                self.display_controller.show_image(self.image_map[status_name])
            return True
            
        # Check for partial matches (e.g., "human_detected" matches "human")
        for img_name, img_path in self.image_map.items():
            if status_name.startswith(img_name) or img_name.startswith(status_name):
                self.get_logger().info(f'Showing partial match image: {img_name} for {status_name}')
                if img_name.endswith('.gif'):
                    self.display_controller.show_gif(img_path)
                else:
                    self.display_controller.show_image(img_path)
                return True
        
        # No matching image found, fall back to text
        if fallback_text:
            self.display_controller.show_text(fallback_text)
        else:
            self.display_controller.show_text(f"Status: {status_name}")
        
        return False

    def wheel_speeds_callback(self, msg: Twist):
        try:
            # Get commanded speeds
            linear_x = msg.linear.x
            angular_z = msg.angular.z
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
    
    def status_callback(self, msg: String):
        """Handle status updates for display"""
        try:
            status_text = msg.data.strip()
            if status_text == "" or status_text == self.previous_image:
                return    # Try to display an image for this status
            if not self.show_status_image(status_text, fallback_text=status_text):
                self.get_logger().info(f'No image for status: {status_text}, showing text only')
            self.previous_image = status_text
            
        except Exception as e:
            self.get_logger().error(f'Error processing display status: {str(e)}')
    
    def alert_callback(self, msg: String):
        """Handle sound alert requests"""
        try:
            if msg.data.strip() == "":
                return
            alert_type = msg.data.strip()
            sound = get_jingle(alert_type)
            self.display_controller.sound_buzzer(sound)
        except Exception as e:
            self.get_logger().error(f'Error processing sound alert: {str(e)}')

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'motor_controller'):
            self.motor_controller.stop_motors()
        
        if hasattr(self, 'display_controller'):
            self.display_controller.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure motors are stopped and display is cleaned up
        if hasattr(node, 'motor_controller'):
            node.motor_controller.stop_motors()
            
        if hasattr(node, 'display_controller'):
            node.display_controller.cleanup()
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()