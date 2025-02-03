#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        
        # Create subscriber for compressed image
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',  # Subscribe to compressed image
            self.image_callback,
            10
        )
        
        # Create publisher for flipped compressed image
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/camera/image_raw_flipped/compressed',  # Publish compressed image
            10
        )
        
        self.get_logger().info('Image flipper initialized')

    def image_callback(self, msg):
        """Process incoming compressed image messages"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Flip image horizontally
            flipped_image = cv2.flip(cv_image, 1)
            
            # Create compressed image message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            
            # Encode flipped image as JPEG
            _, compressed_data = cv2.imencode('.jpg', flipped_image, 
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])  # Adjust quality as needed
            compressed_msg.data = np.array(compressed_data).tobytes()
            
            # Publish compressed flipped image
            self.image_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageFlipper()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in image flipper node: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 