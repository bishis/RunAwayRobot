#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Create publisher and subscriber
        self.pub = self.create_publisher(
            Image, 
            '/camera/image_raw_flipped', 
            10
        )
        
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Image flipper node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Flip image 180° (flip both horizontally and vertically)
            flipped = cv2.flip(cv_image, -1)
            
            # Convert back to ROS image and publish
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped, encoding="bgr8")
            flipped_msg.header = msg.header  # Preserve timestamp and frame_id
            self.pub.publish(flipped_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 