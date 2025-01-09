#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class DualCameraController(Node):
    """Controller for handling two Raspberry Pi cameras."""
    
    def __init__(self):
        super().__init__('dual_camera_controller')
        
        # Declare parameters
        self.declare_parameter('camera1_index', 0)
        self.declare_parameter('camera2_index', 1)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Get parameters
        self.camera1_index = self.get_parameter('camera1_index').value
        self.camera2_index = self.get_parameter('camera2_index').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize cameras
        self.camera1 = None
        self.camera2 = None
        self.running = True
        
        # Create publishers
        self.cam1_pub = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.cam2_pub = self.create_publisher(Image, 'camera2/image_raw', 10)
        self.cam1_info_pub = self.create_publisher(CameraInfo, 'camera1/camera_info', 10)
        self.cam2_info_pub = self.create_publisher(CameraInfo, 'camera2/camera_info', 10)
        
        # Start camera threads
        self.start_cameras()
        
        self.get_logger().info('Dual camera controller initialized')
        
    def start_cameras(self):
        """Initialize and start both cameras."""
        try:
            self.camera1 = cv2.VideoCapture(self.camera1_index)
            self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera1.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            self.camera2 = cv2.VideoCapture(self.camera2_index)
            self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera2.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # Start capture threads
            self.cam1_thread = threading.Thread(target=self.camera1_loop)
            self.cam2_thread = threading.Thread(target=self.camera2_loop)
            
            self.cam1_thread.start()
            self.cam2_thread.start()
            
            self.get_logger().info('Both cameras started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start cameras: {str(e)}')
            
    def camera1_loop(self):
        """Capture and publish frames from camera 1."""
        while self.running and rclpy.ok():
            try:
                ret, frame = self.camera1.read()
                if ret:
                    # Convert to ROS Image message
                    msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera1_link'
                    
                    # Publish image
                    self.cam1_pub.publish(msg)
                    
                    # Publish camera info
                    self.publish_camera_info(self.cam1_info_pub, 'camera1_link')
                    
            except Exception as e:
                self.get_logger().error(f'Camera 1 error: {str(e)}')
                
    def camera2_loop(self):
        """Capture and publish frames from camera 2."""
        while self.running and rclpy.ok():
            try:
                ret, frame = self.camera2.read()
                if ret:
                    # Convert to ROS Image message
                    msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera2_link'
                    
                    # Publish image
                    self.cam2_pub.publish(msg)
                    
                    # Publish camera info
                    self.publish_camera_info(self.cam2_info_pub, 'camera2_link')
                    
            except Exception as e:
                self.get_logger().error(f'Camera 2 error: {str(e)}')
                
    def publish_camera_info(self, publisher, frame_id):
        """Publish camera calibration info."""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = frame_id
        info_msg.height = self.height
        info_msg.width = self.width
        # Add calibration parameters if available
        publisher.publish(info_msg)
        
    def cleanup(self):
        """Clean up camera resources."""
        self.running = False
        if self.camera1:
            self.camera1.release()
        if self.camera2:
            self.camera2.release()
            
    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    controller = DualCameraController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 