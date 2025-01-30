#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # Using nano model for speed
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Create publishers
        self.viz_pub = self.create_publisher(
            MarkerArray,
            '/detected_persons',
            10
        )
        
        # For visualization
        self.debug_img_pub = self.create_publisher(
            CompressedImage,
            '/person_detections/compressed',
            10
        )
        
        self.get_logger().info('Person detector initialized')
        
    def image_callback(self, msg):
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Run detection
            results = self.model(cv_image, classes=[0])  # class 0 is person in COCO
            
            # Process results
            markers = MarkerArray()
            
            # Create visualization image
            viz_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    
                    if confidence > 0.5:  # Confidence threshold
                        # Draw on image
                        cv2.rectangle(
                            viz_image,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            (0, 255, 0),
                            2
                        )
                        
                        # Add confidence text
                        cv2.putText(
                            viz_image,
                            f'Person: {confidence:.2f}',
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )
                        
                        # Create marker for visualization
                        marker = Marker()
                        marker.header = msg.header
                        marker.ns = "persons"
                        marker.id = len(markers.markers)
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        
                        # Set marker position (in camera frame)
                        marker.pose.position.x = 0
                        marker.pose.position.y = 0
                        marker.pose.position.z = 0
                        
                        # Set marker size
                        marker.scale.x = 0.5
                        marker.scale.y = 0.5
                        marker.scale.z = 1.8  # Approximate human height
                        
                        # Set marker color (green, semi-transparent)
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 0.5
                        
                        markers.markers.append(marker)
            
            # Publish markers
            self.viz_pub.publish(markers)
            
            # Publish debug image
            debug_msg = CompressedImage()
            debug_msg.header = msg.header
            debug_msg.format = "jpeg"
            debug_msg.data = np.array(cv2.imencode('.jpg', viz_image)[1]).tobytes()
            self.debug_img_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 