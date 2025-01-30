#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import cv2
import numpy as np
import torch
import os
from pathlib import Path
import requests
import shutil

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # Initialize model path
        self.model = None
        home = str(Path.home())
        self.model_path = os.path.join(home, 'yolov8n.pt')
        
        # Load YOLO model with error handling
        try:
            # First check if we need to install ultralytics
            try:
                from ultralytics import YOLO
            except ImportError:
                self.get_logger().info('Installing ultralytics...')
                os.system('pip3 install ultralytics')
                from ultralytics import YOLO
            
            # Download model directly if it doesn't exist
            if not os.path.exists(self.model_path):
                self.get_logger().info(f'Downloading YOLOv8n model to {self.model_path}...')
                url = "https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt"
                
                # Download with progress reporting
                with requests.get(url, stream=True) as r:
                    r.raise_for_status()
                    with open(self.model_path, 'wb') as f:
                        shutil.copyfileobj(r.raw, f)
            
            # Load the model
            self.get_logger().info(f'Loading model from {self.model_path}')
            self.model = YOLO(self.model_path)
            
            # Force CPU mode and eval mode
            self.model.to('cpu')
            if hasattr(self.model, 'model'):
                self.model.model.eval()
            
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            raise
        
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
        
        # Debug counter
        self.frame_count = 0
        
        self.get_logger().info('Person detector initialized successfully')
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                self.get_logger().info('Processing frame...')
            
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn('Failed to decode image')
                return
            
            # Log image properties
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Image shape: {cv_image.shape}')
            
            # Run detection with error handling
            try:
                results = self.model(cv_image, classes=[0], verbose=False)  # class 0 is person in COCO
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Detection complete, found {len(results[0].boxes)} boxes')
            except Exception as e:
                self.get_logger().error(f'YOLO inference failed: {str(e)}')
                return
            
            # Process results
            markers = MarkerArray()
            
            # Create visualization image
            viz_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    try:
                        # Get box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        
                        if confidence > 0.3:  # Lower confidence threshold for testing
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
                            
                            if self.frame_count % 30 == 0:
                                self.get_logger().info(f'Detected person with confidence: {confidence:.2f}')
                            
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
                    except Exception as e:
                        self.get_logger().warn(f'Error processing detection box: {str(e)}')
                        continue
            
            # Publish markers
            self.viz_pub.publish(markers)
            
            # Publish debug image
            try:
                debug_msg = CompressedImage()
                debug_msg.header = msg.header
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', viz_image)[1]).tobytes()
                self.debug_img_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish debug image: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PersonDetector()
        rclpy.spin(node)
    except Exception as e:
        print(f'Failed to start person detector: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 