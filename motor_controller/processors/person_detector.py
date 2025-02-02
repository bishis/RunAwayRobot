#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray as DetectionArray
import cv2
import numpy as np
import torch
import os
from pathlib import Path
import requests
import shutil
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_geometry_msgs
import math

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
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw_flipped',  # Changed from image_raw to image_raw_flipped
            self.image_callback,
            10
        )
        
        # Create publishers
        self.detection_pub = self.create_publisher(
            DetectionArray, 
            '/person_detections',
            10
        )
        self.viz_pub = self.create_publisher(
            Image,
            '/person_detections/image',
            10
        )
        
        # For visualization
        self.debug_img_pub = self.create_publisher(
            CompressedImage,
            '/person_detections/compressed',
            10
        )
        
        # info counter
        self.frame_count = 0
        
        # Add TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Add map visualization publisher
        self.map_marker_pub = self.create_publisher(
            MarkerArray,
            '/map_person_detections',
            10
        )
        
        # Camera parameters (from camera_info.yaml)
        self.fx = 500.0  # focal length x
        self.fy = 500.0  # focal length y
        self.cx = 320.0  # optical center x
        self.cy = 240.0  # optical center y
        
        # Approximate human height (meters)
        self.human_height = 1.7
        
        self.get_logger().info('Person detector initialized successfully')
        
    def project_to_map(self, x_pixel, y_pixel_pair, header):
        """Project pixel coordinates to map coordinates"""
        try:
            # Get current time
            now = self.get_clock().now()
            
            # Try to get transform with a small delay to ensure it's available
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom',
                    'camera_link',
                    now - rclpy.duration.Duration(seconds=0.1),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception as e:
                self.get_logger().info(f'Transform not ready yet: {str(e)}')
                return None
            
            # Calculate depth using human height and pixel height
            y_top, y_bottom = y_pixel_pair
            pixel_height = float(y_bottom - y_top)
            
            # Add info output
            self.get_logger().info(f'Pixel height: {pixel_height}')
            
            # Adjust depth calculation
            depth = (self.human_height * self.fy) / pixel_height
            depth = min(depth, 5.0)  # Limit max depth to 5 meters
            
            self.get_logger().info(f'Calculated depth: {depth}m')
            
            # Calculate 3D point in camera frame
            # Note: Camera coordinates are:
            # X - right
            # Y - down
            # Z - forward
            x_cam = ((x_pixel - self.cx) * depth) / self.fx  # right/left
            y_cam = ((y_bottom - self.cy) * depth) / self.fy  # up/down
            z_cam = depth  # forward/back
            
            self.get_logger().info(f'Camera frame coords: x={x_cam}, y={y_cam}, z={z_cam}')
            
            # Create pose in camera frame
            pose = PoseStamped()
            pose.header.frame_id = 'camera_link'
            pose.header.stamp = transform.header.stamp
            
            # Convert camera coordinates to ROS standard frame
            pose.pose.position.x = z_cam  # forward
            pose.pose.position.y = -x_cam  # left
            pose.pose.position.z = 0.0    # Set Z to ground level
            
            # Set orientation to align with ground plane
            pose.pose.orientation.w = 1.0
            
            # Transform to odom frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            
            self.get_logger().info(f'Transformed pose: x={transformed_pose.pose.position.x}, '
                                 f'y={transformed_pose.pose.position.y}, '
                                 f'z={transformed_pose.pose.position.z}')
            
            return transformed_pose
            
        except Exception as e:
            self.get_logger().warn(f'Failed to project to map: {str(e)}')
            return None
            
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run detection
            results = self.model(cv_image)
            
            # Create map markers
            map_markers = MarkerArray()
            map_marker_id = 0
            
            # info detection count
            person_count = 0
            
            # Process results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Only process person detections (class 0 in COCO)
                    if box.cls == 0:  # Person class
                        person_count += 1
                        try:
                            # Get box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].numpy()
                            
                            self.get_logger().info(f'Detected person at box coordinates: '
                                                 f'x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}')
                            
                            # Project bottom center of bounding box to map
                            bottom_center_x = (x1 + x2) / 2
                            
                            map_pose = self.project_to_map(
                                bottom_center_x,
                                [y1, y2],  # Pass both top and bottom y for height calculation
                                msg.header
                            )
                            
                            if map_pose:
                                # Create map marker
                                map_marker = Marker()
                                map_marker.header.frame_id = 'odom'
                                map_marker.header.stamp = self.get_clock().now().to_msg()
                                map_marker.ns = 'map_persons'
                                map_marker.id = map_marker_id
                                map_marker_id += 1
                                map_marker.type = Marker.CYLINDER
                                map_marker.action = Marker.ADD
                                
                                # Set position from map pose
                                map_marker.pose = map_pose.pose
                                
                                # Set marker size (make it more visible)
                                map_marker.scale.x = 0.4  # 40cm diameter
                                map_marker.scale.y = 0.4
                                map_marker.scale.z = 1.7  # Human height
                                
                                # Set color (bright red, very visible)
                                map_marker.color.r = 1.0
                                map_marker.color.g = 0.0
                                map_marker.color.b = 0.0
                                map_marker.color.a = 0.8  # More opaque
                                
                                # Set lifetime (longer)
                                map_marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
                                
                                map_markers.markers.append(map_marker)
                                self.get_logger().info(f'Created marker at pose: '
                                                     f'x={map_pose.pose.position.x:.2f}, '
                                                     f'y={map_pose.pose.position.y:.2f}, '
                                                     f'z={map_pose.pose.position.z:.2f}')
                            
                        except Exception as e:
                            self.get_logger().warn(f'Error processing detection box: {str(e)}')
                            continue
            
            # Publish map markers
            if len(map_markers.markers) > 0:
                self.get_logger().info(f'Publishing {len(map_markers.markers)} markers')
                self.map_marker_pub.publish(map_markers)
            else:
                self.get_logger().info('No markers to publish')
            
            if person_count > 0:
                self.get_logger().info(f'Detected {person_count} people in frame')
            
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