#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
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
            # Wait a short time for transform to become available
            if not self.tf_buffer.can_transform('odom', 'camera_link', rclpy.time.Time()):
                self.get_logger().warn('Transform not available, waiting...')
                self.tf_buffer.can_transform('odom', 'camera_link', rclpy.time.Time(), 
                                          timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Calculate depth using human height and pixel height
            y_top, y_bottom = y_pixel_pair
            pixel_height = float(y_bottom - y_top)
            
            self.get_logger().info(f'Pixel height: {pixel_height}')
            
            if pixel_height <= 0:
                self.get_logger().warn('Invalid pixel height')
                return None
            
            depth = (self.human_height * self.fy) / pixel_height
            depth = min(depth, 5.0)  # Limit max depth to 5 meters
            
            self.get_logger().info(f'Calculated depth: {depth}m')
            
            # Calculate 3D point in camera frame
            x_cam = ((x_pixel - self.cx) * depth) / self.fx
            y_cam = ((y_bottom - self.cy) * depth) / self.fy
            z_cam = depth
            
            self.get_logger().info(f'Camera frame coords: x={x_cam}, y={y_cam}, z={z_cam}')
            
            # Create pose in camera frame
            pose = PoseStamped()
            pose.header.frame_id = 'camera_link'
            pose.header.stamp = transform.header.stamp
            
            # Set position
            pose.pose.position = Point(x=z_cam, y=-x_cam, z=0.0)
            
            # Set orientation (identity quaternion)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Transform to odom frame
            try:
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                
                self.get_logger().info(f'Transformed pose: x={transformed_pose.pose.position.x:.2f}, '
                                     f'y={transformed_pose.pose.position.y:.2f}, '
                                     f'z={transformed_pose.pose.position.z:.2f}')
                
                return transformed_pose
                
            except Exception as e:
                self.get_logger().error(f'Transform failed: {str(e)}')
                return None
            
        except Exception as e:
            self.get_logger().warn(f'Failed to project to map: {str(e)}')
            return None
            
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create visualization image (copy of original)
            viz_image = cv_image.copy()
            
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
                            
                            # Draw detection box on visualization image
                            cv2.rectangle(viz_image, 
                                        (int(x1), int(y1)), 
                                        (int(x2), int(y2)), 
                                        (0, 255, 0), 2)
                            
                            self.get_logger().info(f'Detected person at box coordinates: '
                                                 f'x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}')
                            
                            # Project bottom center of bounding box to map
                            bottom_center_x = (x1 + x2) / 2
                            
                            map_pose = self.project_to_map(
                                bottom_center_x,
                                [y1, y2],  # Pass both top and bottom y for height calculation
                                msg.header
                            )
                            
                            if map_pose and map_pose.pose:  # Add check for pose attribute
                                try:
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
                                    
                                    # Make marker more visible
                                    map_marker.scale.x = 0.5  # 50cm diameter
                                    map_marker.scale.y = 0.5
                                    map_marker.scale.z = 1.7  # Human height
                                    
                                    # Bright red color
                                    map_marker.color.r = 1.0
                                    map_marker.color.g = 0.0
                                    map_marker.color.b = 0.0
                                    map_marker.color.a = 1.0  # Fully opaque
                                    
                                    # Longer lifetime
                                    map_marker.lifetime = rclpy.duration.Duration(seconds=5.0).to_msg()
                                    
                                    map_markers.markers.append(map_marker)
                                    self.get_logger().info(f'Created marker at pose: '
                                                         f'x={map_pose.pose.position.x:.2f}, '
                                                         f'y={map_pose.pose.position.y:.2f}, '
                                                         f'z={map_pose.pose.position.z:.2f}')
                                except Exception as e:
                                    self.get_logger().error(f'Failed to create marker: {str(e)}')
                            
                        except Exception as e:
                            self.get_logger().warn(f'Error processing detection box: {str(e)}')
                            continue
            
            # Publish visualization image
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)
                
                # Also publish compressed image for easier viewing
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = np.array(cv2.imencode('.jpg', viz_image)[1]).tobytes()
                self.debug_img_pub.publish(compressed_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish visualization: {str(e)}')
            
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