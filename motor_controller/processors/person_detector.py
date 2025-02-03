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
import yaml

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
        
        # Create subscribers - use compressed image
        self.image_sub = self.create_subscription(
            CompressedImage,  # Changed from Image to CompressedImage
            '/camera/image_raw_flipped/compressed',  # Subscribe to compressed flipped image
            self.image_callback,
            10
        )
        
        # Create publishers
        self.detection_pub = self.create_publisher(
            DetectionArray, 
            '/person_detections',
            10
        )
        
        # Only publish compressed debug image
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
        
        # Load actual camera calibration from camera_info.yaml
        try:
            with open('/path/to/camera_info.yaml', 'r') as f:
                camera_info = yaml.safe_load(f)
                self.fx = camera_info['camera_matrix']['data'][0]
                self.fy = camera_info['camera_matrix']['data'][4]
                self.cx = camera_info['camera_matrix']['data'][2]
                self.cy = camera_info['camera_matrix']['data'][5]
        except:
            # Fallback values
            self.fx = 607.5860
            self.fy = 607.1840
            self.cx = 320.0
            self.cy = 240.0
        
        # Adjust human height for better depth estimation
        self.human_height = 1.7  # meters
        
        # Add position history
        self.position_history = []
        self.max_history = 5
        
        self.get_logger().info('Person detector initialized successfully')
        
    def project_to_map(self, x_pixel, y_pixel_pair, header, box_width):
        """Project pixel coordinates to map coordinates"""
        try:
            # First get transform from camera to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Use multiple measurements for more stable depth
            y_top, y_bottom = y_pixel_pair
            pixel_height = float(y_bottom - y_top)
            
            # Filter out unrealistic measurements
            if pixel_height < 20 or pixel_height > 400:  # Reasonable pixel height range
                self.get_logger().warn('Unrealistic pixel height')
                return None
                
            # Use pixel width as additional check
            pixel_width = box_width
            expected_ratio = 0.3  # width/height ratio for standing person
            measured_ratio = pixel_width / pixel_height
            
            if abs(measured_ratio - expected_ratio) > 0.2:
                self.get_logger().warn('Unusual person proportions, might be inaccurate')
            
            # Improved depth calculation
            depth = (self.human_height * self.fy) / pixel_height
            depth = min(depth, 8.0)  # Limit max depth to 8 meters
            
            # Apply depth confidence factor based on pixel height
            confidence = min(pixel_height / 200.0, 1.0)  # Higher confidence for larger detections
            if confidence < 0.5:
                self.get_logger().warn('Low confidence depth measurement')
            
            # Calculate 3D point in camera frame
            center_x = ((x_pixel - self.cx) * depth) / self.fx
            center_y = ((((y_top + y_bottom) / 2) - self.cy) * depth) / self.fy  # Use center point
            
            # Convert to ROS camera frame
            x_cam = depth        # Camera Z -> ROS X (forward)
            y_cam = -center_x    # Camera -X -> ROS Y (left)
            z_cam = -center_y    # Camera -Y -> ROS Z (up)
            
            self.get_logger().info(f'Camera frame coords: x={x_cam}, y={y_cam}, z={z_cam}')
            
            # Create pose in camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera_link'
            pose_stamped.header.stamp = transform.header.stamp
            
            # Create Pose message first
            pose = Pose()
            pose.position.x = x_cam
            pose.position.y = y_cam
            pose.position.z = 0.0  # Project to ground plane
            
            # Calculate orientation to face the camera
            dx = x_cam
            dy = y_cam
            yaw = math.atan2(dy, dx)  # Calculate yaw angle
            
            # Convert yaw to quaternion
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            
            # Assign the pose to PoseStamped
            pose_stamped.pose = pose
            
            # Transform to map frame
            try:
                transformed_pose = self.tf_buffer.transform(pose_stamped, 'map')
                
                # Apply temporal filtering
                transformed_pose = self.filter_position(transformed_pose)
                
                self.get_logger().info(f'Transformed pose: x={transformed_pose.pose.position.x:.2f}, '
                                     f'y={transformed_pose.pose.position.y:.2f}, '
                                     f'z={transformed_pose.pose.position.z:.2f}')
                
                return transformed_pose, confidence  # Return both pose and confidence
                
            except Exception as e:
                self.get_logger().error(f'Transform failed: {str(e)}')
                return None, 0.0
            
        except Exception as e:
            self.get_logger().warn(f'Failed to project to map: {str(e)}')
            return None, 0.0
            
    def image_callback(self, msg):
        """Process incoming compressed image messages"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
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
                            box_width = x2 - x1  # Calculate width
                            
                            # Draw detection box on visualization image
                            cv2.rectangle(viz_image, 
                                        (int(x1), int(y1)), 
                                        (int(x2), int(y2)), 
                                        (0, 255, 0), 2)
                            
                            self.get_logger().info(f'Detected person at box coordinates: '
                                                 f'x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}')
                            
                            # Project bottom center of bounding box to map
                            bottom_center_x = (x1 + x2) / 2
                            
                            map_pose, confidence = self.project_to_map(  # Get both pose and confidence
                                bottom_center_x,
                                [y1, y2],  # Pass both top and bottom y for height calculation
                                msg.header,
                                box_width  # Pass box width
                            )
                            
                            if map_pose and map_pose.pose:  # Add check for pose attribute
                                try:
                                    # Create map marker
                                    map_marker = Marker()
                                    map_marker.header.frame_id = 'map'  # Changed from 'odom' to 'map'
                                    map_marker.header.stamp = self.get_clock().now().to_msg()
                                    map_marker.ns = 'map_persons'
                                    map_marker.id = map_marker_id
                                    map_marker_id += 1
                                    map_marker.type = Marker.CYLINDER
                                    map_marker.action = Marker.ADD
                                    
                                    # Set position from map pose
                                    map_marker.pose = map_pose.pose
                                    
                                    # Adjust marker color based on confidence
                                    map_marker.color.r = 1.0
                                    map_marker.color.g = confidence  # More green = higher confidence
                                    map_marker.color.b = 0.0
                                    map_marker.color.a = max(0.5, confidence)  # More opaque = higher confidence
                                    
                                    # Adjust marker size based on confidence
                                    base_size = 0.5
                                    map_marker.scale.x = base_size * (0.8 + 0.4 * confidence)
                                    map_marker.scale.y = base_size * (0.8 + 0.4 * confidence)
                                    map_marker.scale.z = 1.7  # Human height
                                    
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
            
            # Only publish compressed visualization
            try:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = np.array(cv2.imencode('.jpg', viz_image, 
                                            [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
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

    def filter_position(self, new_pose):
        """Apply temporal filtering to smooth out position estimates"""
        if not self.position_history:
            self.position_history.append(new_pose)
            return new_pose
            
        # Add new position to history
        self.position_history.append(new_pose)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
            
        # Calculate weighted average (more weight to recent positions)
        weights = [0.1, 0.15, 0.2, 0.25, 0.3]  # Must sum to 1.0
        filtered_x = sum(p.pose.position.x * w for p, w in zip(self.position_history, weights))
        filtered_y = sum(p.pose.position.y * w for p, w in zip(self.position_history, weights))
        
        # Update position with filtered values
        new_pose.pose.position.x = filtered_x
        new_pose.pose.position.y = filtered_y
        return new_pose

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