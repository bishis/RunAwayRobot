#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, Twist
from vision_msgs.msg import Detection2DArray as DetectionArray
from std_msgs.msg import Bool
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
from .sort import Sort
import threading

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
            CompressedImage,
            '/camera/image_raw_flipped/compressed',  # Make sure this matches image_flipper's output topic
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
        self.max_history = 3
        
        # Initialize SORT tracker
        self.tracker = Sort(
            max_age=3,  # Reduce max age for faster tracking updates
            min_hits=1,  # Reduce required hits for faster initial tracking
            iou_threshold=0.25  # Lower IOU threshold for better tracking
        )
        
        # Add tracked persons publisher
        self.tracked_persons_pub = self.create_publisher(
            MarkerArray,
            '/tracked_persons',
            10
        )
        
        # Add LIDAR subscription
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Store latest scan data
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        
        # Add LIDAR parameters
        self.lidar_window_size = 3  # Window size for LIDAR measurements
        
        # Optimize tracking parameters
        self.conf_threshold = 0.5  # Lower threshold for faster detection
        self.model.conf = self.conf_threshold
        self.model.iou = 0.35  # Lower IOU for better tracking
        self.model.max_det = 10  # Increase max detections
        
        # Optimize SORT parameters for faster response
        self.tracker = Sort(
            max_age=2,  # Reduce max age for faster updates
            min_hits=1,  # Immediate tracking
            iou_threshold=0.2  # Lower IOU threshold for better tracking
        )
        
        # Reduce position filtering for faster response
        self.max_history = 2  # Shorter history
        self.position_weights = [0.7, 0.3]  # More weight on current position
        
        # Adjust tracking parameters
        self.min_tracking_confidence = 0.4  # Lower confidence threshold
        self.target_distance = 1.0  # Target following distance
        self.p_gain_angular = 1.5  # Increase angular gain for faster turning
        self.p_gain_linear = 0.8   # Increase linear gain for faster approach
        
        # Add publishers for human tracking
        self.tracking_cmd_pub = self.create_publisher(
            Twist,
            '/human_tracking_cmd',
            10
        )
        
        self.tracking_active_pub = self.create_publisher(
            Bool,
            '/human_tracking_active',
            10
        )
        
        self.get_logger().info('Person detector initialized successfully')
        
    def scan_callback(self, msg):
        """Store latest LIDAR scan"""
        with self.scan_lock:
            self.latest_scan = msg
    
    def get_lidar_distance(self, camera_angle):
        """Optimized LIDAR distance calculation"""
        with self.scan_lock:
            if self.latest_scan is None:
                return None
            
            lidar_angle = -camera_angle
            scan = self.latest_scan
            
            # Direct index calculation
            angle_idx = int((lidar_angle - scan.angle_min) / scan.angle_increment)
            if not (0 <= angle_idx < len(scan.ranges)):
                return None
            
            # Quick window check
            start_idx = max(0, angle_idx - self.lidar_window_size)
            end_idx = min(len(scan.ranges), angle_idx + self.lidar_window_size + 1)
            
            # Use numpy for faster processing
            window_ranges = np.array(scan.ranges[start_idx:end_idx])
            valid_ranges = window_ranges[(window_ranges >= scan.range_min) & 
                                      (window_ranges <= scan.range_max)]
            
            return np.min(valid_ranges) if len(valid_ranges) > 0 else None
            
    def project_to_map(self, x_pixel, y_pixel_pair, header, box_width):
        """Project pixel coordinates to map coordinates using LIDAR data"""
        try:
            y_top, y_bottom = y_pixel_pair
            pixel_height = float(y_bottom - y_top)
            
            # Calculate camera angle from pixel position
            camera_angle = math.atan2((x_pixel - self.cx), self.fx)
            
            # Get LIDAR distance at this angle
            lidar_distance = self.get_lidar_distance(camera_angle)
            
            if lidar_distance is not None:
                depth = lidar_distance
                confidence = 1.2
            else:
                # Improved visual estimation
                depth = (self.human_height * abs(self.fy)) / pixel_height
                # Adjust confidence based on pixel height and position
                confidence = min((pixel_height / 200.0) * (1.0 - (depth / 4.0)), 1.0)
            
            # Get transform from camera to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Calculate 3D point in camera frame
            center_x = ((x_pixel - self.cx) * depth) / self.fx
            center_y = ((((y_top + y_bottom) / 2) - self.cy) * depth) / self.fy
            
            # Convert to ROS camera frame
            x_cam = depth        # Camera Z -> ROS X (forward)
            y_cam = -center_x    # Camera -X -> ROS Y (left)
            z_cam = 0.0         # Project to ground plane
            
            self.get_logger().info(f'Camera frame coords: x={x_cam}, y={y_cam}, z={z_cam}')
            
            # Create pose in camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera_link'
            pose_stamped.header.stamp = transform.header.stamp
            
            pose = Pose()
            pose.position.x = x_cam
            pose.position.y = y_cam
            pose.position.z = z_cam
            
            # Calculate orientation to face the camera
            yaw = math.atan2(y_cam, x_cam)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            
            pose_stamped.pose = pose
            
            # Transform to map frame
            try:
                transformed_pose = self.tf_buffer.transform(pose_stamped, 'map')
                transformed_pose = self.filter_position(transformed_pose)
                
                self.get_logger().info(f'Transformed pose: x={transformed_pose.pose.position.x:.2f}, '
                                     f'y={transformed_pose.pose.position.y:.2f}, '
                                     f'z={transformed_pose.pose.position.z:.2f}')
                
                return transformed_pose, confidence
                
            except Exception as e:
                self.get_logger().error(f'Transform failed: {str(e)}')
                return None, 0.0
            
        except Exception as e:
            self.get_logger().warn(f'Failed to project to map: {str(e)}')
            return None, 0.0
            
    def image_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Run detection with batching
            results = self.model(cv_image, verbose=False)  # Disable verbose output
            
            # Process detections more efficiently
            detections = []
            for result in results:
                boxes = result.boxes.data.cpu().numpy()  # Get all boxes at once
                # Filter person detections (class 0) with confidence
                person_mask = (boxes[:, 5] == 0) & (boxes[:, 4] >= self.conf_threshold)
                person_boxes = boxes[person_mask]
                if len(person_boxes) > 0:
                    detections.extend(person_boxes[:, :5])  # Only take x1,y1,x2,y2,conf
            
            # Update trackers
            tracked_objects = self.tracker.update(np.array(detections)) if detections else np.empty((0, 5))
            
            # Process tracked objects more efficiently
            tracked_markers = MarkerArray()
            viz_image = cv_image.copy() if tracked_objects.size > 0 else None
            
            # Track closest person
            closest_person = None
            min_distance = float('inf')
            
            for i, track in enumerate(tracked_objects):
                x1, y1, x2, y2, track_id = track
                
                # Only process if confidence is high enough
                bottom_center_x = (x1 + x2) / 2
                result = self.project_to_map(
                    bottom_center_x,
                    [y1, y2],
                    msg.header,
                    x2 - x1
                )
                
                if result is not None:
                    map_pose, confidence = result
                    if confidence > self.min_tracking_confidence:
                        # Calculate distance to robot
                        distance = math.sqrt(
                            map_pose.pose.position.x ** 2 + 
                            map_pose.pose.position.y ** 2
                        )
                        
                        # Update closest person
                        if distance < min_distance:
                            min_distance = distance
                            closest_person = (map_pose, track_id, distance)
                        
                        marker = self.create_person_marker(
                            map_pose,
                            int(track_id),
                            i,
                            confidence
                        )
                        tracked_markers.markers.append(marker)
                        
                        # Only draw visualization if we're actually using it
                        if viz_image is not None:
                            cv2.rectangle(viz_image, 
                                        (int(x1), int(y1)), 
                                        (int(x2), int(y2)), 
                                        (0, 255, 0), 2)
                            cv2.putText(viz_image, 
                                      f'ID: {int(track_id)}',
                                      (int(x1), int(y1)-10),
                                      cv2.FONT_HERSHEY_SIMPLEX,
                                      0.5,
                                      (0, 255, 0),
                                      2)
            
            # Generate tracking commands if we have a person to track
            if closest_person is not None:
                pose, track_id, distance = closest_person
                
                # Create tracking command with improved responsiveness
                cmd = Twist()
                
                # Calculate how far the person is from the center of the image
                image_center_x = cv_image.shape[1] / 2
                person_center_x = (x1 + x2) / 2
                center_error = (person_center_x - image_center_x) / image_center_x  # Normalized error (-1 to 1)
                
                # Simple P controller to center person in frame
                cmd.angular.z = -center_error * 1.0  # Negative because positive angle turns left
                
                # Only move forward/back if person is centered
                if abs(center_error) < 0.2:  # Person is roughly centered
                    distance_error = distance - self.target_distance
                    cmd.linear.x = distance_error * 0.5
                else:
                    cmd.linear.x = 0.0  # Don't move while turning to face person
                
                # Apply velocity limits
                cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
                cmd.angular.z = max(min(cmd.angular.z, 0.8), -0.8)
                
                # Publish tracking command
                self.tracking_cmd_pub.publish(cmd)
                
                tracking_active = Bool()
                tracking_active.data = True
                self.tracking_active_pub.publish(tracking_active)
            else:
                # Publish inactive status when no person detected
                tracking_active = Bool()
                tracking_active.data = False
                self.tracking_active_pub.publish(tracking_active)
            
            # Only publish markers if we have any
            if tracked_markers.markers:
                self.tracked_persons_pub.publish(tracked_markers)
            
            # Only publish visualization if we modified the image
            if viz_image is not None:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = np.array(cv2.imencode(
                    '.jpg', viz_image, 
                    [cv2.IMWRITE_JPEG_QUALITY, 60]  # Lower quality for faster transmission
                )[1]).tobytes()
                self.debug_img_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def filter_position(self, new_pose):
        """Optimized position filtering"""
        if not self.position_history:
            self.position_history.append(new_pose)
            return new_pose
            
        self.position_history.append(new_pose)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
            
        # Use numpy for faster calculation
        positions = np.array([[p.pose.position.x, p.pose.position.y] 
                            for p in self.position_history])
        weights = np.array(self.position_weights)
        
        filtered_pos = np.average(positions, weights=weights, axis=0)
        new_pose.pose.position.x = filtered_pos[0]
        new_pose.pose.position.y = filtered_pos[1]
        
        return new_pose

    def create_person_marker(self, pose, track_id, marker_id, confidence):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'tracked_persons'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose = pose.pose
        
        # Adjust marker color based on track ID
        marker.color.r = (track_id * 123) % 255 / 255.0
        marker.color.g = (track_id * 147) % 255 / 255.0
        marker.color.b = (track_id * 213) % 255 / 255.0
        marker.color.a = max(0.5, confidence)
        
        # Set marker size
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.7  # Human height
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        return marker

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