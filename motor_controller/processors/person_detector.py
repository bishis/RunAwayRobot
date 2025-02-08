#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, LaserScan, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, Twist
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import Detection2DArray as DetectionArray
from std_msgs.msg import Bool, ColorRGBA
import cv2
import numpy as np
import torch
import os
from pathlib import Path
import requests
import shutil
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
import math
import yaml
from .sort import Sort
import threading
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Vector3

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
        
        # Cache for synchronized data
        self.latest_scan = None
        self.latest_image = None
        self.latest_timestamp = None
        self.data_lock = threading.Lock()
        
        # Maximum age for cached data (in seconds)
        self.max_data_age = 0.1
        
        # Create synchronized subscribers
        self.image_sub = Subscriber(self, CompressedImage, '/camera/image_raw_flipped/compressed')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')
        
        # Synchronize messages with 0.1 second tolerance
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub],
            queue_size=5,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Create high-frequency processing timer
        self.create_timer(0.05, self.process_data)  # 20Hz processing
        
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
            '/detected_persons',
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
        
        # Add TF availability check
        self.tf_ready = False
        self.create_timer(1.0, self.check_tf_availability)  # Check every second
        
        self.get_logger().info('Person detector initialized successfully')
        
    def synchronized_callback(self, image_msg, scan_msg):
        """Store synchronized data"""
        with self.data_lock:
            self.latest_image = image_msg
            self.latest_scan = scan_msg
            self.latest_timestamp = self.get_clock().now()
    
    def process_data(self):
        """Process latest synchronized data at fixed frequency"""
        with self.data_lock:
            # Check if we have recent data and TF is ready
            if (self.latest_image is None or self.latest_scan is None or 
                self.latest_timestamp is None or not self.tf_ready):
                if not self.tf_ready:
                    self.get_logger().debug('Waiting for TF tree...')
                return
                
            # Check data age
            current_time = self.get_clock().now()
            data_age = (current_time - self.latest_timestamp).nanoseconds / 1e9
            if data_age > self.max_data_age:
                return
                
            # Process the synchronized data
            try:
                # Decode image
                np_arr = np.frombuffer(self.latest_image.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Run detection
                results = self.model(cv_image, verbose=False)
                
                # Process detections with logging
                detections = []
                detection_array = DetectionArray()
                detection_array.header = self.latest_image.header
                
                for result in results:
                    boxes = result.boxes.data.cpu().numpy()
                    # Log all detections
                    for box in boxes:
                        x1, y1, x2, y2, conf, class_id = box
                        if class_id == 0:  # person class
                            self.get_logger().info(
                                f'Person detected: conf={conf:.2f}, '
                                f'pos=({(x1+x2)/2:.0f}, {(y1+y2)/2:.0f})'
                            )
                    
                    person_mask = (boxes[:, 5] == 0) & (boxes[:, 4] >= self.conf_threshold)
                    person_boxes = boxes[person_mask]
                    if len(person_boxes) > 0:
                        detections.extend(person_boxes[:, :5])
                
                # Update trackers
                tracked_objects = self.tracker.update(np.array(detections)) if detections else np.empty((0, 5))
                
                # Create marker array for visualization
                marker_array = MarkerArray()
                
                # Process tracked objects
                if len(tracked_objects) > 0:
                    for i, track in enumerate(tracked_objects):
                        x1, y1, x2, y2, track_id = track
                        person_center_x = (x1 + x2) / 2
                        person_center_y = (y1 + y2) / 2
                        
                        # Create marker for visualization
                        marker = self.create_person_marker(
                            track_id=int(track_id),
                            marker_id=i,
                            x=person_center_x,
                            y=person_center_y,
                            confidence=1.0
                        )
                        
                        if marker is not None:  # Only add valid markers
                            marker_array.markers.append(marker)
                
                    # Only publish if we have valid markers
                    if marker_array.markers:
                        self.map_marker_pub.publish(marker_array)
                    
                    # Find person closest to center
                    image_center_x = cv_image.shape[1] / 2
                    min_center_dist = float('inf')
                    target_person = None
                    
                    for track in tracked_objects:
                        x1, y1, x2, y2, track_id = track
                        person_center_x = (x1 + x2) / 2
                        center_dist = abs(person_center_x - image_center_x)
                        
                        # Log tracked person
                        self.get_logger().info(
                            f'Tracking ID {int(track_id)}: '
                            f'center_dist={center_dist:.0f}, '
                            f'pos=({person_center_x:.0f}, {(y1+y2)/2:.0f})'
                        )
                        
                        if center_dist < min_center_dist:
                            min_center_dist = center_dist
                            target_person = track
                    
                    if target_person is not None:
                        x1, y1, x2, y2, track_id = target_person
                        person_center_x = (x1 + x2) / 2
                        
                        # Calculate distance using LIDAR data
                        angle = -math.atan2((person_center_x - self.cx), self.fx)
                        index = int((angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
                        
                        # Create tracking command
                        cmd = Twist()
                        
                        # Set distance in linear.y (using it as a communication channel)
                        if 0 <= index < len(self.latest_scan.ranges):
                            distance = self.latest_scan.ranges[index]
                            if (distance >= self.latest_scan.range_min and 
                                distance <= self.latest_scan.range_max):
                                cmd.linear.y = distance  # Use linear.y to pass distance
                                self.get_logger().info(f'Human distance: {distance:.2f}m')
                        
                        # Calculate turning command as before
                        center_error = (person_center_x - image_center_x) / image_center_x
                        deadzone = 0.5
                        if abs(center_error) > deadzone:
                            turning_gain = 0.5
                            cmd.angular.z = -center_error * turning_gain
                            cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
                        
                        self.tracking_cmd_pub.publish(cmd)
                        
                        # Publish tracking status
                        tracking_active = Bool()
                        tracking_active.data = True
                        self.tracking_active_pub.publish(tracking_active)
                        
                        # Visualize detection
                        cv2.rectangle(cv_image, 
                                    (int(x1), int(y1)), 
                                    (int(x2), int(y2)), 
                                    (0, 255, 0), 2)
                        cv2.putText(cv_image,
                                  f'ID: {int(track_id)} err: {center_error:.2f}',
                                  (int(x1), int(y1)-10),
                                  cv2.FONT_HERSHEY_SIMPLEX,
                                  0.5,
                                  (0, 255, 0),
                                  2)
                        
                        # Publish visualization
                        compressed_msg = CompressedImage()
                        compressed_msg.header = self.latest_image.header
                        compressed_msg.format = 'jpeg'
                        compressed_msg.data = np.array(cv2.imencode(
                            '.jpg', cv_image, 
                            [cv2.IMWRITE_JPEG_QUALITY, 60]
                        )[1]).tobytes()
                        self.debug_img_pub.publish(compressed_msg)
                
                # Always publish markers and detection array
                self.map_marker_pub.publish(marker_array)
                self.detection_pub.publish(detection_array)
                
                if len(tracked_objects) == 0:
                    # No person detected
                    self.get_logger().info('No persons tracked')
                    tracking_active = Bool()
                    tracking_active.data = False
                    self.tracking_active_pub.publish(tracking_active)
                    
            except Exception as e:
                self.get_logger().error(f'Error processing data: {str(e)}')

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

    def check_tf_availability(self):
        """Check if required TF transforms are available"""
        try:
            # Check if we can get transform from camera to map
            self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                Time(),
                timeout=Duration(seconds=0.1)
            )
            if not self.tf_ready:
                self.get_logger().info('TF tree is now available')
                self.tf_ready = True
        except TransformException:
            if self.tf_ready:
                self.get_logger().warn('TF tree is no longer available')
            self.tf_ready = False

    def create_person_marker(self, track_id, marker_id, x, y, confidence):
        """Create visualization marker for detected person using LIDAR data"""
        if not self.tf_ready or self.latest_scan is None:
            return None
            
        try:
            # Create marker with all required fields
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'detected_persons'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Initialize pose and scale
            marker.pose = Pose()
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale = Vector3()
            marker.scale.x = 0.4  # Diameter
            marker.scale.y = 0.4  # Diameter
            marker.scale.z = 1.7  # Height
            
            # Set color based on track_id
            marker.color = ColorRGBA()
            marker.color.r = float((track_id * 123) % 255) / 255.0
            marker.color.g = float((track_id * 147) % 255) / 255.0
            marker.color.b = float((track_id * 213) % 255) / 255.0
            marker.color.a = float(max(0.5, confidence))
            
            # Calculate angle range for person width
            person_width_rad = math.atan2(0.4, 1.0)  # Approx width of person at 1m
            center_angle = -math.atan2((x - self.cx), self.fx)
            
            # Look at a range of angles around the person
            start_angle = center_angle - person_width_rad/2
            end_angle = center_angle + person_width_rad/2
            
            start_idx = int((start_angle - self.latest_scan.angle_min) / 
                           self.latest_scan.angle_increment)
            end_idx = int((end_angle - self.latest_scan.angle_min) / 
                         self.latest_scan.angle_increment)
            
            # Get all valid readings in the person's angular range
            valid_readings = []
            valid_angles = []
            
            for i in range(start_idx, end_idx + 1):
                if 0 <= i < len(self.latest_scan.ranges):
                    r = self.latest_scan.ranges[i]
                    if (self.latest_scan.range_min <= r <= self.latest_scan.range_max):
                        angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
                        valid_readings.append(r)
                        valid_angles.append(angle)
            
            if valid_readings:
                # Use the closest valid reading as the person's position
                min_dist_idx = valid_readings.index(min(valid_readings))
                depth = valid_readings[min_dist_idx]
                angle = valid_angles[min_dist_idx]
                
                # Create point in camera frame
                camera_point = PointStamped()
                camera_point.header.frame_id = 'camera_link'
                camera_point.header.stamp = self.get_clock().now().to_msg()
                camera_point.point.x = depth * math.cos(angle)
                camera_point.point.y = depth * math.sin(angle)
                camera_point.point.z = 0.0
                
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        'camera_link',
                        Time(),
                        timeout=Duration(seconds=1.0)
                    )
                    
                    map_point = do_transform_point(camera_point, transform)
                    marker.pose.position = map_point.point
                    marker.lifetime = Duration(seconds=0.5).to_msg()
                    
                    return marker
                    
                except TransformException as e:
                    self.get_logger().warn(f'Transform failed: {str(e)}')
                    return None
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Failed to create marker: {str(e)}')
            return None

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