#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist
from std_msgs.msg import ColorRGBA, String
import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class DetectedObject:
    angle: float  # Radians
    distance: float  # Meters
    width: float  # Meters
    point_cluster: List[Tuple[float, float]]  # List of (angle, distance) points

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Parameters
        self.declare_parameter('min_points_per_object', 3)
        self.declare_parameter('max_point_distance', 0.3)  # Max distance between points to be considered same object
        self.declare_parameter('min_object_width', 0.1)  # Minimum width to be considered an object
        self.declare_parameter('max_object_width', 2.0)  # Maximum width to be considered an object
        
        # Get parameters
        self.min_points = self.get_parameter('min_points_per_object').value
        self.max_point_distance = self.get_parameter('max_point_distance').value
        self.min_object_width = self.get_parameter('min_object_width').value
        self.max_object_width = self.get_parameter('max_object_width').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/detected_objects/markers',
            10
        )
        
        # Publish object data using Twist messages
        # linear.x = distance, angular.z = angle
        self.front_object_pub = self.create_publisher(
            Twist, 
            '/detected_objects/front',
            10
        )
        self.rear_object_pub = self.create_publisher(
            Twist,
            '/detected_objects/rear',
            10
        )
        self.left_object_pub = self.create_publisher(
            Twist,
            '/detected_objects/left',
            10
        )
        self.right_object_pub = self.create_publisher(
            Twist,
            '/detected_objects/right',
            10
        )
        
        # Text description publisher
        self.description_pub = self.create_publisher(
            String,
            '/detected_objects/description',
            10
        )
        
        self.get_logger().info('Object detector initialized')

    def detect_objects(self, scan: LaserScan) -> List[DetectedObject]:
        objects = []
        current_cluster = []
        
        # Convert scan to list of (angle, distance) tuples
        points = []
        for i, distance in enumerate(scan.ranges):
            if scan.range_min <= distance <= scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                points.append((angle, distance))
        
        if not points:
            return []
        
        # Start with first point
        current_cluster = [points[0]]
        
        # Cluster points
        for i in range(1, len(points)):
            prev_angle, prev_dist = points[i-1]
            curr_angle, curr_dist = points[i]
            
            # Calculate physical distance between points
            dx = curr_dist * math.cos(curr_angle) - prev_dist * math.cos(prev_angle)
            dy = curr_dist * math.sin(curr_angle) - prev_dist * math.sin(prev_angle)
            point_distance = math.sqrt(dx*dx + dy*dy)
            
            if point_distance <= self.max_point_distance:
                current_cluster.append(points[i])
            else:
                # Process completed cluster
                if len(current_cluster) >= self.min_points:
                    obj = self.create_object_from_cluster(current_cluster)
                    if obj is not None:
                        objects.append(obj)
                # Start new cluster
                current_cluster = [points[i]]
        
        # Process final cluster
        if len(current_cluster) >= self.min_points:
            obj = self.create_object_from_cluster(current_cluster)
            if obj is not None:
                objects.append(obj)
        
        return objects

    def create_object_from_cluster(self, cluster: List[Tuple[float, float]]) -> DetectedObject:
        # Calculate object properties
        angles, distances = zip(*cluster)
        
        # Calculate object width
        x_coords = [d * math.cos(a) for a, d in cluster]
        y_coords = [d * math.sin(a) for a, d in cluster]
        width = math.sqrt((max(x_coords) - min(x_coords))**2 + 
                         (max(y_coords) - min(y_coords))**2)
        
        if not (self.min_object_width <= width <= self.max_object_width):
            return None
            
        # Calculate center point
        mean_x = sum(x_coords) / len(x_coords)
        mean_y = sum(y_coords) / len(y_coords)
        
        center_angle = math.atan2(mean_y, mean_x)
        center_distance = math.sqrt(mean_x**2 + mean_y**2)
        
        return DetectedObject(
            angle=center_angle,
            distance=center_distance,
            width=width,
            point_cluster=cluster
        )

    def create_visualization_markers(self, objects: List[DetectedObject]) -> MarkerArray:
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            # Create marker for object center
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = obj.distance * math.cos(obj.angle)
            marker.pose.position.y = obj.distance * math.sin(obj.angle)
            marker.pose.position.z = 0.0
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Set scale
            marker.scale = Vector3(x=obj.width, y=obj.width, z=0.2)
            
            # Set color (vary by distance)
            distance_scale = min(1.0, obj.distance / 5.0)  # Red->Green based on distance
            marker.color = ColorRGBA(
                r=1.0 - distance_scale,
                g=distance_scale,
                b=0.0,
                a=0.7
            )
            
            marker.lifetime.sec = 1  # Markers last 1 second
            
            marker_array.markers.append(marker)
            
        return marker_array

    def publish_object_data(self, objects: List[DetectedObject]):
        """Publish nearest objects in each quadrant"""
        # Initialize nearest objects for each direction
        front_obj = None  # -45° to 45°
        right_obj = None  # 45° to 135°
        rear_obj = None   # 135° to -135°
        left_obj = None   # -135° to -45°
        
        # Find nearest object in each direction
        for obj in objects:
            angle_deg = math.degrees(obj.angle)
            
            # Normalize angle to -180 to 180
            if angle_deg > 180:
                angle_deg -= 360
                
            # Categorize by direction
            if -45 <= angle_deg <= 45:  # Front
                if front_obj is None or obj.distance < front_obj.distance:
                    front_obj = obj
            elif 45 < angle_deg <= 135:  # Right
                if right_obj is None or obj.distance < right_obj.distance:
                    right_obj = obj
            elif angle_deg > 135 or angle_deg <= -135:  # Rear
                if rear_obj is None or obj.distance < rear_obj.distance:
                    rear_obj = obj
            else:  # Left (-135 to -45)
                if left_obj is None or obj.distance < left_obj.distance:
                    left_obj = obj
        
        # Publish object data for each direction
        description = "Objects detected: "
        
        # Front
        msg = Twist()
        if front_obj:
            msg.linear.x = front_obj.distance
            msg.angular.z = front_obj.angle
            self.front_object_pub.publish(msg)
            description += f"front={front_obj.distance:.2f}m "
            
        # Rear
        msg = Twist()
        if rear_obj:
            msg.linear.x = rear_obj.distance
            msg.angular.z = rear_obj.angle
            self.rear_object_pub.publish(msg)
            description += f"rear={rear_obj.distance:.2f}m "
            
        # Left
        msg = Twist()
        if left_obj:
            msg.linear.x = left_obj.distance
            msg.angular.z = left_obj.angle
            self.left_object_pub.publish(msg)
            description += f"left={left_obj.distance:.2f}m "
            
        # Right
        msg = Twist()
        if right_obj:
            msg.linear.x = right_obj.distance
            msg.angular.z = right_obj.angle
            self.right_object_pub.publish(msg)
            description += f"right={right_obj.distance:.2f}m"
        
        # Publish text description
        desc_msg = String()
        desc_msg.data = description
        self.description_pub.publish(desc_msg)

    def scan_callback(self, msg: LaserScan):
        try:
            # Detect objects
            objects = self.detect_objects(msg)
            
            # Publish object data
            self.publish_object_data(objects)
            
            # Create and publish visualization markers
            marker_array = self.create_visualization_markers(objects)
            self.marker_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 