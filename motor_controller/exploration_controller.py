#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .processors.waypoint_generator import WaypointGenerator

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Create waypoint generator
        self.waypoint_generator = WaypointGenerator()
        
        # Subscribe to map updates
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Set up TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store latest map data
        self.current_map = None
        self.current_pose = None
        
        # Timer for periodic updates
        self.create_timer(5.0, self.update_exploration)
        
    def map_callback(self, msg):
        """Store latest map data"""
        self.current_map = msg
        
    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return None
            
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates"""
        if not self.current_map:
            return None, None
            
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        wx = origin_x + (mx * resolution)
        wy = origin_y + (my * resolution)
        return wx, wy
        
    def is_valid_point(self, x, y):
        """Check if a point is valid for navigation"""
        if not self.current_map:
            return False
            
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Convert world coordinates to map coordinates
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        
        # Check bounds
        if mx < 0 or mx >= self.current_map.info.width or \
           my < 0 or my >= self.current_map.info.height:
            return False
            
        # Check if point is in free space
        index = my * self.current_map.info.width + mx
        return self.current_map.data[index] == 0
        
    def update_exploration(self):
        """Generate and follow new waypoints"""
        if not self.current_map:
            self.get_logger().warning('No map data available')
            return
            
        current_pose = self.get_robot_pose()
        if not current_pose:
            self.get_logger().warning('No robot pose available')
            return
            
        # Convert map message to numpy array for processing
        map_data = [self.current_map.data[i] for i in range(len(self.current_map.data))]
        map_data = np.array(map_data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Generate waypoints
        waypoints = self.waypoint_generator.generate_waypoints(
            current_pose.pose,
            map_data,
            self.current_map.info,
            self.is_valid_point,
            self.map_to_world
        )
        
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()