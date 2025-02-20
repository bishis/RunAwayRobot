#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from scipy.ndimage import distance_transform_edt
from .waypoint_generator import WaypointGenerator
import rclpy

class HumanEscape(WaypointGenerator):
    """Specialized waypoint generator for escaping from humans"""
    
    def __init__(self, node):
        super().__init__(node)
        # Additional parameters for escape behavior
        self.escape_distance = 2.0  # Desired escape distance in meters
        self.min_escape_distance = 1.5  # Minimum acceptable escape distance
        self.escape_search_radius = 3.0  # Maximum radius to search for escape points
        
    def get_furthest_waypoint(self):
        """Generate a waypoint at the furthest reachable point from current position"""
        if self.current_map is None:
            self.node.get_logger().error('No map available for escape planning')
            return None
            
        try:
            # Get current robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            self.node.get_logger().info(
                f'Planning escape from position: ({robot_x:.2f}, {robot_y:.2f})'
            )
            
            # Debug map state
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            
            free_cells = np.sum(map_data < 50)
            self.node.get_logger().info(
                f'Map status: {free_cells} free cells available for escape'
            )
            
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Calculate distance transform from walls
            wall_distance = distance_transform_edt(map_data < 50) * resolution
            
            valid_points = []
            max_attempts = 200
            min_wall_distance = self.safety_margin  # Reduced from 1.5x
            
            # Convert robot position to grid coordinates
            robot_grid_x = int((robot_x - origin_x) / resolution)
            robot_grid_y = int((robot_y - origin_y) / resolution)
            
            # Search in expanding circles for escape points
            for radius in np.linspace(self.min_escape_distance, self.escape_search_radius, 20):  # More points
                angles = np.linspace(0, 2*np.pi, 32)  # More angles to test
                
                for angle in angles:
                    # Calculate potential escape point
                    world_x = robot_x + radius * np.cos(angle)
                    world_y = robot_y + radius * np.sin(angle)
                    
                    # Convert to map coordinates
                    map_x = int((world_x - origin_x) / resolution)
                    map_y = int((world_y - origin_y) / resolution)
                    
                    # Basic validation
                    if (map_x < 0 or map_x >= map_data.shape[1] or 
                        map_y < 0 or map_y >= map_data.shape[0]):
                        continue
                    
                    # Allow unknown space but with lower score
                    unknown_penalty = 0.5 if map_data[map_y, map_x] == -1 else 1.0
                    
                    # Check if point is in free space
                    if map_data[map_y, map_x] >= 50:  # Occupied
                        continue
                    
                    # Relaxed wall distance check
                    if wall_distance[map_y, map_x] < min_wall_distance:  # Too close to walls
                        continue
                    
                    # Calculate scores
                    dist_score = 1.0 - abs(radius - self.escape_distance) / self.escape_distance
                    wall_score = min(wall_distance[map_y, map_x], 2.0) / 2.0
                    
                    total_score = (
                        2.0 * dist_score +      # Prioritize good escape distance
                        1.5 * wall_score +      # Prefer points away from walls
                        1.0 * unknown_penalty   # Slight penalty for unknown space
                    )
                    
                    valid_points.append((world_x, world_y, radius, total_score))
                    
                    if len(valid_points) >= 1:  # Take first valid point in emergency
                        self.node.get_logger().info(f'Found escape point at radius {radius:.2f}m')
                        break
                
                if valid_points:  # Exit after finding points at this radius
                    break
            
            if not valid_points:
                self.node.get_logger().error('No valid escape points found!')
                return None
            
            # Take the point with highest score
            best_point = max(valid_points, key=lambda p: p[3])
            
            # Create waypoint message
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.node.get_clock().now().to_msg()
            waypoint.pose.position.x = best_point[0]
            waypoint.pose.position.y = best_point[1]
            
            # Face away from robot
            dx = waypoint.pose.position.x - robot_x
            dy = waypoint.pose.position.y - robot_y
            yaw = math.atan2(dy, dx)
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Mark this as an escape waypoint
            waypoint.header.stamp.nanosec = 1  # Special flag for escape waypoints
            
            self.node.get_logger().warn(
                f'Generated escape waypoint at ({waypoint.pose.position.x:.2f}, '
                f'{waypoint.pose.position.y:.2f}), '
                f'{best_point[2]:.2f}m from robot'
            )
            
            if valid_points:
                self.node.get_logger().info(
                    f'Found {len(valid_points)} valid escape points'
                )
            
            return waypoint
        
        except Exception as e:
            self.node.get_logger().warn(f'Could not get robot position: {e}')
            return None
    
    def calculate_path_clearance(self, start_x, start_y, end_x, end_y, map_data):
        """Calculate average clearance along path between points"""
        # Use Bresenham's line algorithm to get path cells
        path_cells = self.get_line_cells(start_x, start_y, end_x, end_y)
        
        if not path_cells:
            return 0.0
            
        # Calculate minimum clearance along path
        clearances = []
        for x, y in path_cells:
            if x < 0 or x >= map_data.shape[1] or y < 0 or y >= map_data.shape[0]:
                continue
            # Count free cells in 3x3 neighborhood
            neighborhood = map_data[max(0,y-1):min(y+2,map_data.shape[0]),
                                  max(0,x-1):min(x+2,map_data.shape[1])]
            clearance = np.sum(neighborhood < 50) / neighborhood.size
            clearances.append(clearance)
            
        return min(clearances) if clearances else 0.0
    
    def get_line_cells(self, x0, y0, x1, y1):
        """Get cells along a line using Bresenham's algorithm"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        cells.append((x, y))
        return cells