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
        
    def get_furthest_waypoint(self) -> PoseStamped:
        """Find furthest reachable point from human"""
        if not self.current_map:
            self.node.get_logger().error('No map available for escape planning')
            return None
        
        try:
            # Get robot's current position
            transform = self.node.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert map to numpy array
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            
            # Get map parameters
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Find all free cells (occupancy < 50)
            free_cells = np.where((map_data >= 0) & (map_data < 50))
            if len(free_cells[0]) == 0:
                self.node.get_logger().error('No free cells found in map')
                return None
            
            # Convert cell indices to world coordinates
            world_points = []
            for i, j in zip(free_cells[0], free_cells[1]):
                x = j * resolution + origin_x  # Convert column to x
                y = i * resolution + origin_y  # Convert row to y
                world_points.append((x, y))
            
            # Calculate distances from robot
            distances = []
            for x, y in world_points:
                dist = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                distances.append(dist)
            
            # Find points at desired escape distance
            target_distance = 1.5  # Desired escape distance
            distance_tolerance = 0.2  # Tolerance for distance matching
            
            valid_points = []
            for i, (x, y) in enumerate(world_points):
                if abs(distances[i] - target_distance) < distance_tolerance:
                    # Check if point is actually reachable (no obstacles in direct path)
                    if self.is_path_clear(robot_x, robot_y, x, y, map_data, resolution, origin_x, origin_y):
                        valid_points.append((x, y, distances[i]))
            
            if not valid_points:
                self.node.get_logger().warn('No valid escape points found at target distance, trying closer points')
                # Try points at shorter distances
                for dist in [1.2, 1.0, 0.8]:
                    for i, (x, y) in enumerate(world_points):
                        if distances[i] < dist and self.is_path_clear(robot_x, robot_y, x, y, map_data, resolution, origin_x, origin_y):
                            valid_points.append((x, y, distances[i]))
                    if valid_points:
                        break
            
            if not valid_points:
                self.node.get_logger().error('No valid escape points found')
                return None
            
            # Sort by distance and pick the furthest valid point
            valid_points.sort(key=lambda p: p[2], reverse=True)
            escape_x, escape_y, _ = valid_points[0]
            
            # Create escape goal
            escape_goal = PoseStamped()
            escape_goal.header.frame_id = 'map'
            escape_goal.header.stamp = self.node.get_clock().now().to_msg()
            escape_goal.pose.position.x = escape_x
            escape_goal.pose.position.y = escape_y
            
            # Set orientation to face away from robot
            angle = math.atan2(escape_y - robot_y, escape_x - robot_x)
            escape_goal.pose.orientation.z = math.sin(angle/2)
            escape_goal.pose.orientation.w = math.cos(angle/2)
            
            self.node.get_logger().warn(
                f'Generated escape waypoint at ({escape_x:.2f}, {escape_y:.2f}), {_:.2f}m from robot'
            )
            
            return escape_goal
        
        except Exception as e:
            self.node.get_logger().error(f'Error generating escape point: {str(e)}')
            return None

    def is_path_clear(self, start_x, start_y, end_x, end_y, map_data, resolution, origin_x, origin_y):
        """Check if there's a clear path between two points"""
        # Convert world coordinates to grid coordinates
        start_col = int((start_x - origin_x) / resolution)
        start_row = int((start_y - origin_y) / resolution)
        end_col = int((end_x - origin_x) / resolution)
        end_row = int((end_y - origin_y) / resolution)
        
        # Use Bresenham's line algorithm to check cells along path
        cells = self.get_line_cells(start_row, start_col, end_row, end_col)
        
        # Check each cell
        for row, col in cells:
            if row < 0 or row >= map_data.shape[0] or col < 0 or col >= map_data.shape[1]:
                return False  # Out of bounds
            if map_data[row, col] >= 50 or map_data[row, col] == -1:  # Occupied or unknown
                return False
        return True
    
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