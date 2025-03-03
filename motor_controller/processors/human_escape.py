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
        
    def get_furthest_waypoint(self):
        """Generate a waypoint at the furthest reachable point from human position"""
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
            
            # Get human position
            if hasattr(self.node, 'last_human_position') and self.node.last_human_position is not None:
                human_x, human_y = self.node.last_human_position
                self.node.get_logger().info(
                    f'Planning escape from human at ({human_x:.2f}, {human_y:.2f})'
                )
            else:
                self.node.get_logger().warn('No human position available for escape planning')
                return None
            
            # Calculate current distance from human
            current_dist_to_human = math.sqrt(
                (robot_x - human_x) ** 2 + 
                (robot_y - human_y) ** 2
            )
            
            self.node.get_logger().info(f'Current distance from human: {current_dist_to_human:.2f}m')
            
            # Get map data
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Calculate distance transform from walls
            wall_distance = distance_transform_edt(map_data < 50) * resolution
            
            valid_points = []
            min_wall_distance = self.safety_margin
            best_point = None
            best_score = float('-inf')
            
            # Search entire map for the furthest point
            height, width = map_data.shape
            for y in range(height):
                for x in range(width):
                    # Skip non-free cells and cells too close to walls
                    if map_data[y, x] != 0 or wall_distance[y, x] < min_wall_distance:
                        continue
                        
                    # Convert to world coordinates
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    
                    # Calculate distance to human
                    dist_to_human = math.sqrt(
                        (world_x - human_x) ** 2 + 
                        (world_y - human_y) ** 2
                    )
                    
                    # Skip points that are closer to human than current position
                    if dist_to_human <= current_dist_to_human:
                        continue
                    
                    # Calculate distance from robot (for feasibility)
                    dist_from_robot = math.sqrt(
                        (world_x - robot_x) ** 2 + 
                        (world_y - robot_y) ** 2
                    )
                    
                    # Skip points that are too far from robot to be practical
                    if dist_from_robot > 5.0:  # Max 5 meters from current position
                        continue
                    
                    # Calculate score based on distance from human and wall clearance
                    total_score = (
                        10.0 * dist_to_human +           # Heavily prioritize distance from human
                        2.0 * wall_distance[y, x] +      # Consider wall clearance
                        -1.0 * dist_from_robot           # Slight penalty for distance from robot
                    )
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_point = (world_x, world_y, dist_to_human)
                        self.node.get_logger().info(
                            f'New best point found at ({world_x:.2f}, {world_y:.2f}) - '
                            f'distance from human: {dist_to_human:.2f}m, score: {total_score:.2f}'
                        )
            
            if best_point is None:
                self.node.get_logger().error('No valid escape points found that increase distance from human!')
                return None
            
            # Create waypoint from best point
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.node.get_clock().now().to_msg()
            waypoint.pose.position.x = best_point[0]
            waypoint.pose.position.y = best_point[1]
            
            # Face towards human
            dx = waypoint.pose.position.x - human_x
            dy = waypoint.pose.position.y - human_y
            yaw = math.atan2(dy, dx)  # Point away from human
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Mark this as an escape waypoint
            waypoint.header.stamp.nanosec = 1  # Special flag for escape waypoints
            
            self.node.get_logger().info(
                f'Generated escape waypoint {best_point[2]:.2f}m from human at '
                f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
            )
            
            # Create visualization markers
            markers = self.create_visualization_markers(waypoint, is_escape=True)
            self.node.marker_pub.publish(markers)
            
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