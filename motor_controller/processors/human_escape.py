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
            
            # Convert robot position to grid coordinates
            robot_grid_x = int((robot_x - origin_x) / resolution)
            robot_grid_y = int((robot_y - origin_y) / resolution)
            
            # Convert human position to grid coordinates
            human_grid_x = int((human_x - origin_x) / resolution)
            human_grid_y = int((human_y - origin_y) / resolution)
            
            # Calculate vector from human to robot
            dx = robot_grid_x - human_grid_x
            dy = robot_grid_y - human_grid_y
            
            # Normalize the vector
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                dx /= length
                dy /= length
            
            # Sample points along the direction away from human
            best_point = None
            best_score = float('-inf')
            
            # Try different distances and angles - wider range of options
            for distance in range(5, 40, 3):  # Try distances from 0.5m to 4m with larger steps
                for angle_offset in [-45, -30, -15, 0, 15, 30, 45]:  # Try more angles
                    # Calculate rotated vector
                    angle_rad = math.radians(angle_offset)
                    rotated_dx = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
                    rotated_dy = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
                    
                    # Calculate potential escape point
                    escape_grid_x = robot_grid_x + int(rotated_dx * distance)
                    escape_grid_y = robot_grid_y + int(rotated_dy * distance)
                    
                    # Check if point is within map bounds - FIX SYNTAX ERROR
                    if (escape_grid_x < 0 or escape_grid_x >= map_data.shape[1] or
                        escape_grid_y < 0 or escape_grid_y >= map_data.shape[0]):
                        continue
                    
                    # Check if point is free and has sufficient clearance
                    # Relax the wall distance requirement slightly
                    if map_data[escape_grid_y, escape_grid_x] > 20 or wall_distance[escape_grid_y, escape_grid_x] < 0.2:
                        continue
                    
                    # Check path clearance
                    path_clearance = self.calculate_path_clearance(
                        robot_grid_x, robot_grid_y, 
                        escape_grid_x, escape_grid_y, 
                        map_data
                    )
                    
                    # Relax path clearance requirement
                    if path_clearance < 0.5:  # Require 50% of path cells to be free instead of 70%
                        continue
                    
                    # Convert to world coordinates
                    world_x = origin_x + escape_grid_x * resolution
                    world_y = origin_y + escape_grid_y * resolution
                    
                    # Calculate distance to human
                    dist_to_human = math.sqrt(
                        (world_x - human_x) ** 2 + 
                        (world_y - human_y) ** 2
                    )
                    
                    # Allow points that don't increase distance if we're desperate
                    # Only require increased distance for the best points
                    min_required_distance = current_dist_to_human * 0.8  # Allow points that are at least 80% of current distance
                    
                    if dist_to_human < min_required_distance:
                        continue
                    
                    # Calculate score based on distance from human and wall clearance
                    total_score = (
                        5.0 * dist_to_human +                # Prioritize distance from human
                        2.0 * wall_distance[escape_grid_y, escape_grid_x] +  # Consider wall clearance
                        3.0 * path_clearance                 # Prioritize clear paths
                    )
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_point = (world_x, world_y, dist_to_human, path_clearance)
                        self.node.get_logger().info(
                            f'New best point found at ({world_x:.2f}, {world_y:.2f}) - '
                            f'distance from human: {dist_to_human:.2f}m, '
                            f'path clearance: {path_clearance:.2f}, '
                            f'score: {total_score:.2f}'
                        )
            
            # If no point found, try a fallback approach with even more relaxed constraints
            if best_point is None:
                self.node.get_logger().warn('No primary escape points found, trying fallback approach')
                
                # Try a simpler approach - just move in the opposite direction from human
                for distance in range(3, 20, 2):  # Try shorter distances
                    # Calculate point directly away from human
                    escape_grid_x = robot_grid_x + int(dx * distance)
                    escape_grid_y = robot_grid_y + int(dy * distance)
                    
                    # Check if point is within map bounds
                    if (escape_grid_x < 0 or escape_grid_x >= map_data.shape[1] or
                        escape_grid_y < 0 or escape_grid_y >= map_data.shape[0]):
                        continue
                    
                    # Very basic check - just make sure it's not a wall
                    if map_data[escape_grid_y, escape_grid_x] > 50:
                        continue
                    
                    # Convert to world coordinates
                    world_x = origin_x + escape_grid_x * resolution
                    world_y = origin_y + escape_grid_y * resolution
                    
                    # Use this as fallback
                    best_point = (world_x, world_y, distance * resolution, 0.5)
                    self.node.get_logger().info(
                        f'Using fallback escape point at ({world_x:.2f}, {world_y:.2f})'
                    )
                    break
            
            if best_point is None:
                self.node.get_logger().error('No valid escape points found, even with fallback!')
                
                # Last resort - just return a point 1m away in the direction away from human
                world_x = robot_x + (robot_x - human_x) * 1.0 / current_dist_to_human
                world_y = robot_y + (robot_y - human_y) * 1.0 / current_dist_to_human
                
                best_point = (world_x, world_y, 1.0, 0.5)
                self.node.get_logger().warn(
                    f'Using emergency escape point at ({world_x:.2f}, {world_y:.2f})'
                )
            
            # Create waypoint from best point
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.node.get_clock().now().to_msg()
            waypoint.pose.position.x = best_point[0]
            waypoint.pose.position.y = best_point[1]
            
            # Face away from human
            dx = waypoint.pose.position.x - human_x
            dy = waypoint.pose.position.y - human_y
            yaw = math.atan2(dy, dx)  # Point away from human
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Mark this as an escape waypoint
            waypoint.header.stamp.nanosec = 1  # Special flag for escape waypoints
            
            self.node.get_logger().info(
                f'Generated escape waypoint {best_point[2]:.2f}m from human at '
                f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}) '
                f'with path clearance {best_point[3]:.2f}'
            )
            
            # Create visualization markers
            markers = self.create_visualization_markers(waypoint, is_escape=True)
            self.node.marker_pub.publish(markers)
            
            return waypoint
        
        except Exception as e:
            self.node.get_logger().warn(f'Error generating escape waypoint: {e}')
            import traceback
            self.node.get_logger().error(traceback.format_exc())
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