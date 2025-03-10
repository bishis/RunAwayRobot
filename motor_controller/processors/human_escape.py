#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from scipy.ndimage import distance_transform_edt
from .waypoint_generator import WaypointGenerator
import rclpy
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations

class HumanEscape(WaypointGenerator):
    """Specialized waypoint generator for escaping from humans"""
    
    def __init__(self, node):
        super().__init__(node)
        # Add tracking for dynamic escape monitoring
        self.dynamic_escape_active = False
        self.last_human_distance_to_waypoint = float('inf')
        self.min_intercept_distance = 1.0  # Minimum distance to consider human intercepting
        
        # Initialize waypoint tracking attributes
        self.previous_escape_waypoint = None
        self.failed_waypoints = []
        self.previous_waypoints = []
        self.human_positions = []
        
    def is_human_intercepting_escape(self):
        """Check if human is intercepting current escape path"""
        if not hasattr(self, 'previous_escape_waypoint') or self.previous_escape_waypoint is None:
            return False
            
        if not hasattr(self.node, 'last_human_position') or self.node.last_human_position is None:
            return False
            
        # Get current positions
        try:
            # Get robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Get human position
            human_x, human_y = self.node.last_human_position
            
            # Get waypoint position
            waypoint_x = self.previous_escape_waypoint.pose.position.x
            waypoint_y = self.previous_escape_waypoint.pose.position.y
            
            # Calculate distances
            human_to_waypoint = math.sqrt(
                (human_x - waypoint_x) ** 2 + 
                (human_y - waypoint_y) ** 2
            )
            
            robot_to_waypoint = math.sqrt(
                (robot_x - waypoint_x) ** 2 + 
                (robot_y - waypoint_y) ** 2
            )
            
            # Check if human is getting closer to waypoint
            is_human_getting_closer = human_to_waypoint < self.last_human_distance_to_waypoint
            self.last_human_distance_to_waypoint = human_to_waypoint
            
            # Calculate if human is closer to waypoint than robot or if human could intercept
            # We also consider the rate of approach - if human is getting closer quickly
            human_could_intercept = (
                human_to_waypoint < robot_to_waypoint * 1.2 or  # Human is closer or nearly as close
                (is_human_getting_closer and human_to_waypoint < self.min_intercept_distance * 2.5)  # Human is approaching waypoint quickly
            )
            
            if human_could_intercept:
                self.node.get_logger().warn(
                    f'Human may intercept escape path! Human-to-waypoint: {human_to_waypoint:.2f}m, '
                    f'Robot-to-waypoint: {robot_to_waypoint:.2f}m'
                )
                
            return human_could_intercept
            
        except Exception as e:
            self.node.get_logger().warn(f'Error checking for interception: {e}')
            return False
    
    def get_furthest_waypoint(self, previous_attempt_failed=False):
        """
        Generate a waypoint out of human line of sight when possible,
        otherwise at the furthest reachable point from human position.
        
        Args:
            previous_attempt_failed: If True, indicates previous navigation attempt failed
        """
        # Track previously failed waypoints to avoid reusing them
        if not hasattr(self, 'failed_waypoints'):
            self.failed_waypoints = []
        
        # Track previously used waypoints, even successful ones
        if not hasattr(self, 'previous_waypoints'):
            self.previous_waypoints = []
        
        # Maximum number of previous waypoints to remember
        MAX_PREVIOUS_WAYPOINTS = 3
        
        # Minimum distance between new waypoint and previous waypoints
        MIN_WAYPOINT_SEPARATION = 1.0  # 1 meter minimum separation
        
        # If previous attempt failed, add that waypoint to failed list
        if previous_attempt_failed and self.previous_escape_waypoint is not None:
            # Get coordinates of the failed waypoint
            failed_x = self.previous_escape_waypoint.pose.position.x
            failed_y = self.previous_escape_waypoint.pose.position.y
            
            # Add to failed list if not already there
            failed_waypoint = (failed_x, failed_y)
            if failed_waypoint not in self.failed_waypoints:
                self.failed_waypoints.append(failed_waypoint)
                self.node.get_logger().info(
                    f'Adding failed waypoint ({failed_x:.2f}, {failed_y:.2f}) to excluded points list'
                )
        
        # Always add the previous escape waypoint to the history if it exists
        if self.previous_escape_waypoint is not None:
            prev_x = self.previous_escape_waypoint.pose.position.x
            prev_y = self.previous_escape_waypoint.pose.position.y
            
            prev_waypoint = (prev_x, prev_y)
            if prev_waypoint not in self.previous_waypoints:
                self.previous_waypoints.append(prev_waypoint)
                
                # Limit the size of previous waypoints list
                if len(self.previous_waypoints) > MAX_PREVIOUS_WAYPOINTS:
                    self.previous_waypoints.pop(0)  # Remove oldest waypoint
        
        # Initialize tracking for dynamic escape
        self.last_human_distance_to_waypoint = float('inf')
        
        if self.current_map is None:
            self.node.get_logger().error('No map available for escape planning')
            # Try returning previous waypoint if available
            if self.previous_escape_waypoint is not None and not previous_attempt_failed:
                self.node.get_logger().info('Using previous escape waypoint due to missing map')
                return self.previous_escape_waypoint
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
                
                # Store initial human position for trajectory prediction
                if not hasattr(self, 'human_positions'):
                    self.human_positions = [(human_x, human_y, self.node.get_clock().now())]
                else:
                    # Add new position with timestamp
                    self.human_positions.append((human_x, human_y, self.node.get_clock().now()))
                    # Keep only recent positions (last 3 seconds)
                    current_time = self.node.get_clock().now()
                    self.human_positions = [pos for pos in self.human_positions 
                                           if (current_time - pos[2]).nanoseconds / 1e9 <= 3.0]
                
                self.node.get_logger().info(
                    f'Planning escape from human at ({human_x:.2f}, {human_y:.2f})'
                )
            else:
                self.node.get_logger().warn('No human position available for escape planning')
                if self.previous_escape_waypoint is not None and not previous_attempt_failed:
                    self.node.get_logger().info('Using previous escape waypoint due to missing human position')
                    return self.previous_escape_waypoint
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
            
            # Predict human movement direction if we have multiple positions
            human_velocity_x, human_velocity_y = 0.0, 0.0
            if hasattr(self, 'human_positions') and len(self.human_positions) >= 2:
                # Use the two most recent positions to estimate velocity
                newest_pos = self.human_positions[-1]
                oldest_pos = self.human_positions[0]
                
                time_diff = (newest_pos[2] - oldest_pos[2]).nanoseconds / 1e9
                if time_diff > 0.1:  # Only calculate if time difference is significant
                    human_velocity_x = (newest_pos[0] - oldest_pos[0]) / time_diff
                    human_velocity_y = (newest_pos[1] - oldest_pos[1]) / time_diff
                    
                    self.node.get_logger().info(
                        f'Estimated human velocity: ({human_velocity_x:.2f}, {human_velocity_y:.2f}) m/s'
                    )
            
            # Convert human position to grid coordinates
            human_grid_x = int((human_x - origin_x) / resolution)
            human_grid_y = int((human_y - origin_y) / resolution)
            
            # Make sure human coordinates are within map bounds
            human_grid_x = max(0, min(human_grid_x, map_data.shape[1]-1))
            human_grid_y = max(0, min(human_grid_y, map_data.shape[0]-1))
            
            valid_points = []
            min_wall_distance = self.safety_margin
            best_point = None
            best_los_point = None  # Best point with no line of sight
            best_score = float('-inf')
            best_los_score = float('-inf')  # Best score for point with no line of sight
            
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
                    
                    # Skip points too close to previously failed waypoints
                    too_close_to_failed = False
                    for fx, fy in self.failed_waypoints:
                        dist_to_failed = math.sqrt((world_x - fx)**2 + (world_y - fy)**2)
                        if dist_to_failed < 0.5:  # 0.5m exclusion radius
                            too_close_to_failed = True
                            break
                    
                    if too_close_to_failed:
                        continue
                    
                    # Skip points too close to previously used waypoints
                    too_close_to_previous = False
                    for px, py in self.previous_waypoints:
                        dist_to_previous = math.sqrt((world_x - px)**2 + (world_y - py)**2)
                        if dist_to_previous < MIN_WAYPOINT_SEPARATION:
                            too_close_to_previous = True
                            break
                    
                    if too_close_to_previous:
                        continue
                    
                    # Calculate distance to human
                    dist_to_human = math.sqrt(
                        (world_x - human_x) ** 2 + 
                        (world_y - human_y) ** 2
                    )
                    
                    # Skip points that are closer to human than current position
                    if dist_to_human <= current_dist_to_human:
                        continue
                    
                    # Calculate future position of human based on velocity (if available)
                    future_human_distance = dist_to_human
                    if abs(human_velocity_x) > 0.01 or abs(human_velocity_y) > 0.01:
                        # Project human position 2 seconds into the future
                        projection_time = 2.0
                        future_human_x = human_x + human_velocity_x * projection_time
                        future_human_y = human_y + human_velocity_y * projection_time
                        
                        future_human_distance = math.sqrt(
                            (world_x - future_human_x) ** 2 + 
                            (world_y - future_human_y) ** 2
                        )
                    
                    # Calculate distance from robot (for feasibility)
                    dist_from_robot = math.sqrt(
                        (world_x - robot_x) ** 2 + 
                        (world_y - robot_y) ** 2
                    )
                    
                    # Skip points that are too far from robot to be practical
                    if dist_from_robot > 5.0:  # Max 5 meters from current position
                        continue
                    
                    # Calculate angle from human to potential point
                    angle_from_human = math.atan2(world_y - human_y, world_x - human_x)
                    
                    # Calculate angle from human to robot
                    angle_human_to_robot = math.atan2(robot_y - human_y, robot_x - human_x)
                    
                    # Calculate absolute angle difference (0 means same direction, pi means opposite)
                    angle_diff = abs(normalize_angle(angle_from_human - angle_human_to_robot))
                    
                    # Prefer points in different direction from robot to avoid human pursuing
                    # Angle difference close to pi (180 degrees) is best (opposite direction)
                    direction_score = angle_diff / math.pi  # 1.0 is best (opposite), 0.0 is worst (same direction)
                    
                    # Check if point is out of line of sight from human
                    # Treat the human as a circle with radius 0.25m, not just a point
                    human_radius = 0.25  # Human radius in meters
                    has_line_of_sight = False
                    
                    # Check from multiple points around the human's perimeter
                    num_angles = 16  # Check 16 points around the perimeter
                    for angle_idx in range(num_angles):
                        angle = 2 * math.pi * angle_idx / num_angles
                        
                        # Calculate perimeter point coordinates
                        perimeter_x = human_x + human_radius * math.cos(angle)
                        perimeter_y = human_y + human_radius * math.sin(angle)
                        
                        # Convert to grid coordinates
                        perimeter_grid_x = int((perimeter_x - origin_x) / resolution)
                        perimeter_grid_y = int((perimeter_y - origin_y) / resolution)
                        
                        # Keep points within bounds
                        perimeter_grid_x = max(0, min(perimeter_grid_x, width-1))
                        perimeter_grid_y = max(0, min(perimeter_grid_y, height-1))
                        
                        # Check line of sight from this perimeter point
                        path_cells = self.get_line_cells(perimeter_grid_x, perimeter_grid_y, x, y)
                        
                        # Check if there's a clear path from this perimeter point
                        perimeter_has_los = True
                        for cx, cy in path_cells:
                            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                                continue
                            if map_data[cy, cx] > 50:  # Obstacle
                                perimeter_has_los = False
                                break
                        
                        # If any perimeter point has line of sight, then the human can see the point
                        if perimeter_has_los:
                            has_line_of_sight = True
                            break
                    
                    # Calculate LOS (line of sight) score - GREATLY increase importance for hidden points
                    los_score = 0.0 if has_line_of_sight else 100.0  # Increased from 30 to 100
                    
                    # Calculate score based on distance from human, future distance, wall clearance and direction
                    total_score = (
                        8.0 * dist_to_human +           # Heavily prioritize distance from human
                        4.0 * future_human_distance +   # Consider predicted future distance
                        2.0 * wall_distance[y, x] +     # Consider wall clearance
                        3.0 * direction_score +         # Prefer opposite direction from human-robot vector
                        -1.0 * dist_from_robot +        # Slight penalty for distance from robot
                        los_score                       # Extra points for breaking line of sight
                    )
                    
                    # Track best point and best line-of-sight blocked point separately
                    if not has_line_of_sight and total_score > best_los_score:
                        best_los_score = total_score
                        best_los_point = (world_x, world_y, dist_to_human)
                        self.node.get_logger().info(
                            f'New best hidden point found at ({world_x:.2f}, {world_y:.2f}) - '
                            f'distance from human: {dist_to_human:.2f}m, score: {total_score:.2f}'
                        )
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_point = (world_x, world_y, dist_to_human)
                        self.node.get_logger().info(
                            f'New best point found at ({world_x:.2f}, {world_y:.2f}) - '
                            f'distance from human: {dist_to_human:.2f}m, score: {total_score:.2f}'
                        )
            
            # Prefer a point that breaks line of sight if available, otherwise use best point
            final_point = best_los_point if best_los_point is not None else best_point
            
            if final_point is None:
                self.node.get_logger().error('No valid escape points found that increase distance from human!')
                
                # If we've tried many points and none work, start fresh
                if len(self.failed_waypoints) > 5:
                    self.node.get_logger().warn('Too many failed waypoints, clearing failed list')
                    self.failed_waypoints = []
                
                # Use previous waypoint if available and not already failed
                if self.previous_escape_waypoint is not None and not previous_attempt_failed:
                    self.node.get_logger().info('Using previous escape waypoint as fallback')
                    return self.previous_escape_waypoint
                return None
            
            # Create waypoint from final point
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.node.get_clock().now().to_msg()
            waypoint.pose.position.x = final_point[0]
            waypoint.pose.position.y = final_point[1]
            
            # Face towards human
            dx = waypoint.pose.position.x - human_x
            dy = waypoint.pose.position.y - human_y
            yaw = math.atan2(dy, dx)  # Point away from human
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Mark this as an escape waypoint
            waypoint.header.stamp.nanosec = 1  # Special flag for escape waypoints
            
            # Log line of sight information
            if final_point == best_los_point:
                self.node.get_logger().info(
                    f'Generated hidden escape waypoint {final_point[2]:.2f}m from human at '
                    f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
                )
            else:
                self.node.get_logger().info(
                    f'Generated visible escape waypoint {final_point[2]:.2f}m from human at '
                    f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
                )
            
            # Create visualization markers
            markers = self.create_visualization_markers(waypoint, is_escape=True)
            self.node.marker_pub.publish(markers)
            
            # Store this waypoint for future use if needed
            self.previous_escape_waypoint = waypoint
            
            # Don't clear failed waypoints when finding a new waypoint to maintain history
            # We'll limit the size of the failed list when it gets too large
            if len(self.failed_waypoints) > 10:
                # Remove oldest failed waypoints when list gets too long
                excess = len(self.failed_waypoints) - 10
                self.failed_waypoints = self.failed_waypoints[excess:]
                self.node.get_logger().info(f'Pruned {excess} oldest failed waypoints')
            
            return waypoint
        
        except Exception as e:
            self.node.get_logger().warn(f'Could not get robot position: {e}')
            if self.previous_escape_waypoint is not None and not previous_attempt_failed:
                self.node.get_logger().info('Using previous escape waypoint due to error')
                return self.previous_escape_waypoint
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
    
    def check_and_update_escape_if_needed(self):
        """Check if human is intercepting escape path and find new path if needed"""
        try:
            # Skip if missing required data
            if (not hasattr(self.node, 'last_human_position') or 
                self.node.last_human_position is None or 
                self.node.current_goal is None or
                not hasattr(self.node, 'current_pose') or
                self.node.current_pose is None):
                return None
            
            # Get current positions
            robot_pos = (self.node.current_pose.pose.position.x, self.node.current_pose.pose.position.y)
            target_pos = (self.node.current_goal.pose.position.x, self.node.current_goal.pose.position.y)
            human_pos = self.node.last_human_position
            
            # Check how long we've been on this escape path
            current_time = self.node.get_clock().now()
            if not hasattr(self, 'escape_start_time'):
                self.escape_start_time = current_time
                self.last_replan_time = current_time
                self.node.get_logger().info("Starting new escape path tracking")
                return None
            
            # Don't replan if we just started this escape (give it at least 1 second)
            time_since_escape_start = (current_time - self.escape_start_time).nanoseconds / 1e9
            if time_since_escape_start < 1.0:
                self.node.get_logger().debug(f"Ignoring path check - escape just started {time_since_escape_start:.1f}s ago")
                return None
            
            # Don't replan too frequently (minimum 2 seconds between replans)
            time_since_last_replan = (current_time - self.last_replan_time).nanoseconds / 1e9
            if time_since_last_replan < 2.0:
                self.node.get_logger().debug(f"Waiting for replan cooldown ({time_since_last_replan:.1f}s < 2.0s)")
                return None
            
            # Calculate distances
            human_to_robot_dist = math.sqrt((human_pos[0] - robot_pos[0])**2 + (human_pos[1] - robot_pos[1])**2)
            path_length = math.sqrt((target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2)
            
            # If the human is far away or we're very close to goal, no need to replan
            if human_to_robot_dist > 3.0 or path_length < 0.8:
                return None
            
            # Calculate how far along our path we've moved
            if hasattr(self, 'last_robot_pos'):
                progress = math.sqrt(
                    (robot_pos[0] - self.last_robot_pos[0])**2 + 
                    (robot_pos[1] - self.last_robot_pos[1])**2
                )
                
                # If we're making good progress and human isn't super close, continue on path
                if progress > 0.3 and human_to_robot_dist > 1.2:
                    self.last_robot_pos = robot_pos
                    return None
            
            # Store position for next comparison
            self.last_robot_pos = robot_pos
            
            # Check if human is in the escape path
            result = self.is_human_intercepting_path(robot_pos, target_pos, human_pos)
            
            if result:
                human_to_waypoint = math.sqrt(
                    (human_pos[0] - target_pos[0])**2 + 
                    (human_pos[1] - target_pos[1])**2
                )
                
                self.node.get_logger().warn(
                    f"Human may intercept escape path! Human-to-waypoint: {human_to_waypoint:.2f}m, "
                    f"Robot-to-waypoint: {path_length:.2f}m"
                )
                
                # Check human movement direction (if we have history)
                if hasattr(self, 'last_human_pos') and self.last_human_pos is not None:
                    human_dx = human_pos[0] - self.last_human_pos[0]
                    human_dy = human_pos[1] - self.last_human_pos[1]
                    
                    # Project human movement onto path direction
                    path_dx = target_pos[0] - robot_pos[0]
                    path_dy = target_pos[1] - robot_pos[1]
                    path_mag = math.sqrt(path_dx**2 + path_dy**2)
                    
                    if path_mag > 0:
                        path_dx /= path_mag
                        path_dy /= path_mag
                        
                        # Dot product tells us if human is moving toward path
                        dot_product = human_dx * path_dx + human_dy * path_dy
                        
                        # If human is moving away from our path, don't replan
                        if dot_product < 0:
                            self.node.get_logger().info("Human is moving away from our path, continuing current plan")
                            self.last_human_pos = human_pos
                            return None
                
                # Update time tracking
                self.last_replan_time = current_time
                self.last_human_pos = human_pos
                
                # Human is intercepting, generate new path
                self.node.get_logger().info("Human is intercepting escape path, generating new escape plan")
                return self.get_furthest_waypoint(True)  # Generate new escape point
            
            # Update human position history
            self.last_human_pos = human_pos
            return None
        
        except Exception as e:
            self.node.get_logger().error(f"Error checking path interception: {str(e)}")
            return None

    def force_escape_waypoint_change(self):
        """Force the generator to pick a new escape waypoint by clearing previous state"""
        self.node.get_logger().info('Forcing new escape waypoint')
        self.previous_escape_waypoint = None
        self.last_human_distance_to_waypoint = float('inf')
        
        # Don't clear failed_waypoints list to avoid selecting previously failed waypoints
        
        # Reset dynamic escape monitoring
        self.dynamic_escape_active = True
        
        # Clear visualization
        empty_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.action = Marker.DELETE
        empty_markers.markers.append(marker)
        self.node.marker_pub.publish(empty_markers)

    def find_hiding_spot(self, robot_pos, human_pos):
        """Find a waypoint that would be hidden from human view"""
        try:
            # Skip if no map is available
            if self.current_map is None:
                self.node.get_logger().error('No map available for hiding spot search')
                return None
            
            # Get map info
            map_origin = self.current_map.info.origin
            resolution = self.current_map.info.resolution
            width = self.current_map.info.width
            height = self.current_map.info.height
            
            # Ensure we have obstacles in map data
            if not hasattr(self, 'obstacle_grid'):
                self.node.get_logger().warn('No obstacle grid for hiding spot search')
                return None
            
            # Convert positions to grid coordinates
            robot_grid_x = int((robot_pos[0] - map_origin.position.x) / resolution)
            robot_grid_y = int((robot_pos[1] - map_origin.position.y) / resolution)
            human_grid_x = int((human_pos[0] - map_origin.position.x) / resolution)
            human_grid_y = int((human_pos[1] - map_origin.position.y) / resolution)
            
            # Find potential hiding spots (cells that have obstacles between them and human)
            hiding_candidates = []
            radius = 20  # Search radius in grid cells
            
            # Expand radius until we find at least one candidate
            while len(hiding_candidates) == 0 and radius <= 50:  # Max radius of 50 cells
                for dx in range(-radius, radius+1):
                    for dy in range(-radius, radius+1):
                        test_x = robot_grid_x + dx
                        test_y = robot_grid_y + dy
                        
                        # Skip if out of bounds
                        if test_x < 0 or test_x >= width or test_y < 0 or test_y >= height:
                            continue
                        
                        # Skip if too close to current position
                        if abs(dx) < 5 and abs(dy) < 5:
                            continue
                        
                        # Skip if an obstacle or too close to one
                        if self.obstacle_grid[test_y][test_x] > 0:
                            continue
                        
                        # Check if there's a line of sight to human
                        if not self.has_line_of_sight(test_x, test_y, human_grid_x, human_grid_y):
                            # Convert grid coords back to world coords
                            world_x = test_x * resolution + map_origin.position.x
                            world_y = test_y * resolution + map_origin.position.y
                            
                            # Calculate distance from robot
                            dist = math.sqrt((world_x - robot_pos[0])**2 + (world_y - robot_pos[1])**2)
                            
                            hiding_candidates.append((world_x, world_y, dist))
                
                radius += 10  # Increase radius if no candidates found
            
            if hiding_candidates:
                # Sort by distance (closest first)
                hiding_candidates.sort(key=lambda x: x[2])
                
                # Take the closest candidate
                best_spot = hiding_candidates[0]
                self.node.get_logger().info(f'Found hiding spot at ({best_spot[0]:.2f}, {best_spot[1]:.2f}), '
                                          f'distance: {best_spot[2]:.2f}m')
                
                # Create PoseStamped message
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = best_spot[0]
                pose.pose.position.y = best_spot[1]
                pose.pose.position.z = 0.0
                
                # Set orientation to face away from human
                dx = best_spot[0] - human_pos[0]
                dy = best_spot[1] - human_pos[1]
                yaw = math.atan2(dy, dx)
                
                # Convert yaw to quaternion
                qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                return pose
            else:
                self.node.get_logger().warn('No suitable hiding spots found')
                return None
        
        except Exception as e:
            self.node.get_logger().error(f'Error finding hiding spot: {str(e)}')
            return None

    def has_line_of_sight(self, x1, y1, x2, y2):
        """Check if there's a clear line of sight between two grid cells"""
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        
        err = dx - dy
        
        while x1 != x2 or y1 != y2:
            # Check if current cell is an obstacle
            if self.obstacle_grid[y1][x1] > 0:
                return False
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        
        return True

    def is_human_intercepting_path(self, robot_pos, target_pos, human_pos):
        """Check if human is intercepting the path from robot to target"""
        # Vector from robot to target
        path_vector = (target_pos[0] - robot_pos[0], target_pos[1] - robot_pos[1])
        path_length = math.sqrt(path_vector[0]**2 + path_vector[1]**2)
        
        # Normalize path vector
        if path_length > 0:
            path_unit = (path_vector[0]/path_length, path_vector[1]/path_length)
        else:
            return False
        
        # Vector from robot to human
        human_vector = (human_pos[0] - robot_pos[0], human_pos[1] - robot_pos[1])
        human_length = math.sqrt(human_vector[0]**2 + human_vector[1]**2)
        
        # Project human vector onto path
        if human_length > 0:
            dot_product = human_vector[0]*path_unit[0] + human_vector[1]*path_unit[1]
        else:
            return False
        
        # If human is behind us or beyond target, they're not intercepting
        if dot_product < 0 or dot_product > path_length:
            return False
        
        # Find closest point on path to human
        proj_point = (
            robot_pos[0] + path_unit[0] * dot_product,
            robot_pos[1] + path_unit[1] * dot_product
        )
        
        # Distance from human to path
        dist_to_path = math.sqrt(
            (human_pos[0] - proj_point[0])**2 + 
            (human_pos[1] - proj_point[1])**2
        )
        
        # Calculate intercept threshold based on distance
        # Closer to robot = needs more space due to turning radius
        interception_threshold = 0.8  # Base distance
        
        # If human is really close to the robot, consider it an interception
        if human_length < 1.0:
            return True
        
        # If distance to path is less than threshold, human is intercepting
        return dist_to_path < interception_threshold

def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle