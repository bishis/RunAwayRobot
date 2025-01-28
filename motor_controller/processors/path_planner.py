import numpy as np
import math
from geometry_msgs.msg import Point, Twist, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from typing import List, Tuple, Optional
import rclpy

class PathPlanner:
    """Simplifies paths into straight lines and rotations for binary movement control."""
    
    def __init__(self, angle_threshold: float = 0.2, min_segment_length: float = 0.1):
        """
        Initialize path planner with control parameters.
        
        Args:
            angle_threshold: Minimum angle change (radians) to consider a new segment
            min_segment_length: Minimum length (meters) for a path segment
        """
        self.angle_threshold = 0.5  # About 30 degrees - only make significant turns
        self.min_segment_length = 0.2  # Minimum 20cm segments
        
        # Colors for visualization
        self.colors = {
            'path': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),      # Green for path
            'rotation': ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),  # Magenta for rotations
            'forward': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Blue for forward movements
            'text': ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)       # White for text
        }

    def simplify_path(self, waypoints: List[Point], initial_heading: float = 0.0) -> List[Tuple[str, float]]:
        """Create a simple path with just start and end points"""
        if len(waypoints) < 2:
            return []

        # Just use start and end points
        start = waypoints[0]
        end = waypoints[-1]

        # Calculate direct path
        dx = end.x - start.x
        dy = end.y - start.y
        
        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        
        # Calculate distance
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Create simple two-command path:
        # 1. Turn to face target
        # 2. Move straight to target
        commands = []
        
        # First command: rotate to face target
        # Use the provided initial heading instead of assuming 0
        angle = self._normalize_angle(target_angle - initial_heading)
        if abs(angle) > self.angle_threshold:
            # Round angle to nearest 45 degrees
            angle = round(angle / (math.pi/4)) * (math.pi/4)
            commands.append(('rotate', angle))
        
        # Second command: move forward
        if distance > self.min_segment_length:
            commands.append(('forward', distance))
        
        return commands

    def _douglas_peucker(self, points: List[Point], epsilon: float) -> List[Point]:
        """Douglas-Peucker path simplification algorithm"""
        if len(points) < 3:
            return points
            
        # Find point with maximum distance
        dmax = 0
        index = 0
        for i in range(1, len(points) - 1):
            d = self._point_line_distance(points[i], points[0], points[-1])
            if d > dmax:
                index = i
                dmax = d
        
        # If max distance is greater than epsilon, recursively simplify
        if dmax > epsilon:
            # Recursive call
            rec_results1 = self._douglas_peucker(points[:index + 1], epsilon)
            rec_results2 = self._douglas_peucker(points[index:], epsilon)
            # Build the result list
            return rec_results1[:-1] + rec_results2
        else:
            return [points[0], points[-1]]

    def _point_line_distance(self, point: Point, line_start: Point, line_end: Point) -> float:
        """Calculate perpendicular distance from point to line"""
        if line_start.x == line_end.x and line_start.y == line_end.y:
            return math.sqrt(
                (point.x - line_start.x)**2 + 
                (point.y - line_start.y)**2
            )
            
        num = abs(
            (line_end.y - line_start.y) * point.x -
            (line_end.x - line_start.x) * point.y +
            line_end.x * line_start.y -
            line_end.y * line_start.x
        )
        den = math.sqrt(
            (line_end.y - line_start.y)**2 +
            (line_end.x - line_start.x)**2
        )
        return num/den if den != 0 else 0

    def generate_velocity_commands(self, commands: List[Tuple[str, float]]) -> List[Twist]:
        """
        Convert movement commands into velocity commands.
        
        Args:
            commands: List of (command_type, value) tuples from simplify_path
            
        Returns:
            List of Twist messages with binary velocity values
        """
        velocity_commands = []
        
        for cmd_type, value in commands:
            cmd = Twist()
            
            if cmd_type == 'rotate':
                # Positive angle = counter-clockwise = positive angular velocity
                cmd.angular.z = 1.0 if value > 0 else -1.0
                cmd.linear.x = 0.0
                
            elif cmd_type == 'forward':
                # Always move forward with positive distance
                cmd.linear.x = 1.0
                cmd.angular.z = 0.0
                
            velocity_commands.append(cmd)
            
        return velocity_commands

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def estimate_command_duration(self, command: Tuple[str, float], 
                                linear_speed: float = 0.1, 
                                angular_speed: float = 1.0) -> float:
        """
        Estimate the duration needed for a command based on speeds.
        
        Args:
            command: (command_type, value) tuple
            linear_speed: Robot's linear speed in m/s
            angular_speed: Robot's angular speed in rad/s
            
        Returns:
            Estimated duration in seconds
        """
        cmd_type, value = command
        
        if cmd_type == 'rotate':
            return abs(value) / angular_speed
        else:  # forward
            return abs(value) / linear_speed

    def create_visualization_markers(self, commands: List[Tuple[str, float]], 
                                  start_point: Point,
                                  frame_id: str = 'map') -> MarkerArray:
        """
        Create visualization markers for the simplified path.
        
        Args:
            commands: List of (command_type, value) tuples
            start_point: Starting position
            frame_id: TF frame for markers
            
        Returns:
            MarkerArray containing path visualization
        """
        marker_array = MarkerArray()
        current_pos = np.array([start_point.x, start_point.y])
        current_heading = 0.0
        
        # Single deletion marker with ID 0
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = rclpy.time.Time().to_msg()
        clear_marker.ns = "path_planner"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Path line marker - always ID 1
        path_points = [start_point]
        path_marker = Marker()
        path_marker.header.frame_id = frame_id
        path_marker.header.stamp = rclpy.time.Time().to_msg()
        path_marker.ns = "path_planner"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale = Vector3(x=0.02, y=0.0, z=0.0)
        path_marker.color = self.colors['path']
        path_marker.pose.orientation.w = 1.0
        
        # Start with ID 2 for other markers
        marker_id = 2
        
        for cmd_type, value in commands:
            if cmd_type == 'rotate':
                # Add rotation visualization
                rot_marker = Marker()
                rot_marker.header.frame_id = frame_id
                rot_marker.header.stamp = rclpy.time.Time().to_msg()
                rot_marker.ns = "path_planner"
                rot_marker.id = marker_id
                marker_id += 1
                rot_marker.type = Marker.CYLINDER
                rot_marker.action = Marker.ADD
                
                # Cylinder represents rotation point
                rot_marker.scale = Vector3(x=0.1, y=0.1, z=0.05)
                rot_marker.color = self.colors['rotation']
                rot_marker.pose.position.x = current_pos[0]
                rot_marker.pose.position.y = current_pos[1]
                rot_marker.pose.position.z = 0.025
                rot_marker.pose.orientation.w = 1.0
                marker_array.markers.append(rot_marker)
                
                # Add text showing rotation angle
                text_marker = Marker()
                text_marker.header.frame_id = frame_id
                text_marker.header.stamp = rclpy.time.Time().to_msg()
                text_marker.ns = "path_planner"
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.1  # Text height
                text_marker.color = self.colors['text']
                text_marker.pose.position.x = current_pos[0]
                text_marker.pose.position.y = current_pos[1]
                text_marker.pose.position.z = 0.15
                text_marker.text = f"{math.degrees(value):.1f}°"
                marker_array.markers.append(text_marker)
                
                current_heading += value
                
            else:  # forward
                # Calculate new position
                new_pos = current_pos + value * np.array([
                    math.cos(current_heading),
                    math.sin(current_heading)
                ])
                
                # Add arrow for forward movement
                arrow_marker = Marker()
                arrow_marker.header.frame_id = frame_id
                arrow_marker.header.stamp = rclpy.time.Time().to_msg()
                arrow_marker.ns = "path_planner"
                arrow_marker.id = marker_id
                marker_id += 1
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.scale = Vector3(x=0.05, y=0.08, z=0.08)  # Arrow size
                arrow_marker.color = self.colors['forward']
                
                # Set arrow points
                arrow_marker.points = [
                    Point(x=current_pos[0], y=current_pos[1], z=0.05),
                    Point(x=new_pos[0], y=new_pos[1], z=0.05)
                ]
                marker_array.markers.append(arrow_marker)
                
                # Add distance text
                text_marker = Marker()
                text_marker.header.frame_id = frame_id
                text_marker.header.stamp = rclpy.time.Time().to_msg()
                text_marker.ns = "path_planner"
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.1  # Text height
                text_marker.color = self.colors['text']
                mid_point = (current_pos + new_pos) / 2
                text_marker.pose.position.x = mid_point[0]
                text_marker.pose.position.y = mid_point[1]
                text_marker.pose.position.z = 0.15
                text_marker.text = f"{value:.2f}m"
                marker_array.markers.append(text_marker)
                
                # Update path line
                path_points.append(Point(x=new_pos[0], y=new_pos[1], z=0.0))
                current_pos = new_pos
        
        # Add path line to markers
        path_marker.points = path_points
        marker_array.markers.append(path_marker)
        
        return marker_array
