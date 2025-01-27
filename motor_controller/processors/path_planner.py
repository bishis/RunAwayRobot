import numpy as np
import math
from geometry_msgs.msg import Point, Twist
from typing import List, Tuple, Optional

class PathPlanner:
    """Simplifies paths into straight lines and rotations for binary movement control."""
    
    def __init__(self, angle_threshold: float = 0.2, min_segment_length: float = 0.1):
        """
        Initialize path planner with control parameters.
        
        Args:
            angle_threshold: Minimum angle change (radians) to consider a new segment
            min_segment_length: Minimum length (meters) for a path segment
        """
        self.angle_threshold = angle_threshold
        self.min_segment_length = min_segment_length

    def simplify_path(self, waypoints: List[Point]) -> List[Tuple[str, float]]:
        """
        Convert a list of waypoints into a sequence of straight lines and rotations.
        
        Args:
            waypoints: List of Points representing the path
            
        Returns:
            List of (command_type, value) tuples where:
                command_type is either 'rotate' or 'forward'
                value is angle (radians) for rotate or distance (meters) for forward
        """
        if len(waypoints) < 2:
            return []
            
        commands = []
        current_pos = waypoints[0]
        current_heading = 0.0  # Assume starting heading is 0 (positive x-axis)
        
        for next_point in waypoints[1:]:
            # Calculate angle and distance to next point
            dx = next_point.x - current_pos.x
            dy = next_point.y - current_pos.y
            target_angle = math.atan2(dy, dx)
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Skip very short segments
            if distance < self.min_segment_length:
                continue
                
            # Calculate required rotation
            angle_diff = self._normalize_angle(target_angle - current_heading)
            
            # Add rotation command if angle is significant
            if abs(angle_diff) > self.angle_threshold:
                commands.append(('rotate', angle_diff))
            
            # Add forward command
            commands.append(('forward', distance))
            
            # Update current position and heading
            current_pos = next_point
            current_heading = target_angle
            
        return commands

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

    def visualize_commands(self, commands: List[Tuple[str, float]], 
                         start_point: Point) -> List[Point]:
        """
        Generate visualization points for the simplified path.
        
        Args:
            commands: List of movement commands
            start_point: Starting position
            
        Returns:
            List of Points representing the simplified path
        """
        points = [start_point]
        current_pos = np.array([start_point.x, start_point.y])
        current_heading = 0.0
        
        for cmd_type, value in commands:
            if cmd_type == 'rotate':
                current_heading += value
            else:  # forward
                # Move in current heading direction
                current_pos += value * np.array([
                    math.cos(current_heading),
                    math.sin(current_heading)
                ])
                points.append(Point(
                    x=float(current_pos[0]),
                    y=float(current_pos[1]),
                    z=0.0
                ))
                
        return points
