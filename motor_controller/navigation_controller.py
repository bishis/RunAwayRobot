#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from enum import Enum
from typing import List, Tuple
import heapq

class RobotState(Enum):
    PLANNING = 1
    FOLLOWING = 2
    ROTATING = 3
    STOPPED = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('safety_radius', 0.5)
        self.declare_parameter('waypoint_threshold', 0.3)
        self.declare_parameter('leg_length', 2.0)
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
        
        # Navigation state
        self.state = RobotState.PLANNING
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.current_path: List[Point] = []
        self.current_waypoint_index = 0
        self.square_wave_points = self.generate_square_wave()
        self.target_waypoint = None
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation controller initialized')

    def generate_square_wave(self) -> List[Point]:
        """Generate square wave waypoints."""
        points = []
        x, y = 0, 0
        leg_length = self.leg_length
        
        # Generate 4 legs of the square wave
        for i in range(4):
            if i % 2 == 0:
                y += leg_length
            else:
                x += leg_length
            point = Point()
            point.x, point.y = x, y
            points.append(point)
        
        return points

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* path planning algorithm."""
        if not self.map_data:
            return []

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                new_pos = (pos[0] + dx, pos[1] + dy)
                if (0 <= new_pos[0] < self.map_data.info.width and
                    0 <= new_pos[1] < self.map_data.info.height and
                    self.map_data.data[new_pos[1] * self.map_data.info.width + new_pos[0]] < 50):
                    neighbors.append(new_pos)
            return neighbors

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]
            
            if current == goal:
                break
                
            for next_pos in get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map coordinates."""
        if not self.map_data:
            return (0, 0)
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        return (mx, my)

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map coordinates to world coordinates."""
        if not self.map_data:
            return (0.0, 0.0)
        x = mx * self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = my * self.map_data.info.resolution + self.map_data.info.origin.position.y
        return (x, y)

    def plan_path_to_waypoint(self, target_point: Point):
        """Plan path to next waypoint using A*."""
        if not self.current_pose or not self.map_data:
            return []
            
        start = self.world_to_map(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        goal = self.world_to_map(target_point.x, target_point.y)
        
        path_cells = self.a_star(start, goal)
        
        # Convert cell path to world coordinates
        path = []
        for cell in path_cells:
            x, y = self.map_to_world(cell[0], cell[1])
            point = Point()
            point.x, point.y = x, y
            path.append(point)
        
        return path

    def get_binary_velocity_commands(self, linear_x: float, angular_z: float) -> Tuple[int, int]:
        """Convert continuous velocity commands to binary wheel speeds."""
        # Threshold for movement
        threshold = 0.1
        
        # Calculate wheel speeds
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        # Convert to binary
        left_binary = 1 if left_speed > threshold else (-1 if left_speed < -threshold else 0)
        right_binary = 1 if right_speed > threshold else (-1 if right_speed < -threshold else 0)
        
        return left_binary, right_binary

    def control_loop(self):
        if not self.current_pose or not self.map_data:
            return

        if self.state == RobotState.PLANNING:
            # Get next waypoint
            if self.current_waypoint_index < len(self.square_wave_points):
                self.target_waypoint = self.square_wave_points[self.current_waypoint_index]
                self.current_path = self.plan_path_to_waypoint(self.target_waypoint)
                if self.current_path:
                    self.state = RobotState.FOLLOWING
                    self.get_logger().info(f"Planning complete, following path to waypoint {self.current_waypoint_index}")
            else:
                self.state = RobotState.STOPPED
                self.stop_robot()
                return

        elif self.state == RobotState.FOLLOWING:
            # Check if we've reached the current waypoint
            if self.is_at_waypoint(self.target_waypoint):
                self.current_waypoint_index += 1
                self.state = RobotState.PLANNING
                self.get_logger().info("Reached waypoint, planning next path")
                return

            # Follow the path
            if self.current_path:
                linear_x, angular_z = self.pure_pursuit()
                left_speed, right_speed = self.get_binary_velocity_commands(linear_x, angular_z)
                self.send_wheel_commands(left_speed, right_speed)

    def pure_pursuit(self) -> Tuple[float, float]:
        """Pure pursuit path following algorithm."""
        if not self.current_path:
            return (0.0, 0.0)

        # Find closest point on path
        min_dist = float('inf')
        target_point = None
        lookahead_distance = 0.5  # meters

        for point in self.current_path:
            dist = math.sqrt(
                (point.x - self.current_pose.position.x)**2 +
                (point.y - self.current_pose.position.y)**2
            )
            if dist < min_dist:
                min_dist = dist
                if dist > lookahead_distance:
                    target_point = point

        if not target_point:
            target_point = self.current_path[-1]

        # Calculate angle to target
        target_angle = math.atan2(
            target_point.y - self.current_pose.position.y,
            target_point.x - self.current_pose.position.x
        )
        
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_diff = target_angle - current_yaw
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        # Convert to velocity commands
        if abs(angle_diff) > math.pi/4:
            # Turn in place
            return (0.0, 1.0 if angle_diff > 0 else -1.0)
        else:
            # Move forward while turning
            return (1.0, angle_diff)

    def is_at_waypoint(self, waypoint: Point) -> bool:
        """Check if robot has reached waypoint."""
        if not self.current_pose:
            return False
            
        distance = math.sqrt(
            (waypoint.x - self.current_pose.position.x)**2 +
            (waypoint.y - self.current_pose.position.y)**2
        )
        return distance < self.waypoint_threshold

    def send_wheel_commands(self, left_speed: int, right_speed: int):
        """Send wheel commands to the robot."""
        cmd = Twist()
        if left_speed == right_speed:
            cmd.linear.x = float(left_speed)
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = float(right_speed - left_speed)
        self.cmd_vel_pub.publish(cmd)

    def map_callback(self, msg):
        """Process map updates."""
        self.map_data = msg
    
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg
    
    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def stop_robot(self):
        """Stop the robot."""
        self.send_wheel_commands(0, 0)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 