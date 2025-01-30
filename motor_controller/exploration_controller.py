#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
import random
from scipy.ndimage import distance_transform_edt

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Parameters
        self.declare_parameter('min_distance', 0.5)  # Minimum distance between waypoints
        self.declare_parameter('goal_timeout', 30.0)  # Seconds before giving up on a goal
        self.declare_parameter('safety_margin', 0.3)  # Distance from walls
        self.declare_parameter('waypoint_size', 0.3)  # Size of waypoint markers
        self.declare_parameter('preferred_distance', 1.0)  # Preferred distance for new waypoints
        self.declare_parameter('min_waypoint_time', 30.0)  # Minimum time to keep a waypoint
        self.declare_parameter('goal_reached_distance', 0.5)  # Distance to consider goal reached
        self.declare_parameter('score_hysteresis', 0.3)  # Required score improvement to change waypoint
        
        # Get parameters
        self.min_distance = self.get_parameter('min_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.waypoint_size = self.get_parameter('waypoint_size').value
        self.preferred_distance = self.get_parameter('preferred_distance').value
        self.min_waypoint_time = self.get_parameter('min_waypoint_time').value
        self.goal_reached_distance = self.get_parameter('goal_reached_distance').value
        self.score_hysteresis = self.get_parameter('score_hysteresis').value
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            'map', 
            self.map_callback, 
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            'exploration_markers', 
            10
        )
        
        # State variables
        self.current_map = None
        self.current_waypoint = None
        self.is_navigating = False
        self.goal_start_time = None
        self.waypoint_start_time = None
        self.current_waypoint_score = -float('inf')
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)
        
        self.get_logger().info('Simple exploration controller initialized')

    def map_callback(self, msg):
        """Store map and generate waypoints if needed"""
        self.current_map = msg
        if not self.current_waypoint:
            self.generate_next_waypoint()

    def is_valid_point(self, x, y):
        """Check if a point is valid (free space and away from obstacles)"""
        if not self.current_map:
            return False
            
        # Convert world coordinates to map coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        
        # Check bounds
        if (map_x < 0 or map_x >= self.current_map.info.width or
            map_y < 0 or map_y >= self.current_map.info.height):
            return False
            
        # Get map data as 2D array
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Check if point and surrounding area is free space
        margin = int(self.safety_margin / resolution)
        for dy in range(-margin, margin + 1):
            for dx in range(-margin, margin + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                if (0 <= check_x < self.current_map.info.width and 
                    0 <= check_y < self.current_map.info.height):
                    if map_data[check_y, check_x] > 0:  # Not free space
                        return False
        return True

    def generate_next_waypoint(self):
        """Generate a single new waypoint prioritizing unexplored areas"""
        if not self.current_map:
            return None
        
        attempts = 0
        max_attempts = 100
        
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Get map data as 2D array
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Create exploration potential field
        unknown_area = map_data == -1
        walls = map_data > 50
        
        # Calculate distance transform from walls
        wall_distance = distance_transform_edt(~walls) * resolution
        
        # Calculate distance to unknown areas
        unknown_distance = distance_transform_edt(~unknown_area) * resolution
        
        # Get current robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert to grid coordinates
            robot_grid_x = int((robot_x - origin_x) / resolution)
            robot_grid_y = int((robot_y - origin_y) / resolution)
        except:
            self.get_logger().warn('Could not get robot position')
            return None

        best_point = None
        best_score = -float('inf')
        
        # Sample points and score them
        for _ in range(50):  # Try 50 random points
            # Find valid cells (free space)
            valid_y, valid_x = np.where((map_data == 0) & (wall_distance > self.safety_margin))
            
            if len(valid_x) == 0:
                continue
            
            # Randomly select one of the valid cells
            idx = random.randint(0, len(valid_x) - 1)
            map_x = valid_x[idx]
            map_y = valid_y[idx]
            
            # Convert to world coordinates
            x = origin_x + map_x * resolution
            y = origin_y + map_y * resolution
            
            # Calculate scores for different criteria
            dist_to_robot = math.sqrt((map_x - robot_grid_x)**2 + (map_y - robot_grid_y)**2) * resolution
            if dist_to_robot < self.min_distance:
                continue
            
            # Score based on:
            # 1. Distance to unknown areas (prefer closer to unknown)
            unknown_score = 1.0 / (unknown_distance[map_y, map_x] + 0.1)
            
            # 2. Distance from walls (prefer points away from walls)
            wall_score = min(wall_distance[map_y, map_x], 2.0) / 2.0
            
            # 3. Appropriate distance from robot
            distance_score = 1.0 - abs(dist_to_robot - self.preferred_distance) / self.preferred_distance
            
            # Combine scores with weights
            total_score = (
                3.0 * unknown_score +  # Prioritize exploring unknown areas
                2.0 * wall_score +     # Prefer staying away from walls
                1.0 * distance_score   # Consider distance from robot
            )
            
            if total_score > best_score:
                best_score = total_score
                best_point = (x, y)
        
        if best_point is None:
            return None
        
        # Create waypoint with the best point
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = best_point[0]
        waypoint.pose.position.y = best_point[1]
        waypoint.pose.position.z = 0.0
        
        # Set orientation towards unexplored area
        # Find direction of closest unknown area
        unknown_y, unknown_x = np.where(unknown_area)
        if len(unknown_x) > 0:
            # Find closest unknown point
            dists = [(ux - map_x)**2 + (uy - map_y)**2 for ux, uy in zip(unknown_x, unknown_y)]
            closest_idx = np.argmin(dists)
            target_x = origin_x + unknown_x[closest_idx] * resolution
            target_y = origin_y + unknown_y[closest_idx] * resolution
            
            # Calculate angle towards unknown area
            angle = math.atan2(target_y - best_point[1], target_x - best_point[0])
            waypoint.pose.orientation.z = math.sin(angle / 2)
            waypoint.pose.orientation.w = math.cos(angle / 2)
        else:
            # If no unknown areas, just use default orientation
            waypoint.pose.orientation.w = 1.0
        
        return waypoint

    def is_waypoint_reached(self):
        """Check if we've reached the current waypoint"""
        if not self.current_waypoint:
            return False
            
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            dx = transform.transform.translation.x - self.current_waypoint.pose.position.x
            dy = transform.transform.translation.y - self.current_waypoint.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            return distance < self.goal_reached_distance
        except:
            return False

    def should_generate_new_waypoint(self):
        """Determine if we should generate a new waypoint"""
        if not self.current_waypoint or not self.waypoint_start_time:
            return True
            
        # Check minimum time
        time_elapsed = (self.get_clock().now() - self.waypoint_start_time).nanoseconds / 1e9
        if time_elapsed < self.min_waypoint_time:
            return False
            
        # If we've reached the waypoint, generate a new one
        if self.is_waypoint_reached():
            return True
            
        # If we're stuck (timeout), generate a new one
        if time_elapsed > self.goal_timeout:
            return True
            
        return False

    def exploration_loop(self):
        """Main control loop for exploration"""
        if not self.current_map:
            return
            
        if not self.is_navigating:
            # Only generate new waypoint if needed
            if self.should_generate_new_waypoint():
                new_waypoint = self.generate_next_waypoint()
                if new_waypoint:
                    # Only accept new waypoint if it's significantly better
                    new_score = self.score_waypoint(new_waypoint)
                    if new_score > self.current_waypoint_score + self.score_hysteresis:
                        self.current_waypoint = new_waypoint
                        self.current_waypoint_score = new_score
                        self.waypoint_start_time = self.get_clock().now()
                        self.publish_waypoint_markers()
                        self.get_logger().info(
                            f'Generated new waypoint with score {new_score:.2f}'
                        )
                    else:
                        self.get_logger().info('New waypoint not significantly better, keeping current')
            
            if self.current_waypoint:
                # Send goal
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = self.current_waypoint
                
                self.get_logger().info(
                    f'Navigating to waypoint: ({self.current_waypoint.pose.position.x:.2f}, '
                    f'{self.current_waypoint.pose.position.y:.2f})'
                )
                
                self.nav_client.wait_for_server()
                self.current_goal = self.nav_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.feedback_callback
                )
                self.current_goal.add_done_callback(self.goal_response_callback)
                
                self.is_navigating = True
                self.goal_start_time = self.get_clock().now()

    def generate_new_goal(self):
        """Generate a new goal and reset navigation state"""
        self.is_navigating = False
        self.current_waypoint = None
        self.goal_start_time = None

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.generate_new_goal()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        self.generate_new_goal()

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass

    def publish_waypoint_markers(self):
        """Publish visualization markers for current waypoint"""
        if not self.current_waypoint:
            return
            
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'waypoints'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Cylinder marker
        cylinder_marker = Marker()
        cylinder_marker.header.frame_id = 'map'
        cylinder_marker.header.stamp = self.get_clock().now().to_msg()
        cylinder_marker.ns = 'waypoints'
        cylinder_marker.id = 1
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.action = Marker.ADD
        
        cylinder_marker.pose = self.current_waypoint.pose
        cylinder_marker.pose.position.z = 0.1
        cylinder_marker.scale.x = self.waypoint_size
        cylinder_marker.scale.y = self.waypoint_size
        cylinder_marker.scale.z = 0.2
        
        cylinder_marker.color.r = 1.0
        cylinder_marker.color.g = 0.0
        cylinder_marker.color.b = 0.0
        cylinder_marker.color.a = 0.8
        
        # Arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'map'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = 'waypoints'
        arrow_marker.id = 2
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        arrow_marker.pose = self.current_waypoint.pose
        arrow_marker.pose.position.z = 0.1
        arrow_marker.scale.x = 0.3
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        
        arrow_marker.color = cylinder_marker.color
        arrow_marker.color.a = 1.0
        
        marker_array.markers.append(cylinder_marker)
        marker_array.markers.append(arrow_marker)
        
        self.marker_pub.publish(marker_array)

    def score_waypoint(self, waypoint):
        """Score a waypoint based on exploration potential"""
        if not self.current_map:
            return -float('inf')
            
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Convert waypoint to grid coordinates
        grid_x = int((waypoint.pose.position.x - origin_x) / resolution)
        grid_y = int((waypoint.pose.position.y - origin_y) / resolution)
        
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Calculate scores using the same metrics as in generate_next_waypoint
        unknown_area = map_data == -1
        walls = map_data > 50
        
        wall_distance = distance_transform_edt(~walls) * resolution
        unknown_distance = distance_transform_edt(~unknown_area) * resolution
        
        # Calculate component scores
        unknown_score = 1.0 / (unknown_distance[grid_y, grid_x] + 0.1)
        wall_score = min(wall_distance[grid_y, grid_x], 2.0) / 2.0
        
        # Get robot position for distance score
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_grid_x = int((transform.transform.translation.x - origin_x) / resolution)
            robot_grid_y = int((transform.transform.translation.y - origin_y) / resolution)
            
            dist_to_robot = math.sqrt(
                (grid_x - robot_grid_x)**2 + 
                (grid_y - robot_grid_y)**2
            ) * resolution
            
            distance_score = 1.0 - abs(dist_to_robot - self.preferred_distance) / self.preferred_distance
        except:
            distance_score = 0.0
        
        return (3.0 * unknown_score + 
                2.0 * wall_score + 
                1.0 * distance_score)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()