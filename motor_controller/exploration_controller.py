#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from tf2_ros import Buffer, TransformListener

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Parameters
        self.declare_parameter('min_frontier_size', 5)    # Minimum cells for a valid frontier
        self.declare_parameter('exploration_radius', 1.0) # How far to move for each goal
        self.declare_parameter('wall_threshold', 80)      # Value to consider as wall (0-100)
        self.declare_parameter('unknown_threshold', -1)   # Value for unknown cells
        
        # Get parameters
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.unknown_threshold = self.get_parameter('unknown_threshold').value
        
        # Set up Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)
        
        # Exploration state
        self.current_map = None
        self.is_navigating = False
        self.current_goal = None
        self.marker_id = 0
        
        self.get_logger().info('Exploration controller initialized')

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map data"""
        self.current_map = msg

    def find_frontiers(self):
        """Find frontier regions (unexplored areas next to known areas)"""
        if not self.current_map:
            return []

        # Convert map to numpy array
        width = self.current_map.info.width
        height = self.current_map.info.height
        map_data = np.array(self.current_map.data).reshape(height, width)
        
        # Create binary maps
        unknown = map_data == self.unknown_threshold
        free = map_data < self.wall_threshold
        free[unknown] = False
        
        # Find frontiers (free cells next to unknown cells)
        frontiers = []
        min_gap_size = 3  # Minimum width of gap to consider
        
        # Look for larger gaps
        for y in range(1, height-1):
            for x in range(1, width-1):
                if not free[y, x]:
                    continue
                
                # Check for a significant gap of unknown space
                # Look ahead in both x and y directions
                unknown_count_x = 0
                unknown_count_y = 0
                
                # Check X direction
                for dx in range(min_gap_size):
                    if x + dx < width and unknown[y, x + dx]:
                        unknown_count_x += 1
                        
                # Check Y direction
                for dy in range(min_gap_size):
                    if y + dy < height and unknown[y + dy, x]:
                        unknown_count_y += 1
                
                # If we found a significant gap in either direction
                if unknown_count_x >= min_gap_size or unknown_count_y >= min_gap_size:
                    frontiers.append((x, y))
        
        # Group nearby frontier cells with larger minimum distance
        grouped_frontiers = []
        visited = set()
        min_group_separation = 10  # Minimum cells between frontier groups
        
        for x, y in frontiers:
            if (x, y) in visited:
                continue
                
            # Flood fill to find connected frontier cells
            group = []
            queue = [(x, y)]
            while queue:
                cx, cy = queue.pop(0)
                if (cx, cy) in visited:
                    continue
                    
                visited.add((cx, cy))
                group.append((cx, cy))
                
                # Check neighbors with larger radius
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontiers and (nx, ny) not in visited:
                            queue.append((nx, ny))
            
            # Only keep larger groups
            if len(group) >= self.min_frontier_size * 2:  # Increased size requirement
                # Calculate center of mass
                center_x = sum(x for x, _ in group) / len(group)
                center_y = sum(y for _, y in group) / len(group)
                
                # Check if this group is far enough from existing groups
                is_separate = True
                for other_group in grouped_frontiers:
                    other_center_x = sum(x for x, _ in other_group) / len(other_group)
                    other_center_y = sum(y for _, y in other_group) / len(other_group)
                    dist = math.sqrt((center_x - other_center_x)**2 + (center_y - other_center_y)**2)
                    if dist < min_group_separation:
                        is_separate = False
                        break
                
                if is_separate:
                    grouped_frontiers.append(group)
        
        return grouped_frontiers

    def publish_visualization_markers(self, frontiers, selected_goal=None):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'exploration'
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Show frontiers
        for frontier in frontiers:
            frontier_marker = Marker()
            frontier_marker.header.frame_id = 'map'
            frontier_marker.header.stamp = self.get_clock().now().to_msg()
            frontier_marker.ns = 'exploration'
            frontier_marker.id = self.marker_id
            self.marker_id += 1
            frontier_marker.type = Marker.POINTS
            frontier_marker.action = Marker.ADD
            frontier_marker.scale.x = 0.05
            frontier_marker.scale.y = 0.05
            frontier_marker.color.r = 0.0
            frontier_marker.color.g = 1.0
            frontier_marker.color.b = 0.0
            frontier_marker.color.a = 1.0
            
            # Convert grid cells to world coordinates
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            for x, y in frontier:
                point = Point()
                point.x = origin_x + x * resolution
                point.y = origin_y + y * resolution
                point.z = 0.1
                frontier_marker.points.append(point)
            
            marker_array.markers.append(frontier_marker)
        
        # Show selected goal
        if selected_goal:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'exploration'
            goal_marker.id = self.marker_id
            self.marker_id += 1
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.1
            goal_marker.scale.z = 0.1
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            goal_marker.pose = selected_goal.pose
            
            marker_array.markers.append(goal_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

    def find_exploration_goal(self):
        """Find the next exploration goal based on frontiers"""
        if not self.current_map:
            return None
            
        # Get current robot pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_pos = np.array([robot_x, robot_y])
        except Exception as e:
            self.get_logger().warning(f'Could not get robot pose: {e}')
            return None
        
        # Find frontiers
        frontiers = self.find_frontiers()
        if not frontiers:
            return None
        
        # Convert robot position to grid coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        robot_grid_x = int((robot_x - origin_x) / resolution)
        robot_grid_y = int((robot_y - origin_y) / resolution)
        
        # Find closest frontier
        closest_frontier = None
        min_distance = float('inf')
        
        for frontier in frontiers:
            # Use center of frontier
            center_x = sum(x for x, _ in frontier) / len(frontier)
            center_y = sum(y for _, y in frontier) / len(frontier)
            
            dist = math.sqrt((center_x - robot_grid_x)**2 + (center_y - robot_grid_y)**2)
            if dist < min_distance:
                min_distance = dist
                closest_frontier = frontier
        
        if not closest_frontier:
            return None
        
        # Create goal at frontier center
        center_x = sum(x for x, _ in closest_frontier) / len(closest_frontier)
        center_y = sum(y for _, y in closest_frontier) / len(closest_frontier)
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Convert back to world coordinates
        goal.pose.position.x = origin_x + center_x * resolution
        goal.pose.position.y = origin_y + center_y * resolution
        goal.pose.position.z = 0.0
        
        # Set orientation towards frontier
        angle = math.atan2(
            center_y - robot_grid_y,
            center_x - robot_grid_x
        )
        goal.pose.orientation.w = math.cos(angle / 2)
        goal.pose.orientation.z = math.sin(angle / 2)
        
        # Publish visualization
        self.publish_visualization_markers(frontiers, goal)
        
        return goal

    def send_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
        self.get_logger().info('Sending new navigation goal')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.nav_client.wait_for_server()
        
        self.current_goal = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.current_goal.add_done_callback(self.goal_response_callback)
        self.is_navigating = True

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        result = future.result().result
        self.is_navigating = False
        self.get_logger().info('Navigation completed')

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Can add feedback processing here if needed

    def exploration_loop(self):
        """Main exploration control loop"""
        if self.is_navigating:
            return
            
        # Find and send new exploration goal
        goal_pose = self.find_exploration_goal()
        if goal_pose:
            self.send_goal(goal_pose)
        else:
            self.get_logger().warn('No valid exploration goal found')

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