#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
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
        self.declare_parameter('min_scan_distance', 0.3)  # Minimum distance to obstacle (meters)
        self.declare_parameter('max_scan_distance', 2.0)  # Maximum distance to consider for exploration
        self.declare_parameter('exploration_radius', 1.0)  # How far to move for each exploration goal
        
        # Get parameters
        self.min_distance = self.get_parameter('min_scan_distance').value
        self.max_distance = self.get_parameter('max_scan_distance').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        
        # Set up Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)  # 1Hz control loop
        
        # Exploration state
        self.current_scan = None
        self.current_map = None
        self.is_navigating = False
        self.current_goal = None
        self.marker_id = 0
        
        self.get_logger().info('Exploration controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        self.current_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map data"""
        self.current_map = msg

    def publish_visualization_markers(self, robot_pos, open_regions, selected_goal):
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
        
        # Show open regions as line strips
        if open_regions:
            for start_idx, end_idx in open_regions:
                region_marker = Marker()
                region_marker.header.frame_id = 'map'
                region_marker.header.stamp = self.get_clock().now().to_msg()
                region_marker.ns = 'exploration'
                region_marker.id = self.marker_id
                self.marker_id += 1
                region_marker.type = Marker.LINE_STRIP
                region_marker.action = Marker.ADD
                region_marker.scale.x = 0.05  # Line width
                region_marker.color.r = 0.0
                region_marker.color.g = 1.0
                region_marker.color.b = 0.0
                region_marker.color.a = 0.5
                
                # Add points for the arc
                angles = np.linspace(
                    self.current_scan.angle_min + start_idx * self.current_scan.angle_increment,
                    self.current_scan.angle_min + end_idx * self.current_scan.angle_increment,
                    20
                )
                for angle in angles:
                    point = Point()
                    point.x = robot_pos[0] + self.max_distance * math.cos(angle)
                    point.y = robot_pos[1] + self.max_distance * math.sin(angle)
                    point.z = 0.1
                    region_marker.points.append(point)
                
                marker_array.markers.append(region_marker)
        
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
            goal_marker.scale.x = 0.3  # Arrow length
            goal_marker.scale.y = 0.1  # Arrow width
            goal_marker.scale.z = 0.1  # Arrow height
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            goal_marker.pose = selected_goal.pose
            
            marker_array.markers.append(goal_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

    def find_exploration_goal(self):
        """Find the next exploration goal based on laser scan"""
        if not self.current_scan:
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
            
        # Find open direction from laser scan
        ranges = np.array(self.current_scan.ranges)
        angles = np.arange(
            self.current_scan.angle_min,
            self.current_scan.angle_max + self.current_scan.angle_increment,
            self.current_scan.angle_increment
        )
        
        # Clean up invalid readings
        ranges[np.isnan(ranges)] = 0
        ranges[ranges < self.current_scan.range_min] = 0
        ranges[ranges > self.current_scan.range_max] = self.current_scan.range_max
        
        # Find regions that are open
        valid_regions = (ranges > self.min_distance) & (ranges < self.max_distance)
        
        if not np.any(valid_regions):
            return None
            
        # Find the largest open region
        regions = []
        start_idx = None
        
        for i in range(len(valid_regions)):
            if valid_regions[i] and start_idx is None:
                start_idx = i
            elif not valid_regions[i] and start_idx is not None:
                regions.append((start_idx, i))
                start_idx = None
                
        if start_idx is not None:
            regions.append((start_idx, len(valid_regions)))
            
        if not regions:
            return None
            
        # Select the widest region
        widest_region = max(regions, key=lambda r: r[1] - r[0])
        center_idx = (widest_region[0] + widest_region[1]) // 2
        best_angle = angles[center_idx]
        
        # Create goal pose in map frame
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Set goal position in open direction
        goal.pose.position.x = robot_x + self.exploration_radius * math.cos(best_angle)
        goal.pose.position.y = robot_y + self.exploration_radius * math.sin(best_angle)
        goal.pose.position.z = 0.0
        
        # Set goal orientation (facing the direction of travel)
        goal.pose.orientation.w = math.cos(best_angle / 2)
        goal.pose.orientation.z = math.sin(best_angle / 2)
        
        # Publish visualization
        self.publish_visualization_markers(robot_pos, regions, goal)
        
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