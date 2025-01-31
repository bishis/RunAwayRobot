#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
import time

class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('safety_margin', 0.2)
        self.declare_parameter('scan_threshold', 0.5)  # Distance to consider obstacle
        self.declare_parameter('critical_threshold', 0.3)  # Distance for emergency stop
        self.declare_parameter('avoidance_speed', 0.05)  # Slow speed for avoidance
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.scan_threshold = self.get_parameter('scan_threshold').value
        self.critical_threshold = self.get_parameter('critical_threshold').value
        self.avoidance_speed = self.get_parameter('avoidance_speed').value
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'local_costmap/costmap', 10)
        
        # Nav2 path replanning client
        self.replan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.avoiding = False
        self.last_goal = None
        self.current_scan = None
        self.avoidance_start_time = None
        
        # Create monitoring timer
        self.create_timer(0.1, self.monitor_obstacles)  # 10Hz monitoring
        
        self.get_logger().info('Obstacle monitor initialized')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        self.current_scan = msg

    def detect_obstacles(self):
        """Analyze scan data for obstacles"""
        if not self.current_scan:
            return None
            
        # Convert scan to numpy array
        ranges = np.array(self.current_scan.ranges)
        angles = np.arange(
            self.current_scan.angle_min,
            self.current_scan.angle_max + self.current_scan.angle_increment,
            self.current_scan.angle_increment
        )
        
        # Find points within threshold
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        close_mask = ranges < self.scan_threshold
        obstacle_mask = valid_mask & close_mask
        
        if not np.any(obstacle_mask):
            return None
            
        # Get closest obstacle
        min_dist = np.min(ranges[obstacle_mask])
        min_idx = np.where(ranges == min_dist)[0][0]
        obstacle_angle = angles[min_idx]
        
        return {
            'distance': min_dist,
            'angle': obstacle_angle,
            'critical': min_dist < self.critical_threshold
        }

    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            return transform.transform
        except:
            return None

    def request_new_path(self):
        """Request a new path from Nav2"""
        if not self.last_goal or not self.replan_client.wait_for_server(timeout_sec=1.0):
            return False
            
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.last_goal
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        
        # Get current pose
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return False
            
        goal_msg.start.pose.position = robot_pose.translation
        goal_msg.start.pose.orientation = robot_pose.rotation
        
        self.replan_client.send_goal_async(goal_msg)
        return True

    def execute_avoidance(self, obstacle):
        """Execute avoidance maneuver"""
        cmd = Twist()
        
        if not self.avoidance_start_time:
            self.avoidance_start_time = time.time()
            
        # If we've been avoiding for more than 5 seconds, request new path
        if time.time() - self.avoidance_start_time > 5.0:
            self.request_new_path()
            self.avoiding = False
            self.avoidance_start_time = None
            return
            
        if obstacle['critical']:
            # Emergency stop if too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Basic avoidance - rotate away from obstacle
            cmd.linear.x = -self.avoidance_speed  # Back up slowly
            # Rotate away from obstacle
            cmd.angular.z = -1.0 if obstacle['angle'] > 0 else 1.0
            
        self.cmd_vel_pub.publish(cmd)

    def monitor_obstacles(self):
        """Main monitoring loop"""
        try:
            obstacle = self.detect_obstacles()
            
            if obstacle:
                self.avoiding = True
                self.execute_avoidance(obstacle)
                # Update costmap with obstacle location
                self.update_costmap(obstacle)
            else:
                if self.avoiding:
                    # We're clear of obstacles
                    self.avoiding = False
                    self.avoidance_start_time = None
                    
        except Exception as e:
            self.get_logger().error(f'Error in obstacle monitor: {str(e)}')

    def update_costmap(self, obstacle):
        """Update local costmap with detected obstacle"""
        # Implementation depends on your costmap setup
        # This would add the obstacle to the local costmap
        pass