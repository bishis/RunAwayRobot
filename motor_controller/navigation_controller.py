#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
from enum import Enum

# Import from package root
from motor_controller.processors.obstacle_monitor import ObstacleMonitor
from motor_controller.exploration_controller import ExplorationController

class RobotState(Enum):
    EXPLORING = 1
    AVOIDING = 2
    NAVIGATING = 3

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('safety_margin', 0.2)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.366)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Initialize state
        self.current_state = RobotState.EXPLORING
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Initialize components as nodes
        self.obstacle_monitor = ObstacleMonitor()
        rclpy.spin_once(self.obstacle_monitor)  # Initialize node
        
        self.exploration_controller = ExplorationController()
        rclpy.spin_once(self.exploration_controller)  # Initialize node
        
        # Create control loop timer
        self.create_timer(0.1, self.control_loop)  # 10Hz control
        
        self.get_logger().info('Navigation controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        # Forward scan to obstacle monitor
        if self.obstacle_monitor:
            self.obstacle_monitor.current_scan = msg

    def control_loop(self):
        """Main control loop managing states and behaviors"""
        try:
            # Check for obstacles
            obstacle = self.obstacle_monitor.detect_obstacles() if self.obstacle_monitor else None
            
            if obstacle and obstacle['critical']:
                # Switch to avoidance if obstacle detected
                if self.current_state != RobotState.AVOIDING:
                    self.get_logger().info('Switching to AVOIDING state')
                    self.current_state = RobotState.AVOIDING
            
            # State machine
            if self.current_state == RobotState.AVOIDING:
                # Use obstacle monitor's avoidance behavior
                if obstacle:
                    cmd = self.obstacle_monitor.execute_avoidance(obstacle)
                    if cmd:
                        self.wheel_speeds_pub.publish(cmd)
                else:
                    # Return to exploring when clear
                    self.current_state = RobotState.EXPLORING
                    self.get_logger().info('Switching back to EXPLORING state')
            
            elif self.current_state == RobotState.EXPLORING:
                # Use exploration controller's behavior
                if not obstacle:
                    # Let exploration controller generate next waypoint
                    self.exploration_controller.exploration_loop()
                    # Get current navigation command from exploration
                    cmd = self.get_exploration_command()
                    if cmd:
                        self.wheel_speeds_pub.publish(cmd)
            
            elif self.current_state == RobotState.NAVIGATING:
                # Handle navigation to specific goal
                pass
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.stop_robot()

    def get_exploration_command(self) -> Twist:
        """Get current velocity command from exploration controller"""
        cmd = Twist()
        if self.exploration_controller.current_waypoint:
            # Calculate velocities based on current waypoint
            try:
                # Get robot pose
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                
                # Calculate direction to waypoint
                target_x = self.exploration_controller.current_waypoint.pose.position.x
                target_y = self.exploration_controller.current_waypoint.pose.position.y
                
                dx = target_x - robot_x
                dy = target_y - robot_y
                
                # Calculate angle to target
                target_angle = math.atan2(dy, dx)
                current_angle = 2 * math.atan2(transform.transform.rotation.z,
                                             transform.transform.rotation.w)
                
                # Calculate angle error
                angle_error = target_angle - current_angle
                if angle_error > math.pi:
                    angle_error -= 2 * math.pi
                elif angle_error < -math.pi:
                    angle_error += 2 * math.pi
                
                # Set angular velocity proportional to angle error
                cmd.angular.z = max(min(angle_error, self.max_angular_speed),
                                  -self.max_angular_speed)
                
                # Set forward velocity if roughly pointing at target
                if abs(angle_error) < 0.5:  # ~30 degrees
                    distance = math.sqrt(dx*dx + dy*dy)
                    cmd.linear.x = min(distance * 0.5, self.max_linear_speed)
                
            except Exception as e:
                self.get_logger().error(f'Error calculating exploration command: {str(e)}')
        
        return cmd

    def stop_robot(self):
        """Emergency stop"""
        stop_cmd = Twist()
        self.wheel_speeds_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
