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
from .processors.obstacle_monitor import ObstacleMonitor
from .exploration_controller import ExplorationController

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
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize components (reusing existing code)
        self.obstacle_monitor = ObstacleMonitor()  # From obstacle_monitor.py
        self.exploration_controller = ExplorationController()  # From exploration_controller.py
        
        # Create control loop timer
        self.create_timer(0.1, self.control_loop)  # 10Hz control
        
        self.get_logger().info('Navigation controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        # Forward scan to obstacle monitor
        self.obstacle_monitor.scan_callback(msg)

    def control_loop(self):
        """Main control loop managing states and behaviors"""
        try:
            # Check for obstacles
            obstacle = self.obstacle_monitor.detect_obstacles()
            
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
                    self.wheel_speeds_pub.publish(cmd)
            
            elif self.current_state == RobotState.NAVIGATING:
                # Handle navigation to specific goal
                pass
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.stop_robot()

    def get_exploration_command(self) -> Twist:
        """Get current velocity command from exploration controller"""
        # This would interface with exploration controller's current goal
        cmd = Twist()
        # ... calculate velocities based on current exploration goal
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
