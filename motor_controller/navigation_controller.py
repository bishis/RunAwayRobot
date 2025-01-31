#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.action import ActionClient
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
from enum import Enum
from motor_controller.processors.obstacle_monitor import ObstacleMonitor
from motor_controller.exploration_controller import ExplorationController

class RobotState(Enum):
    EXPLORING = 1
    AVOIDING = 2
    RECOVERING = 3

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Initialize state
        self.current_state = RobotState.EXPLORING
        
        # Create components as part of this node (not as separate nodes)
        self.obstacle_monitor = ObstacleMonitor()
        self.exploration_controller = ExplorationController()
        
        # Share the node's logger with components
        self.obstacle_monitor.get_logger = self.get_logger
        self.exploration_controller.get_logger = self.get_logger
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Control loop timer
        self.create_timer(0.1, self.state_machine_loop)  # 10Hz control loop
        
        self.get_logger().info('Navigation controller initialized in EXPLORING state')

    def scan_callback(self, msg: LaserScan):
        """Forward scan data to obstacle monitor"""
        self.obstacle_monitor.scan_callback(msg)

    def state_machine_loop(self):
        """Main state machine loop"""
        try:
            # Check for obstacles using obstacle monitor's detection
            obstacle = self.obstacle_monitor.detect_obstacles()
            
            if self.current_state == RobotState.EXPLORING:
                if obstacle:
                    # Obstacle detected, switch to avoidance
                    self.get_logger().info('Obstacle detected - Switching to AVOIDING state')
                    self.current_state = RobotState.AVOIDING
                    # Store current exploration goal
                    self.obstacle_monitor.last_goal = self.exploration_controller.current_waypoint
                else:
                    # Continue exploration
                    self.exploration_controller.control_loop()
                    
            elif self.current_state == RobotState.AVOIDING:
                if obstacle:
                    # Execute avoidance using obstacle monitor's logic
                    self.obstacle_monitor.execute_avoidance(obstacle)
                else:
                    # Clear of obstacles, switch to recovery
                    self.get_logger().info('Clear of obstacles - Switching to RECOVERY state')
                    self.current_state = RobotState.RECOVERING
                    # Request new path to original goal
                    self.obstacle_monitor.request_new_path()
                    
            elif self.current_state == RobotState.RECOVERING:
                if obstacle:
                    # Found new obstacle during recovery
                    self.get_logger().info('New obstacle during recovery - Switching to AVOIDING state')
                    self.current_state = RobotState.AVOIDING
                else:
                    # Check if we've reached a good position to resume exploration
                    robot_pose = self.obstacle_monitor.get_robot_pose()
                    if robot_pose:
                        # Resume exploration from current position
                        self.get_logger().info('Recovery complete - Resuming EXPLORATION')
                        self.current_state = RobotState.EXPLORING
                        self.exploration_controller.generate_new_goal()
            
        except Exception as e:
            self.get_logger().error(f'Error in state machine: {str(e)}')
            self.stop_robot()

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
