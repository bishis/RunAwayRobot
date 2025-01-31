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
        
        # Create components as separate nodes
        self.obstacle_monitor = ObstacleMonitor()
        self.exploration_controller = ExplorationController()
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Subscribe to cmd_vel from both components
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Control loop timer
        self.create_timer(0.1, self.state_machine_loop)  # 10Hz control loop
        
        self.get_logger().info('Navigation controller initialized in EXPLORING state')

    def cmd_vel_callback(self, msg: Twist):
        """Forward velocity commands to wheel speeds"""
        # Forward the velocity commands to the motors
        self.wheel_speeds_pub.publish(msg)
        
    def scan_callback(self, msg: LaserScan):
        """Forward scan data to obstacle monitor"""
        self.obstacle_monitor.scan_callback(msg)

    def state_machine_loop(self):
        """Main state machine loop"""
        try:
            # Check for obstacles using obstacle monitor's detection
            obstacle = self.obstacle_monitor.detect_obstacles()
            
            # Log current state and obstacle info
            if obstacle:
                self.get_logger().info(
                    f'State: {self.current_state.name} | ' +
                    f'Obstacle detected at {obstacle["distance"]:.2f}m, ' +
                    f'angle: {math.degrees(obstacle["angle"]):.1f}°, ' +
                    f'critical: {obstacle["critical"]}'
                )
            else:
                self.get_logger().debug(f'State: {self.current_state.name} | No obstacles detected')
            
            if self.current_state == RobotState.EXPLORING:
                if obstacle:
                    self.get_logger().warn(
                        'Switching to AVOIDING state - ' +
                        f'Obstacle at {obstacle["distance"]:.2f}m'
                    )
                    self.current_state = RobotState.AVOIDING
                    self.obstacle_monitor.last_goal = self.exploration_controller.current_waypoint
                else:
                    self.exploration_controller.exploration_loop()
                    
            elif self.current_state == RobotState.AVOIDING:
                if obstacle:
                    self.get_logger().info(
                        f'Executing avoidance maneuver - ' +
                        f'Distance: {obstacle["distance"]:.2f}m, ' +
                        f'Angle: {math.degrees(obstacle["angle"]):.1f}°'
                    )
                    self.obstacle_monitor.execute_avoidance(obstacle)
                else:
                    self.get_logger().info('Clear of obstacles - Switching to RECOVERY state')
                    self.current_state = RobotState.RECOVERING
                    success = self.obstacle_monitor.request_new_path()
                    self.get_logger().info(f'Path replanning {"successful" if success else "failed"}')
                    
            elif self.current_state == RobotState.RECOVERING:
                if obstacle:
                    self.get_logger().warn('New obstacle during recovery - Switching to AVOIDING state')
                    self.current_state = RobotState.AVOIDING
                else:
                    robot_pose = self.obstacle_monitor.get_robot_pose()
                    if robot_pose:
                        self.get_logger().info(
                            'Recovery complete - Resuming EXPLORATION at ' +
                            f'x: {robot_pose.translation.x:.2f}, ' +
                            f'y: {robot_pose.translation.y:.2f}'
                        )
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
