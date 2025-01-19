#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class NavigationInterface(Node):
    """Interface for sending navigation commands to the robot."""
    
    def __init__(self):
        super().__init__('navigation_interface')
        
        # Publisher for direct velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Action client for Nav2 navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Navigation interface initialized')
        
    def send_twist(self, linear_x: float, angular_z: float):
        """Send a twist command to the robot."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Sent twist command - linear: {linear_x}, angular: {angular_z}')
        
    def stop(self):
        """Stop the robot."""
        self.send_twist(0.0, 0.0)
        
    def rotate(self, angular_speed: float):
        """Rotate the robot at the specified speed."""
        self.send_twist(0.0, angular_speed)
        
    def move_forward(self, speed: float):
        """Move the robot forward at the specified speed."""
        self.send_twist(speed, 0.0)
        
    async def navigate_to(self, x: float, y: float, theta: float = 0.0):
        """Navigate to a pose using Nav2."""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
            
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        
        # Set orientation (convert theta to quaternion)
        goal.pose.pose.orientation.z = math.sin(theta/2.0)
        goal.pose.pose.orientation.w = math.cos(theta/2.0)
        
        # Send goal
        self.get_logger().info(f'Navigating to: ({x}, {y}, {theta})')
        send_goal_future = await self.nav_client.send_goal_async(goal)
        
        if not send_goal_future.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        # Wait for result
        goal_handle = send_goal_future.result()
        result_future = await goal_handle.get_result_async()
        
        status = result_future.result().status
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation succeeded')
            return True
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = NavigationInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 