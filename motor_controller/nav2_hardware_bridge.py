#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class Nav2HardwareBridge(Node):
    """Bridge between Nav2 commands and hardware controller."""
    
    def __init__(self):
        super().__init__('nav2_hardware_bridge')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # Add leading slash
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Log subscribed topics
        self.get_logger().info(
            f'\nTopic Configuration:'
            f'\n  Subscribing to cmd_vel topic: {cmd_vel_topic}'
            f'\n  Publishing to wheel_cmd_vel topic: /cmd_vel'  # Change to /cmd_vel
        )
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,  # From parameter
            self.cmd_vel_callback,
            10
        )
        
        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Change to /cmd_vel to match hardware_controller
            10
        )
        
        # Add debug timer to check cmd_vel and print topic info
        self.create_timer(1.0, self.debug_callback)
        
        # Comment out test movement timer to check Nav2
        # self.create_timer(2.0, self.test_movement_callback)
        
        # Status variables
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        self.test_state = 0  # For cycling through test movements
        
        # Create watchdog timer
        self.create_timer(0.1, self.watchdog_callback)
        
        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Add timer to send test goal after a longer delay to ensure Nav2 is ready
        self.create_timer(15.0, self.send_test_goal)  # Wait 15 seconds before sending goal
        
        # Track if we've sent a goal
        self.goal_sent = False
        
        # Define test waypoints [(x, y), ...]
        self.waypoints = [
            (0.5, 0.0),   # 1m ahead
            (0.2, 0.5),   # diagonal
            (0.0, 0.2),   # left
            (0.0, 0.0)    # back to start
        ]
        self.current_waypoint = 0
        
        self.get_logger().info('Nav2 Hardware Bridge initialized')
        
    def cmd_vel_callback(self, msg: Twist):
        """Convert Nav2 velocity commands to wheel velocities."""
        try:
            # Get linear and angular velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Log the received command
            self.get_logger().info(
                f'\nReceived cmd_vel:'
                f'\n  Linear X: {linear_x:.3f} m/s'
                f'\n  Angular Z: {angular_z:.3f} rad/s'
            )
            
            # Clamp velocities to max values
            original_linear = linear_x
            original_angular = angular_z
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            # Log if velocities were clamped
            if original_linear != linear_x or original_angular != angular_z:
                self.get_logger().info(
                    f'Clamped velocities:'
                    f'\n  Linear X: {linear_x:.3f} m/s (from {original_linear:.3f})'
                    f'\n  Angular Z: {angular_z:.3f} rad/s (from {original_angular:.3f})'
                )
            
            # For tank drive:
            # Pure rotation: One wheel forward, one backward
            # Pure translation: Both wheels same direction
            # Mixed: Combine both effects
            
            # Convert to binary commands (-1, 0, 1)
            linear_threshold = 0.05  # Minimum speed to start moving
            angular_threshold = 0.05  # Minimum speed to start turning
            
            # Determine movement type and set binary speeds
            if abs(linear_x) < linear_threshold and abs(angular_z) < angular_threshold:
                # Stop
                left_speed = 0
                right_speed = 0
                self.get_logger().info('Stopping')
            elif abs(angular_z) > angular_threshold:
                # Turning takes priority over forward/backward
                if angular_z > 0:
                    # Turn left
                    left_speed = -1
                    right_speed = 1
                else:
                    # Turn right
                    left_speed = 1
                    right_speed = -1
                self.get_logger().info('Pure rotation')
            else:
                # Forward/Backward
                if linear_x > 0:
                    left_speed = 1
                    right_speed = 1
                else:
                    left_speed = -1
                    right_speed = -1
                self.get_logger().info('Pure straight motion')
            
            # Log detailed movement info
            self.get_logger().info(
                f'Movement calculation:'
                f'\n  Movement type: {"rotation" if abs(angular_z) > abs(linear_x) else "translation"}'
                f'\n  Binary wheel speeds [-1, 0, 1]:'
                f'\n    Left: {left_speed}'
                f'\n    Right: {right_speed}'
                f'\n  Expected behavior:'
                f'\n    Forward/Back: {abs(linear_x) > abs(angular_z)}'
                f'\n    Turning: {abs(angular_z) > abs(linear_x)}'
            )
            
            # Create wheel command message
            wheel_cmd = Twist()
            wheel_cmd.linear.x = float(left_speed)   # Left wheel (-1, 0, 1)
            wheel_cmd.linear.y = float(right_speed)  # Right wheel (-1, 0, 1)
            
            # Publish wheel commands
            self.wheel_cmd_pub.publish(wheel_cmd)
            
            # Update command time
            self.last_cmd_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {str(e)}')
            
    def watchdog_callback(self):
        """Stop robot if no commands received recently."""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_timeout:
            # Send stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0  # Left wheel
            stop_cmd.linear.y = 0.0  # Right wheel
            self.wheel_cmd_pub.publish(stop_cmd)

    def debug_callback(self):
        """Debug timer callback to check if we're receiving cmd_vel."""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # Get publisher and subscriber counts
        n_publishers = self.cmd_vel_sub.get_publisher_count()
        n_subscribers = self.wheel_cmd_pub.get_subscription_count()
        
        # Get node name info
        node_name = self.get_name()
        
        # Check Nav2 topics
        try:
            # Create a one-time subscriber to check Nav2 topics
            goal_sub = self.create_subscription(
                PoseStamped,
                'goal_pose',
                lambda msg: None,
                1
            )
            plan_sub = self.create_subscription(
                Path,
                'plan',
                lambda msg: None,
                1
            )
            
            # Log the basic debug info
            self.get_logger().info(
                f'\nDebug Info for {node_name}:'
                f'\n  Time since last cmd_vel: {time_since_cmd:.2f} seconds'
                f'\n  Number of publishers to cmd_vel: {n_publishers}'
                f'\n  Number of subscribers to wheel_cmd_vel: {n_subscribers}'
                f'\n  Nav2 Status:'
                f'\n    Goal pose publishers: {goal_sub.get_publisher_count()}'
                f'\n    Plan publishers: {plan_sub.get_publisher_count()}'
                f'\n    Controller state: {"Active" if n_publishers > 0 else "Inactive"}'
            )
            
            # Cleanup temporary subscribers
            self.destroy_subscription(goal_sub)
            self.destroy_subscription(plan_sub)
            
        except Exception as e:
            self.get_logger().error(f'Error checking Nav2 status: {str(e)}')
        
        # Check if we have any publishers
        if n_publishers == 0:
            self.get_logger().warn('No publishers on cmd_vel topic - Nav2 may not be running properly')

    def test_movement_callback(self):
        """Send test movement commands to verify hardware chain."""
        # Create a test Twist message
        test_cmd = Twist()
        
        # Cycle through different movements
        if self.test_state == 0:
            # Forward
            test_cmd.linear.x = 0.2
            test_cmd.angular.z = 0.0
            self.get_logger().info('Test: Moving Forward')
        elif self.test_state == 1:
            # Turn left
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = 0.5
            self.get_logger().info('Test: Turning Left')
        elif self.test_state == 2:
            # Turn right
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = -0.5
            self.get_logger().info('Test: Turning Right')
        else:
            # Stop
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = 0.0
            self.get_logger().info('Test: Stopping')
        
        # Process the test command as if it came from Nav2
        self.cmd_vel_callback(test_cmd)
        
        # Cycle to next test state
        self.test_state = (self.test_state + 1) % 4

    def send_test_goal(self):
        """Send next waypoint as goal."""
        if self.goal_sent:
            return
            
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available')
            return
            
        # Get next waypoint
        x, y = self.waypoints[self.current_waypoint]
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (facing forward)
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        
        self.get_logger().info(f'Sending navigation goal: waypoint {self.current_waypoint} ({x}, {y})')
        
        # Send the goal
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)
        
        self.goal_sent = True
        
        # Move to next waypoint
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
    
    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        # Get the result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle the goal result."""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            
        # Reset goal sent flag to allow sending another goal
        self.goal_sent = False
    
    def goal_feedback_callback(self, feedback_msg):
        """Handle the goal feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} meters'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Nav2HardwareBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()