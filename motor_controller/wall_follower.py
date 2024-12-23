#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from gpiozero import Servo
from time import sleep
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
import math
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.linalg import block_diag

# Define GPIO pins for motor control
motor_left = Servo(18)   # GPIO for Left Motor
motor_right = Servo(12)  # GPIO for Right Motor

# Define parameters
WALL_DETECTION_DISTANCE = 0.5  # Distance to start turn (50cm)
SAFETY_RADIUS = 0.3           # Emergency stop distance (30cm)
TURN_SPEED = 1.0             # Speed for turning
MAX_LINEAR_SPEED = 0.3       # Forward speed
TRACK_WIDTH = 0.17           # Distance between tracks
TURN_ANGLE = 90             # Degrees to turn
MIN_TURN_ANGLE = 85         # Minimum acceptable turn angle
MAX_TURN_ANGLE = 95         # Maximum acceptable turn angle

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector [x, y, theta]
        self.state = np.zeros(3)
        
        # State covariance matrix
        self.P = np.diag([0.1, 0.1, 0.1])
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1])
        
        # Measurement noise
        self.R = np.diag([0.1, 0.1, 0.1])

    def predict(self, v, w, dt):
        # State transition matrix
        theta = self.state[2]
        
        # Update state
        self.state[0] += v * np.cos(theta) * dt
        self.state[1] += v * np.sin(theta) * dt
        self.state[2] += w * dt
        
        # Jacobian of state transition
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1, v * np.cos(theta) * dt],
            [0, 0, 1]
        ])
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        # Measurement matrix (identity for direct state measurement)
        H = np.eye(3)
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        innovation = measurement - self.state
        self.state += K @ innovation
        
        # Update covariance
        self.P = (np.eye(3) - K @ H) @ self.P

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # Add odometry publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Add odometry tracking variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        
        # Square pattern variables
        self.pattern_state = 'forward'  # States: 'forward', 'turn'
        self.state_start_time = self.get_clock().now()
        
        # Create timer for odometry publishing
        self.create_timer(0.01, self.publish_odom)  # 100Hz for stable transforms
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Add wheel velocity tracking
        self.left_wheel_vel = 0.0   # radians per second
        self.right_wheel_vel = 0.0  # radians per second

        # Add variables for wall following
        self.turning = False
        self.start_turn_angle = 0.0
        self.target_angle = 0.0
        self.wall_detected = False

        # Initialize EKF
        self.ekf = ExtendedKalmanFilter()
        
        # Update odometry tracking variables to use EKF state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def lidar_callback(self, msg):
        # Store latest scan for EKF update
        self.latest_scan = msg
        
        # Get front-facing LIDAR data
        num_readings = len(msg.ranges)
        front_start = num_readings // 3
        front_end = 2 * num_readings // 3
        front_distances = msg.ranges[front_start:front_end]
        min_front_distance = min(front_distances)

        self.get_logger().info(f"Front distance: {min_front_distance:.2f}m")

        # Emergency stop if too close
        if min_front_distance < SAFETY_RADIUS:
            self.get_logger().warn(f"Emergency stop! Wall at {min_front_distance:.2f}m")
            self.stop_motors()
            return

        # Handle wall detection and turning
        if not self.turning:
            if min_front_distance < WALL_DETECTION_DISTANCE:
                self.get_logger().info(f"Wall detected at {min_front_distance:.2f}m, initiating turn")
                self.start_turn()
            else:
                self.move_forward(0.0)
        else:
            self.execute_turn()

    def start_turn(self):
        self.turning = True
        self.start_turn_angle = self.th
        self.target_angle = self.th + math.radians(TURN_ANGLE)
        self.get_logger().info(f"Starting turn from {math.degrees(self.th):.1f}° to {math.degrees(self.target_angle):.1f}°")

    def execute_turn(self):
        # Calculate how far we've turned
        current_angle = self.th
        angle_turned = math.degrees(current_angle - self.start_turn_angle)
        
        # Normalize angle_turned to be between 0 and 360
        angle_turned = angle_turned % 360
        if angle_turned < 0:
            angle_turned += 360

        self.get_logger().info(f"Turn progress: {angle_turned:.1f}°")

        # Check if we've turned enough
        if MIN_TURN_ANGLE <= angle_turned <= MAX_TURN_ANGLE:
            self.get_logger().info("Turn complete")
            self.turning = False
            self.move_forward(0.0)
        else:
            self.turn_right()

    def move_forward(self, turn_rate):
        # For tracked vehicles, straight motion
        left_track_speed = 1.0
        right_track_speed = 1.0

        self.get_logger().debug(f"Moving forward - Track speeds: L={left_track_speed:.2f}, R={right_track_speed:.2f}")

        # Apply to motors
        motor_left.value = max(min(left_track_speed, 1), -1)
        motor_right.value = max(min(right_track_speed, 1), -1)
        
        # Update odometry
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Calculate velocities
        self.last_linear_vel = MAX_LINEAR_SPEED
        self.last_angular_vel = 0.0
        
        # Update position
        self.x += self.last_linear_vel * math.cos(self.th) * dt
        self.y += self.last_linear_vel * math.sin(self.th) * dt
        
        self.get_logger().debug(f"Position: x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.th):.1f}°")
        
        self.last_time = current_time

    def turn_right(self):
        # Spot turn for tracked vehicle
        motor_left.value = TURN_SPEED
        motor_right.value = -TURN_SPEED
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Update velocities
        self.last_linear_vel = 0.0
        self.last_angular_vel = -(2.0 * TURN_SPEED * MAX_LINEAR_SPEED) / TRACK_WIDTH
        
        # Update orientation
        old_th = self.th
        self.th += self.last_angular_vel * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        
        angle_change = math.degrees(self.th - old_th)
        self.get_logger().debug(f"Turning - Angle change: {angle_change:.1f}°, Current: {math.degrees(self.th):.1f}°")
        
        self.last_time = current_time

    def map_callback(self, msg):
        # Get map dimensions
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Print detailed map info
        self.get_logger().info("\n=== Map Update ===")
        self.get_logger().info(f"Dimensions: {width}x{height} cells")
        self.get_logger().info(f"Resolution: {resolution}m per cell")
        self.get_logger().info(f"Origin: x={msg.info.origin.position.x:.2f}, y={msg.info.origin.position.y:.2f}")
        
        # Count cell types
        unknown = sum(1 for cell in msg.data if cell == -1)
        free = sum(1 for cell in msg.data if cell == 0)
        occupied = sum(1 for cell in msg.data if cell == 100)
        
        self.get_logger().info(f"Cell counts - Unknown: {unknown}, Free: {free}, Occupied: {occupied}")

        # Only print the map visualization if we have some known cells
        if free > 0 or occupied > 0:
            # Print a simplified version of the map
            step = 10
            for y in range(0, height, step):
                line = ""
                for x in range(0, width, step):
                    cell = msg.data[y * width + x]
                    if cell == -1:    # Unknown
                        line += "?"
                    elif cell == 0:   # Free
                        line += "."
                    elif cell == 100: # Occupied
                        line += "#"
                    else:            # Partially occupied
                        line += "o"
                self.get_logger().info(line)

    def publish_odom(self):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            # Predict step with current velocities
            self.ekf.predict(self.last_linear_vel, self.last_angular_vel, dt)
            
            # Update robot state from EKF
            self.x = float(self.ekf.state[0])
            self.y = float(self.ekf.state[1])
            self.th = float(self.ekf.state[2])
            
            # If we have laser scan data, use it to update the filter
            if hasattr(self, 'latest_scan'):
                # Create measurement vector from scan data
                measurement = np.array([self.x, self.y, self.th])
                self.ekf.update(measurement)
            
            # Log odometry data periodically (every second to avoid spam)
            if (current_time - self.last_time).nanoseconds / 1e9 >= 1.0:
                self.get_logger().debug(
                    f"Publishing odometry - "
                    f"Position: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.th):.1f}°), "
                    f"Velocities: (linear: {self.last_linear_vel:.2f} m/s, "
                    f"angular: {self.last_angular_vel:.2f} rad/s)"
                )
            
            # First publish the transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            
            # Create quaternion from yaw
            q = Quaternion()
            q.z = float(math.sin(self.th / 2.0))
            q.w = float(math.cos(self.th / 2.0))
            t.transform.rotation = q
            
            # Publish transform first
            self.tf_broadcaster.sendTransform(t)
            
            # Then publish odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            odom.pose.pose.position.x = float(self.x)
            odom.pose.pose.position.y = float(self.y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q
            
            odom.twist.twist.linear.x = float(self.last_linear_vel)
            odom.twist.twist.angular.z = float(self.last_angular_vel)
            
            # Add covariance matrices
            odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                  0, 0.1, 0, 0, 0, 0,
                                  0, 0, 0.1, 0, 0, 0,
                                  0, 0, 0, 0.1, 0, 0,
                                  0, 0, 0, 0, 0.1, 0,
                                  0, 0, 0, 0, 0, 0.1]
            
            odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                   0, 0.1, 0, 0, 0, 0,
                                   0, 0, 0.1, 0, 0, 0,
                                   0, 0, 0, 0.1, 0, 0,
                                   0, 0, 0, 0, 0.1, 0,
                                   0, 0, 0, 0, 0, 0.1]
            
            self.odom_pub.publish(odom)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {str(e)}')

    def stop_motors(self):
        """Stop both motors immediately."""
        self.get_logger().info("Stopping motors")
        motor_left.value = 0.0
        motor_right.value = 0.0
        
        # Update velocity tracking
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        
        # Update wheel velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()

    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        wall_follower.get_logger().info("Stopping motors")
        wall_follower.stop_motors()
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
