#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pathlib import Path

class PoseVelocityRecorder(Node):
    def __init__(self):
        super().__init__('pose_velocity_recorder')
        self.pose = None
        self.vel  = None

        # Subscribers
        self.create_subscription(PoseStamped, '/robot_current_pose', self.pose_cb,  10)
        self.create_subscription(Odometry,     '/wheel_speeds',      self.odom_cb,  10)

        # CSV setup
        # Prepare Documents directory
        documents_dir = Path.home() / 'Documents'
        documents_dir.mkdir(parents=True, exist_ok=True)

        # Open the CSV in Documents
        log_path = documents_dir / 'pose_velocity_log.csv'
        self.get_logger().info(f'Logging pose+velocity to: {log_path}')
        self.csv_file = open(str(log_path), 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_sec', 'pos_x','pos_y','pos_z',
            'ori_x','ori_y','ori_z','ori_w',
            'lin_vel_x','lin_vel_y','lin_vel_z',
            'ang_vel_x','ang_vel_y','ang_vel_z'
        ])

        # Timer at 10Hz
        self.create_timer(0.1, self.timer_cb)

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def odom_cb(self, msg: Odometry):
        self.vel = msg.twist.twist

    def timer_cb(self):
        if not self.pose or not self.vel:
            return
        t = self.pose.header.stamp.sec + self.pose.header.stamp.nanosec * 1e-9
        p = self.pose.pose.position
        o = self.pose.pose.orientation
        lv = self.vel.linear
        av = self.vel.angular

        self.writer.writerow([
            f"{t:.6f}",
            f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
            f"{o.x:.4f}", f"{o.y:.4f}", f"{o.z:.4f}", f"{o.w:.4f}",
            f"{lv.x:.4f}", f"{lv.y:.4f}", f"{lv.z:.4f}",
            f"{av.x:.4f}", f"{av.y:.4f}", f"{av.z:.4f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseVelocityRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()