#!/usr/bin/env python3

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
import time

class Nav2LifecycleManager(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_manager')
        
        # Create service client
        self.client = self.create_client(
            ManageLifecycleNodes,
            'lifecycle_manager/manage_nodes'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle manager...')
            
        self.req = ManageLifecycleNodes.Request()
        
    def startup(self):
        self.get_logger().info('Starting up Nav2...')
        self.req.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        
    def shutdown(self):
        self.get_logger().info('Shutting down Nav2...')
        self.req.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    manager = Nav2LifecycleManager()
    
    try:
        # Startup Nav2
        manager.startup()
        time.sleep(5)  # Wait for nodes to fully start
        
        # Keep running
        rclpy.spin(manager)
        
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 