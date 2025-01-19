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
        
        # Add timeout with retries
        retry_count = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle manager...')
            retry_count += 1
            if retry_count > 10:
                self.get_logger().error('Lifecycle manager service not available')
                return
            
        self.req = ManageLifecycleNodes.Request()
        
        # Start Nav2 after a short delay
        self.create_timer(5.0, self.startup)
        
    def startup(self):
        self.get_logger().info('Starting up Nav2...')
        self.req.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(self.req)
        
        # Add timeout for the future
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None:
                self.get_logger().info('Nav2 startup complete')
            else:
                self.get_logger().error('Nav2 startup failed')
        except Exception as e:
            self.get_logger().error(f'Nav2 startup error: {str(e)}')
        
    def shutdown(self):
        self.get_logger().info('Shutting down Nav2...')
        self.req.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    manager = Nav2LifecycleManager()
    
    try:
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