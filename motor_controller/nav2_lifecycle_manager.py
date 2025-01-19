#!/usr/bin/env python3

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
import time
from lifecycle_msgs.srv import GetState

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
        
        # Check node states before startup
        self.create_timer(2.0, self.check_nodes)
        # Start Nav2 after initial check
        self.create_timer(5.0, self.startup)
        
    def check_nodes(self):
        """Check the state of all Nav2 nodes"""
        nodes_to_check = [
            'controller_server',
            'planner_server',
            'recoveries_server',
            'bt_navigator',
            'map_server',
            'amcl'
        ]
        
        for node in nodes_to_check:
            client = self.create_client(GetState, f'/{node}/get_state')
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Could not contact {node}')
                continue
                
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                self.get_logger().info(f'{node} state: {future.result().current_state.label}')
            else:
                self.get_logger().error(f'Failed to get state for {node}')
        
    def startup(self):
        self.get_logger().info('Starting up Nav2...')
        self.req.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(self.req)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'Nav2 startup response: {response.success}')
                if not response.success:
                    self.get_logger().error('Nav2 startup failed - checking node states')
                    self.check_nodes()
            else:
                self.get_logger().error('Nav2 startup future returned None')
        except Exception as e:
            self.get_logger().error(f'Nav2 startup error: {str(e)}')
            self.check_nodes()
        
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