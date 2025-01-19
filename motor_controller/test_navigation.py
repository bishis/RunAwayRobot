#!/usr/bin/env python3

import rclpy
import asyncio
from motor_controller.navigation_interface import NavigationInterface

async def test_movement():
    rclpy.init()
    nav = NavigationInterface()
    
    try:
        # Test basic movements
        nav.move_forward(0.5)  # Move forward at half speed
        await asyncio.sleep(2.0)
        
        nav.stop()
        await asyncio.sleep(1.0)
        
        nav.rotate(0.5)  # Rotate clockwise
        await asyncio.sleep(2.0)
        
        nav.stop()
        
        # Test Nav2 navigation
        success = await nav.navigate_to(1.0, 1.0)  # Navigate to point (1,1)
        if success:
            print("Navigation successful!")
        
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(test_movement()) 