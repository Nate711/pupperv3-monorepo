#!/usr/bin/env python3

"""
Simple test script to verify the animation system integration.
This script publishes animation selection messages to test controller switching.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class AnimationTester(Node):
    def __init__(self):
        super().__init__('animation_tester')
        
        # Publisher for animation selection
        self.animation_publisher = self.create_publisher(
            String,
            '/animation_controller_py/animation_select',
            10
        )
        
        # Wait a bit for the system to initialize
        time.sleep(2.0)
        
        self.get_logger().info("Animation system tester ready")
    
    def test_animation_switch(self, animation_name: str):
        """Test switching to a specific animation."""
        msg = String()
        msg.data = animation_name
        
        self.get_logger().info(f"Testing animation switch to: {animation_name}")
        self.animation_publisher.publish(msg)
        
        # Wait for animation to process
        time.sleep(1.0)


def main():
    rclpy.init()
    
    tester = AnimationTester()
    
    try:
        # Test various scenarios
        print("\n=== Animation System Integration Test ===")
        print("This will test controller switching functionality")
        print("Make sure the neural_controller system is running first!")
        
        # Test 1: Switch to a test animation (if any exist)
        print("\nTest 1: Attempting to switch to an animation...")
        tester.test_animation_switch("test_animation")
        
        print("\nTest 2: Switch to another animation...")
        tester.test_animation_switch("another_animation")
        
        print("\nTest complete! Check the logs for controller switching messages.")
        print("Expected behavior:")
        print("1. Neural controllers should deactivate")
        print("2. Forward command controllers should activate")
        print("3. Animation should play") 
        print("4. When animation completes, controllers switch back")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()