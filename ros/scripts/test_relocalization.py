#!/usr/bin/env python3
"""
Test script for KISS-Matcher relocalization functionality
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from kiss_matcher_ros.srv import Relocalize
from std_srvs.srv import Trigger
import time


class RelocalizationTester(Node):
    def __init__(self):
        super().__init__('relocalization_tester')
        
        # Create service clients
        self.relocalize_client = self.create_client(Relocalize, 'relocalize')
        self.reset_client = self.create_client(Trigger, 'reset_localization')
        
        # Subscribe to aligned cloud for verification
        self.aligned_sub = self.create_subscription(
            PointCloud2, '/aligned_scan', self.aligned_callback, 10)
        
        self.aligned_received = False
        
    def aligned_callback(self, msg):
        self.aligned_received = True
        self.get_logger().info(f'Received aligned cloud with {len(msg.data)} bytes')
    
    def test_global_relocalization(self):
        """Test global relocalization without pose hint"""
        self.get_logger().info('Testing global relocalization...')
        
        if not self.relocalize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Relocalize service not available')
            return False
        
        request = Relocalize.Request()
        # Empty pose hint triggers global relocalization
        
        future = self.relocalize_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Global relocalization result: success={response.success}, '
                                 f'confidence={response.confidence:.3f}, message={response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def test_hinted_relocalization(self, x=0.0, y=0.0, z=0.0):
        """Test relocalization with pose hint"""
        self.get_logger().info(f'Testing hinted relocalization at ({x}, {y}, {z})...')
        
        if not self.relocalize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Relocalize service not available')
            return False
        
        request = Relocalize.Request()
        request.pose_hint.header.frame_id = 'map'
        request.pose_hint.header.stamp = self.get_clock().now().to_msg()
        request.pose_hint.pose.pose.position.x = x
        request.pose_hint.pose.pose.position.y = y
        request.pose_hint.pose.pose.position.z = z
        request.pose_hint.pose.pose.orientation.w = 1.0
        
        future = self.relocalize_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Hinted relocalization result: success={response.success}, '
                                 f'confidence={response.confidence:.3f}, message={response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def test_reset(self):
        """Test localization reset"""
        self.get_logger().info('Testing localization reset...')
        
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Reset service not available')
            return False
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Reset result: success={response.success}, message={response.message}')
            return response.success
        else:
            self.get_logger().error('Reset service call failed')
            return False
    
    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('Starting relocalization tests...')
        
        # Test 1: Reset
        if not self.test_reset():
            self.get_logger().error('Reset test failed')
            return False
        
        time.sleep(1)
        
        # Test 2: Global relocalization
        if not self.test_global_relocalization():
            self.get_logger().error('Global relocalization test failed')
        
        time.sleep(1)
        
        # Test 3: Reset again
        if not self.test_reset():
            self.get_logger().error('Second reset test failed')
            return False
        
        time.sleep(1)
        
        # Test 4: Hinted relocalization
        if not self.test_hinted_relocalization(1.0, 2.0, 0.0):
            self.get_logger().error('Hinted relocalization test failed')
        
        # Check if aligned cloud was published
        start_time = time.time()
        while not self.aligned_received and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.aligned_received:
            self.get_logger().info('✓ Aligned cloud visualization working')
        else:
            self.get_logger().warn('⚠ Aligned cloud not received')
        
        self.get_logger().info('Tests completed')
        return True


def main():
    rclpy.init()
    
    tester = RelocalizationTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()