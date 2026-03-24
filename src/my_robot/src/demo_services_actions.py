#!/usr/bin/env python3
"""
demo_services_actions.py - Demo script for testing ROS 2 services and actions.

This script demonstrates all the services and actions implemented for the ECE2318 assignment.
Run this after launching the robot simulation to test all functionality.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
import time


class DemoServicesActionsNode(Node):
    def __init__(self):
        super().__init__('demo_services_actions')
        
        # Service clients
        self.set_mode_client = self.create_client(SetBool, '/set_robot_mode')
        self.get_status_client = self.create_client(Trigger, '/get_robot_status')
        self.trigger_pick_client = self.create_client(Trigger, '/trigger_pick')
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        
        # Publisher for direct velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('🎬 Demo Services & Actions started')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services to be available...')
        self.set_mode_client.wait_for_service()
        self.get_status_client.wait_for_service()
        self.trigger_pick_client.wait_for_service()
        self.emergency_stop_client.wait_for_service()
        
        # Run demo sequence
        self.run_demo_sequence()
    
    def run_demo_sequence(self):
        """Run complete demo of all services and actions"""
        self.get_logger().info('=== Starting ROS 2 Services & Actions Demo ===')
        
        # 1. Get initial status
        self.get_logger().info('\n1. Getting robot status...')
        self.call_get_status()
        time.sleep(2)
        
        # 2. Set manual mode
        self.get_logger().info('\n2. Setting robot to MANUAL mode...')
        self.call_set_mode(True)
        time.sleep(2)
        
        # 3. Test direct velocity control
        self.get_logger().info('\n3. Testing direct velocity control (manual mode)...')
        self.test_manual_control()
        time.sleep(3)
        
        # 4. Set autonomous mode
        self.get_logger().info('\n4. Setting robot to AUTONOMOUS mode...')
        self.call_set_mode(False)
        time.sleep(2)
        
        # 5. Trigger pick sequence
        self.get_logger().info('\n5. Triggering manual pick sequence...')
        self.call_trigger_pick()
        time.sleep(10)
        
        # 6. Get final status
        self.get_logger().info('\n6. Getting final robot status...')
        self.call_get_status()
        time.sleep(2)
        
        # 7. Emergency stop demonstration
        self.get_logger().info('\n7. Testing emergency stop...')
        self.test_emergency_stop()
        
        self.get_logger().info('\n=== Demo Complete ===')
        self.get_logger().info('All ROS 2 services and actions demonstrated successfully!')
    
    def call_set_mode(self, manual_mode):
        """Call set_robot_mode service"""
        request = SetBool.Request()
        request.data = manual_mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Mode change successful: {response.message}')
            else:
                self.get_logger().warn(f'❌ Mode change failed: {response.message}')
        else:
            self.get_logger().error('❌ Service call failed')
    
    def call_get_status(self):
        """Call get_robot_status service"""
        request = Trigger.Request()
        
        future = self.get_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Status: {response.message}')
            else:
                self.get_logger().warn(f'❌ Status check failed: {response.message}')
        else:
            self.get_logger().error('❌ Service call failed')
    
    def call_trigger_pick(self):
        """Call trigger_pick service"""
        request = Trigger.Request()
        
        future = self.trigger_pick_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Pick triggered: {response.message}')
            else:
                self.get_logger().warn(f'❌ Pick trigger failed: {response.message}')
        else:
            self.get_logger().error('❌ Service call failed')
    
    def call_emergency_stop(self):
        """Call emergency_stop service"""
        request = Trigger.Request()
        
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'🛑 Emergency stop: {response.message}')
            else:
                self.get_logger().warn(f'❌ Emergency stop failed: {response.message}')
        else:
            self.get_logger().error('❌ Service call failed')
    
    def test_manual_control(self):
        """Test direct velocity control in manual mode"""
        self.get_logger().info('Moving forward for 2 seconds...')
        
        # Forward motion
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        time.sleep(2)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('Manual control test complete')
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        self.get_logger().info('Starting motion...')
        
        # Start moving
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.2
        self.cmd_vel_pub.publish(cmd)
        
        time.sleep(1)
        
        # Emergency stop
        self.get_logger().info('Activating emergency stop!')
        self.call_emergency_stop()
        
        time.sleep(2)
        self.get_logger().info('Emergency stop test complete')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DemoServicesActionsNode()
        rclpy.spin_once(node)  # Run once to complete demo
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
