#!/usr/bin/env python3
"""
robot_services.py - ROS 2 services for ECE2318 assignment.

Provides services for:
- /set_robot_mode: Change robot operation mode
- /get_robot_status: Get current robot status
- /trigger_pick: Trigger manual pick sequence
- /emergency_stop: Emergency stop all motion
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading


class RobotServicesNode(Node):
    def __init__(self):
        super().__init__('robot_services')
        
        # Publishers for control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/service_status', 10)
        
        # Robot state
        self.robot_mode = 'autonomous'
        self.emergency_stopped = False
        
        # Service definitions
        self.create_service(SetBool, '/set_robot_mode', self.set_robot_mode_cb)
        self.create_service(Trigger, '/get_robot_status', self.get_robot_status_cb)
        self.create_service(Trigger, '/trigger_pick', self.trigger_pick_cb)
        self.create_service(Trigger, '/emergency_stop', self.emergency_stop_cb)
        
        self.get_logger().info('🔧 Robot Services started - 4 services available')
    
    def set_robot_mode_cb(self, request, response):
        """Set robot mode: True=manual, False=autonomous"""
        self.robot_mode = 'manual' if request.data else 'autonomous'
        
        status_msg = String()
        status_msg.data = f'Robot mode set to: {self.robot_mode}'
        self.status_pub.publish(status_msg)
        
        response.success = True
        response.message = f'Mode changed to {self.robot_mode}'
        self.get_logger().info(f'Service: Mode set to {self.robot_mode}')
        return response
    
    def get_robot_status_cb(self, request, response):
        """Get comprehensive robot status"""
        status = {
            'mode': self.robot_mode,
            'emergency_stop': self.emergency_stopped,
            'nodes_running': ['robot_walker', 'arm_controller', 'sensor_simulator', 'fake_odom'],
            'topics_active': ['/cmd_vel', '/odom', '/scan', '/joint_states', '/detected_object']
        }
        
        response.success = True
        response.message = f'Status: {status}'
        self.get_logger().info('Service: Status requested')
        return response
    
    def trigger_pick_cb(self, request, response):
        """Trigger manual pick sequence"""
        # This would interface with pick_controller
        status_msg = String()
        status_msg.data = 'Manual pick triggered'
        self.status_pub.publish(status_msg)
        
        response.success = True
        response.message = 'Pick sequence triggered'
        self.get_logger().info('Service: Manual pick triggered')
        return response
    
    def emergency_stop_cb(self, request, response):
        """Emergency stop all robot motion"""
        self.emergency_stopped = True
        
        # Send zero velocity command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        
        status_msg = String()
        status_msg.data = 'EMERGENCY STOP ACTIVATED'
        self.status_pub.publish(status_msg)
        
        response.success = True
        response.message = 'Emergency stop activated'
        self.get_logger().warn('Service: EMERGENCY STOP ACTIVATED')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotServicesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
