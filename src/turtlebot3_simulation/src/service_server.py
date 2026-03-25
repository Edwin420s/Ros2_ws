#!/usr/bin/env python3

"""
Service Server Node for Turtlebot3 Simulation

This node demonstrates ROS 2 services by providing:
1. Robot status service
2. Speed configuration service
3. Emergency stop service
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from example_interfaces.srv import SetFloat64
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Services
        self.emergency_stop_srv = self.create_service(
            SetBool, 'emergency_stop', self.emergency_stop_callback
        )
        
        self.set_linear_speed_srv = self.create_service(
            SetFloat64, 'set_linear_speed', self.set_linear_speed_callback
        )
        
        self.set_angular_speed_srv = self.create_service(
            SetFloat64, 'set_angular_speed', self.set_angular_speed_callback
        )
        
        self.get_status_srv = self.create_service(
            Trigger, 'get_status', self.get_status_callback
        )
        
        self.reset_odometry_srv = self.create_service(
            Trigger, 'reset_odometry', self.reset_odometry_callback
        )
        
        # State variables
        self.emergency_stopped = False
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.5
        self.robot_active = True
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Service Server Started')
        self.get_logger().info('Available services:')
        self.get_logger().info('  - emergency_stop')
        self.get_logger().info('  - set_linear_speed')
        self.get_logger().info('  - set_angular_speed')
        self.get_logger().info('  - get_status')
        self.get_logger().info('  - reset_odometry')
    
    def emergency_stop_callback(self, request, response):
        """Emergency stop service"""
        self.emergency_stopped = request.data
        
        if self.emergency_stopped:
            # Stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.robot_active = False
            response.success = True
            response.message = "Emergency stop activated - robot stopped"
            self.get_logger().warn('Emergency stop activated!')
        else:
            self.robot_active = True
            response.success = True
            response.message = "Emergency stop deactivated - robot can move again"
            self.get_logger().info('Emergency stop deactivated')
        
        return response
    
    def set_linear_speed_callback(self, request, response):
        """Set maximum linear speed service"""
        if 0.0 <= request.data <= 1.0:
            self.max_linear_speed = request.data
            response.success = True
            response.message = f"Linear speed set to {self.max_linear_speed} m/s"
            self.get_logger().info(f"Linear speed updated: {self.max_linear_speed} m/s")
        else:
            response.success = False
            response.message = "Linear speed must be between 0.0 and 1.0 m/s"
            self.get_logger().warn(f"Invalid linear speed requested: {request.data}")
        
        return response
    
    def set_angular_speed_callback(self, request, response):
        """Set maximum angular speed service"""
        if 0.0 <= request.data <= 2.0:
            self.max_angular_speed = request.data
            response.success = True
            response.message = f"Angular speed set to {self.max_angular_speed} rad/s"
            self.get_logger().info(f"Angular speed updated: {self.max_angular_speed} rad/s")
        else:
            response.success = False
            response.message = "Angular speed must be between 0.0 and 2.0 rad/s"
            self.get_logger().warn(f"Invalid angular speed requested: {request.data}")
        
        return response
    
    def get_status_callback(self, request, response):
        """Get robot status service"""
        status_msg = "Robot Status:\n"
        status_msg += f"  Active: {self.robot_active}\n"
        status_msg += f"  Emergency Stop: {self.emergency_stopped}\n"
        status_msg += f"  Max Linear Speed: {self.max_linear_speed} m/s\n"
        status_msg += f"  Max Angular Speed: {self.max_angular_speed} rad/s"
        
        response.success = True
        response.message = status_msg
        
        return response
    
    def reset_odometry_callback(self, request, response):
        """Reset odometry service"""
        # In a real implementation, this would reset the odometry
        # For simulation, we just acknowledge the request
        response.success = True
        response.message = "Odometry reset (simulated)"
        self.get_logger().info("Odometry reset requested")
        
        return response
    
    def publish_status(self):
        """Publish periodic robot status"""
        status_msg = String()
        status_msg.data = f"Active:{self.robot_active},Emergency:{self.emergency_stopped}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    service_server = ServiceServer()
    
    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        service_server.get_logger().info('Keyboard interrupt, stopping service server')
    finally:
        service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
