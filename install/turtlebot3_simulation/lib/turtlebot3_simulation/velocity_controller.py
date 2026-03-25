#!/usr/bin/env python3

"""
Velocity Controller Node for Turtlebot3 Simulation

This node demonstrates:
1. Publishing to /cmd_vel topic for robot motion control
2. Parameter-based behavior modification
3. Different motion patterns (square, circle, figure-8)
4. Smooth velocity transitions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
import math
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare parameters for speed control and behavior
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('motion_pattern', 'square')  # square, circle, figure8, stop
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameter values
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.motion_pattern = self.get_parameter('motion_pattern').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # State variables
        self.start_time = time.time()
        self.current_phase = 0
        self.phase_start_time = time.time()
        
        # Create timer for periodic commands
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info(f'Velocity Controller started:')
        self.get_logger().info(f'  - Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'  - Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  - Motion pattern: {self.motion_pattern}')
        self.get_logger().info(f'  - Publish rate: {self.publish_rate} Hz')
    
    def parameter_callback(self, params):
        """Handle dynamic parameter updates"""
        for param in params:
            if param.name == 'max_linear_speed':
                self.max_linear_speed = param.value
                self.get_logger().info(f'Updated max_linear_speed: {self.max_linear_speed}')
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = param.value
                self.get_logger().info(f'Updated max_angular_speed: {self.max_angular_speed}')
            elif param.name == 'motion_pattern':
                self.motion_pattern = param.value
                self.current_phase = 0
                self.phase_start_time = time.time()
                self.get_logger().info(f'Updated motion pattern: {self.motion_pattern}')
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
                self.timer.destroy()
                self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
                self.get_logger().info(f'Updated publish rate: {self.publish_rate} Hz')
        
        return SetParametersResult(successful=True)
    
    def timer_callback(self):
        """Publish velocity commands based on motion pattern"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        twist = Twist()
        
        if self.motion_pattern == 'square':
            twist = self.square_pattern(elapsed)
        elif self.motion_pattern == 'circle':
            twist = self.circle_pattern(elapsed)
        elif self.motion_pattern == 'figure8':
            twist = self.figure8_pattern(elapsed)
        elif self.motion_pattern == 'stop':
            twist = Twist()  # All zeros
        else:
            self.get_logger().warn(f'Unknown motion pattern: {self.motion_pattern}')
            twist = Twist()
        
        self.cmd_vel_pub.publish(twist)
        
        # Log current velocities (only when moving)
        if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
            self.get_logger().debug(
                f'Publishing: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}'
            )
    
    def square_pattern(self, elapsed):
        """Generate square motion pattern"""
        twist = Twist()
        phase_duration = 3.0  # seconds per side
        turn_duration = 1.5   # seconds per turn
        
        cycle_time = phase_duration + turn_duration
        phase = int(elapsed / cycle_time) % 4
        phase_elapsed = elapsed % cycle_time
        
        if phase_elapsed < phase_duration:
            # Move forward
            twist.linear.x = self.max_linear_speed
            twist.angular.z = 0.0
        else:
            # Turn
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular_speed
        
        return twist
    
    def circle_pattern(self, elapsed):
        """Generate circular motion pattern"""
        twist = Twist()
        twist.linear.x = self.max_linear_speed
        twist.angular.z = self.max_linear_speed / 0.5  # radius = 0.5m
        return twist
    
    def figure8_pattern(self, elapsed):
        """Generate figure-8 motion pattern"""
        twist = Twist()
        
        # Create figure-8 by varying angular velocity
        period = 10.0  # seconds for one figure-8
        t = (elapsed % period) / period * 2 * math.pi
        
        twist.linear.x = self.max_linear_speed
        twist.angular.z = self.max_angular_speed * math.sin(t)
        
        return twist

def main(args=None):
    rclpy.init(args=args)
    
    controller = VelocityController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt, stopping velocity controller')
    finally:
        # Send stop command before shutdown
        stop_twist = Twist()
        controller.cmd_vel_pub.publish(stop_twist)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
