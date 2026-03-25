#!/usr/bin/env python3
"""
robot_master_controller.py - Central controller for robot behavior modes.

This node coordinates all robot behaviors and prevents conflicts between
different controllers by managing control modes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class RobotMasterControllerNode(Node):
    def __init__(self):
        super().__init__('robot_master_controller')

        # Publishers for control modes
        self.robot_mode_pub = self.create_publisher(String, '/robot_control_mode', 10)
        self.arm_mode_pub = self.create_publisher(String, '/arm_control_mode', 10)
        
        # Current state
        self.current_mode = 'autonomous'
        self.last_switch_time = time.time()
        
        # Start with autonomous mode
        self._set_mode('autonomous')
        
        # Timer for mode management (10 Hz)
        self.create_timer(0.1, self.manage_modes)
        
        self.get_logger().info('🎮 Robot Master Controller started')

    def _set_mode(self, mode):
        """Set robot control mode and update all nodes."""
        self.current_mode = mode
        self.last_switch_time = time.time()
        
        # Publish robot control mode
        robot_msg = String()
        robot_msg.data = mode
        self.robot_mode_pub.publish(robot_msg)
        
        # Set appropriate arm mode
        arm_msg = String()
        if mode == 'autonomous':
            arm_msg.data = 'idle'  # Arm will be activated by pick_controller
        elif mode == 'demo':
            arm_msg.data = 'demo'
        else:
            arm_msg.data = 'idle'
        
        self.arm_mode_pub.publish(arm_msg)
        
        self.get_logger().info(f'🎮 Mode set to: {mode}')

    def manage_modes(self):
        """Manage robot behavior modes based on time and conditions."""
        current_time = time.time()
        
        # Example: Cycle through modes every 30 seconds for demonstration
        # In real usage, this could be controlled by user input or AI decisions
        elapsed = current_time - self.last_switch_time
        
        if self.current_mode == 'autonomous' and elapsed > 30:
            self._set_mode('demo')
        elif self.current_mode == 'demo' and elapsed > 20:
            self._set_mode('autonomous')


def main(args=None):
    rclpy.init(args=args)
    node = RobotMasterControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
