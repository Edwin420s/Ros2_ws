#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_walker')
        # Motion control topic: /cmd_vel [cite: 6, 9]
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameter to modify speed behavior [cite: 10]
        self.declare_parameter('linear_speed', 0.5)
        
        self.timer = self.create_timer(0.5, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        # Use parameter to set speed [cite: 10]
        speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        
        msg.linear.x = speed  # Linear velocity
        msg.angular.z = 0.1   # Constant turn
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity command to /cmd_vel: {speed} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()