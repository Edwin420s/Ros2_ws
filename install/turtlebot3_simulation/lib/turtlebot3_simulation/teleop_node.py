#!/usr/bin/env python3

"""
Teleoperation Node for Turtlebot3 Simulation

This node provides keyboard-based teleoperation control for the Turtlebot3 robot.
It demonstrates:
1. Publishing to /cmd_vel topic
2. Keyboard input handling
3. Smooth velocity control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import select
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Setup keyboard
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop Node Started')
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions"""
        self.get_logger().info("""
        ======================
        TurtleBot3 Teleoperation
        ======================
        Control Your TurtleBot3!
        ---------------------------
        Moving around:
           w
        a    s    d
           x
        
        w/x : increase/decrease linear velocity
        a/d : increase/decrease angular velocity
        s   : stop robot
        
        q : quit
        
        Current speeds:
          Linear: {:.2f} m/s
          Angular: {:.2f} rad/s
        ======================
        """.format(self.linear_speed, self.angular_speed))
    
    def get_key(self):
        """Get keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def timer_callback(self):
        """Publish current velocities"""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        
        self.cmd_vel_pub.publish(twist)
        
        if abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
            self.get_logger().info(
                f'Publishing: linear={self.current_linear:.2f}, angular={self.current_angular:.2f}'
            )
    
    def run(self):
        """Main teleoperation loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    self.current_linear = min(self.current_linear + 0.05, self.linear_speed)
                elif key == 'x':
                    self.current_linear = max(self.current_linear - 0.05, -self.linear_speed)
                elif key == 'a':
                    self.current_angular = min(self.current_angular + 0.1, self.angular_speed)
                elif key == 'd':
                    self.current_angular = max(self.current_angular - 0.1, -self.angular_speed)
                elif key == 's':
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                elif key == 'q':
                    break
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error in teleoperation: {e}')
        finally:
            # Stop robot before exit
            self.current_linear = 0.0
            self.current_angular = 0.0
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    teleop = TeleopNode()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        teleop.get_logger().info('Keyboard interrupt, stopping teleop')
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
