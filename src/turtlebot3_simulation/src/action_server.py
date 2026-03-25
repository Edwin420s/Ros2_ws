#!/usr/bin/env python3

"""
Action Server Node for Turtlebot3 Simulation

This node demonstrates ROS 2 actions by providing:
1. Navigate to position action
2. Patrol area action
3. Scan area action
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
import math
import time

# Custom action messages (these would normally be defined in a separate package)
# For this example, we'll simulate them with basic types
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped

class NavigateToPositionAction:
    """Custom action for navigating to a position"""
    def __init__(self):
        self.goal = Point()
        self.result = PointStamped()

class PatrolAreaAction:
    """Custom action for patrolling an area"""
    def __init__(self):
        self.radius = 2.0
        self.cycles = 3
        self.result = Empty()

class ScanAreaAction:
    """Custom action for scanning an area"""
    def __init__(self):
        self.angle_range = 2.0  # radians
        self.result = LaserScan()

class ActionServer(Node):
    def __init__(self):
        super().__init__('action_server')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Action servers (simulated with basic ROS 2 actions)
        # In a real implementation, these would use custom action definitions
        
        # State variables
        self.current_position = Point()
        self.current_orientation = 0.0
        self.current_scan = None
        self.action_in_progress = False
        
        # Navigation state
        self.target_position = None
        self.patrol_state = 'idle'
        self.patrol_start_time = 0
        self.patrol_cycles_completed = 0
        
        # Timer for action execution
        self.action_timer = self.create_timer(0.1, self.action_callback)
        
        self.get_logger().info('Action Server Started')
        self.get_logger().info('Available actions:')
        self.get_logger().info('  - navigate_to_position')
        self.get_logger().info('  - patrol_area')
        self.get_logger().info('  - scan_area')
    
    def odom_callback(self, msg):
        """Odometry callback"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.current_orientation = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def scan_callback(self, msg):
        """Laser scan callback"""
        self.current_scan = msg
    
    def navigate_to_position(self, target_x, target_y):
        """Navigate to a specific position"""
        self.target_position = Point()
        self.target_position.x = target_x
        self.target_position.y = target_y
        self.action_in_progress = True
        self.patrol_state = 'navigating'
        
        self.get_logger().info(f"Navigating to position: ({target_x}, {target_y})")
    
    def patrol_area(self, radius=2.0, cycles=3):
        """Patrol in a circular area"""
        self.patrol_state = 'patrolling'
        self.patrol_start_time = time.time()
        self.patrol_cycles_completed = 0
        self.action_in_progress = True
        
        self.get_logger().info(f"Starting patrol: radius={radius}m, cycles={cycles}")
    
    def scan_area(self, angle_range=2.0):
        """Scan the surrounding area"""
        self.patrol_state = 'scanning'
        self.scan_start_angle = self.current_orientation
        self.scan_angle_range = angle_range
        self.scan_start_time = time.time()
        self.action_in_progress = True
        
        self.get_logger().info(f"Starting scan: angle_range={angle_range} rad")
    
    def action_callback(self):
        """Execute current action"""
        if not self.action_in_progress:
            return
        
        twist = Twist()
        
        if self.patrol_state == 'navigating':
            # Navigate to target position
            if self.target_position:
                dx = self.target_position.x - self.current_position.x
                dy = self.target_position.y - self.current_position.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance > 0.1:  # Not at target yet
                    # Calculate angle to target
                    target_angle = math.atan2(dy, dx)
                    angle_diff = target_angle - self.current_orientation
                    
                    # Normalize angle difference
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Control robot
                    if abs(angle_diff) > 0.1:
                        # Turn towards target
                        twist.angular.z = 0.5 if angle_diff > 0 else -0.5
                    else:
                        # Move towards target
                        twist.linear.x = min(0.2, distance)
                else:
                    # Reached target
                    self.action_in_progress = False
                    self.patrol_state = 'idle'
                    self.get_logger().info("Reached target position!")
        
        elif self.patrol_state == 'patrolling':
            # Patrol in circular motion
            elapsed = time.time() - self.patrol_start_time
            twist.linear.x = 0.2
            twist.angular.z = 0.3
            
            # Complete patrol after specified time
            if elapsed > 20.0:  # 20 seconds per cycle
                self.patrol_cycles_completed += 1
                if self.patrol_cycles_completed >= 3:
                    self.action_in_progress = False
                    self.patrol_state = 'idle'
                    self.get_logger().info("Patrol completed!")
                else:
                    self.patrol_start_time = time.time()
        
        elif self.patrol_state == 'scanning':
            # Scan by rotating in place
            elapsed = time.time() - self.scan_start_time
            angle_rotated = elapsed * 0.5  # 0.5 rad/s rotation
            
            if angle_rotated < self.scan_angle_range:
                twist.angular.z = 0.5
            else:
                # Scan complete
                self.action_in_progress = False
                self.patrol_state = 'idle'
                self.get_logger().info("Scan completed!")
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
    
    def execute_navigate_to_position(self, goal_handle):
        """Execute navigate to position action"""
        self.navigate_to_position(goal_handle.request.goal.x, goal_handle.request.goal.y)
        
        while self.action_in_progress and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle.succeed()
        result = NavigateToPositionAction()
        result.result.point = self.current_position
        return result
    
    def execute_patrol_area(self, goal_handle):
        """Execute patrol area action"""
        self.patrol_area(goal_handle.request.radius, goal_handle.request.cycles)
        
        while self.action_in_progress and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle.succeed()
        result = PatrolAreaAction()
        return result
    
    def execute_scan_area(self, goal_handle):
        """Execute scan area action"""
        self.scan_area(goal_handle.request.angle_range)
        
        while self.action_in_progress and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle.succeed()
        result = ScanAreaAction()
        if self.current_scan:
            result.result = self.current_scan
        return result

def main(args=None):
    rclpy.init(args=args)
    
    action_server = ActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info('Keyboard interrupt, stopping action server')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
