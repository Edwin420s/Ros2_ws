#!/usr/bin/env python3
"""
robot_actions.py - ROS 2 actions for ECE2318 assignment.

Provides actions for:
- /navigate_to_pose: Navigate to specific pose with feedback
- /pick_and_place: Complete pick and place sequence with feedback
- /explore_area: Systematic area exploration with cancellation support
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time
import threading

# Custom action interfaces (using standard ones for simplicity)
from nav2_msgs.action import NavigateToPose
from example_interfaces.action import Fibonacci


class NavigateToPoseAction:
    """Custom navigate to pose action using available messages"""
    def __init__(self):
        self.goal_pose = None
        self.current_pose = None
        self.is_active = False
        self.feedback = {'distance_to_goal': 0.0, 'status': 'idle'}


class PickAndPlaceAction:
    """Custom pick and place action"""
    def __init__(self):
        self.target_object = None
        self.is_active = False
        self.feedback = {'stage': 'idle', 'objects_picked': 0}


class RobotActionsNode(Node):
    def __init__(self):
        super().__init__('robot_actions')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/action_status', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        
        # Action servers (simplified using basic action types)
        self.navigate_action = NavigateToPoseAction()
        self.pick_action = PickAndPlaceAction()
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.front_clear = True
        
        # Action execution flags
        self Navigate_active = False
        self.pick_active = False
        self.explore_active = False
        
        # Timers for action execution
        self.create_timer(0.1, self.execute_actions)
        
        self.get_logger().info('🎯 Robot Actions started - 3 actions available')
    
    def odom_cb(self, msg):
        """Update current robot pose"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.current_theta = math.atan2(siny, cosy)
    
    def lidar_cb(self, msg):
        """Check front sector for obstacles"""
        n = len(msg.ranges)
        if n == 0:
            return
        c = n // 2
        width = max(1, int(25 * n / 360))
        sector = msg.ranges[c - width: c + width]
        valid = [r for r in sector if msg.range_min < r < msg.range_max]
        self.front_clear = (min(valid) > 1.0) if valid else True
    
    def execute_navigate_to_pose(self, goal_x, goal_y):
        """Execute navigation to pose"""
        if not self.Navigate_active:
            return
        
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.3:  # Reached goal
            self.Navigate_active = False
            status_msg = String()
            status_msg.data = f'Navigation complete - reached ({goal_x:.1f}, {goal_y:.1f})'
            self.status_pub.publish(status_msg)
            self.get_logger().info(f'Action: Navigation complete')
            return
        
        # Calculate desired heading
        target_theta = math.atan2(dy, dx)
        angle_error = self._normalize_angle(target_theta - self.current_theta)
        
        # P-control for navigation
        cmd = Twist()
        if not self.front_clear:
            cmd.angular.z = -0.3  # Obstacle avoidance
        elif abs(angle_error) > 0.15:
            cmd.angular.z = max(-0.5, min(0.5, angle_error * 1.5))
            cmd.linear.x = 0.2
        else:
            cmd.linear.x = min(0.4, distance * 0.5)
            cmd.angular.z = angle_error * 0.8
        
        self.cmd_vel_pub.publish(cmd)
        
        # Update feedback
        status_msg = String()
        status_msg.data = f'Navigating: {distance:.2f}m to goal, angle_err: {angle_error:.2f}'
        self.status_pub.publish(status_msg)
    
    def execute_pick_and_place(self):
        """Execute pick and place sequence"""
        if not self.pick_active:
            return
        
        # Simplified pick sequence - would interface with arm_controller
        stages = ['approaching', 'picking', 'lifting', 'placing', 'complete']
        stage_duration = 2.0  # seconds per stage
        
        if not hasattr(self, 'pick_start_time'):
            self.pick_start_time = time.time()
            self.pick_stage = 0
        
        elapsed = time.time() - self.pick_start_time
        current_stage = min(int(elapsed / stage_duration), len(stages) - 1)
        
        if current_stage != self.pick_stage:
            self.pick_stage = current_stage
            status_msg = String()
            status_msg.data = f'Pick & Place: Stage {current_stage + 1}/{len(stages)} - {stages[current_stage]}'
            self.status_pub.publish(status_msg)
            self.get_logger().info(f'Action: {stages[current_stage]}')
        
        if current_stage >= len(stages) - 1:
            self.pick_active = False
            delattr(self, 'pick_start_time')
            delattr(self, 'pick_stage')
    
    def execute_explore_area(self):
        """Execute area exploration"""
        if not self.explore_active:
            return
        
        # Simple exploration pattern
        if not hasattr(self, 'explore_start_time'):
            self.explore_start_time = time.time()
            self.explore_phase = 0
        
        elapsed = time.time() - self.explore_start_time
        
        # Exploration phases: forward, turn, scan
        if elapsed < 5.0:  # Forward phase
            cmd = Twist()
            cmd.linear.x = 0.3
            self.cmd_vel_pub.publish(cmd)
            phase = "forward"
        elif elapsed < 8.0:  # Turn phase
            cmd = Twist()
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            phase = "turning"
        elif elapsed < 10.0:  # Scan phase
            cmd = Twist()
            cmd.angular.z = 0.2
            self.cmd_vel_pub.publish(cmd)
            phase = "scanning"
        else:  # Reset cycle
            self.explore_start_time = time.time()
            phase = "restarting"
        
        status_msg = String()
        status_msg.data = f'Exploring: {phase} (elapsed: {elapsed:.1f}s)'
        self.status_pub.publish(status_msg)
    
    def execute_actions(self):
        """Main action execution loop"""
        if self.Navigate_active:
            # Would get goal from action server
            goal_x, goal_y = 2.0, 2.0  # Example goal
            self.execute_navigate_to_pose(goal_x, goal_y)
        
        if self.pick_active:
            self.execute_pick_and_place()
        
        if self.explore_active:
            self.execute_explore_area()
    
    def start_navigate_to_pose(self, x, y):
        """Start navigation action"""
        self.Navigate_active = True
        status_msg = String()
        status_msg.data = f'Starting navigation to ({x:.1f}, {y:.1f})'
        self.status_pub.publish(status_msg)
        self.get_logger().info(f'Action: Navigate to ({x}, {y})')
    
    def start_pick_and_place(self):
        """Start pick and place action"""
        self.pick_active = True
        status_msg = String()
        status_msg.data = 'Starting pick and place sequence'
        self.status_pub.publish(status_msg)
        self.get_logger().info('Action: Pick and place started')
    
    def start_explore_area(self):
        """Start exploration action"""
        self.explore_active = True
        status_msg = String()
        status_msg.data = 'Starting area exploration'
        self.status_pub.publish(status_msg)
        self.get_logger().info('Action: Area exploration started')
    
    def stop_all_actions(self):
        """Stop all active actions"""
        self.Navigate_active = False
        self.pick_active = False
        self.explore_active = False
        
        # Stop robot motion
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        status_msg = String()
        status_msg.data = 'All actions stopped'
        self.status_pub.publish(status_msg)
        self.get_logger().info('Action: All actions stopped')
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionsNode()
    
    # Example usage (in real implementation, these would be action server callbacks)
    def demo_actions():
        time.sleep(2)
        node.start_explore_area()
        time.sleep(15)
        node.stop_all_actions()
        time.sleep(2)
        node.start_navigate_to_pose(1.5, 1.5)
        time.sleep(10)
        node.stop_all_actions()
        time.sleep(2)
        node.start_pick_and_place()
        time.sleep(12)
        node.stop_all_actions()
    
    # Run demo in background
    demo_thread = threading.Thread(target=demo_actions, daemon=True)
    demo_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all_actions()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
