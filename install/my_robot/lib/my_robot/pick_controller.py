#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class PickControllerNode(Node):
    def __init__(self):
        super().__init__('pick_controller')
        
        # Subscribers
        self.object_sub = self.create_subscription(
            Point, '/detected_object', self.object_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Target object
        self.target_object = None
        self.state = 'SEARCHING'  # SEARCHING, APPROACHING, PICKING, COMPLETE
        
        # Control parameters
        self.approach_distance = 1.5  # Distance to stop before object
        self.linear_speed = 0.5
        self.angular_speed = 0.3
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Pick Controller Node Started')
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
    
    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle avoidance"""
        # Simple obstacle detection in front
        front_angles = range(-30, 31)  # ±30 degrees in front
        min_front_distance = float('inf')
        
        for i in front_angles:
            if 0 <= i < len(msg.ranges):
                distance = msg.ranges[i]
                if distance < min_front_distance and distance > msg.range_min:
                    min_front_distance = distance
        
        # Store for obstacle avoidance
        self.front_obstacle_distance = min_front_distance
    
    def object_callback(self, msg):
        """Handle detected object"""
        if self.state == 'SEARCHING':
            self.target_object = msg
            self.state = 'APPROACHING'
            self.get_logger().info(f'Target acquired: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')
    
    def control_loop(self):
        """Main control loop"""
        if self.state == 'SEARCHING':
            self.search_behavior()
        elif self.state == 'APPROACHING':
            self.approach_behavior()
        elif self.state == 'PICKING':
            self.picking_behavior()
        elif self.state == 'COMPLETE':
            self.stop_robot()
    
    def search_behavior(self):
        """Search for objects by rotating"""
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(cmd)
    
    def approach_behavior(self):
        """Approach the target object"""
        if not self.target_object:
            self.state = 'SEARCHING'
            return
        
        # Calculate relative position to object
        rel_x = self.target_object.x - self.robot_x
        rel_y = self.target_object.y - self.robot_y
        distance = math.sqrt(rel_x**2 + rel_y**2)
        
        # Calculate angle to object
        target_angle = math.atan2(rel_y, rel_x)
        angle_diff = self.normalize_angle(target_angle - self.robot_theta)
        
        cmd = Twist()
        
        # Check if we're close enough
        if distance < self.approach_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            # Start picking sequence
            self.state = 'PICKING'
            self.get_logger().info('Reached target, starting pick sequence')
            return
        
        # Obstacle avoidance
        if hasattr(self, 'front_obstacle_distance') and self.front_obstacle_distance < 0.8:
            # Stop and turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Obstacle detected, avoiding')
        else:
            # Move towards object
            if abs(angle_diff) > 0.1:  # 5.7 degrees
                # Turn towards object
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                cmd.linear.x = 0.0
            else:
                # Move forward
                cmd.linear.x = min(self.linear_speed, distance * 0.5)
                cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def picking_behavior(self):
        """Wait for arm to complete picking"""
        # This is handled by the arm_controller node
        # We just need to stop the robot and wait
        self.stop_robot()
        
        # After some time, return to searching
        self.create_timer(5.0, self.resume_searching)
        self.state = 'COMPLETE'
    
    def resume_searching(self):
        """Resume searching for next object"""
        self.state = 'SEARCHING'
        self.target_object = None
        self.get_logger().info('Resuming search for objects')
    
    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PickControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
