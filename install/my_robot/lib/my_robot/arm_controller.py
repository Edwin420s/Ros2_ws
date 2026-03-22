#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math
import time

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber for detected objects
        self.object_sub = self.create_subscription(
            Point, '/detected_object', self.object_callback, 10
        )
        
        # Current joint positions
        self.joint_positions = {
            'arm_base_joint': 0.0,
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'left_finger_joint': 0.0,
            'right_finger_joint': 0.0
        }
        
        # Arm parameters (in meters)
        self.arm_link1_length = 1.6
        self.arm_link2_length = 1.2
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info('Arm Controller Node Started')
        
    def object_callback(self, msg):
        """Callback when an object is detected"""
        self.get_logger().info(f'Object detected at: x={msg.x}, y={msg.y}, z={msg.z}')
        self.pick_object(msg.x, msg.y, msg.z)
    
    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        self.joint_pub.publish(msg)
    
    def move_arm_to_position(self, base_angle, shoulder_angle, elbow_angle, wrist_angle):
        """Move arm to specified joint angles"""
        self.joint_positions['arm_base_joint'] = base_angle
        self.joint_positions['shoulder_joint'] = shoulder_angle
        self.joint_positions['elbow_joint'] = elbow_angle
        self.joint_positions['wrist_joint'] = wrist_angle
        
        self.get_logger().info(f'Moving arm - Base: {base_angle:.2f}, Shoulder: {shoulder_angle:.2f}, Elbow: {elbow_angle:.2f}, Wrist: {wrist_angle:.2f}')
    
    def open_gripper(self):
        """Open the gripper"""
        self.joint_positions['left_finger_joint'] = 0.1
        self.joint_positions['right_finger_joint'] = 0.1
        self.get_logger().info('Opening gripper')
    
    def close_gripper(self):
        """Close the gripper"""
        self.joint_positions['left_finger_joint'] = 0.0
        self.joint_positions['right_finger_joint'] = 0.0
        self.get_logger().info('Closing gripper')
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        """Calculate joint angles for target position"""
        # Calculate base rotation
        base_angle = math.atan2(target_y, target_x)
        
        # Calculate distance in x-y plane
        distance = math.sqrt(target_x**2 + target_y**2)
        
        # Calculate arm reach (considering height)
        horizontal_reach = math.sqrt(distance**2 + (target_z - 0.6)**2)
        
        # Check if target is reachable
        max_reach = self.arm_link1_length + self.arm_link2_length
        if horizontal_reach > max_reach:
            self.get_logger().warn('Target out of reach')
            return None, None, None
        
        # Inverse kinematics for 2-link arm
        try:
            # Law of cosines for elbow angle
            cos_elbow = (horizontal_reach**2 - self.arm_link1_length**2 - self.arm_link2_length**2) / (2 * self.arm_link1_length * self.arm_link2_length)
            cos_elbow = max(-1, min(1, cos_elbow))  # Clamp to valid range
            elbow_angle = math.acos(cos_elbow)
            
            # Shoulder angle
            k1 = self.arm_link1_length + self.arm_link2_length * math.cos(elbow_angle)
            k2 = self.arm_link2_length * math.sin(elbow_angle)
            shoulder_angle = math.atan2(target_z - 0.6, distance) - math.atan2(k2, k1)
            
            # Wrist angle to keep gripper level
            wrist_angle = -(shoulder_angle + elbow_angle)
            
            return base_angle, shoulder_angle, elbow_angle, wrist_angle
            
        except Exception as e:
            self.get_logger().error(f'I Kinematics calculation failed: {e}')
            return None, None, None, None
    
    def pick_object(self, x, y, z):
        """Complete picking sequence for an object"""
        self.get_logger().info('Starting pick sequence')
        
        # Calculate target position (offset for gripper approach)
        approach_height = z + 0.2
        
        # Step 1: Move to approach position
        angles = self.inverse_kinematics(x, y, approach_height)
        if angles[0] is None:
            self.get_logger().error('Cannot reach object')
            return
        
        self.move_arm_to_position(angles[0], angles[1], angles[2], angles[3])
        time.sleep(2)
        
        # Step 2: Open gripper
        self.open_gripper()
        time.sleep(1)
        
        # Step 3: Move down to object
        angles = self.inverse_kinematics(x, y, z)
        if angles[0] is None:
            self.get_logger().error('Cannot reach object height')
            return
        
        self.move_arm_to_position(angles[0], angles[1], angles[2], angles[3])
        time.sleep(2)
        
        # Step 4: Close gripper
        self.close_gripper()
        time.sleep(1)
        
        # Step 5: Lift object
        angles = self.inverse_kinematics(x, y, approach_height)
        self.move_arm_to_position(angles[0], angles[1], angles[2], angles[3])
        time.sleep(2)
        
        self.get_logger().info('Pick sequence completed')
    
    def demo_movement(self):
        """Demo movement to show arm capabilities"""
        self.get_logger().info('Starting arm demo')
        
        # Open gripper
        self.open_gripper()
        time.sleep(1)
        
        # Move to different positions
        positions = [
            (0.0, 0.5, 1.0),
            (0.5, 0.5, 1.2),
            (0.0, 0.5, 0.8),
            (-0.5, 0.5, 1.0),
        ]
        
        for x, y, z in positions:
            angles = self.inverse_kinematics(x, y, z)
            if angles[0] is not None:
                self.move_arm_to_position(angles[0], angles[1], angles[2], angles[3])
                time.sleep(2)
        
        # Return to home position
        self.move_arm_to_position(0.0, 0.0, 0.0, 0.0)
        time.sleep(1)
        
        self.get_logger().info('Arm demo completed')

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    
    # Run demo after 3 seconds
    node.create_timer(3.0, lambda: node.demo_movement())
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
