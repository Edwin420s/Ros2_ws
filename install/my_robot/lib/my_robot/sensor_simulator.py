#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, Point
import math
import random

class SensorSimulatorNode(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        
        # LIDAR publisher
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Object detection publisher
        self.object_pub = self.create_publisher(Point, '/detected_object', 10)
        
        # Simulated objects in the environment (x, y, z, size)
        self.objects = [
            {'pos': [2.0, 1.0, 0.5], 'size': 0.2, 'detected': False},
            {'pos': [-1.5, 2.0, 0.3], 'size': 0.15, 'detected': False},
            {'pos': [3.0, -1.0, 0.4], 'size': 0.25, 'detected': False},
            {'pos': [0.5, -2.5, 0.6], 'size': 0.18, 'detected': False},
        ]
        
        # Robot position (will be updated by odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Timer for sensor updates
        self.timer = self.create_timer(0.1, self.update_sensors)  # 10 Hz
        
        self.get_logger().info('Sensor Simulator Node Started')
    
    def update_sensors(self):
        """Update LIDAR scan and object detection"""
        self.publish_lidar_scan()
        self.detect_objects()
    
    def publish_lidar_scan(self):
        """Publish simulated LIDAR scan data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        
        # LIDAR parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree resolution
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = []
        scan.intensities = []
        
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Calculate distance to nearest object in this direction
            min_distance = scan.range_max
            
            for obj in self.objects:
                # Calculate relative position of object from robot
                rel_x = obj['pos'][0] - self.robot_x
                rel_y = obj['pos'][1] - self.robot_y
                
                # Transform to robot's coordinate frame
                cos_theta = math.cos(-self.robot_theta)
                sin_theta = math.sin(-self.robot_theta)
                robot_rel_x = rel_x * cos_theta - rel_y * sin_theta
                robot_rel_y = rel_x * sin_theta + rel_y * cos_theta
                
                # Calculate angle and distance to object
                obj_angle = math.atan2(robot_rel_y, robot_rel_x)
                obj_distance = math.sqrt(robot_rel_x**2 + robot_rel_y**2)
                
                # Check if object is in this laser beam
                angle_diff = abs(self.normalize_angle(angle - obj_angle))
                if angle_diff < scan.angle_increment:
                    # Add some noise to make it realistic
                    noisy_distance = obj_distance + random.gauss(0, 0.01)
                    min_distance = min(min_distance, noisy_distance)
            
            # Add some random noise to empty space readings
            if min_distance >= scan.range_max:
                min_distance = scan.range_max + random.gauss(0, 0.1)
            
            scan.ranges.append(max(scan.range_min, min(scan.range_max, min_distance)))
            scan.intensities.append(100.0 if min_distance < scan.range_max else 0.0)
        
        self.lidar_pub.publish(scan)
    
    def detect_objects(self):
        """Detect objects and publish their positions"""
        for obj in self.objects:
            # Calculate relative position from robot
            rel_x = obj['pos'][0] - self.robot_x
            rel_y = obj['pos'][1] - self.robot_y
            distance = math.sqrt(rel_x**2 + rel_y**2)
            
            # Detect objects within 2 meters
            if distance < 2.0 and not obj['detected']:
                # Publish detected object position
                point = Point()
                point.x = obj['pos'][0]
                point.y = obj['pos'][1]
                point.z = obj['pos'][2]
                
                self.object_pub.publish(point)
                obj['detected'] = True
                
                self.get_logger().info(f'Detected object at ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def update_robot_position(self, x, y, theta):
        """Update robot position (could be subscribed from odometry)"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulatorNode()
    
    # Simulate robot movement for testing
    def simulate_movement():
        # Simple circular motion for testing
        time_elapsed = node.get_clock().now().nanoseconds / 1e9
        node.update_robot_position(
            math.cos(time_elapsed * 0.1) * 2.0,
            math.sin(time_elapsed * 0.1) * 2.0,
            time_elapsed * 0.1
        )
    
    # Create timer for position updates
    node.create_timer(0.1, simulate_movement)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
