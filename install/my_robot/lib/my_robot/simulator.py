import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import math
import time

class FastSimulator(Node):
    def __init__(self):
        super().__init__('fast_simulator')
        self.get_logger().info("Lightweight RViz Simulator Started!")
        
        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.v = 0.0
        self.w = 0.0
        
        self.last_time = time.time()
        
        # Virtual Obstacle (A wall at x=5.0 and a block at x=2.0, y=1.0)
        self.obstacles = [
            {'type': 'circle', 'x': 3.0, 'y': 1.0, 'r': 0.5},
            {'type': 'wall', 'x': 6.0}
        ]

        # Subscribers and Publishers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 30 Hz update loop
        self.create_timer(1.0 / 30.0, self.sim_loop)
        
    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        
    def sim_loop(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 1. Update Kinematics (Differential Drive Integration)
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        
        # 2. Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        # 3. Simulate and Publish LaserScan
        self.publish_scan()

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        
        num_readings = 100
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / num_readings
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        ranges = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.theta + angle
            
            ray_dx = math.cos(global_angle)
            ray_dy = math.sin(global_angle)
            
            min_dist = scan.range_max
            
            # Check intersection with obstacles
            for obs in self.obstacles:
                if obs['type'] == 'wall':
                    # Wall is at x = obs['x'], line is x = self.x + dist * ray_dx
                    if ray_dx > 0:
                        dist = (obs['x'] - self.x) / ray_dx
                        if 0 < dist < min_dist:
                            min_dist = dist
                elif obs['type'] == 'circle':
                    # Simple ray-circle intersection
                    oc_x = self.x - obs['x']
                    oc_y = self.y - obs['y']
                    b = 2.0 * (oc_x * ray_dx + oc_y * ray_dy)
                    c = (oc_x**2 + oc_y**2) - obs['r']**2
                    discriminant = b**2 - 4 * c
                    if discriminant > 0:
                        dist = (-b - math.sqrt(discriminant)) / 2.0
                        if 0 < dist < min_dist:
                            min_dist = dist
            
            # Add some noise
            import random
            if min_dist < scan.range_max:
                min_dist += random.uniform(-0.02, 0.02)
                
            ranges.append(float(min_dist))
            
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FastSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
