#!/usr/bin/env python3
"""
pick_controller.py — Navigation + pick coordination.

States:
  SEARCHING   → spin slowly to scan area
  APPROACHING → drive toward detected object (proportional control)
  WAITING     → stop and let arm handle the pick (6 s wait)
  RETURNING   → drive back toward origin (0,0)
  IDLE        → park and wait for next object
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class PickControllerNode(Node):
    def __init__(self):
        super().__init__('pick_controller')

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(Point,     '/detected_object', self.object_cb, 10)
        self.create_subscription(LaserScan, '/scan',            self.lidar_cb,  10)
        self.create_subscription(Odometry,  '/odom',            self.odom_cb,   10)
        self.create_subscription(String,    '/robot_control_mode', self.mode_cb, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_mode_pub = self.create_publisher(String, '/arm_control_mode', 10)

        # ── Robot state ───────────────────────────────────────────────────
        self.rx = self.ry = self.rtheta = 0.0
        self.front_clear = True          # obstacle-free flag from LiDAR

        # ── Navigation state ──────────────────────────────────────────────
        self.state      = 'SEARCHING'
        self.target     = None           # Point
        self.state_t    = 0              # tick counter
        self.pick_count = 0
        self.control_mode = 'autonomous'  # 'autonomous', 'manual', 'idle'
        self.boundary_limit = 8.0        # meters from origin

        # Parameters
        self.stop_dist    = 1.8          # m to stop before object
        self.lin_speed    = 0.40         # m/s approach speed
        self.ang_speed    = 0.30         # rad/s search spin
        self.big_obstacle_threshold = 0.8  # m radius for big obstacles

        # ── 10 Hz control loop ────────────────────────────────────────────
        self.create_timer(0.1, self.loop)
        self.get_logger().info('🧭 PickController started')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.rtheta = math.atan2(siny, cosy)

    def lidar_cb(self, msg):
        """Check ±25° sector in front for obstacles and classify size."""
        n = len(msg.ranges)
        if n == 0:
            return
        # LiDAR is 360° with index 0 = -π; front ≈ index n//2
        c = n // 2
        width = max(1, int(25 * n / 360))
        sector = msg.ranges[c - width: c + width]
        valid  = [r for r in sector if msg.range_min < r < msg.range_max]
        
        if valid:
            min_dist = min(valid)
            self.front_clear = min_dist > 1.0
            self.current_obstacle_size = min_dist
        else:
            self.front_clear = True
            self.current_obstacle_size = float('inf')

    def object_cb(self, msg: Point):
        if self.control_mode == 'autonomous' and self.state == 'SEARCHING':
            self.target = msg
            self.state  = 'APPROACHING'
            self._log(f'🎯 Target acquired: ({msg.x:.1f}, {msg.y:.1f})')
            # Signal arm to start picking
            mode_msg = String()
            mode_msg.data = 'picking'
            self.arm_mode_pub.publish(mode_msg)
    
    def mode_cb(self, msg: String):
        self.control_mode = msg.data
        self._log(f'🎮 Control mode: {self.control_mode}')
        if self.control_mode == 'idle':
            self._transition('SEARCHING')

    def loop(self):
        self.state_t += 1
        
        # Check boundaries
        if self._check_boundaries():
            self._handle_boundary()
            return
            
        cmd = Twist()

        if self.control_mode == 'idle':
            # Do nothing
            pass
        elif self.control_mode == 'manual':
            # Manual control - let other nodes handle cmd_vel
            return
        elif self.state == 'SEARCHING':
            cmd.angular.z = self.ang_speed

        elif self.state == 'APPROACHING':
            cmd = self._approach()

        elif self.state == 'WAITING':
            # Stand still; after 6 s move on
            if self.state_t >= 60:
                self.pick_count += 1
                self._transition('SEARCHING')

        elif self.state == 'RETURNING':
            cmd = self._return_home()

        self.cmd_pub.publish(cmd)

    # ── Approach logic ────────────────────────────────────────────────────────
    def _approach(self) -> Twist:
        if not self.target:
            self._transition('SEARCHING')
            return Twist()

        dx   = self.target.x - self.rx
        dy   = self.target.y - self.ry
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.stop_dist:
            self._log('🛑 At picking distance — handing off to arm')
            self._transition('WAITING')
            return Twist()

        target_yaw = math.atan2(dy, dx)
        angle_err  = norm_angle(target_yaw - self.rtheta)

        cmd = Twist()
        if not self.front_clear:
            # Check if it's a big obstacle
            if self.current_obstacle_size < self.big_obstacle_threshold:
                # Small obstacle - try to pick it
                self._log('🔍 Small obstacle detected - attempting to pick')
                # This will trigger object detection in sensor_simulator
                return Twist()
            else:
                # Big obstacle - move away
                self._log('⚠️  Big obstacle - moving away')
                cmd.linear.x = -self.lin_speed * 0.5  # Back up
                cmd.angular.z = self.ang_speed * 2.0  # Turn quickly
                return cmd
        elif abs(angle_err) > 0.15:
            # P-control rotation — reduce speed while turning
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, angle_err * 1.2))
            cmd.linear.x  = self.lin_speed * 0.4
        else:
            # Proportional speed: slower near target
            cmd.linear.x  = min(self.lin_speed, dist * 0.35)
            cmd.angular.z = angle_err * 0.8
        return cmd

    # ── Return home ───────────────────────────────────────────────────────────
    def _return_home(self) -> Twist:
        home_x, home_y = 0.0, 0.0
        dx   = home_x - self.rx
        dy   = home_y - self.ry
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.5:
            self._transition('SEARCHING')
            return Twist()

        target_yaw = math.atan2(dy, dx)
        angle_err  = norm_angle(target_yaw - self.rtheta)
        cmd = Twist()
        cmd.linear.x  = min(self.lin_speed, dist * 0.3)
        cmd.angular.z = angle_err * 0.8
        return cmd

    def _check_boundaries(self) -> bool:
        """Check if robot is approaching boundary limits."""
        dist_from_origin = math.sqrt(self.rx**2 + self.ry**2)
        return dist_from_origin > self.boundary_limit
    
    def _handle_boundary(self):
        """Turn robot back toward center when approaching boundary."""
        # Calculate angle back to origin
        angle_to_origin = math.atan2(-self.ry, -self.rx)
        angle_err = norm_angle(angle_to_origin - self.rtheta)
        
        cmd = Twist()
        cmd.linear.x = self.lin_speed * 0.3  # Slow approach
        cmd.angular.z = angle_err * 1.5  # Strong turn toward center
        
        self.cmd_pub.publish(cmd)
        self._log('🚫 Near boundary - turning back')
    
    def _transition(self, state):
        self.get_logger().info(f'NAV: {self.state} → {state}')
        self.state   = state
        self.state_t = 0
        if state == 'SEARCHING':
            self.target = None
            # Reset arm to idle when searching
            mode_msg = String()
            mode_msg.data = 'idle'
            self.arm_mode_pub.publish(mode_msg)

    def _log(self, msg):
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
