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

        # ── Publishers ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Robot state ───────────────────────────────────────────────────
        self.rx = self.ry = self.rtheta = 0.0
        self.front_clear = True          # obstacle-free flag from LiDAR

        # ── Navigation state ──────────────────────────────────────────────
        self.state      = 'SEARCHING'
        self.target     = None           # Point
        self.state_t    = 0              # tick counter
        self.pick_count = 0

        # Parameters
        self.stop_dist    = 1.8          # m to stop before object
        self.lin_speed    = 0.40         # m/s approach speed
        self.ang_speed    = 0.30         # rad/s search spin

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
        """Check ±25° sector in front for obstacles closer than 1.0 m."""
        n = len(msg.ranges)
        if n == 0:
            return
        # LiDAR is 360° with index 0 = -π; front ≈ index n//2
        c = n // 2
        width = max(1, int(25 * n / 360))
        sector = msg.ranges[c - width: c + width]
        valid  = [r for r in sector if msg.range_min < r < msg.range_max]
        self.front_clear = (min(valid) > 1.0) if valid else True

    def object_cb(self, msg: Point):
        if self.state == 'SEARCHING':
            self.target = msg
            self.state  = 'APPROACHING'
            self._log(f'🎯 Target acquired: ({msg.x:.1f}, {msg.y:.1f})')

    # ── Control loop ──────────────────────────────────────────────────────────
    def loop(self):
        self.state_t += 1
        cmd = Twist()

        if self.state == 'SEARCHING':
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
            # Obstacle avoidance: stop and turn right
            cmd.angular.z = -self.ang_speed * 1.5
            self._log('⚠️  Obstacle — avoiding')
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

    def _transition(self, state):
        self.get_logger().info(f'NAV: {self.state} → {state}')
        self.state   = state
        self.state_t = 0
        if state == 'SEARCHING':
            self.target = None

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
