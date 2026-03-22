#!/usr/bin/env python3
"""
walker.py — Realistic differential-drive motion with natural behaviour.

Motion model:
  - Smooth acceleration/deceleration ramps (no instant velocity jumps)
  - Sinusoidal steering sway in EXPLORE (organic feel)
  - TURN uses an S-curve profile (slow in → fast → slow out)
  - SPIRAL widens arc continuously
  - PAUSE decelerates to a true stop

Wheel velocity publishing:
  - Right-side wheels are counter-rotated relative to left (mirrored rpy)
    so positive velocity always means FORWARD on both sides:
    data = [FL, FR, RL, RR]
    FR and RR are negated because their joint rpy="-π/2" flips the axis.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
import math


class RobotWalkerNode(Node):
    def __init__(self):
        super().__init__('robot_walker')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',    0.55)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('accel_rate',      0.06)   # m/s² per tick (smoother)
        self.declare_parameter('decel_rate',      0.10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist,             '/cmd_vel',         10)
        self.status_pub  = self.create_publisher(String,            '/robot_status',     10)
        self.wheel_pub   = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)

        # ── State machine ───────────────────────────────────────────────────
        self.state        = 'EXPLORE'
        self.state_timer  = 0.0
        self.state_target = 7.0
        self.orbit_count  = 0

        # ── Velocity state ──────────────────────────────────────────────────
        self.current_vx  = 0.0
        self.current_vz  = 0.0
        self.target_vx   = 0.0
        self.target_vz   = 0.0

        # ── Physical constants ──────────────────────────────────────────────
        self.WHEEL_RADIUS = 0.55   # metres
        self.HALF_TRACK   = 1.2    # half wheelbase width

        # 20 Hz control loop
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('🚗 Walker started — realistic differential drive')

    def _p(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    # ── Main loop ────────────────────────────────────────────────────────────
    def control_loop(self):
        t       = self.state_timer
        cruise  = self._p('linear_speed')
        ang_max = self._p('max_angular_speed')

        # ── State targets ─────────────────────────────────────────────────────
        if self.state == 'EXPLORE':
            # Gentle S-wave steering — amplitude increases then decreases
            sway = math.sin(t * 0.5) * 0.20 * math.sin(t * 0.15 + 0.5)
            self.target_vx = cruise
            self.target_vz = sway
            if t >= self.state_target:
                self._transition('TURN', 3.5)

        elif self.state == 'TURN':
            # S-curve profile: slow into the turn, fast in middle, slow out
            norm = min(t / self.state_target, 1.0)
            s    = math.sin(norm * math.pi)           # 0 → 1 → 0
            direction = 1.0 if (self.orbit_count % 2 == 0) else -1.0
            self.target_vz = ang_max * direction * s
            self.target_vx = cruise * (0.4 + 0.3 * (1.0 - s))  # slow on apex
            if t >= self.state_target:
                self.orbit_count += 1
                self._transition('PAUSE', 0.9)

        elif self.state == 'SPIRAL':
            decay = max(0.08, 1.0 - t * 0.12)
            self.target_vx = cruise
            self.target_vz = ang_max * 0.55 * decay
            if t >= 5.5:
                self._transition('EXPLORE', 7.0 + abs(math.sin(self.orbit_count)) * 3.0)

        elif self.state == 'PAUSE':
            self.target_vx = 0.0
            self.target_vz = 0.0
            # Only transition once actually stopped
            if t >= self.state_target and abs(self.current_vx) < 0.02:
                if self.orbit_count % 4 == 0:
                    self._transition('SPIRAL', 5.5)
                else:
                    self._transition('EXPLORE', 7.0)

        # ── Smooth ramp current toward target ─────────────────────────────────
        accel = self._p('accel_rate')
        decel = self._p('decel_rate')
        rate_x = accel if self.target_vx >= self.current_vx else decel
        rate_z = 0.12

        self.current_vx = self._ramp(self.current_vx, self.target_vx, rate_x)
        self.current_vz = self._ramp(self.current_vz, self.target_vz, rate_z)

        # ── Publish /cmd_vel ──────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = self.current_vx
        cmd.angular.z = self.current_vz
        self.cmd_vel_pub.publish(cmd)

        # ── Differential drive wheel velocities ───────────────────────────────
        # vl / vr in rad/s (angular velocity of wheel)
        vl = (self.current_vx - self.current_vz * self.HALF_TRACK) / self.WHEEL_RADIUS
        vr = (self.current_vx + self.current_vz * self.HALF_TRACK) / self.WHEEL_RADIUS

        # Right-side wheels have rpy="-π/2", mirroring the spin axis.
        # Negate vr so both sides produce forward motion from positive wheel vel.
        wm = Float64MultiArray()
        wm.data = [vl, -vr, vl, -vr]   # [FL, FR, RL, RR]
        self.wheel_pub.publish(wm)

        # ── Status ────────────────────────────────────────────────────────────
        s = String()
        s.data = (f'STATE:{self.state} | vx={self.current_vx:.3f} m/s '
                  f'vz={self.current_vz:.3f} rad/s | t={t:.1f}s')
        self.status_pub.publish(s)

        self.state_timer += self.dt

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _transition(self, state, duration):
        self.get_logger().info(f'Walker: {self.state} → {state}')
        self.state        = state
        self.state_target = duration
        self.state_timer  = 0.0

    @staticmethod
    def _ramp(cur, tgt, rate):
        diff = tgt - cur
        return cur + math.copysign(min(abs(diff), rate), diff)


def main(args=None):
    rclpy.init(args=args)
    node = RobotWalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()