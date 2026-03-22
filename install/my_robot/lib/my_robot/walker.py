#!/usr/bin/env python3
"""
walker.py — Real-world motion behaviour for ECE2318 robot.

State machine:
  EXPLORE  → robot drives forward with gentle sin-wave steering
  TURN     → performs a smooth banked turn when it has driven far enough
  SPIRAL   → slowly widens arc to sweep an area
  PAUSE    → brief stop before state changes (inertia feel)

Features:
  - Smooth velocity ramping (no instant jumps)
  - Sinusoidal steering overlay for natural sway
  - Wheel spin is published via /wheel_velocities for fake_odom to use
  - Publishes robot_status string for monitoring
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
        self.declare_parameter('linear_speed', 0.55)      # cruise m/s
        self.declare_parameter('max_angular_speed', 0.45) # rad/s
        self.declare_parameter('accel_rate', 0.08)        # m/s² per tick
        self.declare_parameter('decel_rate', 0.12)

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist,             '/cmd_vel',          10)
        self.status_pub   = self.create_publisher(String,            '/robot_status',      10)
        self.wheel_pub    = self.create_publisher(Float64MultiArray, '/wheel_velocities',  10)

        # ── State machine ───────────────────────────────────────────────────
        self.state         = 'EXPLORE'
        self.state_timer   = 0.0      # seconds spent in current state
        self.state_target  = 6.0     # how long to stay in EXPLORE

        # ── Velocity state ──────────────────────────────────────────────────
        self.current_vx    = 0.0
        self.current_vz    = 0.0
        self.target_vx     = 0.0
        self.target_vz     = 0.0

        # ── Environment model (objects to orbit) ────────────────────────────
        self.orbit_count   = 0

        # ── Timer: 20 Hz control loop ────────────────────────────────────────
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('🤖 RobotWalker started — smooth motion mode ON')

    # ────────────────────────────────────────────────────────────────────────
    def get_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    # ── State machine ────────────────────────────────────────────────────────
    def control_loop(self):
        t = self.state_timer
        cruise  = self.get_param('linear_speed')
        max_ang = self.get_param('max_angular_speed')

        # ── Decide targets based on state ────────────────────────────────────
        if self.state == 'EXPLORE':
            # Gentle sinusoidal steering to look organic
            sway = math.sin(t * 0.6) * 0.18
            self.target_vx = cruise
            self.target_vz = sway
            if t >= self.state_target:
                self.transition('TURN', 3.0)

        elif self.state == 'TURN':
            # Hard turn – direction alternates each cycle
            direction = 1.0 if (self.orbit_count % 2 == 0) else -1.0
            self.target_vx = cruise * 0.45
            self.target_vz = max_ang * direction
            if t >= self.state_target:
                self.orbit_count += 1
                self.transition('PAUSE', 0.8)

        elif self.state == 'SPIRAL':
            # Widening spiral arc
            arc = max_ang * max(0.1, 1.0 - t * 0.15)
            self.target_vx = cruise
            self.target_vz = arc * 0.6
            if t >= 5.0:
                self.transition('EXPLORE', 8.0)

        elif self.state == 'PAUSE':
            # Decelerate to a full stop
            self.target_vx = 0.0
            self.target_vz = 0.0
            if t >= self.state_target and abs(self.current_vx) < 0.01:
                # Alternate next state
                if self.orbit_count % 4 == 0:
                    self.transition('SPIRAL', 5.0)
                else:
                    self.transition('EXPLORE', 6.0 + math.sin(self.orbit_count) * 2.0)

        # ── Smooth ramping ────────────────────────────────────────────────────
        accel = self.get_param('accel_rate')
        decel = self.get_param('decel_rate')

        rate_x = accel if self.target_vx > self.current_vx else decel
        rate_z = 0.15

        self.current_vx = self._ramp(self.current_vx, self.target_vx, rate_x)
        self.current_vz = self._ramp(self.current_vz, self.target_vz, rate_z)

        # ── Publish /cmd_vel ─────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = self.current_vx
        cmd.angular.z = self.current_vz
        self.cmd_vel_pub.publish(cmd)

        # ── Publish wheel velocities for fake_odom visual wheel spinning ─────
        wheel_radius = 0.55
        half_track   = 1.2
        # left side: positive angular.z → left wheel slower
        vl = (self.current_vx - self.current_vz * half_track) / wheel_radius
        vr = (self.current_vx + self.current_vz * half_track) / wheel_radius
        wm = Float64MultiArray()
        wm.data = [vl, vr, vl, vr]   # [FL, FR, RL, RR]
        self.wheel_pub.publish(wm)

        # ── Status heartbeat ─────────────────────────────────────────────────
        s = String()
        s.data = (f'STATE:{self.state} | vx={self.current_vx:.2f}m/s '
                  f'| vz={self.current_vz:.2f}rad/s | t={t:.1f}s')
        self.status_pub.publish(s)

        self.state_timer += self.dt

    # ── Helpers ──────────────────────────────────────────────────────────────
    def transition(self, new_state, duration):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state        = new_state
        self.state_target = duration
        self.state_timer  = 0.0

    @staticmethod
    def _ramp(current, target, rate):
        diff = target - current
        step = min(abs(diff), rate)
        return current + math.copysign(step, diff)


def main(args=None):
    rclpy.init(args=args)
    node = RobotWalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()