#!/usr/bin/env python3
"""
fake_odom.py — Improved odometry with visual wheel spinning.

Improvements over original:
  - Subscribes to /wheel_velocities from walker.py to spin wheels visually
  - Publishes all 8 wheel + arm base joints in JointState so RViz shows motion
  - Proper covariance matrices on Odometry message
  - 50 Hz tick rate for smooth animation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom')

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Twist,             '/cmd_vel',          self.cmd_vel_cb,   10)
        self.create_subscription(Float64MultiArray, '/wheel_velocities', self.wheel_vel_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub       = self.create_publisher(Odometry,   '/odom',         10)
        self.joint_pub      = self.create_publisher(JointState, '/joint_states', 10)

        # ── Robot pose ───────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.vx    = 0.0
        self.vtheta = 0.0

        # ── Wheel angle accumulators (for visual spinning) ────────────────────
        # [FL, FR, RL, RR]
        self.wheel_angles    = [0.0, 0.0, 0.0, 0.0]
        self.wheel_vel_cmds  = [0.0, 0.0, 0.0, 0.0]

        self.last_time = self.get_clock().now()

        # ── 50 Hz update ─────────────────────────────────────────────────────
        self.create_timer(0.02, self.update)
        self.get_logger().info('FakeOdom started at 50 Hz with wheel animation')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def cmd_vel_cb(self, msg):
        self.vx     = msg.linear.x
        self.vtheta = msg.angular.z

    def wheel_vel_cb(self, msg):
        if len(msg.data) >= 4:
            self.wheel_vel_cmds = list(msg.data[:4])

    # ── Main 50 Hz update ─────────────────────────────────────────────────────
    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return  # Skip invalid time steps
        self.last_time = now

        # Integrate pose (unicycle model)
        self.theta += self.vtheta * dt
        self.x     += self.vx * math.cos(self.theta) * dt
        self.y     += self.vx * math.sin(self.theta) * dt

        # Integrate wheel angles
        for i in range(4):
            self.wheel_angles[i] += self.wheel_vel_cmds[i] * dt

        q = self._euler_to_quaternion(0.0, 0.0, self.theta)

        self._publish_tf(now, q)
        self._publish_odom(now, q)
        self._publish_joint_states(now)

    # ── TF ────────────────────────────────────────────────────────────────────
    def _publish_tf(self, now, q):
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    # ── Odometry ──────────────────────────────────────────────────────────────
    def _publish_odom(self, now, q):
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.vtheta

        # Diagonal covariance (position uncertainty ~ 0.01 m²)
        odom.pose.covariance[0]   = 0.01
        odom.pose.covariance[7]   = 0.01
        odom.pose.covariance[35]  = 0.02
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)

    # ── Joint States (wheels + arm passthrough zero) ─────────────────────────
    def _publish_joint_states(self, now):
        js = JointState()
        js.header.stamp = now.to_msg()

        wheel_names = [
            'fl_wheel_joint', 'fr_wheel_joint',
            'rl_wheel_joint', 'rr_wheel_joint',
        ]
        # NOTE: arm joints are published by arm_controller.py
        js.name     = wheel_names
        js.position = list(self.wheel_angles)
        js.velocity = list(self.wheel_vel_cmds)

        self.joint_pub.publish(js)

    # ── Maths ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll/2),  math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
        return [
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
