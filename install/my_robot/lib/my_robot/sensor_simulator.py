#!/usr/bin/env python3
"""
sensor_simulator.py — Realistic sensor data for ECE2318 robot.

Publishes:
  /scan          — sensor_msgs/LaserScan   (360° LiDAR, 1° res)
  /imu           — sensor_msgs/Imu         (accel + angular vel from cmd_vel)
  /detected_object — geometry_msgs/Point   (objects within 2.5 m)

Tracks real robot position via /odom subscription.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry
import math
import random


class SensorSimulatorNode(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # ── Publishers ─────────────────────────────────────────────────────
        self.lidar_pub  = self.create_publisher(LaserScan, '/scan',             10)
        self.imu_pub    = self.create_publisher(Imu,       '/imu',              10)
        self.object_pub = self.create_publisher(Point,     '/detected_object',  10)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',    self.odom_cb,    10)
        self.create_subscription(Twist,    '/cmd_vel', self.cmd_vel_cb, 10)

        # ── Simulated objects: (x, y, z, radius) ─────────────────────────
        self.scene_objects = [
            {'pos': [3.0,  1.5,  0.5], 'r': 0.25, 'detected': False, 'id': 0},
            {'pos': [-2.0, 2.5,  0.4], 'r': 0.20, 'detected': False, 'id': 1},
            {'pos': [4.5, -1.0,  0.6], 'r': 0.30, 'detected': False, 'id': 2},
            {'pos': [1.0, -3.0,  0.3], 'r': 0.18, 'detected': False, 'id': 3},
            {'pos': [-3.5, -2.0, 0.5], 'r': 0.22, 'detected': False, 'id': 4},
        ]

        # Simulated walls / obstacles (just large cylinder obstacles)
        self.walls = [
            {'pos': [ 5.0,  0.0], 'r': 0.5},
            {'pos': [-5.0,  0.0], 'r': 0.5},
            {'pos': [ 0.0,  5.0], 'r': 0.5},
            {'pos': [ 0.0, -5.0], 'r': 0.5},
        ]

        # ── Robot state ───────────────────────────────────────────────────
        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self.robot_theta  = 0.0
        self.vx           = 0.0
        self.vtheta       = 0.0

        # For IMU integration
        self.prev_vx     = 0.0
        self.prev_vtheta = 0.0
        self.last_imu_t  = self.get_clock().now()

        # ── 10 Hz sensor tick ──────────────────────────────────────────────
        self.create_timer(0.1, self.sensor_tick)
        self.get_logger().info('📡 Sensor simulator started (LiDAR + IMU + Object detection)')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.robot_theta = math.atan2(siny, cosy)

    def cmd_vel_cb(self, msg: Twist):
        self.vx     = msg.linear.x
        self.vtheta = msg.angular.z

    # ── Main sensor update ────────────────────────────────────────────────────
    def sensor_tick(self):
        self._publish_lidar()
        self._publish_imu()
        self._detect_objects()

    # ── LiDAR ─────────────────────────────────────────────────────────────────
    def _publish_lidar(self):
        now = self.get_clock().now()
        scan = LaserScan()
        scan.header.stamp    = now.to_msg()
        scan.header.frame_id = 'lidar_link'

        scan.angle_min      = -math.pi
        scan.angle_max      =  math.pi
        scan.angle_increment = math.pi / 180.0  # 1° resolution = 361 beams
        scan.scan_time      = 0.1
        scan.range_min      = 0.12
        scan.range_max      = 12.0

        num_beams = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1

        all_obstacles = (
            [{'pos': o['pos'][:2], 'r': o['r']} for o in self.scene_objects] +
            self.walls
        )

        ranges = []
        for i in range(num_beams):
            beam_angle_world = (scan.angle_min + i * scan.angle_increment) + self.robot_theta
            min_dist = scan.range_max

            for obs in all_obstacles:
                dx = obs['pos'][0] - self.robot_x
                dy = obs['pos'][1] - self.robot_y
                obj_dist  = math.sqrt(dx**2 + dy**2)
                obj_angle = math.atan2(dy, dx)

                angle_diff = abs(self._norm_angle(beam_angle_world - obj_angle))

                # Apparent half-angle subtended by the obstacle
                if obj_dist < 0.01:
                    continue
                half_angle = math.atan2(obs['r'], obj_dist)

                if angle_diff <= half_angle:
                    hit = obj_dist - obs['r'] + random.gauss(0, 0.015)
                    min_dist = min(min_dist, max(scan.range_min, hit))

            # Clamp and add noise to empty readings
            if min_dist >= scan.range_max:
                min_dist = scan.range_max - abs(random.gauss(0, 0.05))

            ranges.append(float(clamp(min_dist, scan.range_min, scan.range_max)))

        scan.ranges      = ranges
        scan.intensities = [100.0 if r < scan.range_max - 0.1 else 0.0 for r in ranges]
        self.lidar_pub.publish(scan)

    # ── IMU ───────────────────────────────────────────────────────────────────
    def _publish_imu(self):
        now = self.get_clock().now()
        dt  = (now - self.last_imu_t).nanoseconds / 1e9
        self.last_imu_t = now

        imu = Imu()
        imu.header.stamp    = now.to_msg()
        imu.header.frame_id = 'imu_link'

        # Linear acceleration: a = Δv/dt (forward = x)
        ax = (self.vx - self.prev_vx) / max(dt, 0.001) if dt > 0 else 0.0
        ax += random.gauss(0, 0.02)   # sensor noise

        imu.linear_acceleration = Vector3(
            x=ax + random.gauss(0, 0.01),
            y=random.gauss(0, 0.01),
            z=9.81 + random.gauss(0, 0.02)   # gravity
        )

        # Angular velocity
        imu.angular_velocity = Vector3(
            x=random.gauss(0, 0.005),
            y=random.gauss(0, 0.005),
            z=self.vtheta + random.gauss(0, 0.01)
        )

        # Orientation from yaw (orientation filter simplified)
        q = euler_to_quat(0.0, 0.0, self.robot_theta)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]

        # Covariances (diagonal, reasonable values)
        imu.orientation_covariance[0]         = 0.0025
        imu.orientation_covariance[4]         = 0.0025
        imu.orientation_covariance[8]         = 0.0025
        imu.angular_velocity_covariance[0]    = 0.00025
        imu.angular_velocity_covariance[4]    = 0.00025
        imu.angular_velocity_covariance[8]    = 0.00025
        imu.linear_acceleration_covariance[0] = 0.004
        imu.linear_acceleration_covariance[4] = 0.004
        imu.linear_acceleration_covariance[8] = 0.004

        self.prev_vx     = self.vx
        self.prev_vtheta = self.vtheta
        self.imu_pub.publish(imu)

    # ── Object detection ──────────────────────────────────────────────────────
    def _detect_objects(self):
        for obj in self.scene_objects:
            dx = obj['pos'][0] - self.robot_x
            dy = obj['pos'][1] - self.robot_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 2.5 and not obj['detected']:
                p = Point()
                p.x = obj['pos'][0]
                p.y = obj['pos'][1]
                p.z = obj['pos'][2]
                self.object_pub.publish(p)
                obj['detected'] = True
                self.get_logger().info(
                    f'🎯 Object #{obj["id"]} detected at '
                    f'({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) — dist={dist:.2f}m')

            # Reset detection if robot moves away (> 5 m), allowing re-pick in demo
            if dist > 5.0 and obj['detected']:
                obj['detected'] = False

    # ── Utility ───────────────────────────────────────────────────────────────
    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


# ── Module-level helpers ───────────────────────────────────────────────────────
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def euler_to_quat(roll, pitch, yaw):
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
    node = SensorSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
