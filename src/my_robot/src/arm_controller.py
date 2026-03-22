#!/usr/bin/env python3
"""
arm_controller.py — Smooth, non-blocking arm pick-and-place state machine.

State machine (runs forever):
  HOME  → OPEN_GRIPPER → REACH_APPROACH → REACH_OBJECT → CLOSE_GRIPPER
        → LIFT → DEPOSIT_APPROACH → DEPOSIT → OPEN_GRIPPER → HOME → …

Joint interpolation is done inside a 20 Hz timer — no blocking time.sleep().
The arm picks detected objects from /detected_object; falls back to a
continuous demo loop when idle.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math


# ── Helpers ────────────────────────────────────────────────────────────────────
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp(a, b, t):
    return a + (b - a) * clamp(t, 0.0, 1.0)


class ArmControllerNode(Node):

    # Joint name order must match URDF exactly
    JOINT_NAMES = [
        'arm_base_joint',   # 0  – rotation (yaw)
        'shoulder_joint',   # 1  – pitch up/down
        'elbow_joint',      # 2  – pitch (forward)
        'wrist_joint',      # 3  – wrist pitch
        'left_finger_joint',  # 4
        'right_finger_joint', # 5
    ]

    # Joint limits (rad) matching URDF
    LIMITS = [
        (-math.pi,   math.pi ),   # arm_base
        (-1.5708,    1.5708  ),   # shoulder
        (-2.0944,    0.0     ),   # elbow
        (-1.5708,    1.5708  ),   # wrist
        (0.0,        0.18    ),   # left_finger  (prismatic, m)
        (0.0,        0.18    ),   # right_finger
    ]

    # Named poses (home, deposit, ready)
    POSES = {
        'home':   [0.0,  0.40, -0.80,  0.40, 0.18, 0.18],   # raised rest
        'ready':  [0.0,  0.70, -1.40,  0.70, 0.18, 0.18],   # above pick zone
        'deposit':[math.pi, 0.50, -1.00, 0.50, 0.18, 0.18], # 180° to deposit side
    }

    def __init__(self):
        super().__init__('arm_controller')

        # ── Publishers ─────────────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(Point, '/detected_object',
                                 self.object_cb, 10)

        # ── Joint state ──────────────────────────────────────────────────
        self.current  = list(self.POSES['home'])     # actual published positions
        self.target   = list(self.POSES['home'])
        self.speed    = [0.06] * 6                   # max step per tick (20 Hz)
        self.speed[4] = 0.01                         # fingers slower
        self.speed[5] = 0.01

        # ── Pick-and-place state machine ──────────────────────────────────
        self.state       = 'HOME'
        self.state_ticks = 0         # ticks since entering this state
        self.pick_target: Point | None = None   # queued object to pick
        self.demo_objects = [
            Point(x=1.5, y=0.5,  z=0.5),
            Point(x=1.0, y=-0.6, z=0.4),
            Point(x=1.8, y=0.0,  z=0.6),
        ]
        self.demo_idx = 0

        # ── IK arm geometry (must match URDF) ────────────────────────────
        self.L1 = 1.6   # upper arm length
        self.L2 = 1.2   # forearm length
        self.ARM_BASE_Z = 0.56 + 0.11  # base_link z + arm_base joint z

        # ── 20 Hz timer ──────────────────────────────────────────────────
        self.create_timer(0.05, self.tick)
        self.get_logger().info('🦾 ArmController started — smooth IK pick loop')

    # ── Object detection callback ─────────────────────────────────────────────
    def object_cb(self, msg: Point):
        if self.state == 'HOME':
            self.pick_target = msg
            self.get_logger().info(
                f'↗ Object queued: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

    # ── Main tick ─────────────────────────────────────────────────────────────
    def tick(self):
        self.state_ticks += 1
        self._step_joints()        # smooth interpolation
        self._publish_joints()
        self._state_machine()

    # ── Smooth joint interpolation ────────────────────────────────────────────
    def _step_joints(self):
        for i in range(len(self.current)):
            diff = self.target[i] - self.current[i]
            step = clamp(abs(diff), 0.0, self.speed[i])
            self.current[i] += math.copysign(step, diff)
            lo, hi = self.LIMITS[i]
            self.current[i] = clamp(self.current[i], lo, hi)

    # ── Check if arm reached target ───────────────────────────────────────────
    def _at_target(self, tol=0.02):
        return all(abs(self.current[i] - self.target[i]) < tol
                   for i in range(len(self.current)))

    # ── State machine ─────────────────────────────────────────────────────────
    def _state_machine(self):
        if self.state == 'HOME':
            self._set_pose('home')
            # Use detected object or cycle demo objects
            if self.pick_target is None:
                if self.state_ticks > 60:           # wait 3 s in home
                    obj = self.demo_objects[self.demo_idx % len(self.demo_objects)]
                    self.pick_target = obj
            if self.pick_target and self._at_target():
                self._transition('OPEN_GRIPPER')

        elif self.state == 'OPEN_GRIPPER':
            self._set_fingers(open=True)
            if self._at_target():
                self._transition('REACH_APPROACH')

        elif self.state == 'REACH_APPROACH':
            # Move above object
            if self.pick_target:
                ok = self._ik_to(self.pick_target.x,
                                  self.pick_target.y,
                                  self.pick_target.z + 0.3,
                                  fingers_open=True)
                if not ok:
                    self.get_logger().warn('IK failed for approach — skipping')
                    self.pick_target = None
                    self._transition('HOME')
                    return
            if self._at_target(tol=0.04) and self.state_ticks > 20:
                self._transition('REACH_OBJECT')

        elif self.state == 'REACH_OBJECT':
            if self.pick_target:
                self._ik_to(self.pick_target.x,
                             self.pick_target.y,
                             self.pick_target.z,
                             fingers_open=True)
            if self._at_target(tol=0.04) and self.state_ticks > 20:
                self._transition('CLOSE_GRIPPER')

        elif self.state == 'CLOSE_GRIPPER':
            self._set_fingers(open=False)
            if self._at_target():
                self._transition('LIFT')

        elif self.state == 'LIFT':
            if self.pick_target:
                self._ik_to(self.pick_target.x,
                             self.pick_target.y,
                             self.pick_target.z + 0.5,
                             fingers_open=False)
            if self._at_target(tol=0.04) and self.state_ticks > 20:
                self._transition('DEPOSIT_APPROACH')

        elif self.state == 'DEPOSIT_APPROACH':
            self._set_pose('deposit')
            if self._at_target(tol=0.04) and self.state_ticks > 20:
                self._transition('DEPOSIT')

        elif self.state == 'DEPOSIT':
            self._set_fingers(open=True)
            if self._at_target():
                self.get_logger().info(
                    f'✅ Pick & deposit complete — object #{self.demo_idx}')
                self.demo_idx += 1
                self.pick_target = None
                self._transition('HOME')

    def _transition(self, state):
        self.get_logger().info(f'ARM: {self.state} → {state}')
        self.state       = state
        self.state_ticks = 0

    # ── Pose helpers ──────────────────────────────────────────────────────────
    def _set_pose(self, name):
        self.target = list(self.POSES[name])

    def _set_fingers(self, open: bool):
        val = 0.18 if open else 0.01
        self.target[4] = val
        self.target[5] = val

    # ── Inverse kinematics ────────────────────────────────────────────────────
    def _ik_to(self, x, y, z, fingers_open=True) -> bool:
        """
        2-link planar IK (shoulder + elbow) with base rotation.
        Returns True if reachable.
        """
        base_angle  = math.atan2(y, x)
        planar_dist = math.sqrt(x**2 + y**2)
        dz          = z - self.ARM_BASE_Z

        reach = math.sqrt(planar_dist**2 + dz**2)
        max_r = self.L1 + self.L2

        if reach > max_r * 0.98:
            reach = max_r * 0.98  # clamp to just inside reach

        try:
            cos_e = (reach**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_e = clamp(cos_e, -1.0, 1.0)
            elbow = -math.acos(cos_e)          # negative = arm bends forward

            k1 = self.L1 + self.L2 * math.cos(elbow)
            k2 = self.L2 * math.sin(elbow)
            shoulder = math.atan2(dz, planar_dist) - math.atan2(k2, k1)

            wrist   = -(shoulder + elbow)      # keep gripper level
            fingers = 0.18 if fingers_open else 0.01

            self.target = [
                clamp(base_angle, *self.LIMITS[0]),
                clamp(shoulder,   *self.LIMITS[1]),
                clamp(elbow,      *self.LIMITS[2]),
                clamp(wrist,      *self.LIMITS[3]),
                fingers,
                fingers,
            ]
            return True

        except Exception as e:
            self.get_logger().error(f'IK error: {e}')
            return False

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name         = self.JOINT_NAMES
        js.position     = list(self.current)
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
