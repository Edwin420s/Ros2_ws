#!/usr/bin/env python3
"""
arm_controller.py — Full 360° smooth pick-and-place with multi-direction demo.

Picks from 8 directions (N, NE, E, SE, S, SW, W, NW) at varied heights.
Deposits alternate left / right / rear to show full arm_base rotation range.
All motion is smooth per-joint interpolation at 20 Hz — no blocking sleep().
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ArmControllerNode(Node):

    JOINT_NAMES = [
        'arm_base_joint',
        'shoulder_joint',
        'elbow_joint',
        'wrist_joint',
        'left_finger_joint',
        'right_finger_joint',
    ]

    LIMITS = [
        (-math.pi,   math.pi ),
        (-1.5708,    1.5708  ),
        (-2.0944,    0.0     ),
        (-1.5708,    1.5708  ),
        (0.0,        0.18    ),
        (0.0,        0.18    ),
    ]

    # 8 demo pick points spread 360° at varied heights & distances
    DEMO_TARGETS = [
        Point(x= 1.6, y= 0.0, z= 0.5),   # N  — front, mid height
        Point(x= 1.2, y= 1.2, z= 0.3),   # NE — front-left, low
        Point(x= 0.0, y= 1.8, z= 0.7),   # E  — left side, high
        Point(x=-1.0, y= 1.4, z= 0.4),   # SE — rear-left
        Point(x=-1.5, y= 0.0, z= 0.6),   # S  — rear, mid
        Point(x=-1.0, y=-1.4, z= 0.3),   # SW — rear-right, low
        Point(x= 0.0, y=-1.8, z= 0.5),   # W  — right side
        Point(x= 1.2, y=-1.2, z= 0.7),   # NW — front-right, high
    ]

    # Deposit zones alternate: right-side, left-side, rear
    DEPOSIT_BASES = [math.pi/2, -math.pi/2, math.pi]

    # Named poses
    POSES = {
        'home':    [0.0,  0.40, -0.80,  0.40, 0.18, 0.18],
        'ready':   [0.0,  0.65, -1.30,  0.65, 0.18, 0.18],
        'wave_a':  [math.pi*0.25,  0.80, -1.50,  0.70, 0.18, 0.18],
        'wave_b':  [-math.pi*0.25, 0.80, -1.50,  0.70, 0.18, 0.18],
    }

    def __init__(self):
        super().__init__('arm_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Point, '/detected_object', self.object_cb, 10)

        # Joint motion state
        self.current = list(self.POSES['home'])
        self.target  = list(self.POSES['home'])

        # Per-joint step sizes (rad or m per tick @ 20 Hz)
        self.speed = [0.05, 0.05, 0.06, 0.07, 0.008, 0.008]

        # State machine
        self.state       = 'HOME'
        self.state_ticks = 0
        self.pick_target: Point | None = None
        self.demo_idx    = 0
        self.deposit_idx = 0

        # IK geometry (matches URDF)
        self.L1          = 1.6
        self.L2          = 1.2
        self.ARM_BASE_Z  = 0.67   # base_link z=0 + arm_base_joint z=0.56 + shoulder z=0.11

        self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info('🦾 ArmController — full 360° multi-direction mode')

    # ── Object detection callback ─────────────────────────────────────────────
    def object_cb(self, msg: Point):
        if self.state == 'HOME':
            self.pick_target = msg
            self.get_logger().info(f'↗ External object: ({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})')

    # ── Main tick ─────────────────────────────────────────────────────────────
    def tick(self):
        self.state_ticks += 1
        self._step_joints()
        self._publish()
        self._fsm()

    # ── Joint interpolation ────────────────────────────────────────────────────
    def _step_joints(self):
        for i in range(6):
            diff = self.target[i] - self.current[i]
            step = clamp(abs(diff), 0.0, self.speed[i])
            self.current[i] = clamp(
                self.current[i] + math.copysign(step, diff),
                *self.LIMITS[i]
            )

    def _at_target(self, tol=0.025):
        return all(abs(self.current[i] - self.target[i]) < tol for i in range(6))

    # ── State machine ─────────────────────────────────────────────────────────
    def _fsm(self):
        if self.state == 'HOME':
            self._set_pose('home')
            # Use external pick target or fall through to demo after 2 s
            if self.pick_target is None and self.state_ticks > 40:
                obj = self.DEMO_TARGETS[self.demo_idx % len(self.DEMO_TARGETS)]
                self.pick_target = obj
            if self.pick_target and self._at_target():
                self._go('OPEN_GRIPPER')

        elif self.state == 'OPEN_GRIPPER':
            self._fingers(open=True)
            if self._at_target():
                self._go('REACH_APPROACH')

        elif self.state == 'REACH_APPROACH':
            if self.pick_target:
                ok = self._ik(self.pick_target.x, self.pick_target.y,
                               self.pick_target.z + 0.35, fingers_open=True)
                if not ok:
                    self._abort()
                    return
            if self._at_target(0.04) and self.state_ticks > 15:
                self._go('REACH_OBJECT')

        elif self.state == 'REACH_OBJECT':
            if self.pick_target:
                self._ik(self.pick_target.x, self.pick_target.y,
                          self.pick_target.z, fingers_open=True)
            if self._at_target(0.04) and self.state_ticks > 15:
                self._go('CLOSE_GRIPPER')

        elif self.state == 'CLOSE_GRIPPER':
            self._fingers(open=False)
            if self._at_target():
                self._go('LIFT')

        elif self.state == 'LIFT':
            if self.pick_target:
                self._ik(self.pick_target.x, self.pick_target.y,
                          self.pick_target.z + 0.55, fingers_open=False)
            if self._at_target(0.04) and self.state_ticks > 15:
                self._go('SWEEP_TO_DEPOSIT')

        elif self.state == 'SWEEP_TO_DEPOSIT':
            # Rotate arm_base to deposit zone while keeping arm raised
            dep_base = self.DEPOSIT_BASES[self.deposit_idx % len(self.DEPOSIT_BASES)]
            self.target = [
                clamp(dep_base, *self.LIMITS[0]),
                0.50,             # shoulder mid
                -1.00,           # elbow
                0.50,             # wrist
                self.current[4], # keep fingers closed
                self.current[5],
            ]
            if self._at_target(0.04) and self.state_ticks > 10:
                self._go('DEPOSIT')

        elif self.state == 'DEPOSIT':
            self._fingers(open=True)
            if self._at_target():
                self.get_logger().info(
                    f'✅ Deposited object #{self.demo_idx} at zone {self.deposit_idx % 3}')
                self.demo_idx   += 1
                self.deposit_idx += 1
                self.pick_target = None
                self._go('WAVE')  # celebrate!

        elif self.state == 'WAVE':
            # Quick side-to-side wave to show all directions, then go home
            if self.state_ticks < 30:
                self._set_pose('wave_a')
            elif self.state_ticks < 60:
                self._set_pose('wave_b')
            else:
                self._go('HOME')

    def _go(self, state):
        self.get_logger().info(f'ARM: {self.state} → {state}')
        self.state       = state
        self.state_ticks = 0

    def _abort(self):
        self.get_logger().warn('IK out of reach — skipping to next target')
        self.pick_target = None
        self.demo_idx   += 1
        self._go('HOME')

    # ── Pose/finger helpers ───────────────────────────────────────────────────
    def _set_pose(self, name):
        self.target = list(self.POSES[name])

    def _fingers(self, open: bool):
        v = 0.18 if open else 0.01
        self.target[4] = v
        self.target[5] = v

    # ── 2-link planar inverse kinematics ─────────────────────────────────────
    def _ik(self, x, y, z, fingers_open=True) -> bool:
        base  = math.atan2(y, x)
        planar = math.sqrt(x**2 + y**2)
        dz    = z - self.ARM_BASE_Z
        reach = math.sqrt(planar**2 + dz**2)
        max_r = (self.L1 + self.L2) * 0.97

        if reach > max_r:
            # Scale target back to max reach
            scale  = max_r / reach
            planar *= scale
            dz     *= scale
            reach   = max_r

        try:
            cos_e = (reach**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            elbow = -math.acos(clamp(cos_e, -1.0, 1.0))
            k1    = self.L1 + self.L2 * math.cos(elbow)
            k2    = self.L2 * math.sin(elbow)
            shoulder = math.atan2(dz, planar) - math.atan2(k2, k1)
            wrist    = -(shoulder + elbow)
            fingers  = 0.18 if fingers_open else 0.01

            self.target = [
                clamp(base,     *self.LIMITS[0]),
                clamp(shoulder, *self.LIMITS[1]),
                clamp(elbow,    *self.LIMITS[2]),
                clamp(wrist,    *self.LIMITS[3]),
                fingers, fingers,
            ]
            return True
        except Exception as e:
            self.get_logger().error(f'IK error: {e}')
            return False

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish(self):
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
