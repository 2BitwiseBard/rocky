#!/usr/bin/env python3
"""Gait node — pebble_gait.py wrapped for ROS 2 (D009: the core stays pure).

Subscribes
  /cmd_vel            geometry_msgs/Twist      (teleop_twist_joy-compatible)
  /pebble/gait_cmd    rocky_msgs/GaitCommand   (mode + arm-leg selection)
  /pebble/body_pose   rocky_msgs/BodyPoseCommand
Publishes
  /leg_position_controller/commands  std_msgs/Float64MultiArray  (15 leg joints)
  /claw_position_controller/commands std_msgs/Float64MultiArray  (5 claws)

Joint order matches rocky_control/config/controllers.yaml:
  [yaw0 hip0 knee0 yaw1 ... knee4]  and  [claw0..claw4]

Requires `pip install -e .` at the repo root (installs pebble_gait +
rocky_driver as plain Python packages) — see ros2/README.md.

D052 V2: the gait defaults come from cad/params.yaml (via rocky_model), every
command goes through the gait's budget() (logged when it scales), the
published targets are rate-clamped at the hard 4.7 rad/s and a non-finite
target holds the last good one. BodyPoseCommand.height_m is an offset from
params gait.body_height (the gait is rebuilt when it changes); its lean /
roll / pitch / yaw fields are reserved and not applied yet. Not here: the
reflex supervisor (it needs the IMU and contacts this node does not
subscribe to) — see ros2/README.md. Not run under ROS in CI (no rclpy).

D063: the budgeted command reaches the gait through pebble_gait.CommandSlew
(25 mm/s^2 on the fastest foot + a 0.2 s lag), so a new /cmd_vel eases in
instead of moving a joint target up to 28 deg in one tick, and a zero
/cmd_vel slows down smoothly. A stop is never eased: IDLE / SLEEP / MANIP and
a non-finite command zero it at once, and the gait restarts from standing.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from rocky_msgs.msg import GaitCommand, BodyPoseCommand

import numpy as np

import rocky_model as rm
from pebble_gait import WaveGait, ArmedGait, CommandSlew, stance_manip_targets

RATE_HZ = 50.0
MM = 1e-3
HARD_RAD_S = rm.servo_speed("hard")                  # 4.7: no target moves faster (V2)


class GaitNode(Node):
    def __init__(self):
        super().__init__("pebble_gait")
        d = rm.gait_defaults()                           # V2: params, not the legacy 1.6 s / 32 mm
        self.declare_parameter("cycle_time", d["cycle_time"])
        self.declare_parameter("body_height_mm", d["body_height"])
        self.declare_parameter("stance_radius_mm", d["stance_radius"])
        self.declare_parameter("step_height_mm", d["step_height"])
        self.declare_parameter("duty", d["duty"])
        self._q_last = None
        self._scale_warn_t = -1e9

        self.mode = GaitCommand.MODE_IDLE
        self.arm_legs = (0,)
        self.cmd = [0.0, 0.0, 0.0]                      # vx, vy (m/s), wz
        self.pose = BodyPoseCommand()
        self._gait = self._make_gait()
        self._slew = CommandSlew(self._gait)            # D063: every walk command is eased
        self._t = 0.0

        self.pub_legs = self.create_publisher(
            Float64MultiArray, "/leg_position_controller/commands", 10)
        self.pub_claws = self.create_publisher(
            Float64MultiArray, "/claw_position_controller/commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
        self.create_subscription(GaitCommand, "/pebble/gait_cmd", self.on_gait, 10)
        self.create_subscription(BodyPoseCommand, "/pebble/body_pose",
                                 self.on_pose, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info("pebble gait node up (idle) — publish /cmd_vel")

    # ------------------------------------------------------------ callbacks
    def _make_gait(self):
        p = self.get_parameter
        kw = dict(body_height=p("body_height_mm").value + self.pose.height_m / MM,
                  stance_radius=p("stance_radius_mm").value,
                  cycle_time=p("cycle_time").value,
                  duty=p("duty").value,
                  step_height=p("step_height_mm").value)
        if self.mode == GaitCommand.MODE_ARMED_WALK:
            return ArmedGait(arm_legs=self.arm_legs, **kw)
        return WaveGait(**kw)

    def _set_cmd(self, cmd):
        if all(math.isfinite(c) for c in cmd):
            self.cmd = cmd
        else:                                            # garbage in = a stop, never eased
            self.cmd = [0.0, 0.0, 0.0]
            self._slew.stop()

    def on_twist(self, msg: Twist):
        self._set_cmd([float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)])
        if self.mode == GaitCommand.MODE_IDLE and any(abs(c) > 1e-3 for c in self.cmd):
            self.mode = GaitCommand.MODE_WALK

    def on_gait(self, msg: GaitCommand):
        if msg.mode != self.mode or tuple(msg.arm_legs) != self.arm_legs:
            self.mode = msg.mode
            if msg.arm_legs:
                self.arm_legs = tuple(int(a) for a in msg.arm_legs)
            self._gait = self._make_gait()
            self._slew.g = self._gait                    # same command, the new gait's feet
        self._set_cmd([float(msg.vx), float(msg.vy), float(msg.wz)])

    def on_pose(self, msg: BodyPoseCommand):
        """height_m is applied (an offset from params gait.body_height; the gait
        is rebuilt); lean/roll/pitch/yaw are reserved and ignored for now."""
        if not math.isfinite(msg.height_m):
            return
        changed = abs(float(msg.height_m) - float(self.pose.height_m)) > 1e-6
        self.pose = msg
        if changed:
            self._gait = self._make_gait()
            self._slew.g = self._gait

    # ------------------------------------------------------------ main loop
    def tick(self):
        self._t += 1.0 / RATE_HZ
        if self.mode in (GaitCommand.MODE_IDLE, GaitCommand.MODE_SLEEP):
            self._slew.stop()                            # D063: not walking = a stop, never eased
            return                                       # hold last / limp
        if self.mode == GaitCommand.MODE_MANIP:
            self._slew.stop()
            q, claw = stance_manip_targets(self._gait, self._t,
                                           arm_legs=self.arm_legs or (0, 2))
        else:
            ask = (self.cmd[0] / MM, self.cmd[1] / MM, self.cmd[2])
            vx, vy, wz = self._gait.budget(*ask)         # V2: fitted into the gait's envelope
            if max(abs(a - b) for a, b in zip(ask, (vx, vy, wz))) > 1e-6 and self._t - self._scale_warn_t > 2.0:
                self._scale_warn_t = self._t
                self.get_logger().warn(f"cmd {tuple(round(a, 3) for a in ask)} scaled to the envelope "
                                       f"{(round(vx, 1), round(vy, 1), round(wz, 3))} (mm/s, mm/s, rad/s)")
            vx, vy, wz = self._slew.step((vx, vy, wz), 1.0 / RATE_HZ)   # D063: eased (a stop is not)
            q, stance, _ = self._gait.joint_targets(self._t, vx, vy, wz)
            claw = [0.0] * 5                             # feet stay cones
        q = np.asarray(q, float).reshape(5, 3)
        if not np.isfinite(q).all():
            self.get_logger().warn("unreachable target — holding")
            return
        if self._q_last is not None:                     # V2: no target faster than the servo
            step = HARD_RAD_S / RATE_HZ
            q = np.clip(q, self._q_last - step, self._q_last + step)
        self._q_last = q
        self.pub_legs.publish(Float64MultiArray(
            data=[float(v) for row in q for v in row]))
        self.pub_claws.publish(Float64MultiArray(
            data=[math.radians(55.0) * float(c) for c in claw]))


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
