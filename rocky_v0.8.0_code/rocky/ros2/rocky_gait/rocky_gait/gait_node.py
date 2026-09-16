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
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from rocky_msgs.msg import GaitCommand, BodyPoseCommand

from pebble_gait import WaveGait, ArmedGait, stance_manip_targets

RATE_HZ = 50.0
MM = 1e-3


class GaitNode(Node):
    def __init__(self):
        super().__init__("pebble_gait")
        self.declare_parameter("cycle_time", 1.6)
        self.declare_parameter("body_height_mm", 118.0)
        self.declare_parameter("stance_radius_mm", 185.0)
        self.declare_parameter("step_height_mm", 32.0)
        self.declare_parameter("max_speed_mps", 0.06)

        self.mode = GaitCommand.MODE_IDLE
        self.arm_legs = (0,)
        self.cmd = [0.0, 0.0, 0.0]                      # vx, vy (m/s), wz
        self.pose = BodyPoseCommand()
        self._gait = self._make_gait()
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
                  step_height=p("step_height_mm").value)
        if self.mode == GaitCommand.MODE_ARMED_WALK:
            return ArmedGait(arm_legs=self.arm_legs, **kw)
        return WaveGait(**kw)

    def on_twist(self, msg: Twist):
        vmax = self.get_parameter("max_speed_mps").value
        self.cmd = [max(-vmax, min(vmax, msg.linear.x)),
                    max(-vmax, min(vmax, msg.linear.y)),
                    max(-0.6, min(0.6, msg.angular.z))]
        if self.mode == GaitCommand.MODE_IDLE and any(abs(c) > 1e-3 for c in self.cmd):
            self.mode = GaitCommand.MODE_WALK

    def on_gait(self, msg: GaitCommand):
        if msg.mode != self.mode or tuple(msg.arm_legs) != self.arm_legs:
            self.mode = msg.mode
            if msg.arm_legs:
                self.arm_legs = tuple(int(a) for a in msg.arm_legs)
            self._gait = self._make_gait()
        self.cmd = [msg.vx, msg.vy, msg.wz]

    def on_pose(self, msg: BodyPoseCommand):
        self.pose = msg

    # ------------------------------------------------------------ main loop
    def tick(self):
        self._t += 1.0 / RATE_HZ
        if self.mode in (GaitCommand.MODE_IDLE, GaitCommand.MODE_SLEEP):
            return                                       # hold last / limp
        if self.mode == GaitCommand.MODE_MANIP:
            q, claw = stance_manip_targets(self._gait, self._t,
                                           arm_legs=self.arm_legs or (0, 2))
        else:
            vx, vy, wz = self.cmd[0] / MM, self.cmd[1] / MM, self.cmd[2]
            q, stance, _ = self._gait.joint_targets(self._t, vx, vy, wz)
            claw = [0.0] * 5                             # feet stay cones
        for row in q:
            for v in row:
                if math.isnan(v):
                    self.get_logger().warn("unreachable target — holding")
                    return
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
