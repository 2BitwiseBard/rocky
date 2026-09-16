#!/usr/bin/env python3
"""Hardware bridge — the hw:=topic backend for ros2_control.

topic_based_ros2_control turns the controller side into two topics; this node
owns the actual Feetech bus through the TESTED pure-Python driver stack
(driver/rocky_driver — 58 tests, mock rehearsals, safety monitor). Zero C++
between the gait and the servos for Phase 1–2.

  subscribe /pebble/joint_commands   sensor_msgs/JointState (positions)
  publish   /pebble/joint_states_hw  sensor_msgs/JointState @ 50 Hz
  publish   /pebble/servo_health     rocky_msgs/ServoHealth @ 2 Hz
  publish   /pebble/contacts         rocky_msgs/ContactState @ 50 Hz (stub
                                     until SEA microswitches wire in, D010)

Run with --ros-args -p mock:=true for the full-loop rehearsal (no hardware).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rocky_msgs.msg import ServoHealth, ContactState

from rocky_driver import (FeetechBus, PebbleRobot, SerialTransport,
                          make_pebble_mock, load_calibration)

JOINTS = [f"{j}{i}" for i in range(5) for j in ("yaw", "hip", "knee")]
CLAWS = [f"claw{i}" for i in range(5)]


class HwBridge(Node):
    def __init__(self):
        super().__init__("pebble_hw_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("mock", False)
        self.declare_parameter("calibration", "bench/calibration.yaml")

        if self.get_parameter("mock").value:
            transport = make_pebble_mock()
            self.get_logger().warn("MOCK bus — no hardware will move")
        else:
            transport = SerialTransport(self.get_parameter("port").value,
                                        self.get_parameter("baud").value)
        bus = FeetechBus(transport)
        cal = load_calibration(self.get_parameter("calibration").value)
        self.robot = PebbleRobot(bus, calibration=cal)
        self.robot.enable(True)

        self._q_cmd = None
        self._claw_cmd = [0.0] * 5
        self.create_subscription(JointState, "/pebble/joint_commands",
                                 self.on_cmd, 10)
        self.pub_js = self.create_publisher(JointState,
                                            "/pebble/joint_states_hw", 10)
        self.pub_health = self.create_publisher(ServoHealth,
                                                "/pebble/servo_health", 10)
        self.pub_contact = self.create_publisher(ContactState,
                                                 "/pebble/contacts", 10)
        self.create_timer(1 / 50.0, self.io_tick)
        self.create_timer(1 / 2.0, self.health_tick)

    def on_cmd(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        q = [[by_name.get(f"{j}{i}", 0.0) for j in ("yaw", "hip", "knee")]
             for i in range(5)]
        self._q_cmd = q
        self._claw_cmd = [by_name.get(f"claw{i}", 0.0) / math.radians(55.0)
                          for i in range(5)]

    def io_tick(self):
        if self._q_cmd is not None:
            self.robot.send_leg_targets(self._q_cmd)
            self.robot.send_claws([max(0.0, min(1.0, c))
                                   for c in self._claw_cmd])
        q, tels = self.robot.read_joint_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + CLAWS
        msg.position = [q[i][j] for i in range(5) for j in range(3)] + \
                       [0.0] * 5                      # claw state TODO (v0.3)
        self.pub_js.publish(msg)
        # SEA microswitch contacts arrive with Batch-1 wiring; stub = all True
        c = ContactState()
        c.header.stamp = msg.header.stamp
        c.in_contact = [True] * 5
        c.spring_mm = [0.0] * 5
        self.pub_contact.publish(c)

    def health_tick(self):
        tels = self.robot.health_step()
        msg = ServoHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        for sid in sorted(tels):
            t = tels[sid]
            msg.ids.append(sid)
            msg.temp_c.append(float(t.temp_c))
            msg.voltage_v.append(t.voltage_v)
            msg.load_pct.append(t.load_pct)
            msg.current_a.append(t.current_a if t.current_a is not None
                                 else float("nan"))
            msg.fault_bits.append(0 if not t.faults else 1)
            msg.torque_cut.append(sid in self.robot.monitor.tripped)
        self.pub_health.publish(msg)
        for sid in self.robot.monitor.tripped:
            self.get_logger().error(f"servo {sid} TORQUE-CUT (thermal/fault)")


def main(args=None):
    rclpy.init(args=args)
    node = HwBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.limp()          # never leave servos fighting gravity
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
