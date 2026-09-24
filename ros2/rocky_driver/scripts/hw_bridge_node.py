#!/usr/bin/env python3
"""Hardware bridge — the hw:=topic backend for ros2_control.

topic_based_ros2_control turns the controller side into two topics; this node
owns the actual Feetech bus through the TESTED pure-Python driver stack
(driver/rocky_driver — 58 tests, mock rehearsals, safety monitor). Zero C++
between the gait and the servos for Phase 1–2.

  subscribe /pebble/joint_commands   sensor_msgs/JointState (positions)
  publish   /pebble/joint_states_hw  sensor_msgs/JointState @ 50 Hz
  publish   /pebble/servo_health     rocky_msgs/ServoHealth @ 2 Hz
  (/pebble/contacts rocky_msgs/ContactState: NOT published until the SEA
                                     microswitches are wired, D010 — D052 V2)

Run with --ros-args -p mock:=true for the full-loop rehearsal (no hardware).

D052 V2 (review): this node had none of the D052 bus rules — torque on at
startup toward whatever goal the servos held, the first command at servo max
and full torque, no rate limit, a JointState missing a joint defaulted it to
0.0 rad, and the contact stub said all five feet were down. Now: the stream
goes through rocky_driver.SoftStream (goal parked where each servo IS at
40 % torque BEFORE enable, a smoothstep entry at 200 cps, NaN hold, a
4.7 rad/s rate clamp), a command missing any leg joint is ignored (the last
good one holds), and NO contact message is published until the SEA switches
are wired (absent > lying: a consumer that trusted all-True would walk off
a table). Still missing vs the cockpit path: the reflex supervisor, the
heartbeat, the monitor's leg-level cut — see ros2/README.md. NOT verified
under ROS here (no rclpy on the dev box); SoftStream is tested on the mock.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rocky_msgs.msg import ServoHealth, ContactState

from rocky_driver import (FeetechBus, PebbleRobot, SerialTransport, SoftStream,
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
        self.stream = SoftStream(self.robot)            # D052 V2: never robot.enable(True) first
        up = self.stream.enable()
        self.get_logger().info(f"soft entry: {len(up)} leg servos parked at their present pose, 40 % torque")
        self._t0 = self.get_clock().now()
        self._warn_t = -1e9

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

    def _now(self):
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def on_cmd(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        missing = [n for n in JOINTS if n not in by_name]
        if missing:                                      # V2: a missing joint is not 0.0 rad
            now = self._now()
            if now - self._warn_t > 1.0:
                self._warn_t = now
                self.get_logger().warn(f"joint command missing {missing} — ignored, holding the last one")
            return
        self._q_cmd = [[by_name[f"{j}{i}"] for j in ("yaw", "hip", "knee")] for i in range(5)]
        claws = [by_name.get(f"claw{i}") for i in range(5)]
        if all(c is not None and math.isfinite(c) for c in claws):
            self._claw_cmd = [c / math.radians(55.0) for c in claws]

    def io_tick(self):
        if self._q_cmd is not None:
            self.stream.step(self._q_cmd, self._now())   # entry blend, NaN hold, 4.7 rad/s clamp
            self.robot.send_claws([max(0.0, min(1.0, c))
                                   for c in self._claw_cmd])
        q, tels = self.robot.read_joint_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS + CLAWS
        msg.position = [q[i][j] for i in range(5) for j in range(3)] + \
                       [0.0] * 5                      # claw state TODO (v0.3)
        self.pub_js.publish(msg)
        # SEA microswitch contacts arrive with Batch-1 wiring. V2: nothing is
        # published until then — the old stub said all five feet were down.

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
