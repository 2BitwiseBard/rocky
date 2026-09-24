"""SoftStream — a safe leg-target stream for a bare control loop (D052 V2).

The cockpit's sim/hw_bridge.py carries the D052 rules for putting targets on
the bus; the ROS 2 path (ros2/rocky_driver/scripts/hw_bridge_node.py) had
none of them: torque on at startup toward whatever goal the servos held, the
first command at servo-max speed and full torque, no rate limit, NaN or a
missing joint silently 0.0 rad. This is the same set of rules, pure Python,
no threads, so a loop that owns its own timing can use it:

    s = SoftStream(robot)          # PebbleRobot
    s.enable()                     # soft_enable: goal parked where each servo IS, 40 % torque
    every tick: s.step(q5x3, now)  # blend -> NaN hold -> rate clamp -> send

  * Entry: the first commanded pose is reached by a smoothstep from the
    measured pose over max(blend_s, 1.5 * gap / (0.8 * entry speed)) at the
    entry speed and 40 % torque; the limits are released tail_s after it.
  * NaN / inf anywhere in a leg: that leg keeps its last sent target.
  * Rate clamp: no joint target moves faster than hard_rad_s (the ST3215's
    no-load 4.7 rad/s, params joints.vel_rad_s.hard) per real second.

Not here (and not on the ROS path yet): the reflex supervisor, foot
contacts, the heartbeat — see ros2/README.md.
"""
from __future__ import annotations

import math

import numpy as np

HARD_RAD_S = 4.7            # params joints.vel_rad_s.hard (ST3215 no-load, 12 V)
BLEND_S = 1.5               # same numbers as sim/hw_bridge.py's soft entry
ENTRY_SPEED_CPS = 200
ENTRY_TORQUE_LIMIT = 400
ENTRY_ACC = 10
TAIL_S = 1.0


def _smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


class SoftStream:
    def __init__(self, robot, hard_rad_s=HARD_RAD_S, blend_s=BLEND_S,
                 entry_speed_cps=ENTRY_SPEED_CPS, tail_s=TAIL_S, run_speed_cps=0):
        self.robot = robot
        self.hard = float(hard_rad_s)
        self.blend_s = float(blend_s)
        self.entry_cps = int(entry_speed_cps)
        self.tail_s = float(tail_s)
        self.run_cps = int(run_speed_cps)
        self.q_last = None          # (5,3) rad: the last target sent (or the measured pose)
        self.entry = None           # (t0, q_from (5,3), T)
        self.released = False
        self.t_prev = None
        self.nan_holds = 0
        self.present = {}

    def enable(self):
        """Park + enable (PebbleRobot.soft_enable) and remember where the legs
        are. Returns the ids that answered; the rest stay limp."""
        r = self.robot
        ids = [i for row in r.leg_ids for i in row]
        self.present = r.soft_enable(ids, torque_limit=ENTRY_TORQUE_LIMIT, acc=ENTRY_ACC,
                                     speed_cps=self.entry_cps)
        q = np.zeros((5, 3))
        for leg, row in enumerate(r.leg_ids):
            for j, sid in enumerate(row):
                if sid in self.present:
                    q[leg, j] = r.deg_to_q(leg, j, self.present[sid])
                else:
                    q[leg, j] = np.nan              # never commanded until it has a target
        self.q_last = q
        self.entry = None
        self.released = False
        self.t_prev = None
        return sorted(self.present)

    def step(self, q_cmd, now):
        """One tick: returns the (5,3) rad targets sent, or None (nothing sent)."""
        if q_cmd is None or self.q_last is None:
            return None
        q = np.array(q_cmd, float).reshape(5, 3)
        bad = ~np.isfinite(q).all(axis=1)
        if bad.any():
            self.nan_holds += 1
            q[bad] = self.q_last[bad]
        if self.entry is None:
            known = np.isfinite(self.q_last)
            q_from = np.where(known, self.q_last, q)
            gap = float(np.max(np.abs(q - q_from))) if q.size else 0.0
            v = 0.8 * self.entry_cps * 2.0 * math.pi / 4096.0          # rad/s the servo surely follows
            self.entry = (now, q_from, max(self.blend_s, 1.5 * gap / v))
            self.q_last = q_from.copy()
        t0, q_from, T = self.entry
        s = _smooth((now - t0) / T)
        q_t = q_from + s * (q - q_from)
        dt = 0.02 if self.t_prev is None else min(max(now - self.t_prev, 1e-3), 0.1)
        step = self.hard * dt
        q_t = np.clip(q_t, self.q_last - step, self.q_last + step)
        in_entry = now - t0 < T + self.tail_s
        if not in_entry and not self.released:
            self.robot.release_limits([i for row in self.robot.leg_ids for i in row])
            self.released = True
        self.robot.send_leg_targets(q_t, self.entry_cps if in_entry else self.run_cps)
        self.q_last = q_t
        self.t_prev = now
        return q_t
