"""Follow a planned trajectory with MAVLink position targets (no ROS, no MAVLink in here: testable on its own).

A trajectory is what coybot_avoidance publishes (trajectory_msgs/MultiDOFJointTrajectory): a start time, and
samples of position, velocity and acceleration at times from the start, in the bridge's odom frame
(x north, y west, z up; see mavlink_bridge.py). ``Follower.setpoint(now)`` linearly interpolates them; after the
end it holds the last position at rest. ``to_ned`` turns a setpoint into the values of a
SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_LOCAL_NED, which PX4 (OFFBOARD) and ArduPilot (GUIDED) both accept.

Yaw: along the horizontal velocity when faster than ``yaw_speed_m_s``, otherwise the last yaw (the camera looks
where the vehicle goes). Units: m, s, rad.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Vec = Tuple[float, float, float]

# SET_POSITION_TARGET_LOCAL_NED type_mask: use position, velocity, acceleration and yaw; ignore yaw rate (bit 11).
TYPE_MASK_PVA_YAW = 0b0000100000000000
MAV_FRAME_LOCAL_NED = 1


@dataclass
class Setpoint:
    pos: Vec
    vel: Vec
    acc: Vec
    yaw: float          # rad, in the odom frame (x north, y west, z up): counter-clockwise from north
    active: bool        # False after the end (holding at rest)


@dataclass
class Follower:
    yaw_speed_m_s: float = 0.5
    hold_s: float = 2.0                       # how long after the end the follower still counts as active
    start_s: float = 0.0
    t: List[float] = field(default_factory=list)
    p: List[Vec] = field(default_factory=list)
    v: List[Vec] = field(default_factory=list)
    a: List[Vec] = field(default_factory=list)
    yaw: Optional[float] = None

    def set_trajectory(self, start_s: float, t: Sequence[float], p: Sequence[Vec], v: Sequence[Vec],
                       a: Sequence[Vec]) -> None:
        if not (len(t) == len(p) == len(v) == len(a)) or len(t) < 1:
            raise ValueError("trajectory needs equal, non-empty time, position, velocity and acceleration lists")
        if any(t[i + 1] < t[i] for i in range(len(t) - 1)):
            raise ValueError("trajectory times must not decrease")
        vals = [x for q in (*p, *v, *a) for x in q] + list(t) + [start_s]
        if not all(math.isfinite(x) for x in vals):
            raise ValueError("trajectory is not finite")
        self.start_s, self.t, self.p, self.v, self.a = start_s, list(t), list(p), list(v), list(a)

    def has_trajectory(self) -> bool:
        return bool(self.t)

    def active(self, now: float) -> bool:
        return bool(self.t) and now <= self.start_s + self.t[-1] + self.hold_s

    def setpoint(self, now: float) -> Optional[Setpoint]:
        if not self.t:
            return None
        s = now - self.start_s
        if s >= self.t[-1]:
            pos, vel, acc, live = self.p[-1], (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False
        elif s <= self.t[0]:
            pos, vel, acc, live = self.p[0], self.v[0], self.a[0], True
        else:
            i = bisect.bisect_right(self.t, s) - 1
            f = (s - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
            lerp = lambda x, y: tuple((1 - f) * x[k] + f * y[k] for k in range(3))  # noqa: E731
            pos, vel, acc, live = lerp(self.p[i], self.p[i + 1]), lerp(self.v[i], self.v[i + 1]), \
                lerp(self.a[i], self.a[i + 1]), True
        if math.hypot(vel[0], vel[1]) > self.yaw_speed_m_s:
            self.yaw = math.atan2(vel[1], vel[0])
        return Setpoint(pos, vel, acc, self.yaw if self.yaw is not None else 0.0, live)


def to_ned(sp: Setpoint) -> Tuple[Vec, Vec, Vec, float]:
    """(position, velocity, acceleration, yaw) for SET_POSITION_TARGET_LOCAL_NED, from the odom frame
    (x north, y west, z up) to NED (x north, y east, z down; yaw clockwise from north)."""
    f = lambda q: (q[0], -q[1], -q[2])  # noqa: E731
    return f(sp.pos), f(sp.vel), f(sp.acc), -sp.yaw
