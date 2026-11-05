"""Trajectory following without ROS or MAVLink. Units: m, s, rad."""
import math

import pytest

from coybot_drone.trajectory_follow import Follower, to_ned


def straight(v=2.0, T=2.0, dt=0.5):
    t = [i * dt for i in range(int(T / dt) + 1)]
    return t, [(v * s, 0.0, 2.5) for s in t], [(v, 0.0, 0.0) for _ in t], [(0.0, 0.0, 0.0) for _ in t]


def test_interpolates_between_samples():
    f = Follower()
    f.set_trajectory(100.0, *straight())
    sp = f.setpoint(100.75)
    assert sp.active
    assert sp.pos == pytest.approx((1.5, 0.0, 2.5))
    assert sp.vel == pytest.approx((2.0, 0.0, 0.0))


def test_holds_at_rest_after_the_end():
    f = Follower(hold_s=2.0)
    f.set_trajectory(100.0, *straight())
    sp = f.setpoint(103.0)
    assert not sp.active
    assert sp.pos == pytest.approx((4.0, 0.0, 2.5))
    assert sp.vel == (0.0, 0.0, 0.0) and sp.acc == (0.0, 0.0, 0.0)
    assert f.active(103.9) and not f.active(104.1)


def test_yaw_follows_velocity_and_is_kept_when_slow():
    f = Follower(yaw_speed_m_s=0.5)
    t = [0.0, 1.0]
    f.set_trajectory(0.0, t, [(0, 0, 2), (0, 1, 2)], [(0, 1, 0), (0, 1, 0)], [(0, 0, 0)] * 2)
    assert f.setpoint(0.5).yaw == pytest.approx(math.pi / 2)       # moving +y (west): 90 deg anticlockwise
    f.set_trajectory(1.0, t, [(0, 1, 2), (0, 1, 2)], [(0, 0.1, 0), (0, 0, 0)], [(0, 0, 0)] * 2)
    assert f.setpoint(1.5).yaw == pytest.approx(math.pi / 2)       # slow: keeps the last yaw


def test_ned_conversion():
    f = Follower()
    f.set_trajectory(0.0, [0.0, 1.0], [(1, 2, 3), (1, 2, 3)], [(1, 1, 1), (1, 1, 1)], [(0.5, 0.5, 0.5)] * 2)
    pos, vel, acc, yaw = to_ned(f.setpoint(0.5))
    assert pos == (1, -2, -3) and vel == (1, -1, -1) and acc == (0.5, -0.5, -0.5)
    assert yaw == pytest.approx(-math.pi / 4)                       # north-west in NWU = 315 deg in NED


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_rejects_non_finite(bad):
    f = Follower()
    with pytest.raises(ValueError):
        f.set_trajectory(0.0, [0.0, 1.0], [(0, 0, 2), (bad, 0, 2)], [(0, 0, 0)] * 2, [(0, 0, 0)] * 2)
    assert not f.has_trajectory()
