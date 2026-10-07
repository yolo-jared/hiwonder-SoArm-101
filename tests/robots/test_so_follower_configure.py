from contextlib import contextmanager
from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def _bus_mock(**kwargs) -> MagicMock:
    bus = MagicMock(name="BusMock")
    bus.motors = kwargs["motors"]

    @contextmanager
    def _dummy_cm():
        yield

    bus.torque_disabled.side_effect = _dummy_cm
    return bus


def _p_writes(motor_model: str) -> dict[str, int]:
    target = "lerobot.motors.hiwonder.HiwonderMotorsBus" if motor_model == "hx30hm" else "lerobot.motors.feetech.FeetechMotorsBus"
    with patch(target, side_effect=lambda *a, **kw: _bus_mock(**kw)):
        robot = SO101Follower(SO101FollowerConfig(port="/dev/null", motor_model=motor_model))
    robot.configure()
    return {c.args[1]: c.args[2] for c in robot.bus.write.call_args_list if c.args[0] == "P_Coefficient"}


def test_hx30hm_gravity_joints_get_p32():
    # P16 -> P32 halves hold error on every arm joint (elbow 55 -> 27 ticks); gripper untested on a grasp (audit/LESSONS.md)
    assert _p_writes("hx30hm") == {
        "shoulder_pan": 32,
        "shoulder_lift": 32,
        "elbow_flex": 32,
        "wrist_flex": 32,
        "wrist_roll": 32,
        "gripper": 16,
    }


@pytest.mark.parametrize("motor_model", ["sts3215"])
def test_feetech_keeps_p16_everywhere(motor_model):
    assert set(_p_writes(motor_model).values()) == {16}
