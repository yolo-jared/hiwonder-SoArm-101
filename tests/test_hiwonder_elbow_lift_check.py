"""Software-only checks: these tests never open a serial port."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from examples.hiwonder.elbow_lift_check import (
    ELBOW,
    LIFT_TICKS,
    check_feedback,
    infer_direction,
    preload_and_enable,
    require_rest,
    target_at,
    validate_path,
)


def pose():
    return {
        "shoulder_pan": 2000,
        "shoulder_lift": 2000,
        "elbow_flex": 2000,
        "wrist_flex": 2000,
        "wrist_roll": 2000,
        "gripper": 2000,
    }


@pytest.mark.parametrize("sign", [-1, 1])
def test_teaching_requires_elbow_movement_and_preserves_sign(sign):
    before = pose()
    after = {**before, ELBOW: before[ELBOW] + sign * 80}
    assert infer_direction(before, after) == sign


@pytest.mark.parametrize(
    "changes",
    [
        {},
        {ELBOW: 2001},
        {ELBOW: 2800},
        {ELBOW: 2080, "shoulder_lift": 2100},
        {ELBOW: 2080, "wrist_flex": 2100},
    ],
)
def test_ambiguous_or_excessive_teaching_is_rejected(changes):
    with pytest.raises(ValueError):
        infer_direction(pose(), {**pose(), **changes})


@pytest.mark.parametrize("sign", [-1, 1])
def test_trajectory_starts_at_rest_lifts_only_elbow_and_returns(sign):
    baseline = pose()
    samples = [target_at(baseline, sign, i / 10) for i in range(91)]
    assert len(samples) == 91
    assert samples[0] == baseline == samples[-1]
    offsets = [sign * (p[ELBOW] - baseline[ELBOW]) for p in samples]
    assert max(offsets) == LIFT_TICKS
    assert min(offsets) == 0
    assert all(abs(b - a) <= 3 for a, b in zip(offsets, offsets[1:], strict=False))
    assert offsets[:31] == sorted(offsets[:31])
    assert offsets[40:71] == sorted(offsets[40:71], reverse=True)
    for sample in samples:
        assert {k: v for k, v in sample.items() if k != ELBOW} == {
            k: v for k, v in baseline.items() if k != ELBOW
        }


@pytest.mark.parametrize("sign,low,high", [(1, 1000, 2020), (-1, 1980, 3000)])
def test_path_beyond_recorded_calibration_is_rejected(sign, low, high):
    with pytest.raises(ValueError):
        validate_path(pose(), sign, SimpleNamespace(range_min=low, range_max=high))


def test_valid_path_is_accepted():
    validate_path(pose(), 1, SimpleNamespace(range_min=1000, range_max=3000))


def test_preload_is_read_back_before_torque_enable():
    bus = Mock()
    bus.sync_read.side_effect = [pose(), pose()]
    preload_and_enable(bus, pose())
    assert [call[0] for call in bus.mock_calls] == ["sync_read", "sync_write", "sync_read", "enable_torque"]
    bus.sync_write.assert_called_once_with("Goal_Position", pose(), normalize=False)


def test_ignored_goal_write_never_enables_torque():
    bus = Mock()
    bus.sync_read.side_effect = [pose(), {**pose(), ELBOW: 2100}]
    with pytest.raises(RuntimeError):
        preload_and_enable(bus, pose())
    bus.enable_torque.assert_not_called()


def test_arm_moved_after_baseline_never_enables_torque():
    bus = Mock()
    bus.sync_read.return_value = {**pose(), ELBOW: 2100}
    with pytest.raises(RuntimeError):
        preload_and_enable(bus, pose())
    bus.enable_torque.assert_not_called()


@pytest.mark.parametrize(
    "actual,status",
    [
        ({**pose(), ELBOW: 1950}, {}),  # Wrong direction.
        ({**pose(), "shoulder_lift": 2050}, {}),  # Non-commanded joint drift.
        (pose(), {ELBOW: 1}),  # Motor reports fault.
    ],
)
def test_bad_feedback_aborts(actual, status):
    with pytest.raises(RuntimeError):
        check_feedback(pose(), {**pose(), ELBOW: 2020}, actual, 1, status)


def test_following_error_aborts():
    with pytest.raises(RuntimeError):
        check_feedback(pose(), {**pose(), ELBOW: 2056}, pose(), 1, {})


def test_expected_feedback_passes():
    target = {**pose(), ELBOW: 2020}
    check_feedback(pose(), target, target, 1, {})


@pytest.mark.parametrize("register", ["Torque_Enable", "Operating_Mode", "Status"])
def test_unsafe_initial_motor_state_is_read_only_and_rejected(register):
    bus = Mock(is_calibrated=True)
    bus.sync_read.side_effect = lambda name, **kwargs: {ELBOW: int(name == register)}
    with pytest.raises(RuntimeError):
        require_rest(bus)
    bus.enable_torque.assert_not_called()
    bus.sync_write.assert_not_called()


def test_calibration_mismatch_is_not_automatically_rewritten():
    bus = Mock(is_calibrated=False)
    bus.sync_read.return_value = {ELBOW: 0}
    with pytest.raises(RuntimeError):
        require_rest(bus)
    bus.write_calibration.assert_not_called()


@pytest.mark.parametrize("failure", [None, "partial_enable", "read_failure", "shutdown_failure"])
def test_operator_gates_and_cleanup_in_simulated_run(monkeypatch, tmp_path, failure):
    from serial.tools import list_ports

    from examples.hiwonder import elbow_lift_check as check
    from lerobot.robots.so_follower import so_follower
    from lerobot.teleoperators.so_leader import so_leader

    events = []
    clock = [0.0]

    class FakeBus:
        def __init__(self, name):
            self.name = name
            self.is_connected = False
            self.is_calibrated = True
            self.actual = pose()
            self.goal = pose()
            self.enabled = False
            self.ever_enabled = False

        def connect(self):
            self.is_connected = True

        def sync_read(self, register, normalize=False):
            assert normalize is False
            if register == "Present_Position":
                if failure == "read_failure" and self.enabled:
                    raise OSError("simulated serial failure")
                return self.actual.copy()
            if register == "Goal_Position":
                return self.goal.copy()
            return {k: int(self.enabled and register == "Torque_Enable") for k in pose()}

        def sync_write(self, register, target, normalize=False):
            assert register == "Goal_Position" and normalize is False
            self.goal = target.copy()
            if self.enabled:
                self.actual = target.copy()

        def enable_torque(self):
            assert events.count("enter") == 2
            assert self.name == "follower"
            assert self.goal == self.actual
            self.enabled = self.ever_enabled = True
            events.append("enabled")
            if failure == "partial_enable":
                raise OSError("simulated partial enable")

        def disable_torque(self, **kwargs):
            events.append("disable")
            if failure == "shutdown_failure":
                raise RuntimeError("motor ignored torque-off")
            self.enabled = False

        def disconnect(self, disable_torque=False):
            self.is_connected = False

    leader_bus, follower_bus = FakeBus("leader"), FakeBus("follower")
    calibration = {ELBOW: SimpleNamespace(range_min=1000, range_max=3000)}
    monkeypatch.setattr(
        so_leader, "SOLeader", lambda config: SimpleNamespace(bus=leader_bus, calibration=calibration)
    )
    monkeypatch.setattr(
        so_follower, "SOFollower", lambda config: SimpleNamespace(bus=follower_bus, calibration=calibration)
    )
    monkeypatch.setattr(
        list_ports,
        "comports",
        lambda: [SimpleNamespace(device=p, serial_number=p) for p in ("leader", "follower")],
    )
    monkeypatch.setattr("builtins.input", lambda prompt: events.append("enter"))
    monkeypatch.setattr(check.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(check.time, "sleep", lambda duration: clock.__setitem__(0, clock[0] + duration))

    def fake_say(message):
        events.append(message)
        if message.startswith("Lift the leader"):
            assert not follower_bus.enabled
            leader_bus.actual[ELBOW] += 80

    monkeypatch.setattr(check, "say", fake_say)
    monkeypatch.chdir(tmp_path)
    args = SimpleNamespace(
        leader_port="leader",
        follower_port="follower",
        leader_id="unused-leader",
        follower_id="unused-follower",
    )
    if failure:
        with pytest.raises((OSError, RuntimeError)):
            check.run(args)
    else:
        check.run(args)
        assert follower_bus.actual == pose()
        assert any("verified off" in event for event in events)
    assert follower_bus.ever_enabled
    assert "disable" in events
    assert not leader_bus.is_connected and not follower_bus.is_connected
    if failure == "shutdown_failure":
        assert any("Cut servo power now" in event for event in events)
        assert not any("verified off" in event for event in events)
    else:
        assert not follower_bus.enabled
