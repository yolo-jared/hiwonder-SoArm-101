"""Injected diagnostic failures; no motors or serial devices are used."""

import contextlib
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from examples.hiwonder import elbow_lift_check as check


@pytest.fixture
def flow(monkeypatch, tmp_path):
    from serial.tools import list_ports

    clock = [0.0]
    calls = []

    class Bus:
        motors = {"elbow_flex": object()}
        is_calibrated = True

        def __init__(self):
            self.is_connected = False
            self.enabled = False
            self.actual = 2000
            self.goal = 2000

        def connect(self):
            self.is_connected = True

        def disconnect(self, disable_torque=False):
            assert disable_torque is False
            self.is_connected = False

        def sync_read(self, register, normalize=False):
            values = {
                "Present_Position": self.actual,
                "Goal_Position": self.goal,
                "Torque_Enable": int(self.enabled),
            }
            return {"elbow_flex": values.get(register, 0)}

        def sync_write(self, register, value, normalize=False):
            calls.append(("goal", value["elbow_flex"]))
            self.goal = value["elbow_flex"]
            if self.enabled:
                self.actual = self.goal

        def read(self, register, motor, normalize=False, num_retry=0):
            return self.sync_read(register)[motor]

        def write(self, register, motor, value, **kwargs):
            calls.append(("torque", value))
            assert register == "Torque_Enable"
            self.enabled = bool(value)

        def enable_torque(self):
            calls.append(("enable",))
            self.enabled = True

    leader, follower = Bus(), Bus()
    calibration = {"elbow_flex": SimpleNamespace(range_min=1000, range_max=3000)}
    monkeypatch.setattr(
        check,
        "make_devices",
        lambda args: [SimpleNamespace(bus=b, calibration=calibration) for b in (leader, follower)],
    )
    monkeypatch.setattr(check.session, "BoundedBus", lambda bus: bus)
    monkeypatch.setattr(check.session, "DeviceLocks", lambda *a: contextlib.nullcontext())
    monkeypatch.setattr(check.session, "snapshot", lambda *a: {})
    monkeypatch.setattr(
        list_ports, "comports", lambda: [SimpleNamespace(device=p, serial_number=p) for p in ("L", "F")]
    )
    monkeypatch.setattr(check.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(check.time, "sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    monkeypatch.setattr("builtins.input", lambda p: "")

    def speak(text):
        calls.append(("speech", text))
        if text.startswith("Lift the leader"):
            leader.actual += 80

    monkeypatch.setattr(check, "say", speak)
    monkeypatch.chdir(tmp_path)
    args = SimpleNamespace(leader_port="L", follower_port="F", leader_id="l", follower_id="f", inspect=False)
    return SimpleNamespace(
        args=args, leader=leader, follower=follower, clock=clock, calls=calls, path=tmp_path, speak=speak
    )


def test_inspection_never_changes_initially_enabled_torque(flow):
    flow.args.inspect = True
    flow.follower.enabled = True
    result = check.run(flow.args)
    assert flow.follower.enabled
    assert not any(c[0] in ("goal", "torque", "enable", "speech") for c in flow.calls)
    assert not flow.follower.is_connected
    assert result["execution"] == "completed"


def test_configuration_change_never_enables(flow, monkeypatch):
    monkeypatch.setattr(check.session, "snapshot", Mock(side_effect=[{"p": 16}, {"p": 16}, {"p": 32}]))
    result = check.run(flow.args)
    assert result["execution"] == "rejected"
    assert not any(c[0] in ("goal", "enable", "torque") for c in flow.calls)


def test_rehearsal_rejection_logged_before_motion(flow, monkeypatch):
    monkeypatch.setattr(check, "say", lambda message: None)
    result = check.run(flow.args)
    path = next((flow.path / "test-logs").glob("*.log"))
    persisted = check.evidence.analyze_file(path)
    assert persisted["execution"] == result["execution"] == "rejected"
    assert result["encoder"] == "not attempted"
    assert not any(c[0] == "enable" for c in flow.calls)


@pytest.mark.parametrize("event", ["start", "baseline", "sample", "final"])
def test_disk_failure_still_cleans_up(flow, monkeypatch, event):
    original = check.evidence.Recorder.emit

    def emit(self, kind, **fields):
        if kind == event:
            raise OSError("disk full")
        return original(self, kind, **fields)

    monkeypatch.setattr(check.evidence.Recorder, "emit", emit)
    if event == "start":
        with pytest.raises(OSError):
            check.run(flow.args)
    else:
        result = check.run(flow.args)
        assert check.evidence.exit_code(result) != 0
    assert not flow.follower.enabled
    assert not flow.follower.is_connected
    if event in ("start", "baseline"):
        assert not any(c[0] == "enable" for c in flow.calls)
    else:
        assert ("torque", 0) in flow.calls


@pytest.mark.parametrize("where", ["before", "after"])
def test_speech_failure_never_fakes_motor_state(flow, monkeypatch, where):
    def speak(message):
        if where == "before" or message.startswith("Execution:"):
            raise OSError("speech unavailable")
        flow.speak(message)

    monkeypatch.setattr(check, "say", speak)
    result = check.run(flow.args)
    assert check.evidence.exit_code(result) == 1
    assert not flow.follower.enabled
    if where == "before":
        assert not any(c[0] == "enable" for c in flow.calls)
    else:
        assert result["shutdown"] == "verified off"
    assert result["physical_accepted"] is False


@pytest.mark.parametrize("where", ["feedback", "log"])
def test_aggregate_delay_aborts_without_stale_goal(flow, monkeypatch, where):
    original_read = flow.follower.read
    original_emit = check.evidence.Recorder.emit

    def read(*args, **kwargs):
        if flow.follower.enabled and where == "feedback":
            flow.clock[0] += 0.11
        return original_read(*args, **kwargs)

    def emit(self, kind, **fields):
        if kind == "sample" and where == "log":
            flow.clock[0] += 0.6
        return original_emit(self, kind, **fields)

    monkeypatch.setattr(flow.follower, "read", read)
    monkeypatch.setattr(check.evidence.Recorder, "emit", emit)
    result = check.run(flow.args)
    assert result["execution"] == "aborted"
    assert [c for c in flow.calls if c[0] == "goal"] == [("goal", 2000)]
    assert not flow.follower.enabled


def test_observation_only_after_off_and_linked_to_run(flow, monkeypatch):
    def answer(prompt):
        if prompt.startswith("Torque verified off"):
            assert not flow.follower.enabled
            return "n"
        return ""

    monkeypatch.setattr("builtins.input", answer)
    result = check.run(flow.args)
    assert result["observation"] == "no lift"
    assert check.evidence.exit_code(result) == 2
    path = next((flow.path / "test-logs").glob("*.log"))
    assert check.evidence.analyze_file(path)["observation"] == "no lift"


def test_final_record_failure_keeps_verified_motor_state_in_terminal(flow, monkeypatch):
    original = check.evidence.Recorder.emit

    def emit(self, kind, **fields):
        if kind == "final":
            raise OSError("finalization failed")
        return original(self, kind, **fields)

    monkeypatch.setattr(check.evidence.Recorder, "emit", emit)
    result = check.run(flow.args)
    assert result["shutdown"] == "verified off"
    assert result["physical_accepted"] is False
    assert check.evidence.exit_code(result) == 1


def test_negative_observation_is_spoken_not_only_printed(flow, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda p: "n" if p.startswith("Torque verified") else "")
    check.run(flow.args)
    assert any(c[0] == "speech" and "Observation: no lift" in c[1] for c in flow.calls)


def test_final_record_contains_measurements_and_cleanup(flow):
    check.run(flow.args)
    path = next((flow.path / "test-logs").glob("*.log"))
    records = [json.loads(line) for line in path.read_text().splitlines()]
    final = next(e for e in records if e["event"] == "final")
    assert final["requested_ticks"] == 56
    assert final["peak_ticks"] == 56
    assert final["hold_median_ticks"] == 56
    assert final["encoder"] == "passed"
    assert final["shutdown"] == "verified off"


def test_inspect_read_errors_are_not_success(flow, monkeypatch):
    flow.args.inspect = True
    monkeypatch.setattr(
        check.session,
        "snapshot",
        lambda *args: {
            "motors": {"elbow_flex": {"P_Coefficient": {"state": "error", "message": "unplugged"}}}
        },
    )
    result = check.run(flow.args)
    assert result["errors"]
    assert not any(c[0] in ("goal", "torque", "enable") for c in flow.calls)
