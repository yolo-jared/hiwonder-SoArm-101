"""Follower-only diagnostic failures. No serial ports or cameras are opened."""

import copy
import hashlib
import json
from types import SimpleNamespace

import pytest

from examples.hiwonder import follower_lift_check as check


@pytest.fixture
def record(tmp_path):
    pose = dict.fromkeys(check.JOINTS, 2000)
    source = tmp_path / "evidence.jsonl"
    source.write_text("independent passive observation")
    snap = {
        "calibration_hash": "cal",
        "motors": {
            n: {
                "id": i,
                "configured_model": "hx30hm",
                "Model_Number": {"state": "read", "raw": 777},
                "Firmware_Major_Version": {"state": "read", "raw": 3},
                "Firmware_Minor_Version": {"state": "read", "raw": 15},
            }
            for i, n in enumerate(check.JOINTS, 1)
        },
    }
    cert = {
        "version": 1,
        "serial": "F",
        "calibration_hash": "cal",
        "motor_identity": check.motor_identity(snap),
        "direction": -1,
        "rest_pose": pose,
        "manual_before": pose,
        "manual_lift": {**pose, "elbow_flex": 1920},
        "manual_return": pose,
        "reviewed_upward": True,
        "camera_name": "icspring camera",
        "sources": [{"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}],
    }
    return cert, snap, pose


def test_matching_passive_certificate_is_eligible(record):
    cert, snap, pose = record
    assert check.validate_certificate(cert, "F", snap, pose) == -1


@pytest.mark.parametrize(
    "change",
    [
        "serial",
        "calibration",
        "firmware",
        "source",
        "pose",
        "missing_joint",
        "sign",
        "review",
        "other_joint",
        "return",
        "tiny_lift",
        "nan",
        "bool",
    ],
)
def test_changed_or_invalid_direction_evidence_rejects(record, change):
    cert, snap, pose = copy.deepcopy(record)
    if change == "serial":
        cert["serial"] = "other"
    elif change == "calibration":
        cert["calibration_hash"] = "other"
    elif change == "firmware":
        snap["motors"]["elbow_flex"]["Firmware_Minor_Version"]["raw"] = 16
    elif change == "source":
        cert["sources"][0]["sha256"] = "wrong"
    elif change == "pose":
        pose = {**pose, "wrist_flex": 2040}
    elif change == "missing_joint":
        pose = {"elbow_flex": 2000}
    elif change == "sign":
        cert["direction"] = 1
    elif change == "review":
        cert["reviewed_upward"] = False
    elif change == "other_joint":
        cert["manual_lift"]["shoulder_lift"] = 2080
    elif change == "return":
        cert["manual_return"] = {**pose, "elbow_flex": 1950}
    elif change == "tiny_lift":
        cert["manual_lift"]["elbow_flex"] = 1999
    elif change == "nan":
        cert["rest_pose"] = {**pose, "elbow_flex": float("nan")}
    elif change == "bool":
        cert["direction"] = True
    with pytest.raises((ValueError, RuntimeError)):
        check.validate_certificate(cert, "F", snap, pose)


@pytest.mark.parametrize(
    "change", ["expired", "future", "fourth", "identity", "certificate", "repeat", "unauthorized"]
)
def test_session_budget_rejects_before_enable(tmp_path, change):
    state = {"authorized": True, "started_at": 100, "serial": "F", "certificate_sha256": "C", "trials": []}
    if change == "expired":
        state["started_at"] = 0
    if change == "future":
        state["started_at"] = 9999
    if change == "fourth":
        state["trials"] = [{"purpose": str(i)} for i in range(3)]
    if change == "identity":
        state["serial"] = "other"
    if change == "certificate":
        state["certificate_sha256"] = "other"
    if change == "repeat":
        state["trials"] = [{"purpose": "B initial"}]
    if change == "unauthorized":
        state["authorized"] = False
    p = tmp_path / "budget.json"
    p.write_text(json.dumps(state))
    with pytest.raises((ValueError, RuntimeError)):
        check.reserve_trial(p, "F", "C", "B initial", now=650)
    assert json.loads(p.read_text()) == state


def test_trial_budget_is_durable(tmp_path):
    p = tmp_path / "budget.json"
    p.write_text(
        json.dumps(
            {"authorized": True, "started_at": 100, "serial": "F", "certificate_sha256": "C", "trials": []}
        )
    )
    check.reserve_trial(p, "F", "C", "B initial", now=110)
    assert json.loads(p.read_text())["trials"] == [{"purpose": "B initial", "reserved_at": 110}]


@pytest.fixture
def hardware(monkeypatch):
    pose = dict.fromkeys(check.JOINTS, 2000)
    clock = [0.0]
    writes = []

    class Bus:
        motors = dict.fromkeys(check.JOINTS)
        enabled = False
        goal = pose.copy()
        actual = pose.copy()

        def sync_read(self, field, normalize=False):
            if field == "Present_Position":
                return self.actual.copy()
            if field == "Goal_Position":
                return self.goal.copy()
            if field == "Torque_Enable":
                return dict.fromkeys(pose, int(self.enabled))
            return dict.fromkeys(pose, 0)

        def sync_write(self, field, target, normalize=False):
            assert field == "Goal_Position"
            writes.append(("goal", target.copy()))
            self.goal = target.copy()
            if self.enabled:
                self.actual = target.copy()

        def enable_torque(self):
            self.enabled = True
            writes.append(("enable", 1))

        def write(self, field, motor, value, **kwargs):
            assert field == "Torque_Enable" and value == 0
            self.enabled = False
            writes.append(("off", motor))

        def read(self, field, motor, **kwargs):
            return self.sync_read(field)[motor]

    bus = Bus()
    events = []

    def emit(event, **fields):
        events.append(dict(event=event, **fields))

    def sleep(dt):
        clock[0] += dt

    return SimpleNamespace(
        bus=bus, pose=pose, clock=clock, writes=writes, events=events, emit=emit, sleep=sleep
    )


def perform(h, camera=lambda: None):
    return check.execute(h.bus, h.pose, -1, h.emit, camera, clock=lambda: h.clock[0], sleep=h.sleep)


def test_follower_only_fixed_trajectory_and_cleanup(hardware):
    h = hardware
    r = perform(h)
    assert r["execution"] == "completed" and r["shutdown"] == "verified off"
    goals = [x[1] for x in h.writes if x[0] == "goal"]
    assert len(goals) > 50 and min(p["elbow_flex"] for p in goals) == 1944
    assert goals[-1] == h.pose
    assert all(p[k] == 2000 for p in goals for k in check.JOINTS if k != "elbow_flex")
    assert not h.bus.enabled


@pytest.mark.parametrize("where", ["before", "during"])
def test_camera_failure_blocks_or_cleans_up(hardware, where):
    h = hardware

    def camera():
        if where == "before" or h.bus.enabled:
            raise RuntimeError("camera stopped")

    r = perform(h, camera)
    assert r["execution"] in ("rejected", "aborted") and not h.bus.enabled
    if where == "before":
        assert not h.writes
    else:
        assert any(x[0] == "off" for x in h.writes)


@pytest.mark.parametrize("event", ["baseline", "sample", "command"])
def test_disk_failure_cannot_skip_shutdown(hardware, event):
    h = hardware
    original = h.emit

    def emit(kind, **fields):
        if kind == event:
            raise OSError("disk full")
        original(kind, **fields)

    h.emit = emit
    r = perform(h)
    assert r["execution"] != "completed" and not h.bus.enabled
    if event == "baseline":
        assert not h.writes
    else:
        assert any(x[0] == "off" for x in h.writes)


def test_stale_camera_check_does_not_send_catchup_target(hardware):
    h = hardware

    def camera():
        if h.bus.enabled:
            h.clock[0] += 0.6

    r = perform(h, camera)
    assert r["execution"] == "aborted"
    assert len([w for w in h.writes if w[0] == "goal"]) == 1
    assert not h.bus.enabled


def test_foreign_goal_aborts_and_cleans_up(hardware):
    h = hardware
    original = h.bus.sync_read

    def read(field, **kw):
        values = original(field, **kw)
        if field == "Goal_Position" and h.bus.enabled:
            values["wrist_flex"] += 1
        return values

    h.bus.sync_read = read
    assert perform(h)["execution"] == "aborted"
    assert not h.bus.enabled


def test_initial_torque_on_rejects_without_writes(hardware):
    h = hardware
    h.bus.enabled = True
    assert perform(h)["execution"] == "rejected"
    assert h.bus.enabled and not h.writes


@pytest.mark.parametrize("failure", ["none", "speech", "configuration", "final_log", "camera_start"])
def test_full_orchestration_never_needs_leader_and_preserves_failure_state(
    hardware, tmp_path, monkeypatch, failure
):
    from serial.tools import list_ports

    from lerobot.robots.so_follower import so_follower

    h = hardware
    h.bus.is_connected = False
    h.bus.connect = lambda: setattr(h.bus, "is_connected", True)

    def disconnect(disable_torque=False):
        assert disable_torque is False
        h.bus.is_connected = False

    h.bus.disconnect = disconnect
    f = SimpleNamespace(
        bus=h.bus, calibration={"elbow_flex": SimpleNamespace(range_min=1000, range_max=3000)}
    )
    monkeypatch.setattr(so_follower, "SOFollower", lambda config: f)
    monkeypatch.setattr(list_ports, "comports", lambda: [SimpleNamespace(serial_number="F", device="/fake")])
    monkeypatch.setattr(check.session, "BoundedBus", lambda b: b)
    import contextlib

    monkeypatch.setattr(check.session, "DeviceLocks", lambda *a: contextlib.nullcontext())
    monkeypatch.setattr(check.session, "snapshot", lambda *a: {})

    def unchanged(*a):
        if failure == "configuration":
            raise RuntimeError("changed configuration")

    monkeypatch.setattr(check.session, "require_unchanged", unchanged)
    monkeypatch.setattr(check.lift, "require_rest", lambda b: None)
    monkeypatch.setattr(check, "validate_certificate", lambda *a: -1)
    monkeypatch.setattr(check, "reserve_trial", lambda *a: None)
    monkeypatch.setattr("builtins.input", lambda text: "")

    def say(text):
        if failure == "speech":
            raise RuntimeError("speech unavailable")

    monkeypatch.setattr(check.lift, "say", say)
    monkeypatch.setattr(check.time, "sleep", h.sleep)
    original_execute = check.execute
    monkeypatch.setattr(
        check,
        "execute",
        lambda bus, pose, direction, emit, camera: original_execute(
            bus, pose, direction, emit, camera, clock=lambda: h.clock[0], sleep=h.sleep
        ),
    )

    class FakeCamera:
        def __init__(self, path):
            pass

        def __enter__(self):
            if failure == "camera_start":
                raise RuntimeError("camera missing")
            return self

        def __exit__(self, *a):
            pass

        def check(self):
            return tmp_path / "frame.jpg"

    monkeypatch.setattr(check, "Camera", FakeCamera)
    if failure == "final_log":
        original_emit = check.evidence.Recorder.emit

        def emit(self, event, **fields):
            if event == "final":
                raise OSError("disk full")
            return original_emit(self, event, **fields)

        monkeypatch.setattr(check.evidence.Recorder, "emit", emit)
    cert = tmp_path / "cert.json"
    cert.write_text(json.dumps({"serial": "F"}))
    monkeypatch.chdir(tmp_path)
    args = SimpleNamespace(
        direction_record=cert,
        session_file=tmp_path / "session.json",
        follower_id="f",
        purpose="B initial",
        camera_dir=tmp_path / "camera",
    )
    result = check.run(args)
    assert not h.bus.enabled and not h.bus.is_connected
    assert result["physical_accepted"] is False
    if failure in ("speech", "configuration", "camera_start"):
        assert not h.writes
        assert result["execution"] == "rejected"
    elif failure == "none":
        assert result["execution"] == "completed"
        assert result["encoder"] == "passed"
        assert result["shutdown"] == "verified off"
    else:
        assert result["shutdown"] == "unverified"
        assert any(w[0] == "off" for w in h.writes)


@pytest.mark.parametrize("fault", ["dead", "no_frames", "stale", "empty"])
def test_camera_liveness_check_rejects(tmp_path, fault):
    import os
    import time

    camera = check.Camera(tmp_path)
    camera.process = SimpleNamespace(poll=lambda: 1 if fault == "dead" else None)
    if fault != "no_frames":
        frame = tmp_path / "frame-00001.jpg"
        frame.write_bytes(b"" if fault == "empty" else b"jpeg")
        if fault == "stale":
            os.utime(frame, (time.time() - 5, time.time() - 5))
    with pytest.raises(RuntimeError):
        camera.check()
