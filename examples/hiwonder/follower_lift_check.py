"""Supervised follower-only fixed elbow or shoulder-lift trial. No tuning or leader connection.

Requires a reviewed passive direction record and an explicitly authorized session
file. Camera liveness is a process/frame-age check, NOT collision avoidance.
Visual acceptance is never assigned automatically. Human power cutoff is required.
"""

import argparse
import fcntl
import hashlib
import json
import math
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

if __package__:
    from . import elbow_evidence as evidence, elbow_lift_check as lift, elbow_session as session
else:
    import elbow_evidence as evidence
    import elbow_lift_check as lift
    import elbow_session as session

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
IDENTITY_FIELDS = (
    "id",
    "configured_model",
    "Model_Number",
    "Firmware_Major_Version",
    "Firmware_Minor_Version",
)


def motor_identity(snapshot):
    return json.loads(
        json.dumps(
            {name: {field: snapshot["motors"][name][field] for field in IDENTITY_FIELDS} for name in JOINTS}
        )
    )


def valid_pose(pose):
    if set(pose) != set(JOINTS) or any(type(v) is not int or not 0 <= v <= 4095 for v in pose.values()):
        raise ValueError("Complete, finite, single-turn six-joint pose required")


def validate_certificate(cert, serial, snapshot, pose, *, joint=lift.ELBOW):
    """Identity and pose eligibility, not a claim of current physical clearance."""
    lift.require_joint(joint)
    version = cert.get("version")
    if version == 1:
        if joint != lift.ELBOW or cert.get("joint", lift.ELBOW) != lift.ELBOW:
            raise ValueError("Legacy direction evidence is elbow-only")
    elif version == 2:
        if cert.get("joint") != joint or cert.get("reviewed_clearance") is not True:
            raise ValueError("Joint-specific direction and full-path clearance review required")
    else:
        raise ValueError("Unknown direction evidence version")
    for p in (pose, cert["rest_pose"], cert["manual_before"], cert["manual_lift"], cert["manual_return"]):
        valid_pose(p)
    if (
        cert["serial"] != serial
        or cert["calibration_hash"] != snapshot["calibration_hash"]
        or cert["motor_identity"] != motor_identity(snapshot)
    ):
        raise ValueError("Passive evidence identity/calibration/firmware mismatch")
    if cert.get("reviewed_upward") is not True or cert.get("camera_name") != "icspring camera":
        raise ValueError("Reviewed upward camera evidence required")
    if type(cert["direction"]) is not int or cert["direction"] not in (-1, 1):
        raise ValueError("Invalid direction")
    if lift.infer_direction(cert["manual_before"], cert["manual_lift"], joint=joint) != cert["direction"]:
        raise ValueError("Passive encoder direction contradicts record")
    tolerance = 2 * lift.TICKS_PER_DEGREE
    for a, b in (
        (pose, cert["rest_pose"]),
        (cert["manual_return"], cert["manual_before"]),
        (cert["rest_pose"], cert["manual_return"]),
    ):
        if any(abs(a[k] - b[k]) > tolerance for k in JOINTS):
            raise ValueError("Pose changed; refresh passive direction observation")
    if not cert["sources"]:
        raise ValueError("Passive source artifacts required")
    for item in cert["sources"]:
        if hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Passive evidence artifact changed")
    return cert["direction"]


def reserve_trial(path, serial, certificate_hash, purpose, now=None):
    """Consume a trial before writes; failed attempts are not automatically retried."""
    now = time.time() if now is None else now
    if not purpose.strip():
        raise ValueError("A discriminating trial purpose is required")
    with Path(path).open("r+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = json.load(stream)
        age = now - state["started_at"]
        if (
            state.get("authorized") is not True
            or not math.isfinite(age)
            or not 0 <= age < 600
            or state["serial"] != serial
            or state["certificate_sha256"] != certificate_hash
            or len(state["trials"]) >= 3
            or purpose in [r["purpose"] for r in state["trials"]]
        ):
            raise ValueError("Session expired, exhausted, changed, unauthorized, or repeated purpose")
        state["trials"].append({"purpose": purpose, "reserved_at": now})
        stream.seek(0)
        json.dump(state, stream, allow_nan=False, indent=2)
        stream.truncate()
        stream.flush()
        os.fsync(stream.fileno())


def execute(
    bus, baseline, direction, emit, camera_check, *, joint=lift.ELBOW, clock=time.monotonic, sleep=time.sleep
):
    """Fixed existing trajectory and guards; cleanup independent of camera/logging."""
    attempted = False
    result = {
        "execution": "rejected",
        "shutdown": "not attempted",
        "restoration": "not applicable",
        "return_error_ticks": None,
        "guard_events": [],
        "errors": [],
    }
    try:
        lift.require_joint(joint)
        valid_pose(baseline)
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError("Invalid direction")
        if any(bus.sync_read("Torque_Enable", normalize=False).values()):
            raise RuntimeError("Initial torque is ON; no automatic state change")
        camera_check()
        emit("baseline", baseline=baseline, direction=direction, joint=joint)
        attempted = True
        with session.Deadline(0.5):
            lift.preload_and_enable(bus, baseline)
        if any(v != 1 for v in bus.sync_read("Torque_Enable", normalize=False).values()):
            raise RuntimeError("Not all follower motors enabled")
        start = previous = clock()
        last_sent = baseline.copy()
        while True:
            read_start = clock()
            session.require_fresh(previous, read_start)
            previous = read_start
            camera_check()
            target = lift.target_at(baseline, direction, read_start - start, joint=joint)
            actual = bus.sync_read("Present_Position", normalize=False)
            read_end = clock()
            status = bus.sync_read("Status", normalize=False)
            torque = bus.sync_read("Torque_Enable", normalize=False)
            goals = bus.sync_read("Goal_Position", normalize=False)
            telemetry = {
                name: bus.read(name, joint, normalize=False)
                for name in ("Present_Load", "Present_Current", "Present_Voltage", "Present_Temperature")
            }
            telemetry.update(Goal_Position=goals[joint], Torque_Enable=torque[joint])
            emit("readback", joint=joint, t=clock(), goal=goals[joint], expected=last_sent[joint])
            emit(
                "sample",
                joint=joint,
                read_start=read_start,
                read_end=read_end,
                actual=actual,
                intended_target=target,
                previous_sent_joint=last_sent[joint],
                status=status,
                torque=torque,
                goals=goals,
                joint_telemetry_raw=telemetry,
            )
            if goals != last_sent:
                raise RuntimeError("Foreign or missing goal readback")
            if any(v != 1 for v in torque.values()):
                raise RuntimeError("A follower motor became disabled")
            lift.check_feedback(baseline, target, actual, direction, status, joint=joint)
            session.require_fresh(read_start, clock())
            bus.sync_write("Goal_Position", target, normalize=False)
            last_sent = target.copy()
            emit("command", joint=joint, t=clock(), goal=target[joint])
            if read_start - start >= lift.END_SECONDS:
                result["return_error_ticks"] = max(abs(actual[k] - baseline[k]) for k in JOINTS)
                if result["return_error_ticks"] > 2 * lift.TICKS_PER_DEGREE:
                    raise RuntimeError("Return error exceeds two degrees")
                result["execution"] = "completed"
                break
            sleep(0.1)
    except (Exception, KeyboardInterrupt) as exc:
        result["execution"] = "aborted" if attempted else "rejected"
        result["guard_events"].append(str(exc) or "Ctrl+C")
    finally:
        if attempted:
            result["shutdown"] = session.shutdown(bus)
    return result


class Camera:
    """Separate FFmpeg process; completed JPEG freshness is checked during motion."""

    def __init__(self, root):
        self.root = Path(root)
        self.process = None
        self.log = None

    def __enter__(self):
        self.root.mkdir(parents=True, exist_ok=False)
        self.log = (self.root / "capture.log").open("w")
        try:
            self.process = subprocess.Popen(
                [
                    "/opt/homebrew/bin/ffmpeg",
                    "-hide_banner",
                    "-nostdin",
                    "-f",
                    "avfoundation",
                    "-pixel_format",
                    "uyvy422",
                    "-framerate",
                    "30",
                    "-video_size",
                    "640x480",
                    "-i",
                    "icspring camera:none",
                    "-an",
                    "-t",
                    "180",
                    "-vf",
                    "fps=5",
                    "-q:v",
                    "3",
                    "-atomic_writing",
                    "1",
                    str(self.root / "frame-%05d.jpg"),
                ],
                stdout=self.log,
                stderr=self.log,
            )
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError("Camera failed to start; inspect capture.log")
                if list(self.root.glob("frame-*.jpg")):
                    self.check()
                    return self
                time.sleep(0.1)
            raise TimeoutError("No camera frame; no motion")
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def check(self):
        if self.process.poll() is not None:
            raise RuntimeError("Camera capture stopped")
        frames = list(self.root.glob("frame-*.jpg"))
        if not frames:
            raise RuntimeError("No camera frame")
        newest = max(frames)
        stat = newest.stat()
        if stat.st_size == 0 or not 0 <= time.time() - stat.st_mtime <= 1.0:
            raise RuntimeError("Camera frame stale or empty")
        return newest

    def __exit__(self, *exc):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if self.log:
            self.log.close()


def run(args):
    from serial.tools.list_ports import comports

    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower

    joint = args.joint
    lift.require_joint(joint)
    label = joint.replace("_flex", "").replace("_", " ")
    cert_bytes = Path(args.direction_record).read_bytes()
    cert = json.loads(cert_bytes)
    digest = hashlib.sha256(cert_bytes).hexdigest()
    candidates = [p for p in comports() if p.serial_number == cert["serial"]]
    if len(candidates) != 1:
        raise ValueError("Follower adapter missing or ambiguous")
    device = SOFollower(SOFollowerRobotConfig(port=candidates[0].device, id=args.follower_id))
    bus = session.BoundedBus(device.bus)
    root = Path("test-logs")
    root.mkdir(exist_ok=True)
    path = root / f"follower-lift-{datetime.now():%Y%m%d-%H%M%S-%f}.jsonl"
    print(f"Diagnostic log: {path.resolve()}", flush=True)
    result = {
        "execution": "rejected",
        "shutdown": "not attempted",
        "restoration": "not applicable",
        "guard_events": [],
        "errors": [],
        "return_error_ticks": None,
    }
    records = []
    hardware = session.HardwareSession([bus], [cert["serial"]])
    with evidence.Recorder(path) as recorder:

        def emit(event, **fields):
            with session.Deadline(session.IO_SECONDS):
                records.append(recorder.emit(event, **fields))

        emit(
            "start",
            mode="follower-only-joint-comparison",
            joint=joint,
            adapter_serials=[cert["serial"]],
            purpose=args.purpose,
            certificate_sha256=digest,
        )
        try:
            with Camera(args.camera_dir) as camera, hardware:
                lift.require_rest(bus)
                before = session.snapshot(bus, device.calibration)
                pose = bus.sync_read("Present_Position", normalize=False)
                direction = validate_certificate(cert, cert["serial"], before, pose, joint=joint)
                lift.validate_path(pose, direction, device.calibration[joint], joint=joint)
                emit("configuration", snapshots=[before])
                print(f"Review current full-path image: {camera.check().resolve()}", flush=True)
                input(
                    "Press Enter only after fresh view/setup review; operator present and cutoff reachable: "
                )
                lift.say(
                    "Follower-only test in fifteen seconds. Keep clear of the white arm. Watch the follower."
                )
                time.sleep(12)
                for word in ("Three", "Two", "One"):
                    lift.say(word)
                    time.sleep(1)
                lift.say(f"Starting the small {label} lift after checks. It will return to rest.")
                camera.check()
                current = session.snapshot(bus, device.calibration)
                session.require_unchanged(before, current)
                lift.require_rest(bus)
                pose = bus.sync_read("Present_Position", normalize=False)
                direction = validate_certificate(cert, cert["serial"], current, pose, joint=joint)
                lift.validate_path(pose, direction, device.calibration[joint], joint=joint)
                reserve_trial(args.session_file, cert["serial"], digest, args.purpose)
                result = execute(bus, pose, direction, emit, camera.check, joint=joint)
        except (Exception, KeyboardInterrupt) as exc:
            result["guard_events"].append(str(exc) or "Ctrl+C")
        result["errors"].extend(hardware.close_errors)
        try:
            emit("final", **result)
        except Exception as exc:
            result["errors"].append(f"Final record unavailable: {exc}")
        analyzed = evidence.analyze(records)
        if result["errors"]:
            analyzed["errors"].extend(result["errors"])
            analyzed["physical_accepted"] = False
        # A frame-age check cannot assign visual acceptance. Review retained images separately.
        message = evidence.format_result(analyzed)
        if result["shutdown"] == "unverified":
            message = "CUT SERVO POWER NOW. Shutdown unverified. " + message
        print(message, flush=True)
        try:
            lift.say(message)
        except Exception as exc:
            print(f"Speech failed after cleanup: {exc}", flush=True)
        return analyzed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=("elbow_flex", "shoulder_lift"), default="elbow_flex")
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--direction-record", required=True)
    parser.add_argument("--session-file", required=True)
    parser.add_argument("--purpose", required=True)
    parser.add_argument("--camera-dir", required=True)
    args = parser.parse_args()
    raise SystemExit(evidence.exit_code(run(args)))


if __name__ == "__main__":
    main()
