"""Unix-only diagnostic ownership, read-only snapshots and bounded I/O.

Deadlines are process alarms, not a real-time safety guarantee. Physical power cutoff
remains necessary. No changes to the shared SDK or normal teleoperation behavior.
"""

import dataclasses
import fcntl
import hashlib
import json
import math
import os
import signal
import threading
import time
from contextlib import ExitStack
from pathlib import Path

IO_SECONDS = 0.25
FIELDS = (
    "Model_Number",
    "Firmware_Major_Version",
    "Firmware_Minor_Version",
    "Operating_Mode",
    "Homing_Offset",
    "Min_Position_Limit",
    "Max_Position_Limit",
    "P_Coefficient",
    "I_Coefficient",
    "D_Coefficient",
    "Acceleration",
    "Goal_Velocity",
    "Torque_Limit",
    "Max_Torque_Limit",
    "Protection_Current",
    "Overload_Torque",
    "Protection_Time",
    "Torque_Enable",
    "Lock",
    "Goal_Position",
    "Minimum_Startup_Force",
    "CW_Dead_Zone",
    "CCW_Dead_Zone",
    "Protective_Torque",
    "Over_Current_Protection_Time",
)


class Deadline:
    """Interrupt blocking Unix calls in the main thread; preserve enclosing alarms."""

    def __init__(self, seconds):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("Invalid deadline")
        self.seconds = seconds

    def __enter__(self):
        if threading.current_thread() is not threading.main_thread() or not hasattr(signal, "setitimer"):
            raise RuntimeError("Bounded diagnostic requires Unix main-thread alarm support; no motion")
        self.started = time.monotonic()
        self.old_handler = signal.getsignal(signal.SIGALRM)
        self.old_timer = signal.getitimer(signal.ITIMER_REAL)

        def expired(signum, frame):
            raise TimeoutError("Diagnostic I/O deadline expired")

        signal.signal(signal.SIGALRM, expired)
        remaining = self.old_timer[0]
        signal.setitimer(signal.ITIMER_REAL, min(self.seconds, remaining) if remaining else self.seconds)
        return self

    def __exit__(self, *exc):
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, self.old_handler)
        old, interval = self.old_timer
        remaining = old - (time.monotonic() - self.started)
        if old and remaining > 0:
            signal.setitimer(signal.ITIMER_REAL, remaining, interval)
        elif old and exc[0] is None:
            raise TimeoutError("Enclosing diagnostic deadline expired")


class DeviceLocks:
    def __init__(self, identities, root=None):
        if len(set(identities)) != len(identities) or any(not i for i in identities):
            raise ValueError("Distinct adapter identities required")
        self.identities = sorted(identities)
        self.root = Path(root or Path.home() / "Library/Application Support/hiwonder-soarm101/locks")
        self.stack = ExitStack()

    def __enter__(self):
        self.root.mkdir(parents=True, exist_ok=True)
        try:
            for identity in self.identities:
                path = self.root / (hashlib.sha256(identity.encode()).hexdigest() + ".lock")
                handle = self.stack.enter_context(path.open("a"))
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError("Adapter owned by another diagnostic") from exc
            return self
        except BaseException:
            self.stack.close()
            raise

    def __exit__(self, *exc):
        self.stack.close()


def identify_ports(leader, follower, ports):
    identities = []
    for requested in (leader, follower):
        matches = {
            p.serial_number for p in ports if os.path.realpath(p.device) == os.path.realpath(requested)
        }
        if len(matches) != 1 or not next(iter(matches)):
            raise ValueError("Missing or ambiguous adapter serial; re-identify arms")
        identities.append(next(iter(matches)))
    if identities[0] == identities[1]:
        raise ValueError("Both ports identify the same adapter")
    return identities


class BoundedBus:
    """Per-call alarm includes SDK flush/write/read/retry time, including handshake."""

    def __init__(self, bus):
        self.raw = bus

    def __getattr__(self, name):
        value = getattr(self.raw, name)
        if not callable(value):
            return value

        def call(*args, **kwargs):
            try:
                with Deadline(IO_SECONDS):
                    return value(*args, **kwargs)
            except BaseException:
                # An interrupted SDK operation may leave its internal busy flag set.
                self.raw.port_handler.is_using = False
                raise

        return call

    def connect(self):
        # Open without handshake, install write deadline before the first transaction.
        with Deadline(2):
            self.raw.connect(handshake=False)
            self.raw.port_handler.ser.write_timeout = IO_SECONDS
            self.raw.set_timeout(int(IO_SECONDS * 1000))
            self.raw._handshake()

    def enable_torque(self):
        # SDK enable_torque also mutates Lock; this diagnostic changes torque only.
        for motor in self.raw.motors:
            self.write("Torque_Enable", motor, 1, normalize=False, num_retry=0)

    def write(self, *args, **kwargs):
        return self.__getattr__("write")(*args, **kwargs)


class HardwareSession:
    def __init__(self, buses, identities, lock_root=None):
        self.buses = buses
        self.locks = DeviceLocks(identities, lock_root)
        self.close_errors = []

    def __enter__(self):
        self.locks.__enter__()
        try:
            for bus in self.buses:
                bus.connect()
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *exc):
        try:
            for bus in reversed(self.buses):
                if bus.is_connected:
                    try:
                        bus.disconnect(disable_torque=False)
                    except BaseException as error:
                        self.close_errors.append(str(error))
        finally:
            self.locks.__exit__(*exc)


def calibration_hash(calibration):
    def convert(value):
        if dataclasses.is_dataclass(value):
            return dataclasses.asdict(value)
        return vars(value)

    data = json.dumps(calibration, default=convert, sort_keys=True, allow_nan=False)
    return hashlib.sha256(data.encode()).hexdigest()


def snapshot(bus, calibration):
    motors = {}
    for name, motor in bus.motors.items():
        values = {"id": motor.id, "configured_model": motor.model}
        table = bus.model_ctrl_table.get(motor.model, {})
        for field in FIELDS:
            if field not in table:
                values[field] = {"state": "unsupported"}
                continue
            try:
                values[field] = {"state": "read", "raw": bus.read(field, name, normalize=False)}
            except Exception as exc:
                values[field] = {"state": "error", "message": str(exc)}
        motors[name] = values
    return {
        "motors": motors,
        "calibration_hash": calibration_hash(calibration),
        "register_definitions": "checkout Hiwonder table v1 (inherited STS/SMS); firmware applicability unverified",
        "unit_interpretation": "unverified",
    }


def require_unchanged(before, after):
    if before != after:
        raise RuntimeError("Configuration changed before enable; no motion")
    if "motors" in after:
        for fields in after["motors"].values():
            for name in FIELDS:
                value = fields[name]
                if value["state"] == "error":
                    raise RuntimeError(f"Configuration read failed: {name}")
                if value["state"] == "unsupported" and name not in (
                    "Firmware_Major_Version",
                    "Firmware_Minor_Version",
                ):
                    raise RuntimeError(f"Required configuration unavailable: {name}")


def require_fresh(start, now):
    if not math.isfinite(now - start) or not 0 <= now - start <= 0.5:
        raise RuntimeError("Control loop stale after feedback/logging; no catch-up target sent")


def cleanup_budget(bus):
    # Three passes; each motor gets one bounded write and one bounded read per pass.
    return 3 * len(bus.motors) * 2 * IO_SECONDS + 0.5


def shutdown(bus):
    remaining = set(bus.motors)
    try:
        with Deadline(cleanup_budget(bus)):
            for _ in range(3):
                for motor in sorted(remaining):
                    try:
                        with Deadline(IO_SECONDS):
                            bus.write("Torque_Enable", motor, 0, normalize=False, num_retry=0)
                        with Deadline(IO_SECONDS):
                            off = bus.read("Torque_Enable", motor, normalize=False, num_retry=0) == 0
                        if off:
                            remaining.remove(motor)
                    except (Exception, KeyboardInterrupt):
                        continue
                if not remaining:
                    return "verified off"
    except (Exception, KeyboardInterrupt):
        pass
    return "unverified"
