# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from collections import Counter
from copy import deepcopy
from dataclasses import dataclass, field
from time import monotonic, sleep

from lerobot.motors.feetech.feetech import (
    DriveMode,
    FeetechMotorsBus,
    OperatingMode,
    TorqueMode,
    patch_setPacketTimeout,
)
from lerobot.motors.feetech.tables import SCAN_BAUDRATES
from lerobot.motors.motors_bus import Motor, MotorCalibration, NameOrID, get_address
from lerobot.utils.decorators import check_if_not_connected

from . import hiwonder_sdk as hw
from .tables import (
    MODEL_BAUDRATE_TABLE,
    MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER_TABLE,
    MODEL_PROTOCOL,
    MODEL_RESOLUTION,
)

DEFAULT_PROTOCOL_VERSION = 0
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_MS = 1000

# Same fields that are normalized in Feetech — position data uses calibration offsets.
NORMALIZED_DATA = ["Goal_Position", "Present_Position"]

# Verified torque-off (HX-30HM can acknowledge a Torque_Enable=0 write without applying it, and flags every
# reply with Overload while stalled, so torque-off is confirmed by reading it back).
TORQUE_OFF_TIMEOUT_S = 2.0  # bounds the verification loop; the initial writes always reach every motor
TORQUE_OFF_CONFIRM_GAP_S = 0.5  # a 0 read this long after the first 0, with no non-zero read between
TORQUE_OFF_POLL_S = 0.1
GOAL_MATCH_TICKS = 20  # the two Present_Position reads must agree this closely before Goal=Present
SIGN_BIT = 1 << 15  # sign-magnitude position registers: a raw value with bit 15 set is negative

# Overload latch on connect: hardware 2026-10-08, Goal_Position=Present cleared it within 2 s in 18 of 18 resets.
OVERLOAD_CLEAR_TIMEOUT_S = 2.5
OTHER_FAULT_BITS = 0xFF & ~hw.ERRBIT_OVERLOAD  # any other bit, documented (voltage, sensor, ...) or not

logger = logging.getLogger(__name__)


@dataclass
class _TorqueOffState:
    name: str
    id: int
    model: str
    range_min: int
    range_max: int
    first_zero: float | None = None
    confirmed: bool = False
    last_te: int | None = None  # last successful Torque_Enable read
    last_error: str | None = None  # last failed op, as text
    te_read_failed: bool = False  # the last Torque_Enable read attempt failed
    resends: int = 0
    errors: Counter = field(default_factory=Counter)
    flags: set = field(default_factory=set)


class HiwonderMotorsBus(FeetechMotorsBus):
    """
    MotorsBus implementation for Hiwonder serial bus servos (e.g. HX-30HM).

    The HX-30HM uses the same STS/SMS serial protocol as Feetech motors and is
    fully compatible with the hiwonder_sdk bundled in this module. This class
    registers the Hiwonder model names so they can be used as the ``model``
    argument when constructing :class:`~lerobot.motors.motors_bus.Motor` objects.

    Example::

        from lerobot.motors.motors_bus import Motor
        from lerobot.motors.hiwonder import HiwonderMotorsBus

        bus = HiwonderMotorsBus(
            port="/dev/ttyUSB0",
            motors={
                "shoulder_pan": Motor(1, "hx30hm"),
                "shoulder_lift": Motor(2, "hx30hm"),
            },
        )
    """

    apply_drive_mode = True
    available_baudrates = deepcopy(SCAN_BAUDRATES)
    default_baudrate = DEFAULT_BAUDRATE
    default_timeout = DEFAULT_TIMEOUT_MS
    model_baudrate_table = deepcopy(MODEL_BAUDRATE_TABLE)
    model_ctrl_table = deepcopy(MODEL_CONTROL_TABLE)
    model_encoding_table = deepcopy(MODEL_ENCODING_TABLE)
    model_number_table = deepcopy(MODEL_NUMBER_TABLE)
    model_resolution_table = deepcopy(MODEL_RESOLUTION)
    normalized_data = deepcopy(NORMALIZED_DATA)

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        protocol_version: int = DEFAULT_PROTOCOL_VERSION,
    ):
        # Call grandparent (SerialMotorsBus) directly to avoid FeetechMotorsBus.__init__
        # importing scservo_sdk, which is not installed for Hiwonder setups.
        from lerobot.motors.motors_bus import SerialMotorsBus

        SerialMotorsBus.__init__(self, port, motors, calibration)
        self.protocol_version = protocol_version
        self._assert_same_protocol()

        self.port_handler = hw.PortHandler(self.port)
        self.port_handler.setPacketTimeout = patch_setPacketTimeout.__get__(  # type: ignore[method-assign]
            self.port_handler, hw.PortHandler
        )
        self.packet_handler = hw.PacketHandler(self.port_handler, endianness=0)
        self.sync_reader = hw.GroupSyncRead(self.packet_handler, 0, 0)
        self.sync_writer = hw.GroupSyncWrite(self.packet_handler, 0, 0)
        self._comm_success = hw.COMM_SUCCESS
        self._no_error = 0x00

        if any(MODEL_PROTOCOL[model] != self.protocol_version for model in self.models):
            raise ValueError(f"Some motors are incompatible with protocol_version={self.protocol_version}")

    def _assert_same_protocol(self) -> None:
        if any(MODEL_PROTOCOL[model] != self.protocol_version for model in self.models):
            raise RuntimeError("Some motors use an incompatible protocol.")

    def _handshake(self) -> None:
        self._clear_overload_latches()
        self._assert_motors_exist()

    def _clear_overload_latches(self) -> None:
        """Clear an Overload flag left latched by a stall that torque-off cut short.

        HX-30HM keeps Status Overload set with torque off (> 5 min seen). Every reply then carries the flag, so
        `ping()` reports the motor missing and reads raise. For each motor whose Status shows Overload and no
        other fault: Goal_Position = Present_Position (turns torque on, holding position; this clears the flag),
        wait up to OVERLOAD_CLEAR_TIMEOUT_S, then verified torque-off on those motors, also on error or interrupt.
        A motor with another fault bit is not written. A motor that does not reply is left to the motor check.
        Raises RuntimeError naming each motor that is still latched or was not written.
        """
        latched = []
        for name in self.motors:
            st = self._torque_off_state(name)
            status = self._read_status(st)
            if status is not None and status & hw.ERRBIT_OVERLOAD:
                latched.append((st, status))
        if not latched:
            return

        writable = [st for st, status in latched if not status & OTHER_FAULT_BITS]
        problems = [
            f"{st.name} (id {st.id}): Status 0x{status:02X} has a fault besides Overload, not written"
            for st, status in latched
            if status & OTHER_FAULT_BITS
        ]
        cleared = []
        try:
            for st in writable:
                goal, problem = self._clear_overload_latch(st)
                if problem is None:
                    cleared.append((st, goal))
                else:
                    problems.append(f"{st.name} (id {st.id}): {problem}")
        finally:
            if writable:
                self.port_handler.is_using = False  # an interrupt mid-packet leaves the SDK port lock set
                self.disable_torque([st.name for st in writable])
        for st, goal in cleared:
            logger.warning(
                f"Overload latched on {st.name} (id {st.id}): cleared by Goal_Position=Present ({goal}, torque "
                "briefly on); torque off confirmed"
            )
        if problems:
            raise RuntimeError(
                f"Overload latched on connect and not cleared on {len(problems)} motor(s): {'; '.join(problems)}. "
                "Torque is off on every motor written; power-cycle the 12 V servo supply, then reconnect."
            )

    def _clear_overload_latch(self, st: _TorqueOffState) -> tuple[int | None, str | None]:
        """Goal_Position = Present_Position, then wait for Overload to clear. Returns (goal, problem or None)."""
        deadline = monotonic() + OVERLOAD_CLEAR_TIMEOUT_S
        goal = self._torque_off_goal(st, deadline)
        if goal is None:
            return None, f"no trustworthy Present_Position ({dict(st.errors) or 'read failed'}), Goal not written"
        addr, length = get_address(self.model_ctrl_table, st.model, "Goal_Position")
        self._torque_off_write(st, "Goal_Position", addr, length, goal)
        while True:
            status = self._read_status(st)
            if status is not None and not status & hw.ERRBIT_OVERLOAD:
                return goal, None
            if monotonic() >= deadline:
                return goal, f"still latched {OVERLOAD_CLEAR_TIMEOUT_S} s after Goal_Position=Present ({goal})"
            sleep(TORQUE_OFF_POLL_S)

    def _read_status(self, st: _TorqueOffState) -> int | None:
        """Status register OR the reply's error byte, input flushed first; None if the read failed."""
        flushed, _ = self._torque_off_try(st, "flush input", self.port_handler.ser.reset_input_buffer)
        if not flushed:
            return None
        addr, length = get_address(self.model_ctrl_table, st.model, "Status")
        ok, result = self._torque_off_try(
            st, "read Status", self._read, addr, length, st.id, raise_on_error=False
        )
        if not ok:
            return None
        value, comm, error = result
        if not self._is_comm_success(comm):
            return None
        return value | error

    def _find_single_motor(self, motor: str, initial_baudrate: int | None = None) -> tuple[int, int]:
        # HX-30HM only supports protocol 0 (STS/SMS), so we always use the p0 path
        # which relies on broadcast_ping — no scservo_sdk needed.
        return self._find_single_motor_p0(motor, initial_baudrate)

    def _split_into_byte_chunks(self, value: int, length: int) -> list[int]:
        if length == 1:
            return [value]
        elif length == 2:
            return [value & 0xFF, (value >> 8) & 0xFF]
        elif length == 4:
            lo = value & 0xFFFF
            hi = (value >> 16) & 0xFFFF
            return [lo & 0xFF, (lo >> 8) & 0xFF, hi & 0xFF, (hi >> 8) & 0xFF]
        raise ValueError(f"Unsupported data length: {length}")

    def _broadcast_ping(self) -> tuple[dict[int, int], int]:
        data_list: dict[int, int] = {}
        status_length = 6
        rx_length = 0
        wait_length = status_length * hw.MAX_ID

        txpacket = [0] * 6
        tx_time_per_byte = (1000.0 / self.port_handler.getBaudRate()) * 10.0

        txpacket[hw.PKT_ID] = hw.BROADCAST_ID
        txpacket[hw.PKT_LENGTH] = 2
        txpacket[hw.PKT_INSTRUCTION] = hw.INST_PING

        result = self.packet_handler.txPacket(txpacket)
        if result != hw.COMM_SUCCESS:
            self.port_handler.is_using = False
            return data_list, result

        self.port_handler.setPacketTimeoutMillis((wait_length * tx_time_per_byte) + (3.0 * hw.MAX_ID) + 16.0)

        rxpacket = []
        while not self.port_handler.isPacketTimeout() and rx_length < wait_length:
            rxpacket += self.port_handler.readPort(wait_length - rx_length)
            rx_length = len(rxpacket)

        self.port_handler.is_using = False

        if rx_length == 0:
            return data_list, hw.COMM_RX_TIMEOUT

        while True:
            if rx_length < status_length:
                return data_list, hw.COMM_RX_CORRUPT

            for idx in range(0, rx_length - 1):
                if rxpacket[idx] == 0xFF and rxpacket[idx + 1] == 0xFF:
                    break

            if idx == 0:
                checksum = 0
                for idx in range(2, status_length - 1):
                    checksum += rxpacket[idx]
                checksum = ~checksum & 0xFF

                if rxpacket[status_length - 1] == checksum:
                    result = hw.COMM_SUCCESS
                    data_list[rxpacket[hw.PKT_ID]] = rxpacket[hw.PKT_ERROR]
                    del rxpacket[0:status_length]
                    rx_length -= status_length
                    if rx_length == 0:
                        return data_list, result
                else:
                    result = hw.COMM_RX_CORRUPT
                    del rxpacket[0:2]
                    rx_length -= 2
            else:
                del rxpacket[0:idx]
                rx_length -= idx

    def ping(self, motor: NameOrID, num_retry: int = 0, raise_on_error: bool = False) -> int | None:
        id_ = self._get_motor_id(motor)
        comm = hw.COMM_TX_FAIL
        error = 0
        model_number = 0
        for n_try in range(1 + num_retry):
            rxpacket, comm, error = self.packet_handler.ping(id_)
            if self._is_comm_success(comm):
                # hiwonder ping returns (rxpacket, comm, error) — model number not in response
                model_number = self.model_number_table.get(self.motors[self._id_to_name(id_)].model, 0)
                break
            logger.debug(f"ping failed for {id_=}: {n_try=} got {comm=} {error=}")

        if not self._is_comm_success(comm):
            if raise_on_error:
                raise ConnectionError(self.packet_handler.getTxRxResult(comm))
            return None
        if self._is_error(error):
            if raise_on_error:
                raise RuntimeError(self.packet_handler.getRxPacketError(error))
            return None
        return model_number

    def _read(
        self,
        address: int,
        length: int,
        motor_id: int,
        *,
        num_retry: int = 0,
        raise_on_error: bool = True,
        err_msg: str = "",
    ) -> tuple[int, int, int]:
        if length == 1:
            read_fn = self.packet_handler.read1ByteData
        elif length == 2:
            read_fn = self.packet_handler.read2ByteData
        elif length == 4:
            read_fn = self.packet_handler.read4ByteData
        else:
            raise ValueError(length)

        for n_try in range(1 + num_retry):
            value, comm, error = read_fn(motor_id, address)
            if self._is_comm_success(comm):
                break
            logger.debug(
                f"Failed to read @{address=} ({length=}) on {motor_id=} ({n_try=}): "
                + self.packet_handler.getTxRxResult(comm)
            )

        if not self._is_comm_success(comm) and raise_on_error:
            raise ConnectionError(f"{err_msg} {self.packet_handler.getTxRxResult(comm)}")
        elif self._is_error(error) and raise_on_error:
            raise RuntimeError(f"{err_msg} {self.packet_handler.getRxPacketError(error)}")
        return value, comm, error

    def _write(
        self,
        addr: int,
        length: int,
        motor_id: int,
        value: int,
        *,
        num_retry: int = 0,
        raise_on_error: bool = True,
        err_msg: str = "",
    ) -> tuple[int, int]:
        data = self._split_into_byte_chunks(value, length)
        for n_try in range(1 + num_retry):
            comm, error = self.packet_handler.writeReadData(motor_id, addr, length, data)
            if self._is_comm_success(comm):
                break
            logger.debug(
                f"Failed to write @{addr=} ({length=}) on id={motor_id} with {value=} ({n_try=}): "
                + self.packet_handler.getTxRxResult(comm)
            )

        if not self._is_comm_success(comm) and raise_on_error:
            raise ConnectionError(f"{err_msg} {self.packet_handler.getTxRxResult(comm)}")
        elif self._is_error(error) and raise_on_error:
            raise RuntimeError(f"{err_msg} {self.packet_handler.getRxPacketError(error)}")
        return comm, error

    def disable_torque(self, motors: int | str | list[str] | None = None, num_retry: int = 0) -> None:
        """Turn torque off and confirm it by reading Torque_Enable back.

        1. Torque_Enable=0 and Lock=0 are written to every target motor, whatever the time.
        2. Until every motor is confirmed or TORQUE_OFF_TIMEOUT_S has passed, each unconfirmed motor's
           Torque_Enable is read (input buffer flushed first). A motor is confirmed by a successful 0 read
           taken >= TORQUE_OFF_CONFIRM_GAP_S after its first 0 read, with no other result in between. On a
           non-zero read, Goal_Position is set to the present position (drops a stall load, which otherwise
           keeps Overload set and the torque-off write ignored) and Torque_Enable=0 / Lock=0 are re-sent; on a
           failed read only the writes are re-sent.
        3. Otherwise raises RuntimeError naming each unconfirmed motor and its last known state.

        Every read and write is attempted on its own: an error (including an Overload status) on one motor or
        one operation never skips the others. `num_retry` is accepted for compatibility; the loop is the retry.
        """
        names = self._get_motors_list(motors)
        if not names:
            return

        start = monotonic()
        deadline = start + TORQUE_OFF_TIMEOUT_S
        states = [self._torque_off_state(name) for name in names]
        try:
            for st in states:
                self._torque_off_writes(st)
            while monotonic() < deadline:
                for st in states:
                    if st.confirmed:
                        continue
                    if monotonic() >= deadline:
                        break
                    self._torque_off_verify(st, deadline)
                if all(st.confirmed for st in states):
                    return
                sleep(TORQUE_OFF_POLL_S)

            unconfirmed = [st for st in states if not st.confirmed]
            if not unconfirmed:
                return
            details = "; ".join(
                f"{st.name} (id {st.id}): {self._torque_off_label(st, start)}" for st in unconfirmed
            )
            raise RuntimeError(
                f"Torque-off not confirmed after {TORQUE_OFF_TIMEOUT_S} s on {len(unconfirmed)} motor(s): "
                f"{details}. Cut servo power before touching the arm."
            )
        finally:
            for st in states:
                if st.errors or st.flags or st.resends:
                    logger.warning(
                        f"Torque-off on {st.name} (id {st.id}): {'confirmed' if st.confirmed else 'NOT confirmed'}"
                        f", {st.resends} re-send(s), errors {dict(st.errors)}, status flags {sorted(st.flags)}"
                    )

    def _torque_off_state(self, name: str) -> _TorqueOffState:
        model = self.motors[name].model
        cal = self.calibration.get(name)
        if cal is not None:
            lo, hi = cal.range_min, cal.range_max
        else:
            lo, hi = 0, self.model_resolution_table[model] - 1
        return _TorqueOffState(name, self.motors[name].id, model, lo, hi)

    def _torque_off_verify(self, st: _TorqueOffState, deadline: float) -> None:
        value = self._torque_off_read(st, "Torque_Enable")
        now = monotonic()
        if value == 0:
            if st.first_zero is None:
                st.first_zero = now
            elif now - st.first_zero >= TORQUE_OFF_CONFIRM_GAP_S:
                st.confirmed = True
            return

        st.first_zero = None
        st.resends += 1
        goal = self._torque_off_goal(st, deadline) if value is not None else None
        if goal is not None:
            goal_addr, goal_len = get_address(self.model_ctrl_table, st.model, "Goal_Position")
            self._torque_off_write(st, "Goal_Position", goal_addr, goal_len, goal)
            self._torque_off_writes(st)  # always follows a Goal write, even past the deadline
        else:
            self._torque_off_writes(st, deadline)

    def _torque_off_goal(self, st: _TorqueOffState, deadline: float) -> int | None:
        """Raw Present_Position if two successful reads agree and lie in the motor's range, else None."""
        values = []
        for _ in range(2):
            if monotonic() >= deadline:
                return None
            value = self._torque_off_read(st, "Present_Position")
            if value is None:
                return None
            values.append(value)
        a, b = values
        if a & SIGN_BIT or b & SIGN_BIT:
            st.errors["Present_Position negative"] += 1
        elif abs(a - b) > GOAL_MATCH_TICKS:
            st.errors["Present_Position reads disagree"] += 1
        elif not (st.range_min <= a <= st.range_max and st.range_min <= b <= st.range_max):
            st.errors["Present_Position out of range"] += 1
        else:
            return b
        return None

    def _torque_off_writes(self, st: _TorqueOffState, deadline: float | None = None) -> None:
        for data_name in ("Torque_Enable", "Lock"):
            if deadline is not None and monotonic() >= deadline:
                return
            addr, length = get_address(self.model_ctrl_table, st.model, data_name)
            self._torque_off_write(st, data_name, addr, length, 0)

    def _torque_off_write(
        self, st: _TorqueOffState, data_name: str, addr: int, length: int, value: int
    ) -> None:
        ok, result = self._torque_off_try(
            st, f"write {data_name}", self._write, addr, length, st.id, value, raise_on_error=False
        )
        if not ok:
            return
        comm, error = result
        if error:
            st.flags.add(self.packet_handler.getRxPacketError(error))
        if not self._is_comm_success(comm):
            self._torque_off_failed(st, f"write {data_name}", self.packet_handler.getTxRxResult(comm))

    def _torque_off_read(self, st: _TorqueOffState, data_name: str) -> int | None:
        """Flush the input buffer, then read; None unless the read itself succeeded.

        A failed read returns 0 from the SDK, so the value is only used when comm is COMM_SUCCESS. A status flag
        (e.g. Overload) on a successful read keeps the value.
        """
        is_te = data_name == "Torque_Enable"
        value = None
        flushed, _ = self._torque_off_try(st, "flush input", self.port_handler.ser.reset_input_buffer)
        if flushed:  # a read without the flush could return a stale reply
            addr, length = get_address(self.model_ctrl_table, st.model, data_name)
            ok, result = self._torque_off_try(
                st, f"read {data_name}", self._read, addr, length, st.id, raise_on_error=False
            )
            if ok:
                value, comm, error = result
                if error:
                    st.flags.add(self.packet_handler.getRxPacketError(error))
                if not self._is_comm_success(comm):
                    self._torque_off_failed(st, f"read {data_name}", self.packet_handler.getTxRxResult(comm))
                    value = None
        if is_te:
            st.te_read_failed = value is None
            if value is not None:
                st.last_te = value
        return value

    def _torque_off_try(self, st: _TorqueOffState, op: str, fn, *args, **kwargs) -> tuple[bool, object]:
        """Run one operation; on any Exception (never BaseException) record it and return (False, None)."""
        try:
            return True, fn(*args, **kwargs)
        except Exception as e:
            # An exception mid-packet leaves the SDK's port lock set, which would fail every later op.
            self.port_handler.is_using = False
            logger.debug(
                f"Torque-off {op} on {st.name} (id {st.id}) raised {type(e).__name__}", exc_info=True
            )
            self._torque_off_failed(st, op, f"{type(e).__name__}: {e}")
            return False, None

    def _torque_off_failed(self, st: _TorqueOffState, op: str, reason: str) -> None:
        st.errors[f"{op}: {reason.split(':')[0]}"] += 1
        st.last_error = f"{op}: {reason}"

    @staticmethod
    def _torque_off_label(st: _TorqueOffState, start: float) -> str:
        if st.first_zero is not None:
            return f"pending (read 0 at t={st.first_zero - start:.2f} s, gap not reached)"
        if st.te_read_failed:
            return f"no reply ({st.last_error})"
        if st.last_te is not None:
            return f"ON (read {st.last_te})"
        return "not checked (deadline reached during the initial writes)"

    @check_if_not_connected
    def disconnect(self, disable_torque: bool = True) -> None:
        """Turn torque off (verified, see `disable_torque`) and close the port.

        The port is closed even if torque-off raises (including KeyboardInterrupt); the torque-off error is
        re-raised afterwards and wins over a close error.
        """
        torque_err = None
        try:
            if disable_torque:
                self.port_handler.is_using = False
                try:
                    self.port_handler.clearPort()
                except Exception as e:
                    logger.warning(f"clearPort failed before torque-off ({e!r}); continuing")
                try:
                    self.disable_torque(num_retry=5)
                except Exception as e:
                    torque_err = e
        except BaseException:
            try:
                self.port_handler.closePort()
            except Exception as close_err:
                logger.error(f"closePort failed while handling an interrupt: {close_err!r}")
            raise

        try:
            self.port_handler.closePort()
        except Exception as close_err:
            if torque_err is None:
                raise
            logger.error(f"closePort failed after a torque-off error: {close_err!r}")
        if torque_err is not None:
            raise torque_err
        logger.debug(f"{self.__class__.__name__} disconnected.")

    def _find_single_motor_p0(self, motor: str, initial_baudrate: int | None = None) -> tuple[int, int]:
        model = self.motors[motor].model
        search_baudrates = (
            [initial_baudrate] if initial_baudrate is not None else self.model_baudrate_table[model]
        )
        expected_model_nb = self.model_number_table[model]

        for baudrate in search_baudrates:
            self.set_baudrate(baudrate)
            id_model = self.broadcast_ping()
            if id_model:
                found_id, found_model = next(iter(id_model.items()))
                if found_model != expected_model_nb:
                    raise RuntimeError(
                        f"Found one motor on {baudrate=} with id={found_id} but it has a "
                        f"model number '{found_model}' different than the one expected: '{expected_model_nb}'. "
                        f"Make sure you are connected only to the '{motor}' motor (model '{model}')."
                    )
                return baudrate, found_id

        raise RuntimeError(f"Motor '{motor}' (model '{model}') was not found. Make sure it is connected.")


__all__ = ["HiwonderMotorsBus", "DriveMode", "OperatingMode", "TorqueMode"]
