#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

"""Verified torque-off for HiwonderMotorsBus (issue #2) and the SDK reply-length check."""

import pytest

from lerobot.motors.hiwonder import hiwonder_sdk as hw

pytestmark = pytest.mark.timeout(10)

TE_ADDR, GOAL_ADDR, LOCK_ADDR, PRESENT_ADDR = 40, 42, 55, 56


def _checksum(packet: list[int]) -> int:
    return ~sum(packet[2:-1]) & 0xFF


def status_packet(hw_id: int, params: list[int], error: int = 0, length: int | None = None) -> bytes:
    """A status packet; `length` overrides the LEN byte (default len(params) + 2)."""
    packet = [0xFF, 0xFF, hw_id, len(params) + 2 if length is None else length, error, *params, 0]
    packet[-1] = _checksum(packet)
    return bytes(packet)


class ByteServoLine:
    """Byte-level fake of the serial line: parses instruction packets and answers like HX-30HM servos.

    `memory[id][addr]` holds register bytes. `queued` replies (bytes) are served before the servo's own
    reply, which models a stale reply left in the input buffer.
    """

    def __init__(self, memory: dict[int, dict[int, int]]):
        self.memory = memory
        self.rx = bytearray()
        self.queued: list[bytes] = []
        self.errors: dict[int, int] = {}
        self.raise_on_read: list[BaseException] = []
        self.on_write = None

    # pyserial surface used by PortHandler
    def write(self, data) -> int:
        data = list(data)
        if self.on_write is not None:
            self.on_write(data)
        hw_id, instr, params = data[2], data[4], data[5:-1]
        reply = b""
        if instr == hw.INST_READ:
            addr, n = params
            regs = self.memory[hw_id]
            reply = status_packet(hw_id, [regs.get(addr + i, 0) for i in range(n)], self.errors.get(hw_id, 0))
        elif instr == hw.INST_WRITE:
            addr, values = params[0], params[1:]
            for i, v in enumerate(values):
                self.memory[hw_id][addr + i] = v
            reply = status_packet(hw_id, [], self.errors.get(hw_id, 0))
        for stale in self.queued:
            self.rx += stale
        self.queued.clear()
        self.rx += reply
        return len(data)

    def read(self, n: int) -> bytes:
        if self.raise_on_read:
            raise self.raise_on_read.pop(0)
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        return len(self.rx)


def make_packet_handler(line: ByteServoLine) -> hw.PacketHandler:
    port = hw.PortHandler("/dev/fake")
    port.ser = line
    port.is_open = True
    port.tx_time_per_byte = (1000.0 / port.baudrate) * 10.0
    return hw.PacketHandler(port)


# --------------------------------------------------------------------------------------------------------------
# T1 / R4: readData rejects a reply whose LEN byte does not match the request
# --------------------------------------------------------------------------------------------------------------


def test_read_reply_with_matching_len_is_accepted():
    line = ByteServoLine({6: {TE_ADDR: 1, PRESENT_ADDR: 0x02, PRESENT_ADDR + 1: 0x08}})
    ph = make_packet_handler(line)
    assert ph.read1ByteData(6, TE_ADDR) == (1, hw.COMM_SUCCESS, 0)
    assert ph.read2ByteData(6, PRESENT_ADDR) == (0x0802, hw.COMM_SUCCESS, 0)


def test_stale_write_ack_is_not_decoded_as_a_one_byte_read():
    """A late write ack (LEN=2, no data) for the same ID would otherwise be read as Torque_Enable."""
    line = ByteServoLine({6: {TE_ADDR: 1}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, []))  # stale ack of an earlier write
    value, comm, _ = ph.read1ByteData(6, TE_ADDR)
    assert comm == hw.COMM_RX_CORRUPT
    assert value == 0  # SDK convention on failure; callers must look at comm


def test_one_byte_reply_to_a_two_byte_read_is_rejected():
    line = ByteServoLine({6: {}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, [0x00]))  # stale Torque_Enable reply (LEN=3)
    _, comm, _ = ph.read2ByteData(6, PRESENT_ADDR)
    assert comm == hw.COMM_RX_CORRUPT


def test_wrong_len_reply_is_rejected_by_read_data():
    line = ByteServoLine({6: {}})
    ph = make_packet_handler(line)
    line.queued.append(status_packet(6, [0x10, 0x20]))  # 2-byte reply to a 1-byte read
    data, comm, _ = ph.readData(6, TE_ADDR, 1)
    assert comm == hw.COMM_RX_CORRUPT
    assert data == []
