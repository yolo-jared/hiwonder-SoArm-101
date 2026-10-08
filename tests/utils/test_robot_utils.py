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

import logging

import pytest

from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import disconnect_all


def step(log: list[str], name: str, exc: BaseException | None = None):
    def run():
        log.append(name)
        if exc is not None:
            raise exc

    run.__qualname__ = name
    return run


def test_disconnect_all_runs_every_step_and_raises_first_error(caplog):
    log = []
    first = RuntimeError("follower torque still on")
    with pytest.raises(RuntimeError) as e:
        disconnect_all(
            step(log, "s1", first), step(log, "s2"), step(log, "s3", ValueError("leader port gone"))
        )
    assert e.value is first
    assert log == ["s1", "s2", "s3"]
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert any("s3" in r.getMessage() and "leader port gone" in r.getMessage() for r in errors)


def test_disconnect_all_without_errors_returns():
    log = []
    assert disconnect_all(step(log, "s1"), None, step(log, "s2")) is None
    assert log == ["s1", "s2"]


def test_disconnect_all_runs_remaining_steps_after_keyboard_interrupt():
    """FA-13: a Ctrl+C during the first step does not skip the others, and is re-raised after them."""
    log = []
    with pytest.raises(KeyboardInterrupt):
        disconnect_all(step(log, "s1", KeyboardInterrupt()), step(log, "s2"))
    assert log == ["s1", "s2"]


def test_disconnect_all_second_interrupt_skips_only_current_step(caplog):
    """FA-13: each further Ctrl+C ends only the step in progress; the first is re-raised, later ones logged."""
    log = []
    first = KeyboardInterrupt("first")
    with pytest.raises(KeyboardInterrupt) as e:
        disconnect_all(step(log, "s1", first), step(log, "s2", KeyboardInterrupt("second")), step(log, "s3"))
    assert e.value is first
    assert log == ["s1", "s2", "s3"]
    assert any(r.levelno == logging.ERROR and "s2" in r.getMessage() for r in caplog.records)


def test_disconnect_all_treats_device_not_connected_as_done(caplog):
    """FA-20: an already-disconnected device is not an error."""
    caplog.set_level(logging.INFO)
    log = []
    disconnect_all(step(log, "s1", DeviceNotConnectedError("not connected")), step(log, "s2"))
    assert log == ["s1", "s2"]
    assert any(r.levelno == logging.INFO and "s1" in r.getMessage() for r in caplog.records)
