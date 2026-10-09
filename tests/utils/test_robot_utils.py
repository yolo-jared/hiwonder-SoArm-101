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
import os
import signal
import threading
import time

import pytest

from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import disconnect_all, exit_on_termination_signals


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


# --------------------------------------------------------------------------------------------------------------
# Termination signals (FA-28): SIGTERM/SIGHUP must run the shutdown in `finally`, not kill the process mid-run.
# --------------------------------------------------------------------------------------------------------------

TERM_SIGNALS = [signal.SIGTERM] + ([signal.SIGHUP] if hasattr(signal, "SIGHUP") else [])


class SignalNotHandled(BaseException):
    """Stands in for the default action (process killed, `finally` skipped) so a red test fails cleanly."""


@pytest.fixture
def sentinel_signals():
    def handler(signum, frame):
        raise SignalNotHandled(signum)

    saved = {s: signal.getsignal(s) for s in TERM_SIGNALS}
    for s in TERM_SIGNALS:
        signal.signal(s, handler)
    yield handler
    for s, h in saved.items():
        signal.signal(s, h)


def send(sig):
    os.kill(os.getpid(), sig)
    time.sleep(0.2)  # the handler runs in the main thread at the next bytecode boundary


@pytest.mark.parametrize("sig", TERM_SIGNALS, ids=lambda s: s.name)
def test_termination_signal_raises_system_exit_with_shell_code(sentinel_signals, sig):
    with pytest.raises(SystemExit) as e, exit_on_termination_signals():
        send(sig)
    assert e.value.code == 128 + sig


def test_termination_signal_runs_finally(sentinel_signals):
    log = []
    with pytest.raises(SystemExit), exit_on_termination_signals():
        try:
            send(signal.SIGTERM)
        finally:
            log.append("torque off")
    assert log == ["torque off"]


def test_previous_handlers_restored(sentinel_signals):
    with exit_on_termination_signals():
        assert signal.getsignal(signal.SIGTERM) is not sentinel_signals
    assert all(signal.getsignal(s) is sentinel_signals for s in TERM_SIGNALS)


def test_previous_handlers_restored_after_signal(sentinel_signals):
    with pytest.raises(SystemExit), exit_on_termination_signals():
        send(signal.SIGTERM)
    assert all(signal.getsignal(s) is sentinel_signals for s in TERM_SIGNALS)


def test_repeat_signal_during_shutdown_is_ignored(sentinel_signals, caplog):
    """A second SIGTERM/SIGHUP (closing a terminal can send both) must not end a torque-off step midway: the
    verified torque-off writes Goal=Present, which turns torque on, before it writes Torque_Enable=0."""
    log = []
    first = TERM_SIGNALS[-1]
    with pytest.raises(SystemExit) as e, exit_on_termination_signals():
        try:
            send(first)
        finally:
            log.append("goal=present")
            send(signal.SIGTERM)
            log.append("torque_enable=0")
    assert log == ["goal=present", "torque_enable=0"]
    assert e.value.code == 128 + first
    assert any(r.levelno == logging.WARNING and "SIGTERM" in r.getMessage() for r in caplog.records)


def test_off_main_thread_is_a_no_op(sentinel_signals):
    errors = []

    def run():
        try:
            with exit_on_termination_signals():
                pass
        except BaseException as e:  # noqa: BLE001
            errors.append(e)

    t = threading.Thread(target=run)
    t.start()
    t.join()
    assert errors == []
    assert signal.getsignal(signal.SIGTERM) is sentinel_signals


def test_restore_ends_signal_handling_early(sentinel_signals):
    """record() restores the previous handlers once the arms are off: a later `kill` during a long video encode
    or upload is then not ignored."""
    with exit_on_termination_signals() as restore:
        restore()
        assert signal.getsignal(signal.SIGTERM) is sentinel_signals
        restore()  # idempotent
    assert signal.getsignal(signal.SIGTERM) is sentinel_signals
