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
import platform
import signal
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager

from lerobot.utils.errors import DeviceNotConnectedError

logger = logging.getLogger(__name__)


def disconnect_all(*steps: Callable[[], object] | None) -> None:
    """Run every shutdown step even if earlier ones fail, then re-raise the first failure.

    Used to disconnect several devices so that one failing (or a Ctrl+C while it runs) does not leave the
    others powered. Each step's exception is caught, including KeyboardInterrupt, so a further Ctrl+C only ends
    the step in progress; the first exception is re-raised after the last step and later ones are logged at
    ERROR. DeviceNotConnectedError means the device is already disconnected and is not treated as a failure.
    `None` steps are skipped.
    """
    first: BaseException | None = None
    for step in steps:
        if step is None:
            continue
        name = getattr(step, "__qualname__", repr(step))
        try:
            step()
        except DeviceNotConnectedError as e:
            logger.info(f"{name}: already disconnected ({e})")
        except BaseException as e:  # noqa: BLE001 - shutdown must reach every step
            if first is None:
                first = e
            else:
                logger.error(f"{name} failed during shutdown after an earlier failure: {e!r}")
    if first is not None:
        raise first


@contextmanager
def exit_on_termination_signals() -> Iterator[Callable[[], None]]:
    """Turn SIGTERM and SIGHUP into SystemExit(128 + signal) so the caller's `finally` (motor torque off) runs.

    By default these signals (`kill`, an IDE stop button, closing the terminal) end Python without running
    `finally`, leaving motors powered. After the first one, further SIGTERM/SIGHUP are logged and ignored until
    the block exits: closing a terminal can send both, and a second SystemExit would end a shutdown step midway
    (the verified torque-off writes Goal_Position, which turns torque on, before Torque_Enable=0). Ctrl+C and
    SIGKILL are unchanged. Yields `restore`, which puts the previous handlers back early (once the motors are
    off, a later signal need not wait for slow steps such as video encoding); it also runs on exit. Does nothing
    off the main thread, where Python cannot set signal handlers.
    """
    if threading.current_thread() is not threading.main_thread():
        logger.debug("Not on the main thread; SIGTERM/SIGHUP handlers left unchanged.")
        yield lambda: None
        return
    signals = [signal.SIGTERM] + ([signal.SIGHUP] if hasattr(signal, "SIGHUP") else [])
    previous = {sig: signal.getsignal(sig) for sig in signals}

    def ignore(signum, frame):
        logger.warning(f"Received {signal.Signals(signum).name} again; ignored until shutdown finishes.")

    def exit_(signum, frame):
        for sig in signals:
            signal.signal(sig, ignore)
        logger.warning(f"Received {signal.Signals(signum).name}; shutting down.")
        raise SystemExit(128 + signum)

    def restore():
        while previous:
            sig, handler = previous.popitem()
            if handler is not None:  # None: set outside Python, cannot be restored from here
                signal.signal(sig, handler)

    for sig in signals:
        signal.signal(sig, exit_)
    try:
        yield restore
    finally:
        restore()


def precise_sleep(seconds: float, spin_threshold: float = 0.010, sleep_margin: float = 0.005):
    """
    Wait for `seconds` with better precision than time.sleep alone at the expense of more CPU usage.

    Parameters:
      - seconds: duration to wait
      - spin_threshold: if remaining <= spin_threshold -> spin; otherwise sleep (seconds). Default 10ms
      - sleep_margin: when sleeping leave this much time before deadline to avoid oversleep. Default 5ms

    Note:
        The default parameters are chosen to prioritize timing accuracy over CPU usage for the common 30 FPS use case.
    """
    if seconds <= 0:
        return

    system = platform.system()
    # On macOS and Windows the scheduler / sleep granularity can make
    # short sleeps inaccurate. Instead of burning CPU for the whole
    # duration, sleep for most of the time and spin for the final few
    # milliseconds to achieve good accuracy with much lower CPU usage.
    if system in ("Darwin", "Windows"):
        end_time = time.perf_counter() + seconds
        while True:
            remaining = end_time - time.perf_counter()
            if remaining <= 0:
                break
            # If there's more than a couple milliseconds left, sleep most
            # of the remaining time and leave a small margin for the final spin.
            if remaining > spin_threshold:
                # Sleep but avoid sleeping past the end by leaving a small margin.
                time.sleep(max(remaining - sleep_margin, 0))
            else:
                # Final short spin to hit precise timing without long sleeps.
                pass
    else:
        # On Linux time.sleep is accurate enough for most uses
        time.sleep(seconds)
