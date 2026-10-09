#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import os
import signal
import time
from unittest.mock import MagicMock, patch

import pytest
import rerun as rr

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.scripts import lerobot_record, lerobot_teleoperate
from lerobot.scripts.lerobot_calibrate import CalibrateConfig, calibrate
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.scripts.lerobot_replay import DatasetReplayConfig, ReplayConfig, replay
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate
from tests.fixtures.constants import DUMMY_REPO_ID
from tests.mocks.mock_robot import MockRobot, MockRobotConfig
from tests.mocks.mock_teleop import MockTeleop, MockTeleopConfig


def test_calibrate():
    robot_cfg = MockRobotConfig()
    cfg = CalibrateConfig(robot=robot_cfg)
    calibrate(cfg)


def test_teleoperate():
    robot_cfg = MockRobotConfig()
    teleop_cfg = MockTeleopConfig()
    cfg = TeleoperateConfig(
        robot=robot_cfg,
        teleop=teleop_cfg,
        teleop_time_s=0.1,
    )
    teleoperate(cfg)


def test_record_and_resume(tmp_path):
    robot_cfg = MockRobotConfig()
    teleop_cfg = MockTeleopConfig()
    dataset_cfg = DatasetRecordConfig(
        repo_id=DUMMY_REPO_ID,
        single_task="Dummy task",
        root=tmp_path / "record",
        num_episodes=1,
        episode_time_s=0.1,
        reset_time_s=0,
        push_to_hub=False,
    )
    cfg = RecordConfig(
        robot=robot_cfg,
        dataset=dataset_cfg,
        teleop=teleop_cfg,
        play_sounds=False,
    )

    dataset = record(cfg)

    assert dataset.fps == 30
    assert dataset.meta.total_episodes == dataset.num_episodes == 1
    assert dataset.meta.total_frames == dataset.num_frames == 3
    assert dataset.meta.total_tasks == 1

    cfg.resume = True
    # Mock the revision to prevent Hub calls during resume
    with (
        patch("lerobot.datasets.dataset_metadata.get_safe_version") as mock_get_safe_version,
        patch("lerobot.datasets.dataset_metadata.snapshot_download") as mock_snapshot_download,
    ):
        mock_get_safe_version.return_value = "v3.0"
        mock_snapshot_download.return_value = str(tmp_path / "record")
        dataset = record(cfg)

    assert dataset.meta.total_episodes == dataset.num_episodes == 2
    assert dataset.meta.total_frames == dataset.num_frames == 6
    assert dataset.meta.total_tasks == 1


def test_record_and_replay(tmp_path):
    robot_cfg = MockRobotConfig()
    teleop_cfg = MockTeleopConfig()
    record_dataset_cfg = DatasetRecordConfig(
        repo_id=DUMMY_REPO_ID,
        single_task="Dummy task",
        root=tmp_path / "record_and_replay",
        num_episodes=1,
        episode_time_s=0.1,
        push_to_hub=False,
    )
    record_cfg = RecordConfig(
        robot=robot_cfg,
        dataset=record_dataset_cfg,
        teleop=teleop_cfg,
        play_sounds=False,
    )
    replay_dataset_cfg = DatasetReplayConfig(
        repo_id=DUMMY_REPO_ID,
        episode=0,
        root=tmp_path / "record_and_replay",
    )
    replay_cfg = ReplayConfig(
        robot=robot_cfg,
        dataset=replay_dataset_cfg,
        play_sounds=False,
    )

    record(record_cfg)

    # Mock the revision to prevent Hub calls during replay
    with (
        patch("lerobot.datasets.dataset_metadata.get_safe_version") as mock_get_safe_version,
        patch("lerobot.datasets.dataset_metadata.snapshot_download") as mock_snapshot_download,
    ):
        mock_get_safe_version.return_value = "v3.0"
        mock_snapshot_download.return_value = str(tmp_path / "record_and_replay")
        replay(replay_cfg)


# --------------------------------------------------------------------------------------------------------------
# Shutdown order and error handling (issue #2: R1, R5, FA-12, FA-14, FA-20, FA-22)
# --------------------------------------------------------------------------------------------------------------


def _recording_disconnect(order: list[str], name: str, exc: BaseException | None = None):
    def disconnect(self):
        order.append(name)
        self._is_connected = False
        if exc is not None:
            raise exc

    return disconnect


def _patch_disconnects(monkeypatch, order, robot_exc=None, teleop_exc=None):
    monkeypatch.setattr(MockRobot, "disconnect", _recording_disconnect(order, "robot", robot_exc))
    monkeypatch.setattr(MockTeleop, "disconnect", _recording_disconnect(order, "teleop", teleop_exc))


def _teleop_cfg(display_data=False):
    return TeleoperateConfig(
        robot=MockRobotConfig(), teleop=MockTeleopConfig(), teleop_time_s=0.1, display_data=display_data
    )


def test_teleoperate_disconnects_robot_before_teleop(monkeypatch):
    """FA-12: the follower is shut down first, so a hung leader adapter cannot keep it powered."""
    order = []
    _patch_disconnects(monkeypatch, order)
    teleoperate(_teleop_cfg())
    assert order == ["robot", "teleop"]


def test_teleoperate_robot_disconnect_error_still_disconnects_teleop(monkeypatch):
    """R1: the first disconnect raising does not skip the second; the error is re-raised after."""
    order = []
    _patch_disconnects(monkeypatch, order, robot_exc=RuntimeError("Torque-off not confirmed"))
    with pytest.raises(RuntimeError, match="Torque-off not confirmed"):
        teleoperate(_teleop_cfg())
    assert order == ["robot", "teleop"]


def test_teleoperate_rerun_shutdown_error_still_disconnects_arms(monkeypatch):
    """FA-14: rerun shutdown runs after both arms and its error cannot skip them."""
    order = []
    _patch_disconnects(monkeypatch, order)
    monkeypatch.setattr(lerobot_teleoperate, "init_rerun", lambda **kwargs: None)
    monkeypatch.setattr(lerobot_teleoperate, "log_rerun_data", lambda *args, **kwargs: None)

    def rerun_shutdown():
        order.append("rerun")
        raise RuntimeError("rerun shutdown failed")

    monkeypatch.setattr(rr, "rerun_shutdown", rerun_shutdown)
    with pytest.raises(RuntimeError, match="rerun shutdown failed"):
        teleoperate(_teleop_cfg(display_data=True))
    assert order == ["robot", "teleop", "rerun"]


def _record_cfg(tmp_path):
    return RecordConfig(
        robot=MockRobotConfig(),
        teleop=MockTeleopConfig(),
        dataset=DatasetRecordConfig(
            repo_id=DUMMY_REPO_ID,
            single_task="Dummy task",
            root=tmp_path / "record",
            num_episodes=1,
            episode_time_s=0.1,
            reset_time_s=0,
            push_to_hub=False,
        ),
        play_sounds=False,
    )


def _instrument_record(monkeypatch, order, say_exc=None, finalize_exc=None):
    """Record shutdown events in `order`: say:<text>, finalize, listener.stop (robot/teleop via _patch_disconnects)."""

    def log_say(text, play_sounds=True, blocking=False):
        order.append(f"say:{text}")
        if say_exc is not None and text == "Stop recording":
            raise say_exc

    real_finalize = LeRobotDataset.finalize

    def finalize(self):
        # Logged only when it does work: finalize() is idempotent and record() calls it from both
        # VideoEncodingManager.__exit__ and the outer shutdown.
        if self._is_finalized:
            return real_finalize(self)
        order.append("finalize")
        real_finalize(self)
        if finalize_exc is not None:
            raise finalize_exc

    listener = MagicMock(name="listener")
    listener.stop.side_effect = lambda: order.append("listener.stop")
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    monkeypatch.setattr(lerobot_record, "log_say", log_say)
    monkeypatch.setattr(LeRobotDataset, "finalize", finalize)
    monkeypatch.setattr(lerobot_record, "init_keyboard_listener", lambda: (listener, events))
    monkeypatch.setattr(lerobot_record, "is_headless", lambda: False)


def _shutdown(order):
    return order[order.index("robot") :]


def test_record_shutdown_order(tmp_path, monkeypatch):
    """R5, FA-14, FA-22: arms first, then finalize, then the spoken message, then the keyboard listener."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)
    record(_record_cfg(tmp_path))
    assert _shutdown(order)[:5] == ["robot", "teleop", "finalize", "say:Stop recording", "listener.stop"]


def test_record_finalize_error_still_disconnects_arms(tmp_path, monkeypatch):
    """R5: a finalize error is raised only after both arms are disconnected."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order, finalize_exc=RuntimeError("finalize failed"))
    with pytest.raises(RuntimeError, match="finalize failed"):
        record(_record_cfg(tmp_path))
    assert _shutdown(order)[:3] == ["robot", "teleop", "finalize"]
    assert "listener.stop" in order


def test_record_log_say_error_still_disconnects_arms(tmp_path, monkeypatch):
    """FA-14: the blocking "Stop recording" message runs after the arms; its error cannot skip them."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order, say_exc=RuntimeError("audio device busy"))
    with pytest.raises(RuntimeError, match="audio device busy"):
        record(_record_cfg(tmp_path))
    shutdown = _shutdown(order)
    assert shutdown.index("teleop") < shutdown.index("say:Stop recording")
    assert "finalize" in shutdown


def test_record_listener_stopped_when_arm_disconnect_raises(tmp_path, monkeypatch):
    """FA-22: a torque-off error does not skip finalize or the keyboard listener; it is re-raised after."""
    order = []
    _patch_disconnects(monkeypatch, order, robot_exc=RuntimeError("Torque-off not confirmed"))
    _instrument_record(monkeypatch, order)
    with pytest.raises(RuntimeError, match="Torque-off not confirmed"):
        record(_record_cfg(tmp_path))
    assert _shutdown(order)[:5] == ["robot", "teleop", "finalize", "say:Stop recording", "listener.stop"]


def test_record_does_not_disconnect_devices_that_never_connected(tmp_path, monkeypatch):
    """Devices whose connect() never ran are not disconnected (some robots' disconnect sends commands
    without checking); the original error surfaces."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)

    def create(*args, **kwargs):
        raise RuntimeError("dataset creation failed")

    monkeypatch.setattr(LeRobotDataset, "create", create)
    with pytest.raises(RuntimeError, match="dataset creation failed"):
        record(_record_cfg(tmp_path))
    assert "robot" not in order
    assert "teleop" not in order


def test_record_camera_dropped_still_disconnects_follower(tmp_path, monkeypatch):
    """FA-20: a robot whose is_connected turned False (a camera dropped) is still disconnected."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)
    monkeypatch.setattr(MockRobot, "camera_dropped", False, raising=False)
    monkeypatch.setattr(
        MockRobot, "is_connected", property(lambda self: self._is_connected and not self.camera_dropped)
    )
    real_save_episode = LeRobotDataset.save_episode

    def save_episode(self, *args, **kwargs):
        real_save_episode(self, *args, **kwargs)
        MockRobot.camera_dropped = True

    monkeypatch.setattr(LeRobotDataset, "save_episode", save_episode)
    record(_record_cfg(tmp_path))
    assert "robot" in order


# --------------------------------------------------------------------------------------------------------------
# Remaining shutdown cases: video encoding after torque-off, SIGTERM/SIGHUP (FA-28), startup failing partway (R7)
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


def _send(sig):
    os.kill(os.getpid(), sig)
    time.sleep(0.2)


def test_record_disconnects_arms_before_video_encoding(tmp_path, monkeypatch):
    """VideoEncodingManager.__exit__ encodes the videos (can take minutes); the arms are off before it starts."""
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)
    record(_record_cfg(tmp_path))
    assert order.index("robot") < order.index("teleop") < order.index("finalize")


def test_record_ctrl_c_mid_episode_disconnects_arms_before_finalize(tmp_path, monkeypatch):
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)

    def interrupted(**kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(lerobot_record, "record_loop", interrupted)
    with pytest.raises(KeyboardInterrupt):
        record(_record_cfg(tmp_path))
    assert order.index("robot") < order.index("teleop") < order.index("finalize")


def test_record_disconnects_each_arm_once(tmp_path, monkeypatch):
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)
    record(_record_cfg(tmp_path))
    assert order.count("robot") == 1
    assert order.count("teleop") == 1


def test_record_arm_error_does_not_cancel_video_encoding(tmp_path, monkeypatch):
    """A torque-off error is re-raised after the dataset is saved; it does not reach VideoEncodingManager as an
    exception (which would cancel pending videos)."""
    order = []
    _patch_disconnects(monkeypatch, order, robot_exc=RuntimeError("Torque-off not confirmed"))
    _instrument_record(monkeypatch, order)
    seen = []
    real_exit = lerobot_record.VideoEncodingManager.__exit__

    def vem_exit(self, exc_type, exc_val, exc_tb):
        seen.append(exc_type)
        return real_exit(self, exc_type, exc_val, exc_tb)

    monkeypatch.setattr(lerobot_record.VideoEncodingManager, "__exit__", vem_exit)
    with pytest.raises(RuntimeError, match="Torque-off not confirmed"):
        record(_record_cfg(tmp_path))
    assert seen == [None]
    assert order.index("robot") < order.index("finalize")
    assert "listener.stop" in order


def test_record_failure_before_recording_still_disconnects_arms(tmp_path, monkeypatch):
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)

    def no_listener():
        raise RuntimeError("keyboard listener failed")

    monkeypatch.setattr(lerobot_record, "init_keyboard_listener", no_listener)
    with pytest.raises(RuntimeError, match="keyboard listener failed"):
        record(_record_cfg(tmp_path))
    assert order[:2] == ["robot", "teleop"]


def _signal_in_loop(sig):
    def loop(*args, **kwargs):
        _send(sig)

    return loop


@pytest.mark.parametrize("sig", TERM_SIGNALS, ids=lambda s: s.name)
def test_teleoperate_termination_signal_disconnects_arms(monkeypatch, sentinel_signals, sig):
    """FA-28: closing the terminal (SIGHUP) or `kill` (SIGTERM) runs the shutdown instead of killing the process."""
    order = []
    _patch_disconnects(monkeypatch, order)
    monkeypatch.setattr(lerobot_teleoperate, "teleop_loop", _signal_in_loop(sig))
    with pytest.raises(SystemExit) as e:
        teleoperate(_teleop_cfg())
    assert e.value.code == 128 + sig
    assert order == ["robot", "teleop"]
    assert signal.getsignal(sig) is sentinel_signals


@pytest.mark.parametrize("sig", TERM_SIGNALS, ids=lambda s: s.name)
def test_record_termination_signal_disconnects_arms_then_saves(tmp_path, monkeypatch, sentinel_signals, sig):
    order = []
    _patch_disconnects(monkeypatch, order)
    _instrument_record(monkeypatch, order)
    monkeypatch.setattr(lerobot_record, "record_loop", _signal_in_loop(sig))
    with pytest.raises(SystemExit) as e:
        record(_record_cfg(tmp_path))
    assert e.value.code == 128 + sig
    assert order.index("robot") < order.index("teleop") < order.index("finalize")
    assert signal.getsignal(sig) is sentinel_signals


def test_teleoperate_second_signal_does_not_cut_robot_disconnect_short(monkeypatch, sentinel_signals):
    """Closing a terminal can send SIGHUP then SIGTERM; the second must not end the follower's torque-off midway."""
    order = []

    def robot_disconnect(self):
        order.append("robot:start")
        _send(signal.SIGTERM)
        order.append("robot:end")
        self._is_connected = False

    monkeypatch.setattr(MockRobot, "disconnect", robot_disconnect)
    monkeypatch.setattr(MockTeleop, "disconnect", _recording_disconnect(order, "teleop"))
    monkeypatch.setattr(lerobot_teleoperate, "teleop_loop", _signal_in_loop(TERM_SIGNALS[-1]))
    with pytest.raises(SystemExit):
        teleoperate(_teleop_cfg())
    assert order == ["robot:start", "robot:end", "teleop"]


def _order_when_raised(order, exc_type, match):
    """Disconnects done by the time the error leaves teleoperate(). Read inside `except`, while the traceback
    still holds the devices: once it is dropped, Robot/Teleoperator.__del__ may disconnect them, which a real
    crash does not guarantee."""
    try:
        teleoperate(_teleop_cfg())
    except exc_type as e:
        assert match in str(e)
        return list(order)
    raise AssertionError(f"{exc_type.__name__} not raised")


def test_teleoperate_robot_connect_failure_disconnects_leader(monkeypatch):
    """R7: the leader connected first is disconnected when the follower's connect raises."""
    order = []
    _patch_disconnects(monkeypatch, order)

    def connect(self, calibrate=True):
        raise ConnectionError("follower port busy")

    monkeypatch.setattr(MockRobot, "connect", connect)
    assert _order_when_raised(order, ConnectionError, "follower port busy") == ["teleop"]


def test_teleoperate_leader_connect_failure_disconnects_nothing(monkeypatch):
    order = []
    _patch_disconnects(monkeypatch, order)

    def connect(self, calibrate=True):
        raise ConnectionError("leader port busy")

    monkeypatch.setattr(MockTeleop, "connect", connect)
    assert _order_when_raised(order, ConnectionError, "leader port busy") == []
