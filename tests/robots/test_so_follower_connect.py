"""R7: SOFollower.connect() failing partway closes what it opened and re-raises the original error."""

import logging
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def _bus_mock(connect_exc=None, port_open_after_failure=False, disconnect_exc=None, **kwargs) -> MagicMock:
    bus = MagicMock(name="BusMock")
    bus.motors = kwargs["motors"]
    bus.is_connected = False
    bus.is_calibrated = True

    def connect():
        bus.is_connected = True
        if connect_exc is not None:
            bus.is_connected = port_open_after_failure
            raise connect_exc

    def disconnect(disable_torque=True):
        bus.is_connected = False
        if disconnect_exc is not None:
            raise disconnect_exc

    @contextmanager
    def torque_disabled(*args, **kwargs):
        yield

    bus.connect.side_effect = connect
    bus.disconnect.side_effect = disconnect
    bus.torque_disabled.side_effect = torque_disabled
    return bus


def _robot(**bus_kwargs) -> SO101Follower:
    with patch("lerobot.motors.hiwonder.HiwonderMotorsBus", side_effect=lambda *a, **kw: _bus_mock(**bus_kwargs, **kw)):
        return SO101Follower(SO101FollowerConfig(port="/dev/null", motor_model="hx30hm"))


def test_configure_failure_disconnects_bus_with_torque_off():
    robot = _robot()
    with (
        patch.object(SO101Follower, "configure", side_effect=RuntimeError("Torque-off not confirmed")),
        pytest.raises(RuntimeError, match="Torque-off not confirmed"),
    ):
        robot.connect()
    robot.bus.disconnect.assert_called_once_with(robot.config.disable_torque_on_disconnect)
    assert not robot.bus.is_connected


def test_calibration_interrupted_disconnects_bus():
    """Ctrl+C at the calibration prompt."""
    robot = _robot()
    robot.bus.is_calibrated = False
    with (
        patch.object(SO101Follower, "calibrate", side_effect=KeyboardInterrupt),
        pytest.raises(KeyboardInterrupt),
    ):
        robot.connect()
    robot.bus.disconnect.assert_called_once_with(robot.config.disable_torque_on_disconnect)


def test_camera_failure_disconnects_bus():
    robot = _robot()
    cam = MagicMock(name="cam")
    cam.is_connected = False
    cam.connect.side_effect = ConnectionError("camera busy")
    robot.cameras = {"front": cam}
    with patch.object(SO101Follower, "configure") as configure, pytest.raises(ConnectionError, match="camera busy"):
        robot.connect()
    configure.assert_not_called()
    robot.bus.disconnect.assert_called_once()


def test_bus_connect_failure_closes_port_without_motor_writes():
    """A failed handshake (e.g. a motor with a fault besides Overload, deliberately not written) leaves the port
    open; cleanup closes it with no torque-off, since that can write Goal_Position to the faulted motor."""
    robot = _robot(connect_exc=RuntimeError("power-cycle the servos"), port_open_after_failure=True)
    with pytest.raises(RuntimeError, match="power-cycle"):
        robot.connect()
    robot.bus.disconnect.assert_called_once_with(False)
    robot.bus.write.assert_not_called()
    robot.bus.disable_torque.assert_not_called()


def test_bus_connect_failure_with_port_closed_does_nothing():
    robot = _robot(connect_exc=ConnectionError("no such port"))
    with pytest.raises(ConnectionError, match="no such port"):
        robot.connect()
    robot.bus.disconnect.assert_not_called()


def test_cleanup_error_does_not_hide_the_original(caplog):
    robot = _robot(disconnect_exc=RuntimeError("Torque-off not confirmed on gripper"))
    with (
        patch.object(SO101Follower, "configure", side_effect=ValueError("bad P coefficient")),
        pytest.raises(ValueError, match="bad P coefficient"),
    ):
        robot.connect()
    assert any(
        r.levelno == logging.ERROR and "Torque-off not confirmed on gripper" in r.getMessage() for r in caplog.records
    )


def test_successful_connect_does_not_disconnect():
    robot = _robot()
    with patch.object(SO101Follower, "configure"):
        robot.connect()
    robot.bus.disconnect.assert_not_called()
