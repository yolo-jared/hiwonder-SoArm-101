"""Session safeguards tested with fake hardware and real process locks only."""
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from examples.hiwonder import elbow_session as session


def ports():
    return [SimpleNamespace(device='/dev/leader', serial_number='L'), SimpleNamespace(device='/dev/follower', serial_number='F')]


@pytest.mark.parametrize('which', ['L', 'F'])
def test_contention_releases_partial_acquisitions(tmp_path, which):
    with session.DeviceLocks([which], tmp_path):
        with pytest.raises(RuntimeError):
            with session.DeviceLocks(['L', 'F'], tmp_path): pass
    with session.DeviceLocks(['L', 'F'], tmp_path): pass


def test_same_adapter_alias_rejected():
    candidates = ports()
    candidates[1].serial_number = 'L'
    with pytest.raises(ValueError): session.identify_ports('/dev/leader', '/dev/follower', candidates)


def test_missing_serial_rejected():
    candidates = ports()
    candidates[0].serial_number = None
    with pytest.raises(ValueError): session.identify_ports('/dev/leader', '/dev/follower', candidates)


def test_alias_canonicalization(monkeypatch):
    monkeypatch.setattr(session.os.path, 'realpath', lambda p: '/dev/leader' if p == '/alias' else p)
    assert session.identify_ports('/alias', '/dev/follower', ports()) == ['L', 'F']


def test_timeout_interrupts_blocking_call_and_restores_timer():
    with pytest.raises(TimeoutError):
        with session.Deadline(.02): time.sleep(.2)
    with session.Deadline(.1): pass


def test_nested_deadline_cannot_extend_parent():
    with pytest.raises(TimeoutError):
        with session.Deadline(.02):
            with session.Deadline(.2): time.sleep(.1)


def test_snapshot_distinguishes_zero_unsupported_and_failure():
    bus = Mock()
    bus.motors = {'elbow_flex': SimpleNamespace(id=3, model='hx30hm')}
    bus.model_ctrl_table = {'hx30hm': {'P_Coefficient': (21, 1), 'Torque_Enable': (40, 1)}}
    bus.read.return_value = 0
    snap = session.snapshot(bus, {})
    assert snap['motors']['elbow_flex']['P_Coefficient'] == {'state': 'read', 'raw': 0}
    assert snap['motors']['elbow_flex']['Firmware_Major_Version']['state'] == 'unsupported'
    bus.read.side_effect = OSError('disconnected')
    assert session.snapshot(bus, {})['motors']['elbow_flex']['P_Coefficient']['state'] == 'error'
    assert snap['unit_interpretation'] == 'unverified'


def test_configuration_change_prevents_enable():
    with pytest.raises(RuntimeError): session.require_unchanged({'gain': 16}, {'gain': 32})


@pytest.mark.parametrize('age', [.501, 1, float('nan')])
def test_stale_batch_rejected_after_logging(age):
    with pytest.raises(RuntimeError): session.require_fresh(0, age)


def test_partial_connect_closes_every_owned_port_without_torque_writes(tmp_path):
    first, second = Mock(), Mock()
    first.is_connected = True
    second.is_connected = True
    second.connect.side_effect = KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        with session.HardwareSession([first, second], ['L', 'F'], tmp_path): pass
    first.disconnect.assert_called_once_with(disable_torque=False)
    second.disconnect.assert_called_once_with(disable_torque=False)
    first.disable_torque.assert_not_called()
    second.disable_torque.assert_not_called()
    with session.DeviceLocks(['L', 'F'], tmp_path): pass


def test_shutdown_only_writes_torque_not_persistent_lock():
    bus = Mock()
    bus.motors = {'elbow_flex': object()}
    bus.read.return_value = 0
    assert session.shutdown(bus) == 'verified off'
    bus.write.assert_called_once_with('Torque_Enable', 'elbow_flex', 0, normalize=False, num_retry=0)
    assert session.cleanup_budget(bus) == 2.0


def test_shutdown_tries_other_motor_after_failure():
    bus = Mock()
    bus.motors = {'elbow_flex': object(), 'shoulder_lift': object()}
    bus.write.side_effect = OSError('unplugged')
    assert session.shutdown(bus) == 'unverified'
    assert bus.write.call_count == 6


def test_existing_timer_is_preserved():
    import signal
    signal.setitimer(signal.ITIMER_REAL, 10)
    try:
        with session.Deadline(.1): pass
        remaining, _ = signal.getitimer(signal.ITIMER_REAL)
        assert 9 < remaining <= 10
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
