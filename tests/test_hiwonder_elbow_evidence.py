"""Offline evidence contracts; no hardware imports or ports."""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest

from examples.hiwonder import elbow_evidence as evidence


def dense_run(travel=50):
    events = [
        {'event': 'start', 'baseline': {'elbow_flex': 2000}, 'direction': 1},
        {'event': 'command', 't': 3.0, 'goal': 2056},
        {'event': 'readback', 't': 3.1, 'goal': 2056, 'expected': 2056},
    ]
    events += [{'event': 'sample', 'read_start': t, 'read_end': t + .01,
                'actual': {'elbow_flex': 2000 + travel}} for t in (3.51, 3.62, 3.73, 3.84, 3.95)]
    events += [{'event': 'command', 't': 4.1, 'goal': 2055},
               {'event': 'final', 'execution': 'completed', 'shutdown': 'verified off',
                'restoration': 'not applicable', 'return_error_ticks': 0, 'guard_events': []}]
    # Last 0.5 seconds before 4.1 contain four complete samples, spanning .33s.
    return [dict(e, version=1, run_id='run', seq=i) for i, e in enumerate(events)]


def test_dense_window_and_separate_observation():
    result = evidence.analyze(dense_run())
    assert result['encoder'] == 'passed'
    assert result['hold_median_ticks'] == 50
    assert result['physical_accepted'] is False
    assert evidence.exit_code(result) == 0
    assert 'not physical acceptance' in evidence.format_result(result)


@pytest.mark.parametrize('travel,verdict', [(44, 'insufficient travel'), (45, 'passed'), (16, 'insufficient travel')])
def test_unrounded_threshold(travel, verdict):
    assert evidence.analyze(dense_run(travel))['encoder'] == verdict


def test_spike_cannot_pass():
    events = dense_run(16)
    events[5]['actual']['elbow_flex'] = 2056
    assert evidence.analyze(events)['encoder'] == 'insufficient travel'


@pytest.mark.parametrize('mutation', ['duplicate', 'backward', 'nan', 'read_end_before_start', 'wrong_seq', 'wrong_run'])
def test_invalid_data_cannot_pass(mutation):
    events = dense_run()
    if mutation == 'duplicate': events[5]['read_start'] = events[4]['read_start']
    if mutation == 'backward': events[5]['read_start'] = 1
    if mutation == 'nan': events[5]['actual']['elbow_flex'] = float('nan')
    if mutation == 'read_end_before_start': events[5]['read_end'] = 1
    if mutation == 'wrong_seq': events[5]['seq'] = 99
    if mutation == 'wrong_run': events[5]['run_id'] = 'other'
    result = evidence.analyze(events)
    assert result['encoder'] == 'invalid data'
    assert evidence.exit_code(result) != 0


@pytest.mark.parametrize('mutation', ['few', 'gap', 'late_read', 'no_endpoint', 'no_readback', 'interrupted_hold'])
def test_missing_coverage_cannot_pass(mutation):
    events = dense_run()
    if mutation == 'few': events = events[:4] + events[-2:]
    if mutation == 'gap':
        events[5]['read_start'] = 3.75
        events[5]['read_end'] = 3.76
        events[4]['read_start'] = 3.54
        events[4]['read_end'] = 3.55
    if mutation == 'late_read': events[6]['read_end'] = 4.2
    if mutation == 'no_endpoint': events[1]['goal'] = 2055
    if mutation == 'no_readback': events.pop(2)
    if mutation == 'interrupted_hold': events.insert(4, {'event': 'command', 't': 3.65, 'goal': 2040})
    for i, e in enumerate(events): e.update(version=1, run_id='run', seq=i)
    assert evidence.analyze(events)['encoder'] != 'passed'


def test_missing_final_preserves_undertravel_but_not_success():
    result = evidence.analyze(dense_run(16)[:-1])
    assert result['peak_ticks'] == 16
    assert result['execution'] == 'interrupted/unknown'
    assert result['shutdown'] == 'unverified'
    assert evidence.exit_code(result) == 3


@pytest.mark.parametrize('field,value', [('shutdown', 'unverified'), ('restoration', 'recovery required')])
def test_cleanup_failure_overrides_encoder_pass(field, value):
    events = dense_run()
    events[-1][field] = value
    result = evidence.analyze(events)
    assert evidence.exit_code(result) == 3
    assert result['physical_accepted'] is False


def test_observation_only_counts_after_verified_cleanup():
    events = dense_run()
    events.append(dict(version=1, run_id='run', seq=len(events), event='observation',
                       value='upward/no binding/no oscillation'))
    assert evidence.analyze(events)['physical_accepted'] is True
    events[-2]['shutdown'] = 'unverified'
    assert evidence.analyze(events)['physical_accepted'] is False


@pytest.mark.parametrize('observation', ['no lift', 'downward/binding', 'oscillation'])
def test_negative_observation_exits_incomplete(observation):
    events = dense_run()
    events.append(dict(version=1, run_id='run', seq=len(events), event='observation', value=observation))
    assert evidence.exit_code(evidence.analyze(events)) == 2


def test_malformed_trailing_record_is_not_success(tmp_path):
    path = tmp_path / 'broken.log'
    path.write_text('\n'.join(json.dumps(e) for e in dense_run()) + '\n{"event":')
    result = evidence.analyze_file(path)
    assert result['encoder'] == 'invalid data'
    assert evidence.exit_code(result) != 0


def test_legacy_undertravel_without_invented_shutdown():
    events = [{'baseline': {'elbow_flex': 2543}, 'direction': -1},
              {'t': 0, 'target': {'elbow_flex': 2543}, 'actual': {'elbow_flex': 2543}},
              {'t': 3, 'target': {'elbow_flex': 2487}, 'actual': {'elbow_flex': 2527}}]
    result = evidence.analyze(events)
    assert (result['requested_ticks'], result['peak_ticks']) == (56, 16)
    assert result['encoder'] == 'insufficient travel'
    assert result['shutdown'] == 'unverified'
    assert result['execution'] == 'interrupted/unknown'


def test_import_is_hardware_and_plotting_free():
    code = "import sys; from examples.hiwonder import elbow_evidence; assert not any(x in sys.modules for x in ('serial', 'lerobot', 'matplotlib'))"
    subprocess.run([sys.executable, '-c', code], check=True)


def test_recorder_orders_flushes_and_refuses_nan(tmp_path):
    path = tmp_path / 'run.log'
    with evidence.Recorder(path) as recorder:
        recorder.emit('start', direction=1, baseline={'elbow_flex': 2000})
        assert len(path.read_text().splitlines()) == 1
        with pytest.raises(ValueError): recorder.emit('sample', read_start=float('nan'))
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert records[0]['seq'] == 0
    assert records[0]['version'] == 1
    assert records[0]['run_id']
