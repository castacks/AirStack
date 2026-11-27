"""Tests for the commander <-> goal-retargeting learner file protocol (no ROS needed)."""

from __future__ import annotations

import json
import os
import stat

import numpy as np
import pytest

from svg_ground_control.goal_retarget_bridge import (
    FORMAT_VERSION,
    MANIFEST_FILE,
    DetailLog,
    SESSION_FILE,
    TICK_VECTOR_FIELDS,
    TICKS_FILE,
    PolicyManifestWatcher,
    TickLogReader,
    TickRecorder,
    format_update_summary,
    rows_to_arrays,
    write_json_atomic,
)


def _vectors(value: float) -> dict[str, np.ndarray]:
    return {name: np.full(size, value) for name, size in TICK_VECTOR_FIELDS.items()}


def test_run_directory_is_shared_and_has_session(tmp_path) -> None:
    recorder = TickRecorder(tmp_path / 'root', 'run', {'policy_dt': 0.1})
    assert stat.S_IMODE(os.stat(recorder.run_dir).st_mode) == 0o777
    session = json.loads((recorder.run_dir / SESSION_FILE).read_text())
    assert session['policy_dt'] == 0.1 and session['format_version'] == FORMAT_VERSION
    with pytest.raises(FileExistsError):
        TickRecorder(tmp_path / 'root', 'run', {})


def test_segments_restart_tick_numbering(tmp_path) -> None:
    recorder = TickRecorder(tmp_path, 'run', {})
    assert recorder.next_position == (0, 0)
    assert recorder.record(0.0, 0, **_vectors(1.0)) == (0, 0)
    assert recorder.record(0.1, 0, **_vectors(2.0)) == (0, 1)
    assert recorder.next_position == (0, 2)
    assert recorder.end_segment() and not recorder.end_segment()
    assert recorder.next_position == (1, 0)
    assert recorder.record(5.0, 3, **_vectors(3.0)) == (1, 0)
    recorder.close()

    rows = TickLogReader(recorder.run_dir / TICKS_FILE).poll()
    assert [(row['segment'], row['tick'], row['policy']) for row in rows] == [(0, 0, 0), (0, 1, 0), (1, 0, 3)]
    arrays = rows_to_arrays(rows)
    assert arrays['drone_quat_wxyz'].shape == (3, 4)
    np.testing.assert_allclose(arrays['goal_xy'][:, 0], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(arrays['t'], [0.0, 0.1, 5.0])


def test_missing_or_misshapen_fields_are_rejected(tmp_path) -> None:
    recorder = TickRecorder(tmp_path, 'run', {})
    vectors = _vectors(0.0)
    del vectors['waypoint']
    with pytest.raises(ValueError, match='waypoint'):
        recorder.record(0.0, 0, **vectors)
    vectors = _vectors(0.0)
    vectors['goal_xy'] = np.zeros(3)
    with pytest.raises(ValueError, match='goal_xy'):
        recorder.record(0.0, 0, **vectors)


def test_reader_waits_for_a_half_written_line(tmp_path) -> None:
    path = tmp_path / TICKS_FILE
    reader = TickLogReader(path)
    assert reader.poll() == []
    with open(path, 'w') as file:
        file.write('{"a": 1}\n{"a"')
    assert reader.poll() == [{'a': 1}]
    with open(path, 'a') as file:
        file.write(': 2}\n')
    assert reader.poll() == [{'a': 2}]
    assert reader.poll() == []


def test_manifest_watcher_reports_each_newer_iterate_once(tmp_path) -> None:
    watcher = PolicyManifestWatcher(tmp_path)
    assert watcher.poll() is None

    write_json_atomic(tmp_path / MANIFEST_FILE, {'iterate': 1, 'policy': 'policies/policy_001.zip'})
    manifest = watcher.poll()
    assert manifest['iterate'] == 1
    assert manifest['policy_path'] == str(tmp_path / 'policies' / 'policy_001.zip')
    assert watcher.poll() is None

    # An older iterate (e.g. a stale rewrite) is ignored; a newer one is reported.
    write_json_atomic(tmp_path / MANIFEST_FILE, {'iterate': 1, 'policy': 'x.zip', 'note': 'rewritten'})
    assert watcher.poll() is None
    write_json_atomic(tmp_path / MANIFEST_FILE, {'iterate': 2, 'policy': 'policies/policy_002.zip'})
    assert watcher.poll()['iterate'] == 2


def test_unparsable_manifest_is_retried(tmp_path) -> None:
    watcher = PolicyManifestWatcher(tmp_path)
    (tmp_path / MANIFEST_FILE).write_text('{"iterate": ')
    assert watcher.poll() is None
    write_json_atomic(tmp_path / MANIFEST_FILE, {'iterate': 4, 'policy': 'p.zip'})
    assert watcher.poll()['iterate'] == 4


def test_update_summary_names_the_health_signals() -> None:
    summary = format_update_summary({
        'samples': 180, 'rows_fit': 172, 'loss_before': 0.5, 'loss_after': 0.25,
        'rms_action_change_m': 0.31, 'median_current_offset_m': 0.12,
        'median_accumulated_offset_m': 0.05, 'median_applied_offset_m': 0.16,
        'flight_time_s': 123.4,
    })
    assert 'fit 172 of 180 accepted ticks' in summary
    assert 'loss 0.5000 -> 0.2500' in summary
    assert 'batch action change 0.310 m RMS' in summary
    assert 'goal shift: step 0.12 m, before 0.05 m, labels 0.16 m (medians)' in summary
    assert format_update_summary({}) == ''


def test_detail_log_timestamps_every_line(tmp_path) -> None:
    detail = DetailLog(tmp_path / 'commander.log')
    detail.write('POLICY UPDATE: first\n  second')
    detail.write('third')
    lines = (tmp_path / 'commander.log').read_text().splitlines()
    detail.close()
    assert [line.split(' ', 1)[1] for line in lines] == ['POLICY UPDATE: first', '  second', 'third']
    assert all(line[:4].isdigit() and 'T' in line.split(' ', 1)[0] for line in lines)
