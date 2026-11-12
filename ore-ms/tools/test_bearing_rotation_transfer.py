"""Recording identity, artifact locks, reference direction and transfer gates."""
import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import bearing_rotation_transfer as transfer
import pose_covariance_accuracy


def test_binding_restored_on_failure():
    old = transfer.core.OUT, transfer.core.INPUT, transfer.core.verify, transfer.core.FREEZE
    with pytest.raises(RuntimeError):
        with transfer.binding('day'):
            raise RuntimeError('fixture')
    assert (transfer.core.OUT, transfer.core.INPUT, transfer.core.verify, transfer.core.FREEZE) == old


def setup_case(monkeypatch, tmp_path, sequence=None, stems=None):
    monkeypatch.setattr(transfer, 'ROOT', tmp_path)
    monkeypatch.setattr(transfer, 'BASE', tmp_path/'transfer')
    source = {'sequence': sequence or transfer.CASES['day'][1],
        'records': [{'stem': stem, 'cameras': {'rgb': {'left': {'recorded_time_ns': (i+1)*1000000}}}}
                    for i, stem in enumerate(stems or [f'{i:06}' for i in range(20)])]}
    (tmp_path/'source.json').write_text(json.dumps(source))
    target = transfer.BASE/'inputs/day'; target.mkdir(parents=True)
    metadata = {'condition': 'day', 'sequence': transfer.CASES['day'][1], 'source_manifest': 'source.json',
                'calibration': 'calibration.json', 'reference_rgb_poses': [f'{i}.txt' for i in range(20)]}
    (target/'metadata.json').write_text(json.dumps(metadata))
    return source


def test_wrong_recording_cannot_borrow_reference_cohort(monkeypatch, tmp_path):
    setup_case(monkeypatch, tmp_path, sequence=transfer.CASES['night'][1])
    with pytest.raises(ValueError, match='Recording identity'):
        transfer.read_case('day')


def test_reordered_or_shortened_prefix_rejected(monkeypatch, tmp_path):
    setup_case(monkeypatch, tmp_path, stems=[f'{i:06}' for i in range(1, 20)])
    with pytest.raises(ValueError, match='prefix changed'):
        transfer.read_case('day')


@pytest.mark.parametrize('day,night,passed', [(True, True, True), (False, True, False), (True, False, False)])
def test_no_pooled_rescue_of_failed_recording(day, night, passed):
    recordings = {c: {'primary': {'checks': {'gate': value}}} for c, value in [('day', day), ('night', night)]}
    assert (transfer.transfer_decision(recordings)['decision'] == 'DAY_NIGHT_DEVELOPMENT_GATES_PASS') is passed


def test_reference_pose_direction_and_exact_rgb_times(monkeypatch, tmp_path):
    setup_case(monkeypatch, tmp_path)
    (tmp_path/'calibration.json').write_text('{}')
    monkeypatch.setattr(pose_covariance_accuracy, 'transforms', lambda *args: {'thermal': np.eye(4)})
    root = Rotation.from_rotvec([.2, -.3, .1]).as_matrix()
    step = np.array([.003, -.004, .002])
    for i in range(20):
        pose = np.eye(4); pose[:3, :3] = root@Rotation.from_rotvec(i*step).as_matrix(); pose[:3, 3] = [i, .5*i, 0]
        np.savetxt(tmp_path/f'{i}.txt', pose)
    _, references, clocks, qualification = transfer.reference_data('day')
    for rotation in references.values():
        np.testing.assert_allclose(rotation, Rotation.from_rotvec(step).as_matrix(), atol=1e-12)
    assert clocks[0] == [1000000, 2000000] and len(qualification) == 20


def test_changed_acquired_packet_is_rejected(monkeypatch, tmp_path):
    monkeypatch.setattr(transfer, 'BASE', tmp_path)
    monkeypatch.setattr(transfer, 'verify', lambda: None)
    freeze = tmp_path/'freeze'; freeze.write_text('fixed')
    monkeypatch.setattr(transfer, 'FREEZE', freeze)
    packet = tmp_path/'packet'; packet.write_text('fixed input')
    target = tmp_path/'inputs/day'; target.mkdir(parents=True)
    (target/'input-lock.json').write_text(json.dumps({'execution_freeze_sha256': transfer.core.sha(freeze),
                                                   'pins': {'packet': transfer.core.sha(packet)}}))
    transfer.verify_condition('day')
    packet.write_text('modified input')
    with pytest.raises(ValueError, match='Changed acquired packet'):
        transfer.verify_condition('day')
