import io
import json
from pathlib import Path

import pytest
from PIL import Image

from diagnostics import Recorder
from visual_evidence import (digest, evidence_catalog, resolve_evidence_requests,
                             trusted_preview, trajectory_image)


def jpeg(color='red'):
    buffer = io.BytesIO(); Image.new('RGB', (64, 32), color).save(buffer, format='JPEG')
    return buffer.getvalue()


def publish(runtime, run, sequence=1, stamp=1., data=None, **extra):
    directory = runtime/'inference'; directory.mkdir(exist_ok=True)
    data = data or jpeg()
    metadata = dict(run_id=str(run.resolve()), sequence=sequence, sim_stamp=stamp,
                    planner='pulp_dronet', preview_sha256=digest(data),
                    steering=.35, collision_probability=.81, **extra)
    (directory/'pulp_dronet.jpg').write_bytes(data)
    (directory/'pulp_dronet.json').write_text(json.dumps(metadata))
    return metadata


def scene(stamp):
    return {'sim_time': stamp, 'position': [stamp*.1, 0., 1.2],
            'oracle': {'clearance_m': .4}, 'condition': {'patch_enabled': False}}


def test_preview_rejects_previous_run_and_same_run_jpeg_metadata_race(tmp_path):
    run = tmp_path/'run'; run.mkdir()
    publish(tmp_path, tmp_path/'old')
    path = tmp_path/'inference/pulp_dronet.json'
    assert trusted_preview(path, run) == (None, None, 'different_or_missing_run_id')
    metadata = publish(tmp_path, run)
    path.with_suffix('.jpg').write_bytes(jpeg('blue'))
    actual, data, reason = trusted_preview(path, run)
    assert actual == metadata and data is None and reason == 'preview_hash_mismatch'
    publish(tmp_path, run)
    actual, data, reason = trusted_preview(path, run)
    assert data == jpeg() and reason is None


def test_legacy_preview_without_hash_preserves_telemetry_but_is_not_trusted_image(tmp_path):
    run = tmp_path/'run'; run.mkdir()
    metadata = publish(tmp_path, run)
    del metadata['preview_sha256']
    (tmp_path/'inference/pulp_dronet.json').write_text(json.dumps(metadata))
    recorder = Recorder(run, tmp_path, 'pulp_dronet'); recorder.capture(scene(1.), {})
    summary = recorder.finish(0., 'collision')
    assert summary['snapshots'][0]['model_output']['steering'] == .35
    assert summary['preview_frames'] == []
    assert summary['visual_evidence']['capture_rejections']['missing_preview_hash'] == 1
    assert not (run/'first_inference.jpg').exists()


def test_consumed_and_released_frames_remain_distinct_in_trace_manifest_and_compact_context(tmp_path):
    from agent_context import compact_diagnostics
    run = tmp_path/'run'; run.mkdir()
    consumed = {'schema_version': 1, 'sequence': 1, 'sim_stamp': 1., 'disturbance_revision': 1,
                'sensor_disturbance': {'rgb_noise_stddev': 16., 'fixed_delay_s': .3}, 'rgb_noise_rmse': 12.}
    publish(tmp_path, run, sequence=1, stamp=1., consumed_input=consumed)
    bridge = {'sequence': 2, 'image_stamp': 1.3, 'latest_input_stamp': 1.3,
              'latest_input_disturbance': {'rgb_noise_stddev': 0., 'fixed_delay_s': 0.},
              'latest_input_rgb_noise_rmse': 0.}
    recorder = Recorder(run, tmp_path, 'pulp_dronet')
    recorder.capture(scene(1.5), bridge)
    summary = recorder.finish(0., 'collision')
    trace = json.loads((run/'diagnostic_trace.json').read_text())
    assert trace[0]['worker']['consumed_input'] == consumed
    assert trace[0]['latest_input_disturbance']['rgb_noise_stddev'] == 0.
    assert json.loads((run/'first_inference.json').read_text())['consumed_input'] == consumed
    manifest = json.loads((run/'visual_evidence.json').read_text())
    assert manifest['schema_version'] == 2
    frame = manifest['frames'][0]
    assert frame['model_output']['consumed_input'] == consumed
    assert frame['capture_observation']['bridge_released_input']['sequence'] == 2
    assert frame['capture_observation']['bridge_released_input']['rgb_noise_rmse'] == 0.
    assert summary['snapshots'][0]['model_output']['consumed_input'] == consumed
    compact = compact_diagnostics({'diagnostics': summary}, 'r001_perturbed')
    sample = compact['snapshots'][0]
    assert sample['model_output']['consumed_input']['sequence'] == 1
    assert sample['model_output']['consumed_input']['rgb_noise_rmse'] == 12.
    assert sample['bridge_released_input']['sequence'] == 2
    assert sample['bridge_released_input']['sensor_disturbance']['rgb_noise_stddev'] == 0.
    assert 'effective_input_disturbance' not in sample


def make_run(tmp_path, stamps=(1., 3., 5.)):
    run = tmp_path/'run'; run.mkdir()
    recorder = Recorder(run, tmp_path, 'pulp_dronet')
    for index, stamp in enumerate(stamps):
        publish(tmp_path, run, sequence=index+1, stamp=stamp-.2)
        recorder.capture(scene(stamp), {'last_command': {'speed': .2}, 'image_stamp': stamp-.2})
    summary = recorder.finish(1., 'collision')
    result = {'result_dir': str(run.resolve()), 'outcome': 'collision', 'diagnostics': summary}
    (run/'result.json').write_text(json.dumps(result))
    history = [{'decision': {'round': 1}, 'pairs': [{'perturbed': result}]}]
    return run, summary, history


def test_recorder_bound_retention_and_first_last_telemetry(tmp_path):
    run, summary, history = make_run(tmp_path, range(1, 502))
    manifest = json.loads((run/'visual_evidence.json').read_text())
    assert len(manifest['frames']) <= 31
    assert len(summary['preview_frames']) == 6
    assert manifest['frames'][0]['capture_sim_s'] == 1
    assert manifest['frames'][-1]['capture_sim_s'] == 501
    assert manifest['frames'][-1]['model_output']['collision_probability'] == .81
    assert manifest['frames'][-1]['sensor_sim_s'] == pytest.approx(500.8)
    assert manifest['trajectory']['samples'] == 501
    assert manifest['trajectory']['kind'] == 'derived_gt_trajectory'
    for frame in manifest['frames']:
        assert digest((run/frame['file']).read_bytes()) == frame['sha256']
        assert frame['verified_preview_hash'] is True


def test_evidence_requests_resolve_nearest_real_time_and_output_metadata(tmp_path):
    run, _, history = make_run(tmp_path)
    catalog = evidence_catalog(history, tmp_path)
    assert catalog[0]['evidence_id'] == 'r001_perturbed'
    assert catalog[0]['inference_times_sim_s'] == [1., 3., 5.]
    requests = [{'evidence_id': 'r001_perturbed', 'view': 'inference', 'sim_time_s': 3.1},
                {'evidence_id': 'r001_perturbed', 'view': 'trajectory', 'sim_time_s': None}]
    bundle = resolve_evidence_requests(history, requests, tmp_path, tmp_path/'selection')
    assert len(bundle['image_paths']) == 2
    evidence = bundle['manifest']['requests'][0]
    assert evidence['artifact']['capture_sim_s'] == 3.
    assert evidence['requested_time_delta_s'] == pytest.approx(-.1)
    assert 'no depth or optical-flow' in evidence['description']
    assert (tmp_path/'selection/evidence_manifest.json').is_file()
    assert bundle['manifest']['requests'][1]['artifact']['coordinate_frame'] == 'ws2_world'


def test_catalog_never_uses_results_outside_runtime_or_foreign_manifest(tmp_path):
    run, _, history = make_run(tmp_path)
    assert evidence_catalog(history, tmp_path/'other') == []
    manifest = json.loads((run/'visual_evidence.json').read_text())
    manifest['run_id'] = str(tmp_path/'different_run')
    (run/'visual_evidence.json').write_text(json.dumps(manifest))
    assert evidence_catalog(history, tmp_path)[0]['image_count'] == 0
    bundle = resolve_evidence_requests(history, [{'evidence_id': 'r001_perturbed',
                    'view': 'inference', 'sim_time_s': None}], tmp_path)
    assert bundle['image_paths'] == []
    assert bundle['manifest']['requests'][0]['status'] == 'unavailable'


def test_request_limits_paths_integrity_and_finite_time(tmp_path):
    run, _, history = make_run(tmp_path)
    request = {'evidence_id': 'r001_perturbed', 'view': 'inference', 'sim_time_s': None}
    for invalid in ([request]*4, [{**request, 'path': '/etc/passwd'}],
                    [{**request, 'evidence_id': '../run'}], [{**request, 'sim_time_s': float('nan')}],
                    [{**request, 'sim_time_s': True}]):
        with pytest.raises(ValueError):
            resolve_evidence_requests(history, invalid, tmp_path)
    manifest = json.loads((run/'visual_evidence.json').read_text())
    latest = manifest['frames'][-1]
    (run/latest['file']).write_bytes(jpeg('blue'))
    with pytest.raises(ValueError, match='hash mismatch'):
        resolve_evidence_requests(history, [request], tmp_path)
    latest['file'] = '../../outside.jpg'
    (run/'visual_evidence.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='escapes'):
        resolve_evidence_requests(history, [request], tmp_path)


def test_trajectory_is_absent_without_motion_samples(tmp_path):
    assert trajectory_image([], {}, tmp_path/'map.png') is None
    assert not (tmp_path/'map.png').exists()
