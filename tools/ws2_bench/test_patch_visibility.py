"""Synthetic artifacts only; these tests do not certify the live renderer."""
import copy
import ast
import hashlib
import importlib.util
import io
from pathlib import Path

import pytest

from patch_visibility import BINDING, CAPTURE_VERSION, PREPROCESS, TRANSPORT, kim_crop, source_token, verify_capture


def sha(value):
    return hashlib.sha256(value).hexdigest()


@pytest.fixture
def evidence(tmp_path):
    import cv2
    import numpy as np

    def save(name, value):
        if isinstance(value, np.ndarray):
            handle = io.BytesIO(); np.save(handle, value, allow_pickle=False); value = handle.getvalue()
        (tmp_path / name).write_bytes(value)
        return {'path': name, 'sha256': sha(value)}

    image = np.arange(8 * 12 * 3, dtype=np.uint8).reshape(8, 12, 3)
    labels = np.zeros((8, 12), np.uint32); labels[2:5, 4:8] = 7
    texture = save('patch.png', b'synthetic texture fixture, never flight evidence')
    manifest = {'binding': BINDING, 'run_id': 'synthetic_run', 'model': 'kim', 'config_sha256': 'a'*64,
                'camera_prim': '/World/TestCamera', 'camera_frame': 'test_optical',
                'render_product': '/Render/TestProduct', 'patch_prim_path': '/World/PatchSurface',
                'texture': texture, 'material': {'texture_sha256': texture['sha256'],
                    'bound_prim': '/World/PatchSurface', 'opacity': 1., 'scale': [1, 1, 1, 1], 'bias': [0, 0, 0, 0]},
                'preprocess': {'name': PREPROCESS, 'opencv_version': cv2.__version__,
                    'k': [5., 0., 6., 0., 5., 4., 0., 0., 1.],
                    'input_fov_deg': 0., 'preserve_input_aspect': True},
                'transport': {'name': TRANSPORT, 'numpy_version': np.__version__,
                    'opencv_version': cv2.__version__, 'jpeg_quality': 95, 'disturbance_seed': 827}}
    context = {k: manifest[k] for k in ('run_id', 'model', 'config_sha256')}
    renderer = dict(context, rgb=save('rgb.npy', image), instances=save('instances.npy', labels),
                    id_to_prim={'0': None, '7': '/World/PatchSurface'}, patch_enabled=True,
                    reference_time={'numerator': 17, 'denominator': 3}, source_stamp_ns=5666666667)
    token = source_token(manifest, renderer); renderer['source_token'] = token
    ok, jpeg = cv2.imencode('.jpg', image[:, :, ::-1]); assert ok
    bridge = dict(context, source_token=token, source_stamp_ns=5666666667, sequence=4,
                  raw_rgb_sha256=sha(image.tobytes()), jpeg=save('input.jpg', jpeg.tobytes()),
                  post_noise_bgr=save('post_noise.npy', np.ascontiguousarray(image[:, :, ::-1])),
                  effective_rgb_noise_stddev=0.)
    decoded = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
    roi = kim_crop(decoded.shape, manifest['preprocess']['k'], 0, True)
    x, y, w, h = roi
    tensor = cv2.resize(cv2.cvtColor(decoded[y:y+h, x:x+w], cv2.COLOR_BGR2RGB),
                        (304, 228), interpolation=cv2.INTER_CUBIC).astype(np.float32)[None]
    worker = dict(context, source_token=token, source_stamp_ns=5666666667, sequence=4,
                  received_jpeg_sha256=bridge['jpeg']['sha256'], effective_crop_roi=roi,
                  fcrn_input_sha256=sha(tensor.tobytes()))
    capture = {'schema': CAPTURE_VERSION, 'manifest': manifest,
               'frames': [{'renderer': renderer, 'bridge': bridge, 'worker': worker}]}
    kwargs = dict(run_id='synthetic_run', model='kim', config_sha256='a'*64,
                  expected_patch_sha256=texture['sha256'])
    return tmp_path, capture, kwargs, save


def test_full_chain_visible_compatible_with_factor_gate_and_no_causal_claim(evidence):
    from factor_exposure import assess
    root, capture, kwargs, _ = evidence; old = copy.deepcopy(capture)
    result = verify_capture(root, capture, **kwargs)
    assert result['status'] == 'visible' and result['visible_consumed_frames'] == 1
    assert result['frames'][0]['visible_patch_pixels'] == 12
    assert result['frames'][0]['effective_crop_roi'] == [0, 0, 11, 8]
    assert result['causal_validation'] is False and capture == old
    pair = {'clean': {'outcome': 'route_completed'}, 'perturbed': {'outcome': 'collision', 'patch_visibility': result}}
    assert assess({'patch_enabled': True}, pair)['eligible'] is True


def replace_labels(evidence, labels, enabled=True):
    root, capture, kwargs, save = evidence
    frame = capture['frames'][0]; frame['renderer']['instances'] = save('instances.npy', labels)
    frame['renderer']['patch_enabled'] = enabled
    token = source_token(capture['manifest'], frame['renderer'])
    for key in ('renderer', 'bridge', 'worker'): frame[key]['source_token'] = token
    return verify_capture(root, capture, **kwargs)


@pytest.mark.parametrize('mode', ['occluded', 'off', 'cropped_out'])
def test_verified_zero_visibility_is_distinct_from_missing_evidence(evidence, mode):
    import numpy as np
    labels = np.zeros((8, 12), np.uint32)
    if mode == 'cropped_out': labels[:, 11] = 7
    result = replace_labels(evidence, labels, mode != 'off')
    assert result['status'] == 'not_visible' and result['visibility_verified'] is False
    assert result['zero_visible_consumed_frames'] == 1
    assert result['verified_frame_records'][0]['camera_visible_patch_pixels'] == (8 if mode == 'cropped_out' else 0)


@pytest.mark.parametrize('part,field,value,reason', [
    ('renderer', 'source_stamp_ns', 5666666668, 'source_timestamp_mismatch'),
    ('bridge', 'raw_rgb_sha256', '0'*64, 'raw_source_pixels_mismatch'),
    ('bridge', 'source_token', 'wrong-render', 'source_token_mismatch'),
    ('worker', 'run_id', 'another_run', 'frame_context_mismatch'),
    ('worker', 'model', 'pulp_dronet', 'frame_context_mismatch'),
    ('worker', 'config_sha256', 'b'*64, 'frame_context_mismatch'),
    ('worker', 'sequence', 5, 'consumed_sequence_mismatch'),
    ('worker', 'received_jpeg_sha256', '0'*64, 'received_payload_mismatch'),
    ('worker', 'effective_crop_roi', [0, 0, 12, 8], 'actual_crop_mismatch'),
    ('worker', 'fcrn_input_sha256', '0'*64, 'actual_fcrn_input_mismatch'),
])
def test_rejects_broken_actual_input_chain_even_if_caller_sets_verified(evidence, part, field, value, reason):
    root, capture, kwargs, _ = evidence
    capture['frames'][0][part][field] = value
    capture['visibility_verified'] = True
    capture['frames'][0]['worker_consumption_verified'] = True
    result = verify_capture(root, capture, **kwargs)
    assert result['status'] == 'unknown' and reason in result['issues']


@pytest.mark.parametrize('change,reason', [
    ({'binding': 'nearest_timestamp'}, 'unverified_render_binding'),
    ({'patch_prim_path': '/World/Obstacle'}, 'wrong_patch_prim'),
    ({'config_sha256': 'b'*64}, 'episode_context_mismatch'),
])
def test_requires_producer_and_episode_contract(evidence, change, reason):
    root, capture, kwargs, _ = evidence; capture['manifest'].update(change)
    assert reason in verify_capture(root, capture, **kwargs)['issues']


def test_texture_material_and_artifacts_must_match(evidence):
    root, capture, kwargs, _ = evidence
    assert 'original_texture_mismatch' in verify_capture(root, capture, **dict(kwargs, expected_patch_sha256='0'*64))['issues']
    capture['manifest']['material']['opacity'] = .5
    assert 'material_not_original_opaque_patch' in verify_capture(root, capture, **kwargs)['issues']
    capture['manifest']['material']['opacity'] = 1
    (root / 'instances.npy').write_bytes(b'changed')
    assert 'artifact_hash_mismatch' in verify_capture(root, capture, **kwargs)['issues']


def test_unrelated_jpeg_with_matching_worker_digest_and_tensor_is_rejected(evidence):
    import cv2
    import numpy as np
    root, capture, kwargs, save = evidence; frame = capture['frames'][0]
    _, jpeg = cv2.imencode('.jpg', np.zeros((8, 12, 3), np.uint8))
    frame['bridge']['jpeg'] = save('unrelated.jpg', jpeg.tobytes())
    frame['worker']['received_jpeg_sha256'] = sha(jpeg.tobytes())
    frame['worker']['fcrn_input_sha256'] = sha(np.zeros((1, 228, 304, 3), np.float32).tobytes())
    assert 'source_to_jpeg_mismatch' in verify_capture(root, capture, **kwargs)['issues']
    frame['bridge']['post_noise_bgr'] = save('unrelated.npy', np.zeros((8, 12, 3), np.uint8))
    assert 'post_noise_source_pixels_mismatch' in verify_capture(root, capture, **kwargs)['issues']


@pytest.mark.parametrize('sigma', [0., .1, 12., 64.])
def test_noise_and_jpeg_replay_matches_actual_bridge_function(evidence, sigma):
    import cv2
    import numpy as np
    root, capture, kwargs, save = evidence; frame = capture['frames'][0]
    bridge_source = Path(__file__).resolve().parents[2] / 'robot/ros_ws/src/local/planners/mononav_bridge/mononav_bridge/disturbances.py'
    spec = importlib.util.spec_from_file_location('patch_noise_bridge_reference', bridge_source)
    disturbances = importlib.util.module_from_spec(spec); spec.loader.exec_module(disturbances)
    rgb = np.load(root / 'rgb.npy', allow_pickle=False)
    transport = capture['manifest']['transport']
    actual_pixels = disturbances.add_rgb_noise(np.ascontiguousarray(rgb[:, :, ::-1]), sigma,
                                              transport['disturbance_seed'], frame['bridge']['sequence'])
    frame['bridge']['post_noise_bgr'] = save('post_noise.npy', actual_pixels)
    frame['bridge']['effective_rgb_noise_stddev'] = sigma
    _, jpeg = cv2.imencode('.jpg', actual_pixels, [cv2.IMWRITE_JPEG_QUALITY, transport['jpeg_quality']])
    frame['bridge']['jpeg'] = save('input.jpg', jpeg.tobytes())
    frame['worker']['received_jpeg_sha256'] = sha(jpeg.tobytes())
    decoded = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
    x, y, w, h = frame['worker']['effective_crop_roi']
    tensor = cv2.resize(cv2.cvtColor(decoded[y:y+h, x:x+w], cv2.COLOR_BGR2RGB),
                        (304, 228), interpolation=cv2.INTER_CUBIC).astype(np.float32)[None]
    frame['worker']['fcrn_input_sha256'] = sha(tensor.tobytes())
    assert verify_capture(root, capture, **kwargs)['status'] == 'visible'
    if sigma:
        frame['bridge']['effective_rgb_noise_stddev'] += 2
        assert 'post_noise_source_pixels_mismatch' in verify_capture(root, capture, **kwargs)['issues']


def test_transport_parameters_are_required_and_versioned(evidence):
    root, capture, kwargs, _ = evidence
    capture['manifest']['transport']['numpy_version'] = 'unknown'
    assert 'transport_version_mismatch' in verify_capture(root, capture, **kwargs)['issues']


def test_mask_provenance_lookup_cannot_be_omitted(evidence):
    root, capture, kwargs, _ = evidence; frame = capture['frames'][0]
    frame['renderer']['id_to_prim'].pop('7')
    token = source_token(capture['manifest'], frame['renderer'])
    for key in ('renderer', 'bridge', 'worker'): frame[key]['source_token'] = token
    assert 'incomplete_instance_lookup' in verify_capture(root, capture, **kwargs)['issues']


def test_duplicate_consumption_not_inflated(evidence):
    root, capture, kwargs, _ = evidence
    capture['frames'].append(copy.deepcopy(capture['frames'][0]))
    assert verify_capture(root, capture, **kwargs)['visible_consumed_frames'] == 1
    capture['frames'][1]['worker']['sequence'] += 1
    capture['frames'][1]['bridge']['sequence'] += 1
    assert 'conflicting_duplicate_source' in verify_capture(root, capture, **kwargs)['issues']


def test_legacy_or_non_kim_inputs_stay_unknown(tmp_path):
    kwargs = dict(run_id='x', model='kim', config_sha256='a'*64, expected_patch_sha256='b'*64)
    assert verify_capture(tmp_path, {'visibility_verified': True}, **kwargs)['status'] == 'unknown'
    assert 'unsupported_model' in verify_capture(tmp_path, {}, **dict(kwargs, model='mononav'))['issues']


@pytest.mark.parametrize('shape,fov,preserve', [((300, 480, 3), 0, True), ((480, 300, 3), 0, True),
                                                ((300, 480, 3), 0, False), ((300, 480, 3), 60, True)])
def test_crop_matches_existing_kim_transport_and_runtime_math(shape, fov, preserve):
    import numpy as np
    transport_path = Path(__file__).resolve().parents[3] / 'Collision-avoidance/airstack_integration/transport.py'
    spec = importlib.util.spec_from_file_location('kim_transport_crop_reference', transport_path)
    transport = importlib.util.module_from_spec(spec); spec.loader.exec_module(transport)
    # Unique pixel values make exact ROI comparison independent of the implementation's reported ROI.
    image = np.arange(np.prod(shape), dtype=np.int64).reshape(shape)
    k = [200., 0., shape[1]/2, 0., 200., shape[0]/2, 0., 0., 1.]
    cropped, _, _ = transport.crop_camera_fov(image, None, k, fov)
    if preserve:
        # Execute only the actual repository's pure crop function, avoiding TensorFlow/model imports.
        runtime_path = transport_path.with_name('runtime.py')
        function = next(node for node in ast.parse(runtime_path.read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == 'center_crop_to_aspect')
        namespace = {'np': np}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(runtime_path), 'exec'), namespace)
        cropped = namespace['center_crop_to_aspect'](cropped)
    x, y, w, h = kim_crop(shape, k, fov, preserve)
    assert np.array_equal(cropped, image[y:y+h, x:x+w])
