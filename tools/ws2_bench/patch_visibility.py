"""Offline verifier for a future synchronized rendered-patch evidence producer.

NOT wired into the simulator. Existing runs lack this evidence and stay unknown.
The accepted producer contract publishes RGB and its source token from the same
render payload as instance segmentation and reference time. Independent latest
images, approximate timestamp joins and caller-supplied verification flags are
not accepted. This verifies artifacts from trusted bench instrumentation; it is
not authentication against a malicious evidence producer.

Input: an episode-local patch_visibility_capture.json containing ``manifest``
and ``frames``. Each frame has renderer, bridge and worker records. Artifact
references are {path: relative-to-episode, sha256: file digest}; RGB is uint8 HWC
RGB in .npy, instances are uint32 HW in .npy, JPEG is the actual HTTP payload.
The renderer stores id_to_prim, reference_time and patch_enabled; bridge records
source RGB digest and sequence; worker records actual received JPEG digest,
source identity and the exact effective crop/FCRN tensor digest.
The transport manifest freezes JPEG quality, RNG seed and library versions;
each bridge record also retains post-noise BGR pixels and effective sigma. The
verifier replays the actual Gaussian transform and JPEG encoding from renderer
pixels, so a correctly copied token cannot bind an unrelated payload.

No version currently emitted by the live runtime satisfies this contract. The
future producer requires live synchronization/occlusion tests before wiring.
"""
import hashlib
import io
import json
import math
from fractions import Fraction
from pathlib import Path

VERSION = 'rendered_patch_visibility_audit_v1'
CAPTURE_VERSION = 'patch_visibility_capture_v1'
BINDING = 'same_render_payload_v1'
PREPROCESS = 'kim_fcrn_opencv_v1'
TRANSPORT = 'bridge_rgb_gaussian_jpeg_v1'
MAX_BYTES = 32 * 1024 * 1024


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _artifact(root, ref):
    _require(isinstance(ref, dict) and isinstance(ref.get('path'), str), 'missing_artifact')
    path = root / ref['path']
    _require(not Path(ref['path']).is_absolute() and path.resolve().is_relative_to(root),
             'artifact_outside_episode')
    _require(path.is_file() and path.stat().st_size <= MAX_BYTES, 'missing_or_oversize_artifact')
    raw = path.read_bytes()
    _require(_digest(raw) == ref.get('sha256'), 'artifact_hash_mismatch')
    return raw


def _array(raw, np, dtype, dimensions):
    value = np.load(io.BytesIO(raw), allow_pickle=False)
    _require(value.dtype == np.dtype(dtype) and value.ndim == dimensions
             and all(0 < n <= 8192 for n in value.shape), 'invalid_render_array')
    return np.ascontiguousarray(value)


def source_token(manifest, renderer):
    """Content-bound render identity, independent of file names or caller flags."""
    record = {key: manifest[key] for key in
              ('run_id', 'model', 'config_sha256', 'camera_prim', 'camera_frame', 'render_product')}
    record.update(reference_time=renderer['reference_time'],
                  rgb_sha256=renderer['rgb']['sha256'],
                  instances_sha256=renderer['instances']['sha256'],
                  id_to_prim=renderer['id_to_prim'], patch_enabled=renderer['patch_enabled'])
    return _digest(json.dumps(record, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def kim_crop(shape, intrinsics, input_fov, preserve_aspect):
    """Exact ROI arithmetic from Kim transport.py and runtime.py (no TF import)."""
    import numpy as np
    height, width = shape[:2]
    _require(_finite(input_fov) and 0 <= input_fov < 180 and type(preserve_aspect) is bool,
             'invalid_kim_preprocess')
    _require(isinstance(intrinsics, list) and len(intrinsics) == 9
             and all(_finite(v) for v in intrinsics), 'invalid_camera_calibration')
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    _require(min(fx, fy) > 0, 'invalid_camera_calibration')
    x = y = 0
    w, h = width, height
    if input_fov:
        w = 4 * round(2 * fx * np.tan(np.deg2rad(input_fov) / 2) / 4)
        h = w * 3 // 4
        x, y = round(cx - w / 2), round(cy - h / 2)
        _require(w >= 4 and x >= 0 and y >= 0 and x + w <= width and y + h <= height,
                 'crop_outside_image')
    if preserve_aspect:
        if w / h > 4 / 3:
            new_width = round(h * 4 / 3)
            x += (w - new_width) // 2
            w = new_width
        elif w / h < 4 / 3:
            new_height = round(w / (4 / 3))
            y += (h - new_height) // 2
            h = new_height
    return [x, y, w, h]


def _transport_pixels(rgb, standard_deviation, seed, sequence, np):
    """Replay disturbances.py:add_rgb_noise on the actual BGR byte order."""
    image = np.ascontiguousarray(rgb[:, :, ::-1])
    if standard_deviation <= 0:
        return image
    words = [seed & 0xFFFFFFFF, (seed >> 32) & 0xFFFFFFFF, sequence & 0xFFFFFFFF, 0]
    rng = np.random.default_rng(np.random.SeedSequence(words))
    noisy = image.astype(np.float32) + rng.normal(0., standard_deviation, image.shape).astype(np.float32)
    return np.clip(np.rint(noisy), 0., 255.).astype(np.uint8)


def _frame(root, manifest, frame, cv2, np):
    render, bridge, worker = (frame[key] for key in ('renderer', 'bridge', 'worker'))
    token = source_token(manifest, render)
    for record in (render, bridge, worker):
        _require(record.get('source_token') == token, 'source_token_mismatch')
        _require(all(record.get(key) == manifest[key] for key in ('run_id', 'model', 'config_sha256')),
                 'frame_context_mismatch')
    reference = render['reference_time']
    numerator, denominator = reference['numerator'], reference['denominator']
    _require(type(numerator) is int and numerator >= 0 and type(denominator) is int
             and denominator > 0, 'invalid_render_reference_time')
    stamp = round(Fraction(numerator * 1_000_000_000, denominator))
    _require(all(type(record.get('source_stamp_ns')) is int and record['source_stamp_ns'] == stamp
                 for record in (render, bridge, worker)), 'source_timestamp_mismatch')
    _require(type(bridge.get('sequence')) is int and bridge['sequence'] >= 0
             and type(worker.get('sequence')) is int and worker['sequence'] == bridge['sequence'],
             'consumed_sequence_mismatch')
    rgb = _array(_artifact(root, render['rgb']), np, 'uint8', 3)
    labels = _array(_artifact(root, render['instances']), np, 'uint32', 2)
    _require(rgb.shape[2] == 3 and labels.shape == rgb.shape[:2], 'render_shape_mismatch')
    _require(bridge.get('raw_rgb_sha256') == _digest(rgb.tobytes()), 'raw_source_pixels_mismatch')
    mapping = render['id_to_prim']
    _require(isinstance(mapping, dict) and all(str(v) in mapping for v in np.unique(labels)),
             'incomplete_instance_lookup')
    patch_ids = [int(key) for key, value in mapping.items() if value == manifest['patch_prim_path']]
    _require(all(isinstance(value, str) or value is None for value in mapping.values()),
             'invalid_instance_lookup')
    mask = np.isin(labels, patch_ids)
    _require(type(render.get('patch_enabled')) is bool, 'missing_render_patch_state')
    _require(render['patch_enabled'] or not mask.any(), 'disabled_patch_has_visible_pixels')
    transport = manifest['transport']
    sigma = bridge.get('effective_rgb_noise_stddev')
    _require(_finite(sigma) and sigma >= 0, 'invalid_effective_rgb_noise')
    expected_pixels = _transport_pixels(rgb, sigma, transport['disturbance_seed'], bridge['sequence'], np)
    post_noise = _array(_artifact(root, bridge['post_noise_bgr']), np, 'uint8', 3)
    _require(np.array_equal(post_noise, expected_pixels), 'post_noise_source_pixels_mismatch')
    ok, expected_jpeg = cv2.imencode('.jpg', expected_pixels,
                                    [cv2.IMWRITE_JPEG_QUALITY, transport['jpeg_quality']])
    _require(ok, 'jpeg_replay_failed')
    jpeg = _artifact(root, bridge['jpeg'])
    _require(jpeg == expected_jpeg.tobytes(), 'source_to_jpeg_mismatch')
    _require(worker.get('received_jpeg_sha256') == _digest(jpeg), 'received_payload_mismatch')
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    _require(image is not None and image.shape == rgb.shape, 'invalid_worker_jpeg')
    preprocess = manifest['preprocess']
    roi = kim_crop(image.shape, preprocess['k'], preprocess['input_fov_deg'],
                   preprocess['preserve_input_aspect'])
    _require(worker.get('effective_crop_roi') == roi, 'actual_crop_mismatch')
    x, y, width, height = roi
    resized = cv2.resize(cv2.cvtColor(image[y:y+height, x:x+width], cv2.COLOR_BGR2RGB),
                         (304, 228), interpolation=cv2.INTER_CUBIC).astype(np.float32)
    _require(worker.get('fcrn_input_sha256') == _digest(resized[None].tobytes()),
             'actual_fcrn_input_mismatch')
    visible = int(mask[y:y+height, x:x+width].sum())
    return {'source_token': token, 'sequence': bridge['sequence'], 'input_stamp_sim_s': stamp / 1e9,
            'rgb_sha256': render['rgb']['sha256'], 'segmentation_sha256': render['instances']['sha256'],
            'visible_patch_pixels': visible, 'camera_visible_patch_pixels': int(mask.sum()),
            'crop_pixel_count': width * height, 'effective_crop_roi': roi,
            'patch_enabled': render['patch_enabled'], 'frame_binding_verified': True,
            'worker_consumption_verified': True}


def verify_capture(episode_root, capture, *, run_id, model, config_sha256, expected_patch_sha256):
    """Verify a future bench-produced capture against independently known identity.

    ``capture`` is parsed local instrumentation output, never LLM output. The
    caller supplies expected episode/config/texture identity from saved bench
    provenance. No raw artifacts or passed objects are modified. Unknown remains
    unknown if OpenCV is unavailable or preprocessing versions differ.
    """
    result = {'schema': VERSION, 'status': 'unknown', 'visibility_verified': False,
              'method': 'rendered_instance_segmentation', 'source_validation': 'unverified',
              'frames': [], 'issues': [], 'causal_validation': False,
              'scope': 'Observed consumed source frames after the actual Kim RGB crop; not full D3QN history.',
              'limitation': 'Patch visibility does not prove a depth change, attack potency, or causal flight effect.'}
    try:
        _require(model == 'kim', 'unsupported_model')
        _require(isinstance(capture, dict) and capture.get('schema') == CAPTURE_VERSION,
                 'missing_capture_contract')
        manifest = capture['manifest']
        _require(manifest.get('binding') == BINDING, 'unverified_render_binding')
        _require(all(manifest.get(key) == value for key, value in
                     (('run_id', run_id), ('model', model), ('config_sha256', config_sha256))),
                 'episode_context_mismatch')
        _require(manifest.get('patch_prim_path') == '/World/PatchSurface', 'wrong_patch_prim')
        _require(all(isinstance(manifest.get(k), str) and manifest[k]
                     for k in ('camera_prim', 'camera_frame', 'render_product')), 'missing_camera_identity')
        root = Path(episode_root).resolve()
        texture = _artifact(root, manifest['texture'])
        _require(_digest(texture) == expected_patch_sha256, 'original_texture_mismatch')
        material = manifest['material']
        _require(material.get('texture_sha256') == expected_patch_sha256
                 and material.get('bound_prim') == manifest['patch_prim_path']
                 and type(material.get('opacity')) in (int, float) and material['opacity'] == 1
                 and material.get('scale') == [1, 1, 1, 1] and material.get('bias') == [0, 0, 0, 0],
                 'material_not_original_opaque_patch')
        preprocess = manifest['preprocess']
        _require(preprocess.get('name') == PREPROCESS, 'unsupported_preprocess')
        import cv2
        import numpy as np
        _require(preprocess.get('opencv_version') == cv2.__version__, 'preprocess_version_mismatch')
        transport = manifest['transport']
        _require(transport.get('name') == TRANSPORT
                 and transport.get('numpy_version') == np.__version__
                 and transport.get('opencv_version') == cv2.__version__, 'transport_version_mismatch')
        _require(type(transport.get('jpeg_quality')) is int and 0 <= transport['jpeg_quality'] <= 100
                 and type(transport.get('disturbance_seed')) is int, 'invalid_transport_parameters')
        frames = capture['frames']
        _require(isinstance(frames, list) and 0 < len(frames) <= 100000, 'missing_consumed_frames')
        verified = {}
        for frame in frames:
            item = _frame(root, manifest, frame, cv2, np)
            key = item['source_token']
            _require(key not in verified or verified[key] == item, 'conflicting_duplicate_source')
            verified[key] = item
        visible = [item for item in verified.values() if item['visible_patch_pixels'] > 0]
        result.update(status='visible' if visible else 'not_visible', visibility_verified=bool(visible),
                      source_validation='bench_verified', patch_prim_path=manifest['patch_prim_path'],
                      patch_texture_sha256=expected_patch_sha256, frames=visible,
                      observed_consumed_frames=len(verified), visible_consumed_frames=len(visible),
                      zero_visible_consumed_frames=len(verified)-len(visible),
                      verified_frame_records=list(verified.values()))
    except (ValueError, TypeError, KeyError, OSError, ImportError, OverflowError, AttributeError) as exc:
        result['issues'].append(str(exc) if isinstance(exc, ValueError) else type(exc).__name__)
    return result
