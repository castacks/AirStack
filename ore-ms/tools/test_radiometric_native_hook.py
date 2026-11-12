import ast
import hashlib
import numpy as np
import pytest
import radiometric_native_hook as hook
from test_radiometric_tracking import image


def test_insertion_is_only_one_statement_and_precedes_target_sampling():
    original = ast.parse(hook.SOURCE.read_bytes())
    changed = hook.instrument(hook.SOURCE.read_bytes())
    pairs = [n for n in ast.walk(changed) if isinstance(n, ast.FunctionDef) and n.name == 'run_pair']
    body = pairs[0].body
    inserted = [i for i, n in enumerate(body) if ast.unparse(n).startswith('kp1_uv = _radiometric_hook(')]
    assert len(inserted) == 1
    i = inserted[0]
    assert ast.unparse(body[i-1]) == hook.ANCHOR
    assert ast.unparse(body[i+1]).startswith('inbound_mask =')
    body.pop(i)
    assert ast.dump(original, include_attributes=False) == ast.dump(changed, include_attributes=False)


def test_changed_native_source_refused():
    with pytest.raises(ValueError, match='pinned'):
        hook.instrument(hook.SOURCE.read_bytes()+b'\n')


def test_passthrough_preserves_object_and_dtype():
    endpoints = np.array([[1., 2.]], dtype=np.float32)
    assert hook.passthrough(None, None, None, endpoints) is endpoints


def test_refined_and_rejected_points_keep_order_and_inputs():
    points = np.array([[45., 40.], [1., 1.]])
    initial = np.array([[46.3, 39.7], [2., 1.]], dtype=np.float32)
    old = initial.copy()
    result, rows = hook.refine_endpoints(image(), image(1, 0), points, initial)
    assert result.dtype == initial.dtype and np.array_equal(initial, old)
    assert rows[0]['status'] == 'OK' and rows[1]['status'] == 'BORDER'
    assert np.linalg.norm(result[0]-[46, 40]) < .05
    assert np.array_equal(result[1], initial[1])


def test_no_point_dropped_when_all_refinements_reject():
    flat = np.ones((80, 96), np.uint16)*15000
    points = np.array([[45., 40.], [30., 30.]])
    result, rows = hook.refine_endpoints(flat, flat, points, points)
    assert np.array_equal(result, points) and len(rows) == 2
    assert all(r['status'] == 'LOW_TEXTURE' for r in rows)


def test_unequal_point_arrays_rejected():
    with pytest.raises(ValueError, match='Nx2'):
        hook.refine_endpoints(image(), image(), np.zeros((3, 2)), np.zeros((2, 2)))


def test_integer_endpoint_truncation_refused():
    with pytest.raises(ValueError, match='floating'):
        hook.refine_endpoints(image(), image(), np.array([[45, 40]]), np.array([[45, 40]]))


def test_runner_binding_cleanup_without_neural_runtime(monkeypatch):
    from types import SimpleNamespace
    original = object()
    runner = SimpleNamespace(MACVO=original)
    # This checks binding lifetime only; the real source transformation is
    # independently checked above and native class loading remains separate.
    monkeypatch.setattr(hook, 'instrument', lambda _: ast.parse('class MACVO: pass'))
    with pytest.raises(RuntimeError, match='abort'):
        with hook.install(runner):
            assert runner.MACVO is not original
            raise RuntimeError('abort')
    assert runner.MACVO is original


def test_native_tensor_install_cleanup():
    pytest.importorskip('torch')
    # Loading the real class also requires the original pypose/Module runtime.
    pytest.importorskip('pypose')
    import sys
    sys.path.insert(0, str(hook.SOURCE.parents[1]))
    from types import SimpleNamespace
    from Odometry.MACVO import MACVO
    runner = SimpleNamespace(MACVO=MACVO)
    with pytest.raises(RuntimeError, match='abort'):
        with hook.install(runner):
            assert runner.MACVO is not MACVO
            raise RuntimeError('abort')
    assert runner.MACVO is MACVO


def test_raw_lookup_requires_exact_time_and_hash(tmp_path):
    import cv2
    import json
    from types import SimpleNamespace
    pixels = image()
    path = tmp_path/'raw.png'
    assert cv2.imwrite(str(path), pixels)
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps({'records': [{'cameras': {'thr': {'left': {
        'recorded_time_ns': 123, 'path': 'raw.png', 'shape': list(pixels.shape),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}}}}]}))
    provider = hook.raw_provider(tmp_path, manifest)
    frame = SimpleNamespace(stereo=SimpleNamespace(frame_ns=123))
    assert np.array_equal(provider(frame, frame)[0], pixels)
    with pytest.raises(KeyError):
        provider(SimpleNamespace(stereo=SimpleNamespace(frame_ns=124)), frame)
    path.write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash'):
        provider(frame, frame)
