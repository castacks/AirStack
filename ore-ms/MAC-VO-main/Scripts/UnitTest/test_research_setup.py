from pathlib import Path
import torch
import pytest

from Module.Network.FlowFormerCov.checkpoint import (
    DELTA_KEY, LEGACY_UNUSED_KEYS, load_cov_checkpoint,
)
from Scripts.Setup.prepare_td3_sample import load_stream, pair_streams
from Scripts.Setup.audit_td3_geometry import pose_matrix, rectification_residual, compare_calibration, static_query
import numpy as np
import yaml


def tiny_model():
    model = torch.nn.Module()
    model.weight = torch.nn.Parameter(torch.ones(2))
    model.memory_decoder = torch.nn.Module()
    model.memory_decoder.register_buffer("delta", torch.zeros(1, 9, 9, 2))
    return model


def legacy_weights():
    return {"weight": torch.tensor([2., 3.]),
            **{key: torch.zeros(1) for key in LEGACY_UNUSED_KEYS}}


def test_audited_legacy_and_native_load():
    model = tiny_model()
    report = load_cov_checkpoint(model, legacy_weights())
    assert report["migration"] == "flowformer-pre-cleanup-rgb"
    assert model.weight.tolist() == [2., 3.]
    assert model.memory_decoder.delta[0, 0, 0].tolist() == [-4., -4.]
    assert model.memory_decoder.delta[0, 8, 8].tolist() == [4., 4.]
    assert load_cov_checkpoint(model, model.state_dict())["migration"] == "none"


@pytest.mark.parametrize("change", ["unknown", "missing_parameter", "partial_legacy", "wrong_shape", "strict_only", "duplicate"])
def test_checkpoint_corruption_rejected(change):
    state = legacy_weights()
    kwargs = {}
    if change == "unknown": state["mystery"] = torch.ones(1)
    if change == "missing_parameter": del state["weight"]
    if change == "partial_legacy": del state[next(iter(LEGACY_UNUSED_KEYS))]
    if change == "wrong_shape": state["weight"] = torch.ones(3)
    if change == "strict_only": kwargs["allow_legacy"] = False
    if change == "duplicate": state["module.weight"] = state["weight"]
    with pytest.raises((RuntimeError, ValueError)):
        load_cov_checkpoint(tiny_model(), state, **kwargs)


def records(stamps):
    return [{"index": i, "time_ns": stamp, "path": str(i)} for i, stamp in enumerate(stamps)]


def test_pairing_drops_gaps_and_never_reuses_acquisitions():
    streams = {"rgb_left": records([0, 10, 20, 30]),
               "rgb_right": records([0, 10, 30]),
               "thermal_left": records([1, 31]),
               "thermal_right": records([2, 32])}
    rows = pair_streams(streams, stereo_ns=1, cross_ns=3)
    assert [r["rgb_left"]["time_ns"] for r in rows] == [0, 30]
    for key in streams:
        times = [r[key]["time_ns"] for r in rows]
        assert times == sorted(set(times))


def test_tie_prefers_earlier_and_every_cross_pair_is_bounded():
    streams = {"rgb_left": records([10]), "rgb_right": records([11]),
               "thermal_left": records([8, 12]), "thermal_right": records([8, 12])}
    assert pair_streams(streams, 1, 3)[0]["thermal_left"]["time_ns"] == 8
    # Left thermal is within anchor bound, but exceeds RGB-right bound.
    streams["thermal_left"] = records([8]); streams["thermal_right"] = records([8])
    assert pair_streams(streams, 1, 2) == []


def test_loading_preserves_sparse_rows_and_deduplicates(tmp_path):
    d = tmp_path / "sensors" / "test"; d.mkdir(parents=True)
    # Out of order, duplicate, NaN, and a missing image; preserve epoch precision.
    (d / "timestamps.txt").write_text("1761590487.200000001\n1761590487.100000001\n1761590487.100000001\nnan\n1761590487.3\n")
    for i in [0, 1, 2]: (d / f"{i:08d}.png").touch()
    rows, stats = load_stream(tmp_path, "test")
    assert [r["index"] for r in rows] == [1, 0]
    assert rows[0]["time_ns"] == 1761590487100000001
    assert stats["duplicates"] == 1 and stats["missing"] == 2
    assert stats["out_of_order_steps"] == 1


def test_rectification_check_detects_wrong_direction_and_axis():
    left = {"p": [[100., 0, 50., 0], [0, 100., 50., 0], [0, 0, 1., 0]], "r": np.eye(3)}
    right = {"p": [[100., 0, 50., -20.], [0, 100., 50., 0], [0, 0, 1., 0]], "r": np.eye(3)}
    correct = pose_matrix([.2, 0, 0, 0, 0, 0, 1])
    assert rectification_residual(correct, left, right) == {
        "horizontal_baseline_error_m": 0., "off_axis_translation_m": 0.,
        "rectified_rotation_error_deg": 0.}
    inverse = rectification_residual(np.linalg.inv(correct), left, right)
    assert inverse["horizontal_baseline_error_m"] == pytest.approx(-.4)
    wrong_axis = pose_matrix([0, .2, 0, 0, 0, 0, 1])
    assert rectification_residual(wrong_axis, left, right)["off_axis_translation_m"] == pytest.approx(.2)
    with pytest.raises(ValueError): pose_matrix([0, 0, 0, 0, 0, 0, 2])


def test_yaml_detects_reversed_labels_without_inverting_values(tmp_path):
    entries = []
    for name, x in [("thermal_left/optical_frame", 0.), ("thermal_right/optical_frame", .2)]:
        entry = {"from_frame": "vehicle", "to_frame": name,
                 "translation": [x, 0., 0.], "quaternion": [0., 0., 0., 1.]}
        entries.append(entry)
        folder = tmp_path / "tf" / (name.replace('/', '-') + '_to_vehicle')
        folder.mkdir(parents=True)
        np.savetxt(folder / 'static_transform.txt', entry['translation'] + entry['quaternion'])
    calib = tmp_path / 'yamaha.yaml'
    calib.write_text(yaml.safe_dump({'transform_params': entries}))
    for name, px in [('thermal_left_processed_camera_info', 0.), ('thermal_right_processed_camera_info', -20.)]:
        folder = tmp_path / 'sensors' / name; folder.mkdir(parents=True)
        (folder / 'data.yaml').write_text(yaml.safe_dump({
            'p': [[100., 0., 50., px], [0., 100., 50., 0.], [0., 0., 1., 0.]],
            'r': np.eye(3).tolist()}))
    result = compare_calibration(tmp_path, calib)
    right = result['transform_comparisons'][1]['archive_candidates'][0]
    assert right['labels_reversed']
    assert right['same_matrix_residual']['translation_m'] == 0
    assert right['inverse_matrix_residual']['translation_m'] == pytest.approx(.4)
    assert result['thermal_projection_residuals_using_yaml']['horizontal_baseline_error_m'] == 0
    edges = [(e['from_frame'], e['to_frame'], pose_matrix(e['translation'] + e['quaternion'])) for e in entries]
    a = static_query(edges, entries[0]['to_frame'], entries[1]['to_frame'])
    b = static_query(edges, entries[1]['to_frame'], entries[0]['to_frame'])
    np.testing.assert_allclose(a @ b, np.eye(4), atol=1e-12)
