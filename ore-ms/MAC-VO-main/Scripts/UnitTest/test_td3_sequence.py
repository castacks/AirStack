import json
import cv2
import numpy as np
import pytest
import torch
import yaml

from DataLoader.Dataset.TD3 import TD3ManifestSequence, read_uint8_rgb


@pytest.fixture
def sample(tmp_path):
    sensor = tmp_path / 'sensors'; sensor.mkdir()
    for name, tx in [('thermal_left_processed_camera_info', 0.), ('thermal_right_processed_camera_info', 20.)]:
        directory = sensor / name; directory.mkdir()
        (directory / 'data.yaml').write_text(yaml.safe_dump({
            'k': [[90., 0., 3.], [0., 90., 2.], [0., 0., 1.]],
            'p': [[100., 0., 2., tx], [0., 100., 2., 0.], [0., 0., 1., 0.]],
            'height': 4, 'width': 5}))
    rows = []
    for index in [3, 9]:
        row = {}
        for side in ['left', 'right']:
            path = sensor / f'{side}-{index}.png'
            cv2.imwrite(str(path), np.full((4, 5), index, dtype=np.uint8))
            row['thermal_' + side] = {'path': str(path.relative_to(tmp_path)),
                                     'index': index, 'time_ns': 1761590487000000000 + index}
        rows.append(row)
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({'pairs': rows}))
    return tmp_path, manifest


def test_sequence_uses_rectified_projection_and_source_timestamps(sample):
    root, manifest = sample
    seq = TD3ManifestSequence(root, manifest, 'thermal', acknowledge_provisional_geometry=True)
    assert seq.signed_baseline == pytest.approx(-.2)
    assert seq.baseline == pytest.approx(.2)
    frame = seq.clip(start_idx=1)[0]
    assert frame.stereo.fx == 100.  # P, not the raw K=90
    assert frame.stereo.frame_ns == 1761590487000000009
    assert frame.stereo.imageL.shape == (1, 3, 4, 5)
    torch.testing.assert_close(frame.stereo.imageL[0, 0], frame.stereo.imageL[0, 2])


def test_provisional_opt_in_and_raw_thermal_rejection(sample):
    root, manifest = sample
    with pytest.raises(ValueError, match='acknowledgement'):
        TD3ManifestSequence(root, manifest, 'thermal')
    raw = root / 'raw.png'
    cv2.imwrite(str(raw), np.full((4, 5), 25000, dtype=np.uint16))
    with pytest.raises(ValueError, match='uint8'):
        read_uint8_rgb(raw)


def test_manifest_cannot_escape_dataset(sample):
    root, manifest = sample
    data = json.loads(manifest.read_text())
    data['pairs'][0]['thermal_left']['path'] = '../outside.png'
    manifest.write_text(json.dumps(data))
    seq = TD3ManifestSequence(root, manifest, 'thermal', acknowledge_provisional_geometry=True)
    with pytest.raises(ValueError, match='leaves dataset'):
        seq[0]
