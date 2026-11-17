"""Recompute frozen private-render evidence; no ROS or renderer startup."""
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np

from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import select_viewpoint


def audit(root):
    root = Path(root)
    complete = json.loads((root/'result.json').read_text())
    assert complete['capture_complete'] is True
    assert hashlib.sha256((root/'renderer.py').read_bytes()).hexdigest() == complete['source_sha256']
    spec = importlib.util.spec_from_file_location('frozen_baseline_marker', root/'baseline-uav-marker.py')
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    expected = {'nominal': 'OBSERVED', 'absent': 'ABSENT', 'duplicate-red': 'AMBIGUOUS',
                'split-occlusion': 'AMBIGUOUS', 'full-occlusion': 'ABSENT',
                'partial-occlusion': 'DEPTH_INCONSISTENT'}
    results, hashes = [], {}
    for case, status in expected.items():
        directory = root/case
        metadata = json.loads((directory/'metadata.json').read_text())
        observations, controls = [], []
        assert len(metadata['frames']) == 3
        for frame in metadata['frames']:
            for name, digest in frame['input_sha256'].items():
                actual = hashlib.sha256((directory/name).read_bytes()).hexdigest()
                assert actual == digest, (case, name)
                hashes[f'{case}/{name}'] = actual
            prefix = frame['prefix']
            rgb = cv2.cvtColor(cv2.imread(str(directory/(prefix+'.png'))), cv2.COLOR_BGR2RGB)
            camera = json.loads((directory/(prefix+'-camera.json')).read_text())
            with np.load(directory/(prefix+'.npz'), allow_pickle=False) as saved:
                depth, k = saved['depth_m'], saved['intrinsic'].reshape(-1)
            assert camera['cameraModel'] == 'pinhole'
            assert camera['cameraApertureOffset'] == [0., 0.]
            assert camera['metersPerSceneUnit'] == 1.
            width, height = camera['renderProductResolution']
            assert rgb.shape == (height, width, 3) and depth.shape == (height, width)
            # Verify saved intrinsics against independent projection output as well
            # as authored optical parameters. cameraOpenCVFx/Fy are zero in this SDK.
            projection = camera['cameraProjection']
            np.testing.assert_allclose(k, [projection[0]*width/2, 0., width/2,
                                          0., projection[5]*height/2, height/2, 0., 0., 1.],
                                       atol=1e-5, rtol=0)
            np.testing.assert_allclose([k[0], k[4]],
                [camera['cameraFocalLength']/camera['cameraAperture'][0]*width,
                 camera['cameraFocalLength']/camera['cameraAperture'][1]*height], atol=1e-5, rtol=0)
            observed = detect_red_surface(rgb, depth, k.tolist())
            observations.append(observed)
            controls.append(baseline.detect_red_surface(rgb, depth, k.tolist()))
            assert observed['status'] == status, (case, prefix, observed)
        decision = select_viewpoint(observations, [width, height], [1., 0., 0.])
        if status != 'OBSERVED':
            assert decision['status'] == 'ABSTAIN' and decision['delta_map_m'] is None
        results.append(dict(case=case, expected_observation=status, observations=observations,
                            baseline_observations=controls, decision=decision,
                            passed=True))
    return dict(scope='private static renderer development replay; no robot camera or flight',
                physical_execution=False, learned_reasoning=False, held_out=False,
                notes=['Partial-occlusion expectation records a measured local depth discontinuity; not general occlusion detection.',
                       'Three separately rendered frames of a static stage do not establish robot camera synchronization or identity.',
                       'Nominal control informed the local continuity revision; this is not an accuracy estimate.',
                       'Old total-spread profile rejected nominal and partial in all saved frames.',
                       'Viewpoint decision is reported independently; OBSERVED need not imply SELECTED.'],
                cases=results, passed=len(results), total=len(expected), frames=18,
                input_sha256=hashes,
                source_sha256={name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                               for name in ('audit.py','renderer.py','fixture-author.py','baseline-uav-marker.py')},
                detector_sha256=hashlib.sha256(Path(__import__('inspect').getfile(detect_red_surface)).read_bytes()).hexdigest())


if __name__ == '__main__':
    print(json.dumps(audit(Path(__file__).parent), indent=2, allow_nan=False))
