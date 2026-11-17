"""Frozen-detector envelope replay; records positive-control failures unchanged."""
import hashlib
import inspect
import json
from pathlib import Path
import cv2
import numpy as np
from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import select_viewpoint


def audit(root):
    root=Path(root)
    detector_hash=hashlib.sha256(Path(inspect.getfile(detect_red_surface)).read_bytes()).hexdigest()
    assert detector_hash=='7507196036b7f60358b06c4e7c0cce7b67a5838236eb0750f8092a405c7ed68a'
    complete=json.loads((root/'result.json').read_text())
    assert complete['capture_complete'] is True and len(complete['cases'])==9
    assert not (root/'error.json').exists()
    assert hashlib.sha256((root/'renderer.py').read_bytes()).hexdigest()==complete['source_sha256']
    cases=[]
    hashes={}
    for record in complete['cases']:
        name=record['case']; directory=root/name
        metadata=json.loads((directory/'metadata.json').read_text())
        assert len(metadata['frames'])==3
        observations=[]
        for frame in metadata['frames']:
            for filename,expected in frame['input_sha256'].items():
                digest=hashlib.sha256((directory/filename).read_bytes()).hexdigest()
                assert digest==expected
                hashes[f'{name}/{filename}']=digest
            prefix=frame['prefix']
            rgb=cv2.cvtColor(cv2.imread(str(directory/(prefix+'.png'))),cv2.COLOR_BGR2RGB)
            camera=json.loads((directory/(prefix+'-camera.json')).read_text())
            width,height=camera['renderProductResolution']
            with np.load(directory/(prefix+'.npz'),allow_pickle=False) as data:
                depth=data['depth_m']; k=data['intrinsic'].reshape(-1)
            assert rgb.shape==(height,width,3) and depth.shape==(height,width)
            assert camera['cameraModel']=='pinhole' and camera['metersPerSceneUnit']==1.
            assert camera['cameraApertureOffset']==[0.,0.]
            projection=camera['cameraProjection']
            np.testing.assert_allclose(k,[projection[0]*width/2,0.,width/2,
                                          0.,projection[5]*height/2,height/2,0.,0.,1.],atol=1e-5,rtol=0)
            observations.append(detect_red_surface(rgb,depth,k.tolist()))
        decision=select_viewpoint(observations,[width,height],[1.,0.,0.])
        control_passed=all(o['status']=='OBSERVED' for o in observations)
        if not control_passed:
            assert decision['status']=='ABSTAIN' and decision['delta_map_m'] is None
        cases.append(dict(case=name,resolution=[width,height],camera_position_world_m=metadata['camera_position_world_m'],
                          intended_observation='OBSERVED',observations=observations,
                          positive_control_passed=control_passed,decision=decision))
    return dict(scope='frozen local-depth detector resolution/view development limits',
                physical_execution=False,held_out=False,learned_reasoning=False,
                notes=['Nine nominal-cube development views, not independent trials or held-out accuracy.',
                       'Every scene contains the same cube; abstention is retained as a conservative envelope limit.',
                       'Detector source and numeric limits frozen before rendering this matrix.',
                       'No threshold revision or live scene mutation occurred.'],
                cases=cases,positive_controls_passed=sum(c['positive_control_passed'] for c in cases),
                total_controls=9,frames=27,input_sha256=hashes,detector_sha256=detector_hash,
                source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest()
                               for name in ('renderer.py','fixture-author.py','audit.py')})


if __name__=='__main__':
    print(json.dumps(audit(Path(__file__).parent),indent=2,allow_nan=False))
