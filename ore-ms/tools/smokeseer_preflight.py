"""Whole-population SmokeSeer input preflight; never estimates image motion."""
import concurrent.futures
import hashlib
import json
import math
import struct
import zipfile
import zlib
from pathlib import Path

import cv2
import numpy as np

ROOT = Path('/root/AirStack/ore-ms')
INTAKE = ROOT/'results/smokeseer-synthetic-intake'
OUT = ROOT/'results/smokeseer-preflight'
DATA = ROOT/'data/smokeseer-synthetic'
ARCHIVE = Path('/tmp/ore-smokeseer-synthetic.zip')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def png_check(data):
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    offset, chunks = 8, []
    while offset < len(data):
        size = struct.unpack('>I', data[offset:offset+4])[0]
        end = offset+12+size
        assert end <= len(data)
        kind = data[offset+4:offset+8]
        assert zlib.crc32(data[offset+4:end-4]) & 0xffffffff == struct.unpack('>I', data[end-4:end])[0]
        chunks.append(kind)
        offset = end
        if kind == b'IEND':
            break
    assert chunks[0] == b'IHDR' and b'IDAT' in chunks and chunks[-1] == b'IEND' and offset == len(data)


def decode(name):
    # Separate ZIP handles avoid sharing mutable seek state across workers.
    with zipfile.ZipFile(ARCHIVE) as archive:
        data = archive.read(name)  # Includes selected member's ZIP CRC check.
    png_check(data)
    array = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    assert array is not None and array.dtype in (np.uint8, np.uint16)
    assert array.shape[:2] == (800, 800), (name, array.shape)
    channels = 1 if array.ndim == 2 else array.shape[2]
    assert channels in (1, 3, 4)
    normalized = array.astype(np.float64)/np.iinfo(array.dtype).max
    alpha_range = None
    if channels == 1:
        gray = normalized
    else:
        # OpenCV supplies BGR(A). Alpha composites onto fixed black.
        gray = normalized[..., 2]*.299 + normalized[..., 1]*.587 + normalized[..., 0]*.114
        if channels == 4:
            gray *= normalized[..., 3]
            alpha_range = [int(array[..., 3].min()), int(array[..., 3].max())]
    gray = np.clip(np.rint(gray*255), 0, 255).astype(np.uint8)
    path = DATA/name
    assert path.resolve().is_relative_to(DATA.resolve())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return {'member': name, 'sha256': sha(data), 'bytes': len(data),
            'dtype': str(array.dtype), 'channels': channels, 'shape': list(array.shape),
            'decoded_sha256': sha(array.tobytes()), 'gray_sha256': sha(gray.tobytes()),
            'alpha_range': alpha_range}


def main():
    cv2.setNumThreads(1)
    OUT.mkdir(parents=True, exist_ok=True)
    index = json.loads((INTAKE/'zip-index.json').read_text())
    assert ARCHIVE.stat().st_size == index['archive_bytes']
    with zipfile.ZipFile(ARCHIVE) as archive:
        infos = {x.filename: x for x in archive.infolist()}
        assert len(infos) == len(index['members'])
        for m in index['members']:
            z = infos[m['name']]
            assert (z.CRC,z.compress_size,z.file_size,z.header_offset,z.compress_type) == (
                m['crc32'],m['compressed_bytes'],m['uncompressed_bytes'],m['offset'],m['method'])
    images, pairs, scenes = set(), [], []
    flip = np.diag([1., -1., -1., 1.])
    for camera in sorted((INTAKE/'metadata').rglob('transforms_train.json')):
        scene = str(camera.parent.relative_to(INTAKE/'metadata'))
        scenes.append(scene)
        for arm, filename in [('rgb_smoke','transforms_train.json'), ('rgb_clean','transforms_train.json'), ('thermal','transforms_thermaltrain.json')]:
            path = camera.parent/filename
            meta = json.loads(path.read_text())
            bound, poses, times = [], [], []
            for ordinal, frame in enumerate(meta['frames']):
                member = scene+'/'+'/'.join(frame['file_path'].split('/')[-2:])+'.png'
                expected = f'r_thermal{ordinal}' if arm == 'thermal' else f'r_{ordinal}'
                assert Path(member).stem == expected
                if arm == 'rgb_clean':
                    member = member[:-4]+'_gt.png'
                assert member in infos
                images.add(member)
                bound.append(member)
                c = np.asarray(frame['transform_matrix'], np.float64)@flip
                assert c.shape == (4,4) and np.isfinite(c).all()
                np.testing.assert_allclose(c[3], [0,0,0,1], atol=1e-12)
                np.testing.assert_allclose(c[:3,:3].T@c[:3,:3], np.eye(3), atol=1e-5)
                assert abs(np.linalg.det(c[:3,:3])-1) < 1e-5
                poses.append(c)
                times.append(float(frame['frame']))
            assert all(math.isfinite(t) for t in times) and all(b>a for a,b in zip(times,times[1:])), scene
            focal = 800/(2*math.tan(meta['camera_angle_x']/2))
            for i in range(len(bound)-1):
                t = np.linalg.solve(poses[i+1], poses[i])
                np.testing.assert_allclose(poses[i+1]@t, poses[i], atol=1e-10)
                # Rendered rotations have small float export error; use closest SO3
                # only to describe their angle, retaining original transform bytes.
                u, _, vt = np.linalg.svd(t[:3,:3])
                rotation = u@vt
                assert np.linalg.det(rotation)>0
                angle = float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
                magnitude = float(np.linalg.norm(t[:3,3]))
                pairs.append({'scene':scene,'arm':arm,'index':i,'source':bound[i],
                    'target':bound[i+1], 'camera_json_sha256':sha(path.read_bytes()),
                    'K':[[focal,0,400],[0,focal,400],[0,0,1]],
                    'source_to_next_optical':t.tolist(), 'reference_rotation_deg':angle,
                    'reference_translation_render_units':magnitude,
                    'translation_direction_defined':magnitude>1e-9,
                    'animation_times':[times[i],times[i+1]]})
    assert len(scenes)==10 and len(images)==3495 and len(pairs)==3465
    (OUT/'declared-pairs.json').write_text(json.dumps(pairs, indent=2)+'\n')
    receipts=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for record in pool.map(decode, sorted(images)):
            receipts.append(record)
            if len(receipts)%250==0:
                print('Decoded',len(receipts),'/',len(images),flush=True)
    (OUT/'image-receipts.json').write_text(json.dumps(receipts,indent=2)+'\n')
    archive_sha=hashlib.file_digest(ARCHIVE.open('rb'),'sha256').hexdigest()
    summary={'status':'input-preflight-pass','images':len(receipts),'pairs':len(pairs),
             'scenes':len(scenes),'archive_bytes':ARCHIVE.stat().st_size,'archive_sha256':archive_sha,
             'opencv':cv2.__version__,'numpy':np.__version__,'predictions':0,'arms':{}}
    for arm in ['rgb_smoke','rgb_clean','thermal']:
        selected=[x for x in pairs if x['arm']==arm]
        angles=[x['reference_rotation_deg'] for x in selected]
        magnitudes=[x['reference_translation_render_units'] for x in selected]
        summary['arms'][arm]={'pairs':len(selected),'zero_translation':sum(not x['translation_direction_defined'] for x in selected),
            'rotation_deg_quantiles':np.percentile(angles,[0,25,50,75,100]).tolist(),
            'translation_render_units_quantiles':np.percentile(magnitudes,[0,25,50,75,100]).tolist(),
            'identity_rotation_success_5deg':sum(a<=5 for a in angles)/len(angles)}
    pins={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in [
        Path(__file__).resolve(), ROOT/'docs/research/smokeseer-relative-pose-protocol-2026-11-15.md',
        OUT/'declared-pairs.json',OUT/'image-receipts.json',INTAKE/'zip-index.json']}
    (OUT/'preflight.json').write_text(json.dumps(summary,indent=2)+'\n')
    (OUT/'freeze.json').write_text(json.dumps({'pins':pins,'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09',
        'scope':'Input/protocol freeze only; prediction runner not yet qualified'},indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    main()
