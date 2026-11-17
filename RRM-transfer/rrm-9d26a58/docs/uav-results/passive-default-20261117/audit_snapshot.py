"""Frozen passive capture audit: headers for all records, raw samples only."""
import collections
import hashlib
import inspect
import json
from pathlib import Path
import sys
import cv2
import numpy as np
from rrm.uav_marker import detect_red_surface
from rrm.camera_pairing import camera_pair_metadata

root = Path(sys.argv[1])
rows = [json.loads(line) for line in (root/'observations.jsonl').read_text().splitlines()]
summary = json.loads((root/'summary.json').read_text())
provenance = json.loads((root/'provenance.json').read_text())
assert len(rows) == summary['exact_stamp_observations'] == 20
assert dict(collections.Counter(row['observation']['status'] for row in rows)) == summary['statuses']
for obj, key in ((detect_red_surface, 'detector_sha256'), (camera_pair_metadata, 'pairing_sha256')):
    assert hashlib.sha256(Path(inspect.getfile(obj)).read_bytes()).hexdigest() == provenance[key]
stamps = []
raw = []
for row in rows:
    headers = row['headers']
    assert set(headers) == {'rgb', 'depth', 'info'}
    assert all(header == headers['rgb'] for header in headers.values())
    stamp = (row['stamp_sec'], row['stamp_nanosec'])
    assert headers['rgb'] == dict(frame_id=row['frame'], stamp=dict(sec=stamp[0], nanosec=stamp[1]))
    assert row['frame'] and row['exact_stamp_pair'] and row['depth_encoding'] == '32FC1'
    stamps.append(stamp)
    assert row['pair_receipt_span_wall_s'] >= 0
    assert all(0 <= row['wall_unix']-t < 1 for t in row['receipt_wall_unix'].values())
    if 'raw_prefix' not in row:
        continue
    prefix = row['raw_prefix']
    assert Path(prefix).name == prefix
    for name, digest in row['raw_sha256'].items():
        assert Path(name).name == name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest
    image = cv2.cvtColor(cv2.imread(str(root/(prefix+'.png'))), cv2.COLOR_BGR2RGB)
    with np.load(root/(prefix+'.npz'), allow_pickle=False) as saved:
        intrinsic = saved['intrinsic'].tolist()
        assert intrinsic == row['intrinsic']
        assert [image.shape[1], image.shape[0]] == row['image_size']
        assert saved['depth_m'].shape == image.shape[:2]
        assert detect_red_surface(image, saved['depth_m'], intrinsic) == row['observation']
    raw.append(prefix)
assert stamps == sorted(set(stamps))
assert len(raw) == 3
result = dict(scope='all retained headers; only three saved raw detector inputs independently recomputed',
              header_records_checked=len(rows), raw_samples_checked=len(raw),
              raw_unrecomputed_records=len(rows)-len(raw), statuses=summary['statuses'],
              source_span_s=(stamps[-1][0]-stamps[0][0])+(stamps[-1][1]-stamps[0][1])*1e-9,
              maximum_pair_receipt_span_wall_s=max(row['pair_receipt_span_wall_s'] for row in rows),
              input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.is_file()},
              verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              renderer_synchronization_proven=False, physical_execution=False, semantic_truth_proven=False)
print(json.dumps(result, indent=2))
