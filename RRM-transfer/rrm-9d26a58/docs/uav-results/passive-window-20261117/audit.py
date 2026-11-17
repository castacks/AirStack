"""Independent arithmetic/header/raw checks on saved passive receipt windows."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from rrm.uav_marker import detect_red_surface
from rrm.camera_pairing import camera_window_metadata

root=Path(__file__).parent
records=[json.loads(line) for line in (root/'observations.jsonl').read_text().splitlines()]
provenance=json.loads((root/'provenance.json').read_text())
assert len(records)==20
assert provenance['capture_sha256']==hashlib.sha256((root/'capture.py').read_bytes()).hexdigest()
assert provenance['pairing_sha256']==hashlib.sha256((root/'assessed-camera-pairing.py').read_bytes()).hexdigest()
assert provenance['detector_sha256']==hashlib.sha256(Path('rrm/uav_marker.py').read_bytes()).hexdigest()
checks=[]
raw=0
for index, record in enumerate(records):
 for key in ('rgb','depth','info'):
  assert record['headers'][key]==record['headers']['rgb']
 if 'raw_prefix' in record:
  for name,digest in record['raw_sha256'].items():
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest
  prefix=record['raw_prefix']
  rgb=cv2.cvtColor(cv2.imread(str(root/(prefix+'.png'))),cv2.COLOR_BGR2RGB)
  with np.load(root/(prefix+'.npz'),allow_pickle=False) as data:
   actual=detect_red_surface(rgb,data['depth_m'],data['intrinsic'].tolist())
  assert actual==record['observation']
  raw+=1
 if index>=2:
  evidence=record['window_check']['evidence']
  window=records[index-2:index+1]
  times=[r['receipt_monotonic_s'][key] for r in window for key in ('rgb','depth','info')]
  stamps=[r['stamp_sec']*1_000_000_000+r['stamp_nanosec'] for r in window]
  assert stamps[0]<stamps[1]<stamps[2]
  assert evidence['started_monotonic_s']<=min(times)<=max(times)<=evidence['checked_monotonic_s']
  assert evidence['checked_monotonic_s']-min(times)<2
  assert abs(evidence['max_channel_receipt_age_s']-(evidence['checked_monotonic_s']-min(times)))<1e-10
  assert abs(evidence['source_span_s']-(stamps[-1]-stamps[0])*1e-9)<1e-10
  # Recorded same-process check times only: replay is not fresh live permission.
  assert camera_window_metadata(window,started_monotonic=evidence['started_monotonic_s'],
                                now_monotonic=evidence['checked_monotonic_s'])==evidence
  checks.append(evidence)
report=dict(scope='saved passive source headers/receipt arithmetic; no fresh permission on replay',
            records=20,windows=len(checks),raw_recomputed=raw,unsaved_raw=20-raw,
            statuses={s:sum(r['observation']['status']==s for r in records) for s in ('ABSENT',)},
            max_window_receipt_age_s=max(c['max_channel_receipt_age_s'] for c in checks),
            max_window_receipt_span_s=max(c['window_receipt_span_s'] for c in checks),
            physical_execution=False,source_sha256=provenance,
            input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.is_file()})
print(json.dumps(report,indent=2,allow_nan=False))
