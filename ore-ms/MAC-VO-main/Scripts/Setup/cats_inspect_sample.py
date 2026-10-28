"""Inspect retrieved native CATS members; preserve sign/NaNs and original pixels."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def inspect(root):
    declaration = json.loads((root / 'sample-declaration.json').read_text())
    access = json.loads((root / 'access-validation.json').read_text())
    scene = root / 'sample' / declaration['scene']
    rows = {}
    for name, digest in access['sample_sha256'].items():
        path = root / 'sample' / name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
        relative = str(path.relative_to(scene))
        item = {'bytes': path.stat().st_size, 'sha256': digest}
        if path.suffix == '.png':
            a = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            item.update(shape=list(a.shape), dtype=str(a.dtype), min=int(a.min()), max=int(a.max()),
                        unique_values=np.unique(a).tolist() if 'mask' in path.name else None)
        elif path.suffix == '.txt':
            a = np.loadtxt(path, delimiter=',')
            finite = a[np.isfinite(a)]
            item.update(shape=list(a.shape), dtype=str(a.dtype), finite_count=int(np.isfinite(a).sum()),
                        nonfinite_count=int((~np.isfinite(a)).sum()), min=float(finite.min()), max=float(finite.max()),
                        positive_count=int((a > 0).sum()), zero_count=int((a == 0).sum()), negative_count=int((a < 0).sum()))
        elif path.suffix == '.asc':
            with path.open() as f:
                item['first_three_rows'] = [next(f).strip() for _ in range(3)]
        rows[relative] = item
    for camera in ('color', 'thermal', 'cross'):
        a = rows[f'gt_disparity/{camera}/gt_disparity.txt']
        image = rows[f'rectified/{camera}/left_thermal_default.png' if camera != 'color'
                     else 'rectified/color/left_color_default.png']
        assert a['shape'] == image['shape'][:2]
        assert a['positive_count'] + a['zero_count'] + a['negative_count'] == a['finite_count']
    result = {'status': 'PASS', 'scene': declaration['scene'],
              'scope': 'schema/hash/count inspection only; signed disparity and mask meanings unverified; no model evaluation',
              'files': rows, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (root / 'schema-inspection.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': 'PASS', 'files': len(rows), 'scene': declaration['scene']}))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    inspect(p.parse_args().run)
