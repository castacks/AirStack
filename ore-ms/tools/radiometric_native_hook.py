"""Opt-in sparse hook; original MACVO source and global class stay unchanged."""
import ast
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import types
import numpy as np
from radiometric_tracking import refine

SOURCE = Path(__file__).resolve().parents[1]/'MAC-VO-main/Odometry/MACVO.py'
SOURCE_SHA256 = '568c8219ffa910332dad0812f951c42ae2f11f7ca615131e0e46bae19964df32'
ANCHOR = 'kp1_uv = kp0_uv + self.Frontend.retrieve_pixels(kp0_uv, match01.flow).T'


def instrument(source):
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA256:
        raise ValueError('Native MACVO source differs from pinned version')
    tree = ast.parse(source)
    methods = [m for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'MACVO'
               for m in c.body if isinstance(m, ast.FunctionDef) and m.name == 'run_pair']
    if len(methods) != 1:
        raise ValueError('Expected exactly one native run_pair')
    body = methods[0].body
    positions = [i for i, n in enumerate(body) if ast.unparse(n) == ANCHOR]
    if len(positions) != 1:
        raise ValueError('Expected exactly one endpoint-construction anchor')
    body.insert(positions[0]+1, ast.parse(
        'kp1_uv = _radiometric_hook(frame0, frame1, kp0_uv, kp1_uv)').body[0])
    return ast.fix_missing_locations(tree)


def passthrough(frame0, frame1, source_uv, endpoint_uv):
    return endpoint_uv


def refine_endpoints(source, target, points, endpoints):
    points, endpoints = np.asarray(points), np.asarray(endpoints)
    if points.ndim != 2 or points.shape[1:] != (2,) or points.shape != endpoints.shape:
        raise ValueError('equal Nx2 source and endpoint arrays required')
    if not np.issubdtype(endpoints.dtype, np.floating):
        raise ValueError('floating endpoint dtype required; refuse coordinate truncation')
    output = endpoints.copy()
    receipts = []
    for i, (point, endpoint) in enumerate(zip(points, endpoints)):
        result = refine(source, target, point, endpoint)
        if result['status'] == 'OK':
            output[i] = result['endpoint_uv']
        receipts.append({'index': i, **result})
    return output, receipts


def tensor_hook(raw_pair, receipts):
    """raw_pair(frame0,frame1) must supply hash-verified native uint16 images.

    Rejects retain original endpoints, original covariance and original masks.
    Raw lookup failures abort; they never silently become passthrough results.
    """
    def hook(frame0, frame1, points, endpoints):
        import torch
        a, b = raw_pair(frame0, frame1)
        values, rows = refine_endpoints(a, b, points.detach().cpu().numpy(),
                                       endpoints.detach().cpu().numpy())
        receipts.append({'source_time_ns': frame0.stereo.frame_ns,
                         'target_time_ns': frame1.stereo.frame_ns,
                         'rows': rows, 'rejection_policy': 'retain original endpoint',
                         'covariance_policy': 'original control; not calibrated for refinement'})
        return torch.as_tensor(values, device=endpoints.device, dtype=endpoints.dtype)
    return hook


def raw_provider(root, manifest):
    """Look up raw images by exact left-camera time, never nearest timestamps."""
    root = Path(root)
    items = {}
    for row in json.loads(Path(manifest).read_text())['records']:
        item = row['cameras']['thr']['left']
        time = item['recorded_time_ns']
        if time in items:
            raise ValueError('Duplicate thermal source timestamp')
        items[time] = item

    def load(frame):
        import cv2
        item = items[int(frame.stereo.frame_ns)]
        path = root/item['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('Raw source hash mismatch')
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None or image.dtype != np.uint16 or list(image.shape) != item['shape']:
            raise ValueError('Raw thermal source schema mismatch')
        return image

    return lambda frame0, frame1: (load(frame0), load(frame1))


@contextmanager
def install(runner, callback=passthrough):
    """Temporarily replace only this runner's class, restoring it on failure."""
    original = runner.MACVO
    module = types.ModuleType('Odometry._radiometric_experiment')
    module.__package__ = 'Odometry'
    module.__file__ = str(SOURCE)
    module.__dict__['_radiometric_hook'] = callback
    exec(compile(instrument(SOURCE.read_bytes()), str(SOURCE), 'exec'), module.__dict__)
    runner.MACVO = module.MACVO
    try:
        yield module.MACVO
    finally:
        runner.MACVO = original
