"""Actual native persistent-state CUDA allocation; no images/weights/tracking/backend."""
import gc
import hashlib
import json
from pathlib import Path
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/whole-record-preflight/run-001'


def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()


def run():
    p = json.loads((ROOT/'experiments/whole-record-preflight-2026-11-04.json').read_text())
    for path, digest in p['source_sha256'].items():
        if sha(ROOT/path) != digest: raise ValueError('changed source '+path)
    sys.path.insert(0, '/opt/DROID-SLAM/droid_slam')
    from depth_video import DepthVideo
    import depth_video
    if sha(Path(depth_video.__file__)) != sha(ROOT/'results/whole-record-preflight/source-001/depth_video.py'):
        raise ValueError('runtime DepthVideo differs from frozen source')
    device = torch.cuda.get_device_properties(0)
    rows = []
    for camera, shape in p['geometry'].items():
        for buffer in p['probe_buffers']:
            gc.collect(); torch.cuda.empty_cache(); torch.cuda.synchronize()
            before = torch.cuda.memory_allocated(); torch.cuda.reset_peak_memory_stats()
            video = DepthVideo(image_size=shape, buffer=buffer, stereo=True, device='cuda:0')
            torch.cuda.synchronize()
            tensors = {key: {'shape': list(value.shape), 'dtype': str(value.dtype),
                       'elements': value.numel(), 'element_bytes': value.element_size(),
                       'bytes': value.numel()*value.element_size()}
                       for key, value in vars(video).items() if isinstance(value, torch.Tensor)}
            allocated = torch.cuda.memory_allocated()-before
            rows.append({'camera': camera, 'shape': shape, 'buffer': buffer, 'tensors': tensors,
                         'tensor_bytes': sum(v['bytes'] for v in tensors.values()),
                         'cuda_allocated_delta_bytes': allocated,
                         'cuda_peak_delta_bytes': torch.cuda.max_memory_allocated()-before})
            del video; gc.collect(); torch.cuda.empty_cache(); torch.cuda.synchronize()
    free, total = torch.cuda.mem_get_info()
    report = {'gpu': device.name, 'gpu_total_bytes': total, 'gpu_free_bytes_after_probe': free,
              'runtime_image': p['runtime_image'], 'torch': torch.__version__,
              'depth_video_sha256': sha(Path(depth_video.__file__)), 'probes': rows,
              'weights_loaded': False, 'images_read': False, 'tracking_executed': False,
              'backend_executed': False, 'physical_sync_verified': False,
              'scope': 'native persistent tensor storage only; model/workspace/correlation/backend excluded'}
    with (OUT/'capacity-probe.json').open('x') as f: json.dump(report, f, indent=2); f.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'probes'}, indent=2))
    for r in rows: print(r['camera'], r['buffer'], r['tensor_bytes'], r['cuda_allocated_delta_bytes'])


if __name__ == '__main__': run()
