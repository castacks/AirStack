#!/usr/bin/env python3
"""Offline frozen-capture comparison. Predict never reads simulator truth arrays.

Requires diagnostic NumPy/OpenCV; learned candidate additionally upstream RAFT and
PyTorch. Does not install dependencies, publish ROS data, or alter runtime settings.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rrm.stereo_benchmark import comparison_metrics, disparity_depth, rectangle_mask

NAMES = ('recorded_bm', 'sgbm64', 'sgbm128', 'raft_middlebury')
REGIONS = {'whole': None, 'central_view': (200,130,80,60),
           'low_texture': (120,60,100,60), 'floor_patch': (200,220,80,60)}
SETTINGS = dict(blockSize=5, minDisparity=0, P1=8*25, P2=32*25,
                disp12MaxDiff=1, preFilterCap=31, uniquenessRatio=10,
                speckleWindowSize=100, speckleRange=2)


def sha(path):
    return hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def corpus_rows(captures):
    rows, missing = [], []
    for capture in captures:
        manifest = json.loads((capture/'capture.json').read_text())
        for record in manifest['records']:
            if record['status'] != 'HEADER_PAIRED_ASSESSMENT':
                missing.append(dict(corpus=capture.name, record=record))
                continue
            folder = capture/f"sample-{record['sample_index']:02}"
            rows.append((capture.name, folder, record))
    if len({name for name, _, _ in rows}) != len(captures):
        raise ValueError('nonunique corpus names or empty corpus')
    return rows, missing


def build_sgbm(num_disparities):
    import cv2
    return cv2.StereoSGBM_create(numDisparities=num_disparities,
                               mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY, **SETTINGS)


def infer_sgbm(matcher, left, right):
    import cv2
    return matcher.compute(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY),
                           cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)).astype(np.float32)/16


def build_raft(repo, weights):
    import torch
    sys.path.insert(0, str(repo.resolve()))
    from core.raft_stereo import RAFTStereo
    from core.utils.utils import InputPadder
    config = argparse.Namespace(hidden_dims=[128]*3, corr_implementation='reg',
        shared_backbone=False, corr_levels=4, corr_radius=4, n_downsample=2,
        context_norm='batch', slow_fast_gru=False, n_gru_layers=3, mixed_precision=False)
    model = RAFTStereo(config)
    state = torch.load(weights, map_location='cpu', weights_only=True)
    model.load_state_dict({k.removeprefix('module.'): v for k,v in state.items()}, strict=True)
    model.to('cuda').eval()
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False

    def predict(left, right):
        tensors = [torch.from_numpy(np.ascontiguousarray(x)).permute(2,0,1)[None].float().cuda()
                   for x in (left, right)]
        padder = InputPadder(tensors[0].shape, divis_by=32)
        with torch.inference_mode():
            _, flow = model(*padder.pad(*tensors), iters=32, test_mode=True)
            # Upstream flow = right coordinate minus left; disparity is the negative.
            return -padder.unpad(flow)[0,0].cpu().numpy()
    return predict, vars(config), torch


def predict(args):
    import cv2
    args.output.mkdir(parents=True, exist_ok=False)
    rows, missing = corpus_rows(args.captures)
    raft, architecture, torch = build_raft(args.raft_repo, args.weights)
    matchers = {'sgbm64': build_sgbm(64), 'sgbm128': build_sgbm(128)}
    manifest = dict(written_at_utc=datetime.now(timezone.utc).isoformat(),
        evidence_scope='OFFLINE_DIAGNOSTIC_ONLY', execution_dispatch=False,
        script_sha256=sha(Path(__file__)), metric_sha256=sha(Path(__file__).resolve().parents[1]/'rrm/stereo_benchmark.py'),
        source_commit=subprocess.check_output(['git','-C',str(args.raft_repo),'rev-parse','HEAD'], text=True).strip(),
        source_dirty=bool(subprocess.check_output(['git','-C',str(args.raft_repo),'status','--porcelain'], text=True).strip()),
        weights_sha256=sha(args.weights), weights_name=args.weights.name,
        raft_architecture=architecture, raft_iterations=32, sgbm_settings=SETTINGS,
        sgbm_mode='SGBM_3WAY', versions=dict(torch=torch.__version__, numpy=np.__version__, opencv=cv2.__version__),
        gpu=torch.cuda.get_device_name(), native_acquisition_binding_verified=False,
        missing_attempts=missing, records=[], input_manifest_sha256={str(p):sha(p/'capture.json') for p in args.captures})
    cv2.setNumThreads(1)
    # One untimed warm-up per predictor. Saved separately, not in frame timing.
    _, first, _ = rows[0]
    l, r = [np.load(first/f'{name}.npy', allow_pickle=False) for name in ('left','right')]
    torch.cuda.synchronize()
    started = time.perf_counter()
    raft(l,r)
    torch.cuda.synchronize()
    manifest['raft_warmup_s'] = time.perf_counter()-started
    for matcher in matchers.values():
        infer_sgbm(matcher,l,r)
    for corpus, folder, record in rows:
        dest = args.output/corpus/folder.name
        dest.mkdir(parents=True)
        input_hashes = {name:sha(folder/f'{name}.npy') for name in ('left','right','disparity')}
        left, right = [np.load(folder/f'{name}.npy', allow_pickle=False) for name in ('left','right')]
        if left.shape != (300,480,3) or right.shape != left.shape or left.dtype != np.uint8 or right.dtype != np.uint8:
            raise ValueError('expected original 480x300 RGB uint8 capture')
        if hashlib.sha256(left.tobytes()).hexdigest() != record['image_sha256']:
            raise ValueError('left image binding mismatch')
        predictions = {'recorded_bm':np.load(folder/'disparity.npy', allow_pickle=False)}
        timings = {}
        for name, matcher in matchers.items():
            started = time.perf_counter()
            predictions[name] = infer_sgbm(matcher, left, right)
            timings[name] = time.perf_counter()-started
        torch.cuda.synchronize()
        started = time.perf_counter()
        predictions['raft_middlebury'] = raft(left,right)
        torch.cuda.synchronize()
        timings['raft_middlebury'] = time.perf_counter()-started
        for name, value in predictions.items():
            if value.shape != left.shape[:2]:
                raise ValueError('prediction shape mismatch')
            np.save(dest/f'{name}.npy', value, allow_pickle=False)
        if input_hashes != {name:sha(folder/f'{name}.npy') for name in input_hashes}:
            raise ValueError('input mutated during prediction')
        manifest['records'].append(dict(corpus=corpus,sample=folder.name,
            input_sha256=input_hashes,record_sha256=sha(folder/'record.json'),calibration=record['calibration'],
            elapsed_s=timings,output_sha256={name:sha(dest/f'{name}.npy') for name in predictions}))
        write_json(args.output/'predictions.json',manifest)
        print(f'{corpus}/{folder.name} {timings}', flush=True)


def assess(args):
    args.output.mkdir(parents=True, exist_ok=False)
    rows, missing = corpus_rows(args.captures)
    manifest = json.loads((args.predictions/'predictions.json').read_text())
    if manifest['input_manifest_sha256'] != {str(p):sha(p/'capture.json') for p in args.captures}:
        raise ValueError('capture manifest changed')
    records = {(r['corpus'],r['sample']):r for r in manifest['records']}
    collected = {}
    for corpus, folder, record in rows:
        provenance = records[(corpus,folder.name)]
        for name, expected in provenance['input_sha256'].items():
            if sha(folder/f'{name}.npy') != expected:
                raise ValueError('input checksum mismatch')
        if sha(folder/'record.json') != provenance['record_sha256']:
            raise ValueError('record checksum mismatch')
        truth = np.load(folder/'truth.npy', allow_pickle=False)
        cal = record['calibration']
        fb = cal['fx']*cal['baseline_m']
        predictions = {}
        for name in NAMES:
            path = args.predictions/corpus/folder.name/f'{name}.npy'
            if sha(path) != provenance['output_sha256'][name]:
                raise ValueError('prediction checksum mismatch')
            upper = {'recorded_bm':cal['max_disparity'],'sgbm64':63,'sgbm128':127}.get(name)
            predictions[name] = disparity_depth(np.load(path, allow_pickle=False), fb, upper)
        whole = rectangle_mask(truth.shape, cal['valid_window'])
        observable = rectangle_mask(truth.shape, (128,7,344,285)) & (truth >= fb/63)
        for label, rectangle in REGIONS.items():
            area = whole if rectangle is None else whole & rectangle_mask(truth.shape, rectangle)
            for domain, mask in [('original',area),('observable',area & observable)]:
                key = (corpus,label,domain)
                collected.setdefault(key,[]).append((truth[mask], {n:p[mask] for n,p in predictions.items()}))
    summary = {}
    for (corpus,label,domain), samples in collected.items():
        truth = np.concatenate([t for t,_ in samples])
        predictions = {n:np.concatenate([p[n] for _,p in samples]) for n in NAMES}
        summary.setdefault(corpus,{}).setdefault(domain,{})[label] = comparison_metrics(truth,predictions,np.ones(truth.shape,dtype=bool))
    timing = {n:np.quantile([r['elapsed_s'][n] for r in manifest['records']], [.5,.95]).tolist() for n in NAMES[1:]}
    write_json(args.output/'assessment.json',dict(written_at_utc=datetime.now(timezone.utc).isoformat(),
        script_sha256=sha(Path(__file__)), metric_sha256=sha(Path(__file__).resolve().parents[1]/'rrm/stereo_benchmark.py'),
        prediction_manifest_sha256=sha(args.predictions/'predictions.json'),
        evidence_scope='MATCHED_HEADER_STATIONARY_DIAGNOSTIC_ONLY',execution_dispatch=False,
        truth_sha256={f'{c}/{f.name}':sha(f/'truth.npy') for c,f,_ in rows},
        paired_frames=len(rows),missing_attempts=missing,latency_p50_p95_s=timing,summary=summary))
    print(json.dumps(dict(paired_frames=len(rows),latency_p50_p95_s=timing)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('predict','assess'):
        child = sub.add_parser(command)
        child.add_argument('--captures', type=Path, nargs='+', required=True)
        child.add_argument('--output', type=Path, required=True)
        if command == 'predict':
            child.add_argument('--raft-repo',type=Path,required=True)
            child.add_argument('--weights',type=Path,required=True)
        else:
            child.add_argument('--predictions',type=Path,required=True)
    args = parser.parse_args()
    (predict if args.command == 'predict' else assess)(args)


if __name__ == '__main__':
    main()
