"""Rasterized challenge scenes, immutable M10 frontend and policy, separate scoring."""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
from time import perf_counter

import numpy as np

import reference as ref

ROOT = Path(__file__).resolve().parent.parent / 'm10'
sys.path.extend(str(ROOT / folder) for folder in ('preflight','experiment'))
import baseline
import model

ARMS = ('B01','B02','B03','B04','A01')
SHAPE = (25,33,4)
ROWS,COLS = np.indices(SHAPE[:2])
METADATA = dict(zip(baseline.FEATURES,('2000','2000','1','10','1')))


def scenes():
    rng = np.random.default_rng(110010)
    for cluster in range(48):
        x,y = rng.uniform((13,9),(19,15))
        contrast = rng.uniform(.12,.32,2)
        noise = rng.integers(-8,9,size=SHAPE)
        # Common scene and observation perturbations across counterfactual offsets.
        for size in (2,4,6):
            target = ref.square(x,y,size/2,size/2)
            for dx,dy in [(0,0),*((d,0) for d in range(1,5)),*((d,d) for d in range(1,5))]:
                for control in ('clear','edge','partial','distractor'):
                    silhouette = target + ([7,0] if control=='distractor' else [0,0])
                    rgb = ref.raster(COLS,ROWS,silhouette)
                    lw = ref.raster(COLS,ROWS,silhouette+[dx,dy])
                    if control=='edge':
                        # Projected foreground polygon, stationary in RGB coordinates.
                        blocker = ref.square(x-size/2,y,size/5,size)
                        occluded = ref.raster(COLS,ROWS,blocker)
                        rgb &= ~occluded
                        lw &= ~occluded
                    if control=='partial':
                        # Deliberately under a quarter of the unobstructed 6x6 case.
                        for mask in (rgb,lw):
                            indices = np.flatnonzero(mask)
                            limited = indices[:min(4,max(1,len(indices)//5))]
                            mask[:] = False
                            mask.flat[limited] = True
                    codes = 740+noise
                    codes[...,:3] += rgb[...,None]*round(4095*contrast[0])
                    codes[...,3] += lw*round(4095*contrast[1])
                    feasible = control in ('clear','edge') and int(rgb.sum())>=4
                    yield (cluster,size,dx,dy,control,feasible,x,y),codes.astype(np.uint16)


def threshold_diagnostic(pair):
    if any(candidate is None for candidate in pair):
        return {}
    distance = math.dist((pair[0].x,pair[0].y),(pair[1].x,pair[1].y))
    return {str(limit):int(distance>limit) for limit in (1,2,3)}


def summarize(rows):
    n = len(rows)
    if not n:
        raise ValueError('empty score population')
    feasible = sum(r['feasible'] for r in rows)
    issues = [r for r in rows if r['issued']]
    hits = sum(r['correct'] and r['issued'] for r in rows)
    negatives = n-feasible
    false_negative_scene = sum(not r['feasible'] for r in issues)
    calibrated = [r for r in rows if r['probability'] is not None]
    bins = []
    for i in range(10):
        values = [r for r in calibrated if min(int(r['probability']*10),9)==i]
        if values:
            bins.append({'n':len(values),'mean_probability':float(np.mean([r['probability'] for r in values])),
                         'accuracy':float(np.mean([r['correct'] for r in values]))})
    return {'n':n,'feasible':feasible,'issued':len(issues),'correct_issued':hits,
            'coverage':len(issues)/n,'selective_risk':1-hits/len(issues) if issues else None,
            'supported_localization':hits/feasible if feasible else None,
            'negative_scene_issues':false_negative_scene,'negative_scenes':negatives,
            'wrong_issued_including_negative':len(issues)-hits,
            'conflict_warnings':sum(r['warning'] for r in rows),
            'reason_counts':{reason:sum(r['reason']==reason for r in rows)
                             for reason in sorted({r['reason'] for r in rows})},
            'calibration_n':len(calibrated),
            'candidate_brier':float(np.mean([(r['probability']-r['correct'])**2
                                              for r in calibrated])) if calibrated else None,
            'reliability_bins':bins}


def fusion_checks(output, timings):
    deployment = json.loads((ROOT/'experiment/evidence/deployment.json').read_text())
    historical = json.loads((ROOT/'preflight/evidence/calibration.json').read_text())
    old_rules = {arm:{'cutoff':v['chosen']['cutoff'],'mapping':v['mapping']}
                 for arm,v in historical['calibration'].items()}
    if [old_rules[arm]['cutoff'] for arm in ARMS[:3]] != [.2,.2,.3]:
        raise ValueError('unexpected baseline calibration')
    masks = np.ones(SHAPE,dtype=np.uint8)
    inputs, raw, durations = [], [], []
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        with (directory/'frames.csv').open('w',newline='') as frames, (
                directory/'candidates.csv').open('w',newline='') as candidates:
            frame_writer = csv.writer(frames)
            frame_writer.writerow(model.OBS)
            candidate_writer = csv.writer(candidates)
            candidate_writer.writerow(model.FROZEN)
            for i,(label,codes) in enumerate(scenes()):
                start = perf_counter()
                predictions = baseline.predict_frame(METADATA,codes,masks)
                durations.append(perf_counter()-start)
                inputs.append(label)
                raw.append(predictions)
                frame_writer.writerow((i,'holdout',*METADATA.values()))
                candidate_writer.writerows((i,*p) for p in predictions[:2])
        store = np.lib.format.open_memmap(directory/'masks.npy',mode='w+',dtype=np.uint8,
                                         shape=(len(inputs),*SHAPE))
        store[:] = masks
        del store
        diagnostics = dict.fromkeys(('1','2','3'),0)
        paired_n = 0
        rows = []
        policy_times = []
        for i,(_,_,pair) in enumerate(model.features(directory)):
            start = perf_counter()
            proposals = model.propose(pair,deployment['model'])
            requests = [(p[0],(*p[3:],0,''),old_rules[p[0]]) for p in raw[i]]
            requests += [(arm,proposals[arm],deployment['qualification'][arm]) for arm in ARMS[3:]]
            decisions = [model.qualified(p,rule) for _,p,rule in requests]
            policy_times.append(perf_counter()-start)
            contrast = threshold_diagnostic(pair)
            paired_n += bool(contrast)
            for key,value in contrast.items():
                diagnostics[key] += value
            cluster,size,dx,dy,control,feasible,x,y = inputs[i]
            for (arm,proposal,_),(status,_,_,probability,why) in zip(requests,decisions):
                correct = bool(feasible and proposal[0]!='' and math.dist(proposal[:2],(x,y))<=1)
                rows.append(dict(cluster=cluster,size=size,dx=dx,dy=dy,control=control,
                                 arm=arm,feasible=feasible,issued=status=='issue',
                                 correct=correct,reason=why,warning=proposal[5],
                                 probability=None if probability=='' else probability))
    with (output/'fusion_decisions.csv').open('w',newline='') as file:
        writer = csv.DictWriter(file,fieldnames=rows[0],lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    timing = {}
    for name,values in [('frontend',durations),('policy',policy_times)]:
        timing[name] = {'p50_p95_p99_ms':(1000*np.percentile(values,[50,95,99])).tolist(),
                        'calls':len(values),'sum_s':sum(values)}
    timings['fusion_latency'] = timing
    summary = {}
    offset_strata = {}
    group_deltas = {}
    size_results = {}
    for arm in ARMS:
        selection = [r for r in rows if r['arm']==arm]
        summary[arm] = summarize(selection)
        offset_strata[arm] = {f'{dx},{dy}/{control}':summarize([
            r for r in selection if (r['dx'],r['dy'],r['control'])==(dx,dy,control)])
            for dx,dy in sorted({(r['dx'],r['dy']) for r in selection})
            for control in ('clear','edge','partial','distractor')}
        size_results[arm] = {str(size):summarize([r for r in selection if r['size']==size])
                             for size in (2,4,6)}
        group_deltas[arm] = np.array([[sum(r['issued'] and r['correct'] for r in selection
                                          if r['cluster']==cluster),
                                       sum(r['feasible'] for r in selection if r['cluster']==cluster)]
                                      for cluster in range(48)])
    diff = group_deltas['A01'][:,0]-group_deltas['B03'][:,0]
    count = group_deltas['A01'][:,1]
    clusters = np.random.default_rng(110011).integers(0,48,(2000,48))
    draws = diff[clusters].sum(axis=1)/count[clusters].sum(axis=1)
    return {'frames':len(inputs),'independent_clusters':48,'summary':summary,
            'offset_control_strata':offset_strata,'size_strata':size_results,
            'A01_minus_B03_feasible_gain':float(diff.sum()/count.sum()),
            'paired_cluster_interval':np.percentile(draws,[2.5,97.5]).tolist(),
            'paired_candidates':paired_n,'conflict_threshold_diagnostic':diagnostics}
