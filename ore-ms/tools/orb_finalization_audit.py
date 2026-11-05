"""Independent native final-factor, official-map-export and metric audit."""
from collections import Counter,defaultdict
from copy import deepcopy
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import provided_reference_full_record_audit as metric

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-finalization/run-001'
PLAN=ROOT/'experiments/orb-finalization-2026-11-05.json'
COUNT=0
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(value,label):
    global COUNT
    COUNT+=1
    if not value:raise ValueError(label)

def factors(row,gauges):
    need(row['final_map_id'] in gauges,'all-map gauge identity')
    product=np.array(row['T_frame_from_reference'],dtype=float);magnitude=np.linalg.norm(product[:3,3])
    chain=row['reference_chain'];visited=set()
    for i,link in enumerate(chain):
        need(link['keyframe_id'] not in visited,'acyclic reference chain');visited.add(link['keyframe_id'])
        expected=chain[i+1]['keyframe_id'] if i+1<len(chain) else row['resolved_keyframe_id']
        need(link['parent_id']==expected,'culled-parent lineage')
        a=np.array(link['T_child_from_parent']);product=product@a;magnitude+=np.linalg.norm(a[:3,3])
    ref=np.array(row['T_reference_from_map']);gauge=np.array(gauges[row['final_map_id']]['T_map_from_origin'])
    need(np.array_equal(gauge,np.array(row['T_map_from_origin'])),'same frozen map gauge')
    result=np.linalg.inv(product@ref@gauge)
    magnitude+=np.linalg.norm(ref[:3,3])+np.linalg.norm(gauge[:3,3])
    need(np.allclose(result,row['T_world_from_camera'],rtol=1e-10,atol=1e-8),'independent final factor reconstruction')
    return result,float(magnitude),len(chain)+3

def export_audit(directory):
    p=read(directory/'plan.json');received=read(directory/'received.json');final=read(directory/'final.json')
    metadata=read(directory/'native.jsonl.maps.json');rows=[json.loads(s) for s in (directory/'native.jsonl.final.jsonl').read_text().splitlines()]
    need(metadata['local_mapper_finished'] is True and metadata['loop_closer_finished'] is True and metadata['gba_running'] is False,'workers quiescent')
    need(len(rows)==len(p['stems'])==len(final['poses']),'all final source slots')
    gauges={m['map_id']:m for m in metadata['maps']};need(len(gauges)==len(metadata['maps']),'unique map identities')
    official=defaultdict(list);official_rows=0
    for path in sorted(directory.glob('native.jsonl.map-*.euroc.txt')):
        mid=int(path.name.split('.map-')[1].split('.')[0]);need(mid in gauges,'official map declared')
        for line in path.read_text().splitlines():
            v=line.split();need(len(v)==8,'official pose columns')
            stamp=Decimal(v[0]);integer=int(stamp.to_integral_value())
            need(abs(stamp-integer)<=Decimal('.001'),'official elapsed-clock serialization, no timestamp search')
            matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat([float(x) for x in v[4:]]).as_matrix();matrix[:3,3]=[float(x) for x in v[1:4]]
            official[(mid,integer)].append(matrix);official_rows+=1
    compared=ambiguous=eligible=0;worst=0.;reasons=Counter();native_maprows=received['rows'] if 'rows' in received else read(directory/'estimate.json')['rows']
    for i,row in enumerate(rows):
        need(row['index']==i and row['source_id']==p['stems'][i] and row['time_ns']==p['time_ns'][i] and row['right_time_ns']==p['right_time_ns'][i],'exact native final axis')
        pose=row['T_world_from_camera']
        if pose is None:
            need(final['poses'][i] is None,'unavailable final stays null');reasons[row['reason']]+=1;continue
        need(received['poses'][i] is not None and row['received_valid'] is True,'no received-output resurrection')
        need(row['candidate_count']==1 and row['native_lost'] is False,'unique native retained history')
        need(row['final_map_id']==native_maprows[i]['map_id'],'no final map stitching')
        elapsed=p['time_ns'][i]-p['time_ns'][0]
        need(row['native_time']==elapsed*1e-9,'exact native source clock')
        expected,magnitude,links=factors(row,gauges);eligible+=1
        canonical=np.array(pose,dtype=float)
        need(np.allclose(canonical[3],[0,0,0,1],rtol=0,atol=1e-10),'native homogeneous serialization bound')
        canonical[3]=[0,0,0,1]
        if metric.rigid(canonical) is None:need(final['poses'][i] is None,'invalid final pose admission')
        else:
            need(np.allclose(final['poses'][i],pose,atol=1e-10,rtol=0),'native finalized values preserved')
        controls=official[(row['final_map_id'],elapsed)]
        if len(controls)!=1:ambiguous+=1;continue
        control=controls[0];dt=np.max(np.abs(expected[:3,3]-control[:3,3]));dr=np.max(np.abs(expected[:3,:3]-control[:3,:3]));worst=max(worst,float(dt))
        # Prior fixed float32 operation/coordinate-scale allowance, not a metric tolerance.
        need(dt<=5e-5+5e-6*(1+magnitude) and dr<=5e-6*links,'official float per-map export equivalence')
        compared+=1
    need(compared>0 or eligible==0,'official compatibility controls present')
    return dict(source_frames=len(rows),final_native_eligible=eligible,official_rows=official_rows,official_unambiguous_compared=compared,
                official_ambiguous_or_absent=ambiguous,max_official_translation_component_difference_m=worst,
                final_unavailable_reasons=dict(reasons),surviving_map_ids=sorted(gauges))

def audit():
    p=read(PLAN);lock=read(OUT/'prediction-lock.json');need(sha(PLAN)==sha(OUT/'study-lock.json')==lock['study_sha256'],'study/prediction locks')
    for n,h in {**p['source_sha256'],**lock['source_sha256']}.items():need(sha(ROOT/n)==h,'source/input/output pin '+n)
    summary=read(OUT/'summary.json');reports=[];mutations=0
    source=read(OUT/'source-manifest.json');need(len(source['records'])==p['source_groups']==2000,'fixed engineering prefix')
    refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
    for mode in ['rgb','thermal']:
        directory=OUT/'captures'/(mode+'-paired');report=export_audit(directory);report['modality']=mode
        received=read(directory/'received.json');final=read(directory/'final.json');droid=read(OUT/'captures'/(mode+'-droid')/'estimate.json')
        native=[json.loads(s) for s in (directory/'native.jsonl').read_text().splitlines()];original=[r for r in native if 'index' in r]
        need(len(original)==2000,'paired native full prefix')
        for i,r in enumerate(original):
            need(r['index']==i and r['time_ns']==received['time_ns'][i],'received native axis')
            if r['T_camera_from_world'] is None:need(received['poses'][i] is None,'received missing')
            else:
                t=np.array(r['T_camera_from_world']);expected=np.eye(4);expected[:3,:3]=t[:3,:3].T;expected[:3,3]=-t[:3,:3].T@t[:3,3]
                need(np.allclose(expected,np.asarray(received['poses'][i]),atol=1e-10,rtol=0),'original rigid-transpose received convention')
        mask=[all(e['poses'][i] is not None for e in [received,final,droid]) for i in range(2000)]
        expected_segments=[];sid=-1;active=False;last=None
        for ok,segment in zip(mask,received['segments']):
            if not ok:expected_segments.append(None);active=False;continue
            if not active or segment!=last:sid+=1
            expected_segments.append(sid);active=True;last=segment
        raw=[];modality='rgb' if mode=='rgb' else 'thr'
        for row in source['records']:
            path=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/modality/(row['stem']+'.txt')
            need(sha(path)==refs['member_sha256'][str(path.relative_to(ROOT))],'raw reference identity');raw.append(np.loadtxt(path).reshape(4,4))
        for name,individual in [('received',received),('final',final),('droid',droid)]:
            view=read(OUT/'scores'/mode/(name+'-view.json'));score=read(OUT/'scores'/mode/(name+'.json'))
            need(view['time_ns']==received['time_ns'] and view['segments']==expected_segments,'common exact source/segment axis')
            for i,ok in enumerate(mask):need(view['poses'][i]==individual['poses'][i] if ok else view['poses'][i] is None,'common support, no pose edits')
            want=metric.independently_derive(view['time_ns'],raw,view);metric.compare(score,want)
            entry=next(r for r in summary['runs'] if r['modality']==mode)['results'][name]
            for k,v in want.items():
                if k not in ['rpe_rows','segment_ate','excluded_indices']:metric.close(entry[k],v,'summary '+k)
            for key,val in [('pose_coverage',score['pose_coverage']+.1),('scale_fitted',True),('rpe_expected_pairs',score['rpe_expected_pairs']+1)]:
                bad=deepcopy(score);bad[key]=val
                try:metric.compare(bad,want)
                except ValueError:mutations+=1
                else:raise ValueError('undetected score corruption')
        report['common_pose_frames']=sum(mask);reports.append(report);print('INDEPENDENTLY AUDITED',mode,flush=True)
    result=dict(passed=True,independent_checks=COUNT+metric.COUNT,mutations_rejected=mutations,runs=reports,
                auditor_sha256=sha(__file__),plan_sha256=sha(PLAN),physical_sync_verified=False,benchmark_execution_ready=False)
    with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':audit()
