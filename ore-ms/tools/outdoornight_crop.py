"""Unchanged crop/count geometry transfer; separate inherited nominal gyro tier.

Composes frozen native/subset/pose, policy and gyro-evaluation helpers. Adds only
OutdoorNight preflight/parity, variant binding, origin receipts and paired changes.
"""
import argparse,json
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from pre_essential_exclusion import kept_indices,image_masks,index_sha,fit,exact_pose,POSE_FIELDS
from matched_count_control import count_indices
from subt_export_comparison import Comparison
from camera_priority_controls import choices,evaluate,POLICIES
from essential_inlier_mixture import native
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1];VARIANTS=['original','bottom','count']
def lines(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def cohorts(pop,n):
    common=set(pop['common_gyro']);return {'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}}
def paired(pred,selected,errors,pop,n):
    dm={(v,d['frontend'],d['index']):d for v,ds in selected.items() for d in ds};em={(r['variant'],r['frontend'],r['arm'],r['index']):r['disagreement_deg'] for r in errors};result={};common=set(pop['common_gyro'])
    for before,after in [('original','count'),('original','bottom'),('count','bottom')]:
        label=before+'_to_'+after;result[label]={}
        for f in ['sift','learned']:
            result[label][f]={}
            for group,ids in cohorts(pop,n).items():
                result[label][f][group]={}
                for policy in POLICIES:
                    def get(v,i):
                        a=dm[v,f,i]['policies'][policy]['sensor'];return a,em[v,f,a,i] if a else None
                    pairs=[(get(before,i),get(after,i),i) for i in ids];x={'anchors':len(ids),'gained_outputs':sum(a[0] is None and b[0] is not None for a,b,i in pairs),'lost_outputs':sum(a[0] is not None and b[0] is None for a,b,i in pairs)}
                    for t in [1,5]:
                        bits=[(a[1] is not None and a[1]<=t,b[1] is not None and b[1]<=t) for a,b,i in pairs if i in common];x[str(t)]={'eligible':len(bits),'before_agrees':sum(a for a,b in bits),'after_agrees':sum(b for a,b in bits),'gained':sum(not a and b for a,b in bits),'lost':sum(a and not b for a,b in bits)}
                    result[label][f][group][policy]=x
    return result
class Transfer:
    def __init__(self,config):
        self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output'];self.parent=ROOT/self.cfg['matching'];self.gyro=ROOT/self.cfg['gyro'];self.model=Comparison(ROOT/self.cfg['comparison_config'])
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def subsets(self,p):
        a=p['arm'];c=self.model.cameras[a+'_camera'];h,w=c['image_height'],c['image_width'];bottom=kept_indices(p,self.model.camera(a),image_masks([np.zeros((h,w),np.uint8)])[1]);return {'original':np.arange(p.get('matches',0),dtype=np.int64),'bottom':bottom,'count':count_indices(p.get('matches',0),len(bottom),'OutdoorNight',p['frontend'],a,p['index'])}
    def freeze(self):
        assert read(self.parent/'audit.json')['status']==read(self.gyro/'audit.json')['status']=='pass';assert sha(self.parent/'predictions.jsonl')==read(self.parent/'prediction-lock.json')['predictions_sha256'];old=read(self.gyro/'target-lock.json');files=[self.config,Path(__file__),ROOT/self.cfg['protocol'],ROOT/self.cfg['comparison_config']];files.extend(ROOT/x for x in ['tools/pre_essential_exclusion.py','tools/matched_count_control.py','tools/essential_inlier_mixture.py','tools/deployment_spatial_support.py','tools/smokeseer_sift.py','tools/subt_export_comparison.py','tools/camera_priority_controls.py','tools/smoke_rgb_first_fallback.py','tools/subt_gyro_consistency.py','tools/audit_subt_gyro_consistency.py','tools/audit_subt_export.py','tools/deployment_lighting.py','tools/audit_outdoornight_crop.py','tools/test_outdoornight_crop.py']);files.append(ROOT/self.model.cfg['intrinsics']);files.extend(self.parent/x for x in ['input-freeze.json','execution-freeze.json','predictions.jsonl','prediction-lock.json','matcher-inputs.json','decisions.json','decision-lock.json','audit.json']);files.extend(self.gyro/x for x in ['target-lock.json','gyro-labels.json','population.json','disagreements.json','summary.json','completion.json','audit.json']);qual=self.model.parent;files.extend(qual/x for x in ['images.json','calibration.json','export-imu.npz','audit.json']);priority=ROOT/'results/camera-priority-controls/run-001';files.extend(priority/x for x in ['summary.json','completion.json','audit.json']);pins={str(p.relative_to(ROOT)):sha(p) for p in files}
        for p in [self.gyro/'gyro-labels.json',self.gyro/'population.json',qual/'calibration.json',qual/'export-imu.npz']:assert pins[str(p.relative_to(ROOT))]==old['pins'][str(p.relative_to(ROOT))]
        images={r['path']:r for r in read(qual/'images.json')};inputs=read(self.parent/'matcher-inputs.json');shapes={};roundtrip={}
        for row in inputs:
            a=row['arm'];c=self.model.cameras[a+'_camera'];shape=(c['image_height'],c['image_width'])
            for key in ['source','target']:
                name=row[key]
                if name not in shapes:
                    p=ROOT/name;digest=sha(p);assert digest==images[name]['sha256'];im=np.load(p,allow_pickle=False);assert im.shape==shape and im.dtype==np.uint8;shapes[name]=list(im.shape);pins[name]=digest
        for a in ['rgb','thermal']:
            c=self.model.cameras[a+'_camera'];h,w=c['image_height'],c['image_width'];grid=np.array([[0,0],[w-1,0],[0,h-1],[w-1,h-1],[w/2,h/2]]);error=float(np.linalg.norm(native(self.model.lift(grid,a),self.model.camera(a))-grid,axis=1).max());assert error<=.05;roundtrip[a]=error
        raw=lines(self.parent/'predictions.jsonl');n=read(self.parent/'input-freeze.json')['anchors'];assert len(raw)==len({(p['frontend'],p['arm'],p['index']) for p in raw})==4*n==3884;plan=[]
        for p in raw:
            subsets=self.subsets(p)
            for v,ids in subsets.items():plan.append({'variant':v,'frontend':p['frontend'],'arm':p['arm'],'index':p['index'],'qualified':p['qualified'],'input_matches':p.get('matches',0),'kept':len(ids),'kept_sha256':index_sha(ids),'fit_eligible':bool(p['qualified'] and len(ids)>=8)})
        assert len(plan)==11652;self.out.mkdir(parents=True,exist_ok=False);write(self.out/'preflight.json',plan);pins[str((self.out/'preflight.json').relative_to(ROOT))]=sha(self.out/'preflight.json');write(self.out/'input-freeze.json',{'pins':pins,'anchors':n,'expected_rows':len(plan),'image_shapes':shapes,'roundtrip_max_px':roundtrip,'runtime_image':self.cfg['runtime_image'],'scope':'Full native/subset/support preflight; inherited gyro tier, no new fits/errors yet'});print('All transfer rows preflighted',len(plan),flush=True)
    def baseline(self):
        self.verify();assert np.__version__=='2.2.6' and cv2.__version__=='5.0.0';cv2.setNumThreads(1);checks=calls=rows=0
        for p in lines(self.parent/'predictions.jsonl'):
            ids=np.arange(p.get('matches',0),dtype=np.int64);result,call,_=fit(p,self.model.camera(p['arm'])[3],ids);checks+=exact_pose(p,result);calls+=call;rows+=1
        self.verify();write(self.out/'baseline-parity.json',{'status':'pass','rows':rows,'pose_field_checks':checks,'solver_calls':calls,'numpy':np.__version__,'opencv':cv2.__version__});print('Exact baseline parity',rows,checks,flush=True)
    def run(self):
        self.verify();assert read(self.out/'baseline-parity.json')['status']=='pass';assert np.__version__=='2.2.6' and cv2.__version__=='5.0.0';cv2.setNumThreads(1);plan={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in read(self.out/'preflight.json')};rows=calls=reuses=0
        with (self.out/'predictions.jsonl').open('x') as f:
            for p in lines(self.parent/'predictions.jsonl'):
                subsets=self.subsets(p);cache={}
                for v in VARIANTS:
                    ids=subsets[v];digest=index_sha(ids);assert digest==plan[v,p['frontend'],p['arm'],p['index']]['kept_sha256'];prev=next((q for q in cache if index_sha(subsets[q])==digest),None)
                    if v=='original':result={k:p[k] for k in POSE_FIELDS if k in p};call=False;seconds=0.;origin='parent'
                    elif prev is not None:result=cache[prev];call=False;seconds=0.;origin='cache';reuses+=1
                    else:result,call,seconds=fit(p,self.model.camera(p['arm'])[3],ids);origin='fresh';calls+=call
                    cache[v]=result;r={k:p[k] for k in ['frontend','arm','index','qualified','seconds']};r.update(result,variant=v,origin=origin,input_matches=p.get('matches',0),kept_sha256=digest,solver_called=call,reuse_variant=prev,geometry_seconds=seconds);f.write(json.dumps(r,allow_nan=False)+'\n');rows+=1
                if rows%1500==0:print('Transfer rows',rows,flush=True)
        assert rows==11652;self.verify();write(self.out/'prediction-lock.json',{'rows':rows,'predictions_sha256':sha(self.out/'predictions.jsonl'),'inherited_rows':3884,'new_solver_calls':calls,'new_reused_rows':reuses,'new_inference_calls':0,'gyro_disagreements_computed':False,'baseline_parity_sha256':sha(self.out/'baseline-parity.json')});print('Transfer predictions locked',rows,flush=True)
    def select(self):
        self.verify();assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'];raw=lines(self.out/'predictions.jsonl');inputs=read(self.parent/'matcher-inputs.json');ds=[]
        for v in VARIANTS:ds.extend(dict(d,variant=v) for d in choices([p for p in raw if p['variant']==v],inputs,971))
        write(self.out/'choices.json',ds);write(self.out/'choice-lock.json',{'rows':len(ds),'choices_sha256':sha(self.out/'choices.json'),'gyro_errors_used':False});print('Status choices locked',len(ds),flush=True)
    def score(self):
        self.verify();assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'] and sha(self.out/'choices.json')==read(self.out/'choice-lock.json')['choices_sha256'];raw=lines(self.out/'predictions.jsonl');selected=read(self.out/'choices.json');refs={(p['arm'],p['index']):p for p in read(self.gyro/'gyro-labels.json')};cal=read(self.model.parent/'calibration.json');pop=read(self.gyro/'population.json');errors=[]
        for p in raw:
            ref=refs[p['arm'],p['index']];e=None
            if p['status']=='ok' and ref['supported']:
                rc=np.asarray(cal[p['arm']]['effective_camera_to_imu'])[:3,:3];delta=rc@np.asarray(p['R'])@rc.T@np.asarray(ref['target_from_source_imu']).T;e=float(np.degrees(Rotation.from_matrix(delta).magnitude()))
            errors.append({k:p[k] for k in ['variant','frontend','arm','index','status']}|{'input_supported':p['qualified'],'gyro_supported':ref['supported'],'disagreement_deg':e})
        summary={};control={};by={v:[d for d in selected if d['variant']==v] for v in VARIANTS};em={(r['variant'],r['frontend'],r['arm'],r['index']):r['disagreement_deg'] for r in errors};common=set(pop['common_gyro'])
        for v in VARIANTS:
            summary[v]=evaluate([p for p in raw if p['variant']==v],by[v],[p for p in errors if p['variant']==v],pop,971);control[v]={}
            for f in ['sift','learned']:
                control[v][f]={}
                for group,ids in cohorts(pop,971).items():
                    eligible=ids&common;ds=[d for d in by[v] if d['frontend']==f and d['index'] in ids];added=[d for d in ds if d['policies']['rgb_first']['sensor']=='thermal'];valid=[d for d in added if d['index'] in eligible];ee=[em[v,f,'thermal',d['index']] for d in valid];zero=[refs['thermal',d['index']]['rotation_deg'] for d in valid];control[v][f][group]={'zero_rotation':{a:{str(t):sum(refs[a,i]['rotation_deg']<=t for i in eligible) for t in [1,5]} for a in ['rgb','thermal']},'added_thermal':{'outputs':len(added),'gyro_eligible':len(valid),**{'within'+str(t)+'deg':sum(e<=t for e in ee) for t in [1,5]},**{'zero_within'+str(t)+'deg':sum(e<=t for e in zero) for t in [1,5]}}}
        assert summary['original']==read(ROOT/'results/camera-priority-controls/run-001/summary.json');changes=paired(raw,by,errors,pop,971)
        for name,data in [('disagreements',errors),('summary',summary),('controls',control),('transitions',changes)]:write(self.out/(name+'.json'),data)
        self.verify();write(self.out/'completion.json',{**{name+'_sha256':sha(self.out/(name+'.json')) for name in ['disagreements','summary','controls','transitions']},'scope':'Separate nominal gyro tier; no translation or independently certified physical truth'});print('Transfer gyro scores locked',len(errors),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','baseline','run','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Transfer(a.config),a.stage)()
