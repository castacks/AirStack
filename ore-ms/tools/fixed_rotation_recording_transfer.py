"""Unchanged fixed-R native translation baseline on two different recordings."""
from pathlib import Path
import sys
import datetime
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import component_constrained_native as component

feedback=component.feedback
native=component.native
ROOT=native.ROOT
OUT=ROOT/'results/fixed-rotation-transfer'
REC=OUT/'recovery'
METHODS=['native_rgb','rgb_refine','thermal_refine']
SEQUENCES={'day':'_2021-08-06-11-23-45','night':'_2021-08-13-21-18-04'}


def load_at(condition):
    original=native.REC
    try:
        native.REC=REC
        return native.load(condition)
    finally:
        native.REC=original


def check_implementation():
    path=ROOT/'results/component-constrained-native/run-001/freeze.json'
    freeze=native.read(path)
    assert native.sha(component.__file__)==freeze['source_sha256']
    assert native.sha(feedback.__file__)==freeze['feedback_sha256']
    assert native.sha(native.__file__)==freeze['native_extension_sha256']
    return path


def predict(condition,count):
    source,E,saved,events=load_at(condition)
    assert source['sequence']==SEQUENCES[condition]
    for camera,sensor in [('rgb','rgb'),('thermal','thr')]:
        assert len(saved[camera]['frames//time_ns'])==100
        assert saved[camera]['frames//time_ns'].reshape(-1).tolist()==[
            r['cameras'][sensor]['left']['recorded_time_ns'] for r in source['records'][:100]]
    states={m:torch.tensor([0,0,0,0,0,0,1],dtype=torch.float32) for m in METHODS}
    rows=[];reproductions=[]
    for pair in range(count):
        event=events['rgb'].get((pair,pair+1));methods={};binding={}
        available=event is not None and not event['original_need_interp'] and event['derived_camera_delta_xyzw'] is not None
        if available:
            try:
                packet,meta=native.bind_camera(saved['thermal'],events['thermal'].get((pair,pair+1)),pair,
                                              source['records'],'thr',E['thermal']);binding={'thermal':meta}
            except ValueError as error:packet=None;binding={'thermal_unavailable':str(error)}
            try:
                states['native_rgb'],backbone=feedback.step(saved['rgb'],event,pair,source['records'],states['native_rgb'])
                methods['native_rgb']=backbone
                basis=np.eye(4);basis[:3,:3]=native.O
                expected=basis@native.pose(event['derived_camera_delta_xyzw'])@basis.T
                error=np.linalg.solve(expected,np.asarray(backbone['transform']))
                reproductions.append({'pair':pair,'translation_m':float(np.linalg.norm(error[:3,3])),
                                      'rotation_rad':float(Rotation.from_matrix(error[:3,:3]).magnitude())})
            except (RuntimeError,ValueError,np.linalg.LinAlgError) as error:
                available=False;binding['native_failure']=str(error)
        if not available:
            for m in METHODS:
                methods[m]={'status':'UNAVAILABLE_RGB_STATE_HELD','transform':None,
                            'parent_xyzw':states[m].tolist(),'committed_xyzw':states[m].tolist()}
        else:
            for m in ['rgb_refine','thermal_refine']:
                try:
                    states[m],methods[m]=component.step(saved['rgb'],event,pair,source,states[m],backbone,
                                                        packet if m=='thermal_refine' else None)
                except (RuntimeError,ValueError,np.linalg.LinAlgError) as error:
                    methods[m]={'status':'FIT_FAILED_STATE_HELD','transform':None,'reason':str(error),
                                'parent_xyzw':states[m].tolist(),'committed_xyzw':states[m].tolist()}
        rows.append({'pair':pair,'methods':methods,'binding':binding})
        if pair%25==0:print('TRANSFER',condition,pair,flush=True)
    return source,rows,reproductions


def qualify():
    torch.set_num_threads(1);check_implementation()
    target=OUT/'qualification-001';target.mkdir(parents=True,exist_ok=False);checks=[]
    for c in SEQUENCES:
        _,rows,reproduction=predict(c,3)
        for row in rows:
            backbone=row['methods']['native_rgb']
            for m in ['rgb_refine','thermal_refine']:
                assert row['methods'][m]['transform'] is not None
                assert row['methods'][m]['committed_xyzw'][3:]==backbone['committed_xyzw'][3:]
        checks.append({'condition':c,'reproduction':reproduction,'rotation_preserved':True})
    passed=all(x['translation_m']<1e-3 and x['rotation_rad']<1e-4
               for c in checks for x in c['reproduction'])
    native.write(target/'checks.json',{'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_sha256':native.sha(__file__),'checks':checks,'passed':passed})
    print('QUALIFIED',passed,checks,flush=True);assert passed


def capture():
    torch.set_num_threads(1);parent=check_implementation()
    qualification=native.read(OUT/'qualification-001/checks.json')
    assert qualification['passed'] and qualification['source_sha256']==native.sha(__file__)
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    paths=sorted(p for p in REC.rglob('*') if p.is_file() and '/references/' not in str(p))
    freeze={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':native.sha(__file__),
            'implementation_pins':{str(Path(p).relative_to(ROOT)):native.sha(p) for p in
                [component.__file__,feedback.__file__,native.__file__]},
            'parent_freeze_sha256':native.sha(parent),'qualification_sha256':native.sha(OUT/'qualification-001/checks.json'),
            'input_pins':{str(p.relative_to(ROOT)):native.sha(p) for p in paths},'sequences':SEQUENCES,
            'groups':100,'methods':METHODS,'thermal_precision':.1,'new_references_read':False,
            'scope':'Different historically exposed recordings; no tuning, not blind validation.'}
    native.write(target/'freeze.json',freeze);pins=[]
    for c in SEQUENCES:
        source,rows,reproductions=predict(c,99);path=target/f'predictions-{c}.json'
        native.write(path,{'sequence':source['sequence'],'rows':rows,'native_reproduction':reproductions,
                          'new_references_read':False})
        pins.append({'condition':c,'prediction_sha256':native.sha(path)})
    native.write(target/'prediction-lock.json',{'rows':pins,'freeze_sha256':native.sha(target/'freeze.json'),
                                               'new_references_read':False})


def score():
    target=OUT/'run-001';lock=native.read(target/'prediction-lock.json');summary={}
    for entry in lock['rows']:
        c=entry['condition'];path=target/f'predictions-{c}.json';assert native.sha(path)==entry['prediction_sha256']
        rows=native.read(path)['rows'];refs=[feedback.rigid(np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4))[0]
                                           for i in range(100)];methods={}
        for method in METHODS:
            full=native.path_stats(rows,refs,method,0,99)
            if full['status']=='COMPLETE':full.pop('positions');full.pop('reference')
            errors=[]
            for row in rows:
                value=row['methods'][method]['transform']
                if value is None:continue
                i=row['pair'];reference=np.linalg.solve(refs[i],refs[i+1]);difference=np.linalg.solve(reference,np.asarray(value))
                errors.append({'pair':i,'translation_m':float(np.linalg.norm(difference[:3,3])),
                               'rotation_deg':float(np.rad2deg(Rotation.from_matrix(difference[:3,:3]).magnitude()))})
            def stats(key):
                values=np.array([x[key] for x in errors])
                return {'mean':float(np.mean(values)),'rmse':float(np.sqrt(np.mean(values**2))),
                        'p95':float(np.percentile(values,95))} if len(values) else None
            methods[method]={'available':len(errors),'full_path':full,'increment_errors':errors,
                             'increment_translation_m':stats('translation_m'),'increment_rotation_deg':stats('rotation_deg')}
        summary[c]={'sequence':SEQUENCES[c],'methods':methods};print('SCORED',c,{m:v['full_path'] for m,v in methods.items()},flush=True)
    native.write(target/'scores.json',{'summary':summary,'prediction_lock_sha256':native.sha(target/'prediction-lock.json'),
                                      'scope':'Unchanged method, cross-recording exposed100group replay; nominal references.'})


if __name__=='__main__':{'qualify':qualify,'capture':capture,'score':score}[sys.argv[1]]()
