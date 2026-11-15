"""Continue the frozen three-arm candidate from its own saved source599 states."""
from pathlib import Path
import datetime
import sys
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import component_constrained_native as component

feedback=component.feedback
native=component.native
ROOT=native.ROOT
OUT=ROOT/'results/fixed-rotation-continuation'
RGB=ROOT/'results/native-pose-feedback/run-001'
FIXED=ROOT/'results/component-constrained-native/run-001'
METHODS=['native_rgb','rgb_refine','thermal_refine']
SEQUENCES={'day':'_2021-08-13-16-31-10','night':'_2021-08-13-22-16-02'}


def prefix(condition):
    rgb_path=RGB/f'predictions-{condition}.json';fixed_path=FIXED/f'predictions-{condition}.json'
    for folder,path in [(RGB,rgb_path),(FIXED,fixed_path)]:
        lock=native.read(folder/'prediction-lock.json')
        pin=next(x for x in lock['rows'] if x['condition']==condition)
        assert native.sha(path)==pin['prediction_sha256']
    a=native.read(rgb_path)['rows'];b=native.read(fixed_path)['rows']
    assert len(a)==len(b)==599
    return [{'pair':i,'methods':{'native_rgb':a[i]['methods']['rgb_feedback'],
             'rgb_refine':b[i]['methods']['rgb_fixed_rotation'],
             'thermal_refine':b[i]['methods']['thermal_fixed_rotation']},
             'binding':b[i]['binding']} for i in range(599)]


def check_implementation():
    parent=native.read(FIXED/'freeze.json')
    for p,k in [(component.__file__,'source_sha256'),(feedback.__file__,'feedback_sha256'),
                (native.__file__,'native_extension_sha256')]:assert native.sha(p)==parent[k]


def capture():
    torch.set_num_threads(1);check_implementation()
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    parent=native.read(FIXED/'freeze.json')
    for condition in SEQUENCES:
        for p,digest in parent['input_pins'][condition].items():assert native.sha(ROOT/p)==digest
    freeze={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':native.sha(__file__),
        'implementation_pins':{str(Path(p).relative_to(ROOT)):native.sha(p) for p in
            [component.__file__,feedback.__file__,native.__file__]},
        'input_pins':{c:parent['input_pins'][c] for c in SEQUENCES},
        'prefix_pins':{str(p.relative_to(ROOT)):native.sha(p) for c in SEQUENCES for p in
            [RGB/f'predictions-{c}.json',FIXED/f'predictions-{c}.json']},
        'reference_recovery_proof_sha256':native.sha(OUT/'recovery/proof.json'),
        'sequences':SEQUENCES,'methods':METHODS,'new_pairs':[599,1198],'initial_state_source':599,
        'primary_source_span':[599,1199],'secondary_full_groups':1200,
        'block_starts':list(range(600,1200,100)),'thermal_precision':.1,'new_references_read':False}
    native.write(target/'freeze.json',freeze);pins=[]
    for c in SEQUENCES:
        source,E,saved,events=native.load(c);assert source['sequence']==SEQUENCES[c]
        for camera,sensor in [('rgb','rgb'),('thermal','thr')]:
            assert len(saved[camera]['frames//time_ns'])>=1200
            assert saved[camera]['frames//time_ns'].reshape(-1)[:1200].tolist()==[
                r['cameras'][sensor]['left']['recorded_time_ns'] for r in source['records'][:1200]]
        rows=prefix(c);states={m:torch.tensor(rows[-1]['methods'][m]['committed_xyzw'],dtype=torch.float32) for m in METHODS}
        reproduction=[]
        for pair in range(599,1199):
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
                    reproduction.append({'pair':pair,'translation_m':float(np.linalg.norm(error[:3,3])),
                                         'rotation_rad':float(Rotation.from_matrix(error[:3,:3]).magnitude())})
                except (RuntimeError,ValueError,np.linalg.LinAlgError) as error:
                    available=False;binding['native_failure']=str(error)
            if not available:
                for m in METHODS:methods[m]={'status':'UNAVAILABLE_RGB_STATE_HELD','transform':None,
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
            if (pair-599)%100==0:print('CONTINUATION',c,pair,flush=True)
        path=target/f'predictions-{c}.json'
        native.write(path,{'sequence':source['sequence'],'rows':rows,'new_native_reproduction':reproduction,
                          'new_references_read':False})
        pins.append({'condition':c,'prediction_sha256':native.sha(path)})
        del saved
    native.write(target/'prediction-lock.json',{'rows':pins,'freeze_sha256':native.sha(target/'freeze.json'),
                                               'new_references_read':False})


def score():
    target=OUT/'run-001';lock=native.read(target/'prediction-lock.json');summary={}
    for entry in lock['rows']:
        c=entry['condition'];p=target/f'predictions-{c}.json';assert native.sha(p)==entry['prediction_sha256']
        payload=native.read(p);rows=payload['rows']
        assert all(x['translation_m']<1e-3 and x['rotation_rad']<1e-4 for x in payload['new_native_reproduction']), 'Native reproduction bound exceeded'
        refs=[];invalid={}
        for i in range(1200):
            try:refs.append(feedback.rigid(np.loadtxt(OUT/'recovery/references'/c/f'{i:06}.txt').reshape(4,4))[0])
            except ValueError as error:refs.append(None);invalid[i]=str(error)
        methods={}
        for method in METHODS:
            def stats(start,end):
                bad=[i for i in invalid if start<=i<=end]
                if bad:return {'status':'UNQUALIFIED_REFERENCE','reference_source_ids':bad}
                result=native.path_stats(rows,refs,method,start,end)
                if result['status']=='COMPLETE':result.pop('positions');result.pop('reference')
                return result
            methods[method]={'available_new':sum(r['methods'][method]['transform'] is not None for r in rows[599:]),
                'new_span':stats(599,1199),'full_path':stats(0,1199),
                'new_blocks':[{'start':i,**stats(i,i+99)} for i in range(600,1200,100)]}
        source=native.read(native.REC/'inputs'/c/'source-manifest.json')['records']
        duration=(source[1199]['cameras']['rgb']['left']['recorded_time_ns']-source[599]['cameras']['rgb']['left']['recorded_time_ns'])/1e9
        summary[c]={'sequence':SEQUENCES[c],'new_span_recorded_rgb_seconds':duration,
                    'invalid_references':invalid,'methods':methods}
        print('SCORED',c,{m:v['new_span'] for m,v in methods.items()},flush=True)
    native.write(target/'scores.json',{'summary':summary,'prediction_lock_sha256':native.sha(target/'prediction-lock.json'),
        'scope':'New600interval continuation of originalcandidate recordings; previous prefix reused exactly, not longer short-transfer captures.'})


if __name__=='__main__':{'capture':capture,'score':score}[sys.argv[1]]()
