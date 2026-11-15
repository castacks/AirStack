"""Carry each arm's committed native pose through fixed-measurement replay."""
from pathlib import Path
import datetime
import json
import sys
import numpy as np
import torch
import pypose as pp
from scipy.spatial.transform import Rotation
import native_thermal_constraints as native
from Utility.Point import pixel2point_NED
from selected_motion_trajectory import rigid

ROOT = native.ROOT
OUT = ROOT/'results/native-pose-feedback'
METHODS = ['rgb_feedback', 'thermal_feedback']


def feedback_input(saved, event, pair, records, state):
    """Rebuild the exact native point-registration operations at this arm's pose."""
    data = native.rgb_input(saved, event, pair, records)
    observations = data.observations.data
    k0 = torch.from_numpy(saved['frames//K'][pair].copy())
    local = pixel2point_NED(observations['pixel1_uv'], observations['pixel1_d'].squeeze(-1), k0)
    parent = pp.SE3(state.clone())
    data.init_motion = pp.SE3(state.clone()[None])
    data.points.data['pos_Tw'] = pp.SE3_type.Act(parent, local)[..., :3]
    rotation = parent.rotation().matrix().double()
    data.points.data['cov_Tw'] = rotation @ observations['obs1_covTc'] @ rotation.T
    return data


def step(saved, event, pair, records, state, packet=None):
    before = state.clone()
    data = feedback_input(saved, event, pair, records, before)
    if packet is None:
        graph = native.Analytic_ReprojDisp_TwoFramePGO(data)
    else:
        graph = native.NativeThermalGraph(data, packet, native.pose(before.numpy()))
    _, losses = native.optimize(graph)
    # Native write_graph_data commits float32, without a new normalization policy.
    after = graph.pose2opt.detach().tensor()[0].float().clone()
    assert torch.isfinite(after).all()
    relative = np.linalg.solve(native.pose(before.numpy()), native.pose(after.numpy()))
    basis = np.eye(4); basis[:3,:3] = native.O
    return after, {'status': 'RGB_FEEDBACK' if packet is None else 'THERMAL_FEEDBACK',
                    'transform': (basis@relative@basis.T).tolist(), 'losses': losses,
                    'parent_xyzw': before.tolist(), 'committed_xyzw': after.tolist()}


def qualify():
    torch.set_num_threads(1)
    target = OUT/'qualification-002'
    target.mkdir(parents=True, exist_ok=False)
    records = []
    for condition in ['day', 'night', 'rain']:
        source, _, saved, events = native.load(condition)
        state = torch.tensor([0,0,0,0,0,0,1],dtype=torch.float32)
        for pair in range(20):
            event = events['rgb'][(pair,pair+1)]
            historical = torch.tensor(event['parent_pose_camera_map_xyzw'],dtype=torch.float32)
            data = feedback_input(saved['rgb'], event, pair, source['records'], historical)
            archived = native.rgb_input(saved['rgb'],event,pair,source['records'])
            point_error = float(torch.max(torch.abs(data.points.data['pos_Tw']-archived.points.data['pos_Tw'])))
            # A metre-only threshold is inappropriate for float32 points tens
            # of kilometres away. Use a conservative standard gamma_n rounding
            # budget for fewer than64 scalar backprojection/rigid-action ops.
            local = pixel2point_NED(data.observations.data['pixel1_uv'],
                data.observations.data['pixel1_d'].squeeze(-1),
                torch.from_numpy(saved['rgb']['frames//K'][pair].copy()))
            epsilon = np.finfo(np.float32).eps
            gamma64 = 64*epsilon/(1-64*epsilon)
            scale = 1 + 3*local.abs().amax(dim=-1).double() + historical[:3].abs().max().double()
            budget = gamma64*scale
            source_errors = (data.points.data['pos_Tw']-archived.points.data['pos_Tw']).abs().amax(dim=-1).double()
            roundoff_pass = bool(torch.all(source_errors <= budget))
            covariance_error = float(torch.max(torch.abs(data.points.data['cov_Tw']-archived.points.data['cov_Tw'])))
            previous = state.clone()
            state, result = step(saved['rgb'],event,pair,source['records'],state)
            assert result['parent_xyzw'] == previous.tolist()
            expected = native.pose(event['derived_camera_delta_xyzw'])
            basis = np.eye(4);basis[:3,:3] = native.O
            actual = basis.T@np.asarray(result['transform'])@basis
            difference = np.linalg.solve(expected,actual)
            t = float(np.linalg.norm(difference[:3,3]))
            r = float(Rotation.from_matrix(difference[:3,:3]).magnitude())
            records.append({'condition':condition,'pair':pair,'historical_source_point_max_error_m':point_error,
                            'source_covariance_max_error':covariance_error,
                            'source_roundoff_bound_pass':roundoff_pass,
                            'max_source_error_fraction_of_roundoff_bound':float(torch.max(source_errors/budget)),
                            'relative_translation_error_m':t,'relative_rotation_error_rad':r,
                            'state_parent_matches_previous_commit':True})
        del saved
    passed = all(x['relative_translation_error_m']<1e-3 and x['relative_rotation_error_rad']<1e-4
                 and x['source_roundoff_bound_pass'] and x['source_covariance_max_error']==0 for x in records)
    native.write(target/'checks.json',{'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_sha256':native.sha(__file__),'native_extension_sha256':native.sha(native.__file__),
        'records':records,'passed':passed})
    print('QUALIFICATION',passed,'max source m',max(x['historical_source_point_max_error_m'] for x in records),
          'max relative m',max(x['relative_translation_error_m'] for x in records),flush=True)
    assert passed, 'State reconstruction qualification failed; do not score'


def capture():
    torch.set_num_threads(1)
    qualification = native.read(OUT/'qualification-002/checks.json')
    assert qualification['passed'] and qualification['source_sha256']==native.sha(__file__)
    target = OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    parent_freeze_path = ROOT/'results/native-thermal-constraints/run-001/freeze.json'
    parent_freeze = native.read(parent_freeze_path)
    for pins in parent_freeze['input_pins'].values():
        for path, digest in pins.items():assert native.sha(ROOT/path)==digest
    freeze = {'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'source_sha256':native.sha(__file__),'native_extension_sha256':native.sha(native.__file__),
              'qualification_sha256':native.sha(OUT/'qualification-002/checks.json'),
              'parent_freeze_sha256':native.sha(parent_freeze_path),
              'input_pins':parent_freeze['input_pins'],'methods':METHODS,'thermal_precision':.1,
              'pairs':[0,598],'missing_rgb_policy':'hold_internal_static_state_keep_output_null',
              'contract':'supplied_same_ID_no_duration_scaling','new_references_read':False}
    native.write(target/'freeze.json',freeze)
    receipts=[]
    for condition in ['day','night','rain']:
        source,E,saved,events = native.load(condition)
        states={m:torch.tensor([0,0,0,0,0,0,1],dtype=torch.float32) for m in METHODS}
        rows=[]
        for pair in range(599):
            event = events['rgb'].get((pair,pair+1))
            methods={};binding={}
            available=event is not None and not event['original_need_interp'] and event['derived_camera_delta_xyzw'] is not None
            if available:
                try:
                    packet,metadata=native.bind_camera(saved['thermal'],events['thermal'].get((pair,pair+1)),
                                                       pair,source['records'],'thr',E['thermal'])
                    binding={'thermal':metadata}
                except ValueError as error:
                    packet=None;binding={'thermal_unavailable':str(error)}
            for method in METHODS:
                if not available:
                    methods[method]={'status':'UNAVAILABLE_RGB_STATIC_STATE_HELD','transform':None,
                                     'parent_xyzw':states[method].tolist(),'committed_xyzw':states[method].tolist()}
                    continue
                try:
                    states[method],result=step(saved['rgb'],event,pair,source['records'],states[method],
                                               packet if method=='thermal_feedback' else None)
                    methods[method]=result
                except (RuntimeError,ValueError,np.linalg.LinAlgError) as error:
                    methods[method]={'status':'FIT_FAILED_STATE_HELD','transform':None,'reason':str(error),
                                     'parent_xyzw':states[method].tolist(),'committed_xyzw':states[method].tolist()}
            rows.append({'pair':pair,'methods':methods,'binding':binding})
            if pair%100==0:print('FEEDBACK',condition,pair,flush=True)
        path=target/f'predictions-{condition}.json'
        native.write(path,{'sequence':source['sequence'],'rows':rows,'new_references_read':False})
        receipts.append({'condition':condition,'prediction_sha256':native.sha(path)})
        del saved
    native.write(target/'prediction-lock.json',{'rows':receipts,'freeze_sha256':native.sha(target/'freeze.json'),
                                               'new_references_read':False})


def score():
    target=OUT/'run-001';lock=native.read(target/'prediction-lock.json');summary={}
    for receipt in lock['rows']:
        c=receipt['condition'];path=target/f'predictions-{c}.json'
        assert native.sha(path)==receipt['prediction_sha256'];rows=native.read(path)['rows']
        refs=[rigid(np.loadtxt(native.REC/'references'/c/f'{i:06}.txt').reshape(4,4))[0] for i in range(600)]
        methods={}
        for method in METHODS:
            def stats(start,end):
                value=native.path_stats(rows,refs,method,start,end)
                if value['status']=='COMPLETE':value.pop('positions');value.pop('reference')
                return value
            methods[method]={'available':sum(r['methods'][method]['transform'] is not None for r in rows),
                             'full_path':stats(0,599),
                             'blocks':[{'start':i,**stats(i,i+99)} for i in range(0,600,100)]}
        summary[c]=methods
        print('SCORED',c,{m:v['full_path'] for m,v in methods.items()},flush=True)
    native.write(target/'scores.json',{'summary':summary,'prediction_lock_sha256':native.sha(target/'prediction-lock.json'),
                                      'scope':'Pose feedback, native point registration/static initialization, fixed accepted image measurements.'})


if __name__=='__main__':
    {'qualify':qualify,'capture':capture,'score':score}[sys.argv[1]]()
