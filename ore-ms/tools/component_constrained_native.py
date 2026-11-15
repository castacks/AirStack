"""Fixed-R measurement-level translation ablation; not pose blending or SFVO."""
from pathlib import Path
import sys
import datetime
import numpy as np
import torch
import native_pose_feedback as feedback
from Module.Optimization.PyposeOptimizers import AnalyticModule

native = feedback.native
ROOT = native.ROOT
OUT = ROOT/'results/component-constrained-native'
BASE = ROOT/'results/native-pose-feedback/run-001'
METHODS = ['rgb_fixed_rotation', 'thermal_fixed_rotation']


class FixedRotationGraph(AnalyticModule):
    """The optimizer sees three translations; all residual code stays native."""
    def __init__(self, base, fixed_pose):
        super().__init__()
        base = base.double()
        # Keep the original graph outside nn registration so its SE3 parameter
        # cannot enter the optimizer's parameter list.
        object.__setattr__(self, 'base', base)
        with torch.no_grad():
            base.pose2opt.copy_(fixed_pose)
        base.pose2opt.requires_grad_(False)
        self.translation = torch.nn.Parameter(fixed_pose.tensor()[:, :3].clone())

    def sync(self):
        with torch.no_grad():
            self.base.pose2opt[:, :3].copy_(self.translation)

    @property
    def pose2opt(self):
        self.sync()
        return self.base.pose2opt

    def forward(self):
        self.sync()
        return self.base()

    def build_jacobian(self):
        return self.base.build_jacobian()[:, :3]

    def covariance_array(self):
        return self.base.covariance_array()


def step(saved, event, pair, source, state, backbone, packet=None):
    data = feedback.feedback_input(saved, event, pair, source['records'], state)
    parent = native.pose(state.numpy())
    base = native.Analytic_ReprojDisp_TwoFramePGO(data) if packet is None else native.NativeThermalGraph(data, packet, parent)
    target = torch.tensor(backbone['committed_xyzw'], dtype=torch.float64)[None]
    displacement = target[0,:3] - torch.tensor(backbone['parent_xyzw'][:3], dtype=torch.float64)
    target[0,:3] = state[:3].double() + displacement
    graph = FixedRotationGraph(base, native.pp.SE3(target))
    assert sum(p.numel() for p in graph.parameters()) == 3
    _, losses = native.optimize(graph)
    current = graph.pose2opt.detach().tensor()[0].float().clone()
    assert current[3:].tolist() == backbone['committed_xyzw'][3:]
    relative = np.linalg.solve(parent, native.pose(current.numpy()))
    basis = np.eye(4);basis[:3,:3]=native.O
    return current, {'status':'THERMAL_FIXED_R' if packet is not None else 'RGB_FIXED_R',
                     'transform':(basis@relative@basis.T).tolist(),'parent_xyzw':state.tolist(),
                     'committed_xyzw':current.tolist(),'losses':losses}


def qualify():
    torch.set_num_threads(1)
    target=OUT/'qualification-001';target.mkdir(parents=True,exist_ok=False);checks=[]
    for condition in ['day','night','rain']:
        source,E,saved,events=native.load(condition)
        baseline=native.read(BASE/f'predictions-{condition}.json')['rows']
        for pair in [0,1,2]:
            event=events['rgb'][(pair,pair+1)];backbone=baseline[pair]['methods']['rgb_feedback']
            state=torch.tensor(backbone['parent_xyzw'],dtype=torch.float32)
            packet,_=native.bind_camera(saved['thermal'],events['thermal'][(pair,pair+1)],pair,
                                        source['records'],'thr',E['thermal'])
            data=feedback.feedback_input(saved['rgb'],event,pair,source['records'],state)
            base=native.NativeThermalGraph(data,packet,native.pose(state.numpy()))
            fixed=native.pp.SE3(torch.tensor(backbone['committed_xyzw'],dtype=torch.float64)[None])
            wrapper=FixedRotationGraph(base,fixed)
            initial=wrapper().detach().numpy();analytic=wrapper.build_jacobian().numpy()
            assert sum(p.numel() for p in wrapper.parameters())==3
            original_t=wrapper.translation.detach().clone();columns=[]
            for axis in range(3):
                with torch.no_grad():wrapper.translation.copy_(original_t);wrapper.translation[0,axis]+=1e-6
                plus=wrapper().detach().numpy().reshape(-1)
                with torch.no_grad():wrapper.translation.copy_(original_t);wrapper.translation[0,axis]-=1e-6
                minus=wrapper().detach().numpy().reshape(-1);columns.append((plus-minus)/2e-6)
            with torch.no_grad():wrapper.translation.copy_(original_t)
            numeric=np.stack(columns,axis=-1)
            error=float(np.max(np.abs(analytic-numeric)/(1+np.abs(numeric))))
            data_rgb=feedback.feedback_input(saved['rgb'],event,pair,source['records'],state)
            rgb=FixedRotationGraph(native.Analytic_ReprojDisp_TwoFramePGO(data_rgb),fixed)
            n=len(data_rgb.points.data['pos_Tw'])
            assert torch.equal(rgb(),wrapper()[:n])
            assert torch.equal(rgb.build_jacobian(),wrapper.build_jacobian()[:3*n])
            _,result=step(saved['rgb'],event,pair,source,state,backbone,packet)
            assert result['committed_xyzw'][3:]==backbone['committed_xyzw'][3:]
            checks.append({'condition':condition,'pair':pair,'optimized_parameters':3,
                           'jacobian_relative_error':error,'rgb_residual_and_jacobian_prefix_exact':True,
                           'native_rgb_quaternion_exact':True})
        del saved
    passed=all(x['jacobian_relative_error']<1e-4 for x in checks)
    native.write(target/'checks.json',{'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_sha256':native.sha(__file__),'feedback_sha256':native.sha(feedback.__file__),
        'native_extension_sha256':native.sha(native.__file__),'checks':checks,'passed':passed})
    print('QUALIFIED',passed,'max Jacobian relative',max(x['jacobian_relative_error'] for x in checks),flush=True)
    assert passed


def capture():
    torch.set_num_threads(1)
    qualified=native.read(OUT/'qualification-001/checks.json')
    assert qualified['passed'] and qualified['source_sha256']==native.sha(__file__)
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    parent_freeze=native.read(BASE/'freeze.json');baseline_lock=native.read(BASE/'prediction-lock.json')
    for pins in parent_freeze['input_pins'].values():
        for path,digest in pins.items():assert native.sha(ROOT/path)==digest
    freeze={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':native.sha(__file__),
            'feedback_sha256':native.sha(feedback.__file__),'native_extension_sha256':native.sha(native.__file__),
            'qualification_sha256':native.sha(OUT/'qualification-001/checks.json'),
            'baseline_lock_sha256':native.sha(BASE/'prediction-lock.json'),'input_pins':parent_freeze['input_pins'],
            'methods':METHODS,'thermal_precision':.1,'pairs':[0,598],
            'warm_start':'own_parent_translation_plus_current_native_RGB_displacement',
            'fixed_rotation':'same_frame_native_RGB_committed_quaternion','new_references_read':False}
    native.write(target/'freeze.json',freeze);receipts=[]
    for condition in ['day','night','rain']:
        pin=next(x for x in baseline_lock['rows'] if x['condition']==condition)
        baseline_path=BASE/f'predictions-{condition}.json';assert native.sha(baseline_path)==pin['prediction_sha256']
        baseline=native.read(baseline_path)['rows'];source,E,saved,events=native.load(condition)
        states={m:torch.tensor([0,0,0,0,0,0,1],dtype=torch.float32) for m in METHODS};rows=[]
        for pair in range(599):
            event=events['rgb'].get((pair,pair+1));backbone=baseline[pair]['methods']['rgb_feedback'];methods={};binding={}
            available=backbone['transform'] is not None
            if available:
                try:
                    packet,meta=native.bind_camera(saved['thermal'],events['thermal'].get((pair,pair+1)),pair,
                                                  source['records'],'thr',E['thermal']);binding={'thermal':meta}
                except ValueError as error:packet=None;binding={'thermal_unavailable':str(error)}
            for method in METHODS:
                if not available:
                    methods[method]={'status':'UNAVAILABLE_RGB_STATE_HELD','transform':None,
                                     'parent_xyzw':states[method].tolist(),'committed_xyzw':states[method].tolist()}
                else:
                    try:
                        states[method],methods[method]=step(saved['rgb'],event,pair,source,states[method],backbone,
                                                           packet if method=='thermal_fixed_rotation' else None)
                    except (RuntimeError,ValueError,np.linalg.LinAlgError) as error:
                        methods[method]={'status':'FIT_FAILED_STATE_HELD','transform':None,'reason':str(error),
                                         'parent_xyzw':states[method].tolist(),'committed_xyzw':states[method].tolist()}
            rows.append({'pair':pair,'methods':methods,'binding':binding})
            if pair%100==0:print('FIXED_R',condition,pair,flush=True)
        p=target/f'predictions-{condition}.json';native.write(p,{'rows':rows,'new_references_read':False,'sequence':source['sequence']})
        receipts.append({'condition':condition,'prediction_sha256':native.sha(p)})
        del saved
    native.write(target/'prediction-lock.json',{'rows':receipts,'freeze_sha256':native.sha(target/'freeze.json'),
                                               'new_references_read':False})


def score():
    target=OUT/'run-001';lock=native.read(target/'prediction-lock.json');summary={}
    for receipt in lock['rows']:
        c=receipt['condition'];p=target/f'predictions-{c}.json';assert native.sha(p)==receipt['prediction_sha256']
        rows=native.read(p)['rows'];refs=[feedback.rigid(np.loadtxt(native.REC/'references'/c/f'{i:06}.txt').reshape(4,4))[0]
                                        for i in range(600)];methods={}
        for method in METHODS:
            def stats(start,end):
                value=native.path_stats(rows,refs,method,start,end)
                if value['status']=='COMPLETE':value.pop('positions');value.pop('reference')
                return value
            methods[method]={'available':sum(r['methods'][method]['transform'] is not None for r in rows),
                             'full_path':stats(0,599),'blocks':[{'start':i,**stats(i,i+99)} for i in range(0,600,100)]}
        summary[c]=methods;print('SCORED',c,{m:v['full_path'] for m,v in methods.items()},flush=True)
    native.write(target/'scores.json',{'summary':summary,'prediction_lock_sha256':native.sha(target/'prediction-lock.json'),
                                      'scope':'Measurement-level translation reoptimization at fixed nativeRGB rotations, cached observations.'})


if __name__=='__main__':{'qualify':qualify,'capture':capture,'score':score}[sys.argv[1]]()
