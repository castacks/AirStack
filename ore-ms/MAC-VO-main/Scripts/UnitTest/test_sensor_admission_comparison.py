import copy
import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from Scripts.Setup.native_pose_information import disparity_jacobian,information,from_graph_data,to_body_information
from Scripts.Setup.sensor_admission_comparison import predict,score,adequate,SCALES
from Scripts.Setup.geometry_event_recorder import GeometryEventRecorder
from Scripts.UnitTest.test_arbiter_event_recorder import System,frame,events


def points():return np.array([[3.,.2,.3],[4.,-.4,.8],[6.,.9,-.5],[2.,-.3,-.5]])
K=np.array([[300.,0.,160.],[0.,310.,120.],[0.,0.,1.]])

def test_projection_right_tangent_finite_difference():
    p=points();j=disparity_jacobian(p,K,.2)
    def project(q):return np.column_stack((300*q[:,1]/q[:,0]+160,310*q[:,2]/q[:,0]+120,60/q[:,0]))
    for k in range(6):
        e=np.eye(6)[k]*1e-6
        def moved(d):return (p-d[:3])@Rotation.from_rotvec(d[3:]).as_matrix()
        fd=(project(moved(e))-project(moved(-e)))/2e-6
        assert np.allclose(fd,j[:,:,k],rtol=1e-6,atol=1e-7)


def test_information_matches_native_graph_and_pinv():
    import torch
    import pypose as pp
    from pathlib import Path
    from Scripts.Setup.solver_repair_experiment import native_graph_class
    root=Path(__file__).resolve().parents[3]
    cls=native_graph_class(root)
    pose=np.r_[.4,.2,-.1,Rotation.from_rotvec([.1,-.2,.3]).as_quat()]
    world=points()@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
    n=len(world);uvcov=torch.tensor([[2.,3.,.4]]*n)
    data=NS(from_idx=torch.tensor([0]),frame_idx=torch.tensor([1]),init_motion=pp.SE3(torch.tensor(pose[None],dtype=torch.float32)),
        edges_index=torch.zeros(n,dtype=torch.long),points=NS(data={'pos_Tw':torch.tensor(world,dtype=torch.float32),'cov_Tw':torch.eye(3).repeat(n,1,1)}),
        observations=NS(data={'pixel2_uv_cov':uvcov,'pixel2_uv':torch.zeros(n,2),'pixel2_disp':torch.zeros(n,1),'pixel2_disp_cov':torch.full((n,1),.7)}),
        images_intrinsic=torch.tensor(K,dtype=torch.float32),baseline=torch.tensor([.2]))
    before=data.points.data['pos_Tw'].clone();result=from_graph_data(data)
    native=cls(data).double();native();j=native.build_jacobian().detach().numpy().reshape(n,3,7)[:,:,:6]
    # Native uses a left tangent in its optimization frame. Transport current-camera right twist.
    t=data.init_motion.detach().numpy().reshape(7);r=Rotation.from_quat(t[3:]).as_matrix()
    from Scripts.Setup.native_pose_information import skew
    adj=np.block([[r,skew(t[:3])@r],[np.zeros((3,3)),r]])
    j=j@adj
    c=native.covariance_array().detach();w=torch.pinverse(c).numpy();expected=np.einsum('nki,nkl,nlj->ij',j,w,j)
    assert np.allclose(result['information_6x6'],expected,rtol=2e-6,atol=.02)
    assert torch.equal(before,data.points.data['pos_Tw'])
    assert not result['marginal_pose_covariance']


def test_singular_directional_and_rigid_transport():
    p=points();h=np.array(information(p,K,.2,np.zeros((4,3,3)))['information_6x6']);assert np.array_equal(h,np.zeros((6,6)))
    h=np.diag([1,2,3,4,5,6.]);t=np.eye(4);t[:3,:3]=Rotation.from_rotvec([.2,.1,-.4]).as_matrix();t[:3,3]=[1,2,3]
    b=to_body_information(h,t);assert np.allclose(b,b.T) and np.min(np.linalg.eigvalsh(b))>0
    assert np.allclose(to_body_information(b,np.linalg.inv(t)),h)
    bad=np.tile(np.diag([-1,1,1]),(4,1,1))
    with pytest.raises(ValueError):information(p,K,.2,bad)
    assert not adequate(np.diag([1e9]*5+[0]))[0]


def contract(n=4):
    rows=[];matrix=np.diag(2/SCALES**2).tolist()
    for i in range(n):
        interval=[i*100,(i+1)*100]
        camera={'available':True,'supported':True,'information':{'frame_id':'nominal_body','interval_ns':interval,'received_ns':i*1000,'matrix':matrix,'evidence_ref':f'journal/{i}'}}
        rows.append({'sample_id':f'x/{i}','recording_id':'development','window':'a','pair_ordinal':i,'interval_ns':interval,'deadline_ns':i*1000+1,'cameras':{'rgb':copy.deepcopy(camera),'thermal':copy.deepcopy(camera)},'baselines':{'raw':'RGB','history':'RGB'}})
    return {'schema_version':1,'frame_id':'nominal_body','rows':rows}


def test_recovery_defers_when_other_adequate_and_reconsiders_when_needed():
    c=contract();c['rows'][0]['cameras']['rgb'].update(available=False,supported=False,information=None);c['rows'][0]['baselines']={'raw':'THERMAL','history':'ABSTAIN'}
    d=predict(c);assert [r['actions']['candidate'] for r in d['rows']]==['THERMAL','THERMAL','RGB','RGB']
    c['rows'][1]['cameras']['thermal']['information']['matrix']=np.zeros((6,6)).tolist()
    d=predict(c);assert d['rows'][1]['actions']['candidate']=='RGB' and d['rows'][1]['reason']=='reconsider_recovery_remaining_inadequate'
    assert not d['labels_used']


def test_missing_geometry_labels_future_receipts_and_frame_mismatch_reject():
    for mutate in [lambda c:c.update(labels={}),lambda c:c['rows'][0]['cameras']['rgb'].update(information=None),
                   lambda c:c['rows'][0]['cameras']['rgb']['information'].update(received_ns=10**9),
                   lambda c:c['rows'][0]['cameras']['rgb']['information'].update(frame_id='camera')]:
        c=contract();mutate(c)
        with pytest.raises(ValueError):predict(c)


def test_abstention_and_exact_matched_scoring():
    c=contract(2)
    for camera in c['rows'][1]['cameras'].values():camera['information']['matrix']=np.zeros((6,6)).tolist()
    d=predict(c);assert d['rows'][1]['actions']['candidate']=='ABSTAIN'
    labels={r['sample_id']:{cam:{'translation_m':1.,'rotation_deg':1.} for cam in ['rgb','thermal']} for r in c['rows']}
    scored=score(d,labels);record=scored['records']['development'];assert record['methods']['candidate']['scored']==1
    assert len(record['matched']['raw']['ids'])==1 and not scored['passes_all_records']
    assert not scored['candidate_adopted']
    assert record['admission_costs']['rgb']['discarded_supported_motion_ids']==['x/1']


def test_geometry_recorder_observes_returned_pose_before_write_without_mutation(tmp_path):
    system=System();system.Optimizer.config=NS(graph_type='disp')
    old=system.make_graph_data
    def data(index):
        d=old(index);d.init_motion=np.array([[0,0,0,0,0,0,1.]])
        d.points=NS(data={'pos_Tw':points()});d.edges_index=np.zeros(4,dtype=int)
        d.observations=NS(data={'pixel2_uv_cov':np.tile([2.,3.,.4],(4,1)),'pixel2_disp_cov':np.ones((4,1))})
        d.images_intrinsic=K;d.baseline=np.array([.2]);return d
    system.make_graph_data=data
    rec=GeometryEventRecorder(system,tmp_path/'events','rgb');system.run_pair(frame(10),frame(20));system.terminate();summary=rec.finish()
    out=events(rec);info=next(e for e in out if e['kind']=='native_disparity_information');submit=next(e for e in out if e['kind']=='optimization_submitted')
    write=next(e for e in out if e['kind']=='motion_result_written')
    assert submit['sequence']<info['sequence']<write['sequence'] and info['submission_id']==submit['submission_id']==1
    assert info['time0_ns']==10 and info['time1_ns']==20 and info['information']['factors']==4
    assert system.Optimizer.starts==1 and summary['status']=='COMPLETE'


def test_interval_information_transport_preserves_model_and_duration_scaling():
    from Scripts.Setup.native_pose_information import project_information
    from Scripts.Setup.causal_interval_model import project
    h=np.diag([2.,3.,4.,5.,6.,7.]);delta=np.eye(4);delta[:3,3]=[.1,.2,.3]
    t=np.eye(4);t[:3,:3]=Rotation.from_rotvec([.2,-.1,.3]).as_matrix();t[:3,3]=[.3,.2,-.1]
    out=project_information(h,delta,t,[0,100],[0,100]);expected=to_body_information(h,t)
    assert np.allclose(out['information_body_6x6'],expected,atol=1e-7)
    assert np.allclose(out['delta_model'],project(t@delta@np.linalg.inv(t),[0,100],[0,100])['delta_model'])
    out=project_information(h,np.eye(4),np.eye(4),[0,100],[0,50])
    assert np.allclose(out['information_body_6x6'],4*h,atol=1e-7)
    assert not out['additional_model_uncertainty_included']
    with pytest.raises(ValueError):project_information(h,delta,t,[0,100],[1000,1100])


def test_geometry_contract_binds_current_prefix_and_rejects_missing_or_late_geometry():
    from Scripts.Setup.prepare_sensor_admission_contract import prepare
    native=[0,100];matrix=np.diag(2/SCALES**2).tolist();timeline=[];experts={}
    for camera in ['rgb','thermal']:
        info={'kind':'native_disparity_information','sequence':0,'submission_id':1,'time0_ns':0,'time1_ns':100,'from_idx':0,'frame_idx':1,
            'linearization':'returned optimizer pose before first write; copied submitted fixed factors','information':{'information_6x6':matrix}}
        motion={'kind':'motion_result_written','sequence':1,'submission_id':1,'first_write_for_submission':True,'original_need_interp':False,
            'time0_ns':0,'time1_ns':100,'from_idx':0,'frame_idx':1,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]}
        for e in [info,motion]:timeline.append({'kind':'observer_event','camera':camera,'sequence':len(timeline),'controller_monotonic_ns':10+len(timeline),'event':e})
        experts[camera]={'available_by_prefix':True,'valid_model_projection':True,'source':{'motion_event_sequence':1,'received_controller_ns':20},'native_interval_ns':native,'delta_model':np.eye(4).tolist()}
    predictions={'rows':[{'window':'a','reference_used_for_prediction':False,'controller_deadline_ns':1000,'pair_ordinal':0,'common_interval_ns':native,'experts':experts,'model_actions':{'raw_advice':'RGB','history':'ABSTAIN'}}]}
    static={'a':{'nominal_body_from_camera':{c:np.eye(4).tolist() for c in ['rgb','thermal']}}}
    c,proof=prepare(predictions,static,timeline,'dev','a');assert len(proof)==2;assert predict(c)['rows'][0]['actions']['candidate']=='RGB'
    late=copy.deepcopy(timeline);late[0]['controller_monotonic_ns']=1001
    with pytest.raises(ValueError,match='missing/duplicate causal geometry'):prepare(predictions,static,late,'dev','a')
    wrong=copy.deepcopy(timeline);wrong[0]['event']['time1_ns']=99
    with pytest.raises(ValueError,match='bindings differ'):prepare(predictions,static,wrong,'dev','a')


def test_information_degradation_resets_confirmation_even_when_tracking_supported():
    c=contract(4);c['rows'][0]['cameras']['rgb']['information']['matrix']=np.zeros((6,6)).tolist()
    rows=predict(c)['rows']
    assert rows[0]['streak']['rgb']==0 and rows[1]['streak']['rgb']==1
    assert [r['actions']['candidate'] for r in rows]==['THERMAL','THERMAL','RGB','RGB']


def test_prediction_and_scoring_cli_are_separate_and_exclusive(tmp_path):
    import subprocess
    import sys
    inputs=tmp_path/'inputs.json';actions=tmp_path/'actions.json';labels=tmp_path/'labels.json';scores=tmp_path/'scores.json'
    inputs.write_text(json.dumps(contract(2)))
    labels.write_text(json.dumps({f'x/{i}':{c:{'translation_m':1.,'rotation_deg':1.} for c in ['rgb','thermal']} for i in range(2)}))
    command=[sys.executable,'-m','Scripts.Setup.sensor_admission_comparison']
    subprocess.run(command+['predict','--input',str(inputs),'--output',str(actions)],check=True,capture_output=True)
    assert json.loads(actions.read_text())['labels_used'] is False
    subprocess.run(command+['score','--input',str(actions),'--labels',str(labels),'--output',str(scores)],check=True,capture_output=True)
    assert not json.loads(scores.read_text())['passes_all_records']
    old=actions.read_bytes()
    assert subprocess.run(command+['predict','--input',str(inputs),'--output',str(actions)],capture_output=True).returncode!=0
    assert actions.read_bytes()==old
    assert subprocess.run(command+['predict','--input',str(inputs),'--labels',str(labels),'--output',str(tmp_path/'bad.json')],capture_output=True).returncode!=0
