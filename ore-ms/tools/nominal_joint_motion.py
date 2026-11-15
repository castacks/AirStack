"""Fixed-budget conditional stereo joint solve under declared same-ID groups.

Optical XYZ; T maps source body to target body. Source landmarks are fixed.
This separate interface never changes/rewrites camera timestamps or the strict
synthetic solver. It models supplied same-ID groups as nominal simultaneity.
"""
import numpy as np
from scipy.spatial.transform import Rotation

CONTRACT='ms2_supplied_same_id_nominal_simultaneity_v1'


def skew(x):
    x=np.asarray(x);z=np.zeros(len(x));a,b,c=x.T
    return np.stack((z,-c,b,c,z,-a,-b,a,z),axis=-1).reshape(-1,3,3)


def project(points, camera_R, camera_t, Kb, T):
    q=points@T[:3,:3].T+T[:3,3]
    c=np.einsum('nij,nj->ni',camera_R,q)+camera_t
    if not np.isfinite(c).all() or np.any(c[:,2]<=0):raise ValueError('invalid_target_depth')
    x,y,z=c.T;fx,fy,cx,cy,b=Kb.T;o=np.zeros(len(z))
    uv=np.column_stack((fx*x/z+cx,fy*y/z+cy,fx*b/z))
    D=np.stack((fx/z,o,-fx*x/z**2,o,fy/z,-fy*y/z**2,o,o,-fx*b/z**2),axis=-1).reshape(-1,3,3)
    B=np.concatenate((np.broadcast_to(np.eye(3),(len(z),3,3)),-skew(q)),axis=-1)
    return uv,D@camera_R@B


def validate(p):
    n=len(p['points'])
    if n<3:raise ValueError('insufficient_native_factors')
    for k,shape in [('points',(n,3)),('observations',(n,3)),('camera_R',(n,3,3)),('camera_t',(n,3)),('Kb',(n,5)),('covariance',(n,3,3))]:
        if np.shape(p[k])!=shape or not np.isfinite(p[k]).all():raise ValueError('invalid_'+k)
    if np.any(p['Kb'][:,[0,1,4]]<=0):raise ValueError('invalid_intrinsics_baseline')
    if not np.allclose(p['covariance'],p['covariance'].transpose(0,2,1),rtol=0,atol=1e-12):raise ValueError('asymmetric_covariance')
    np.linalg.cholesky(p['covariance']) # Reject entire camera; never drop factors.
    if not np.allclose(p['camera_R']@p['camera_R'].transpose(0,2,1),np.eye(3),rtol=0,atol=1e-10):raise ValueError('invalid_camera_rotation')
    if not np.allclose(np.linalg.det(p['camera_R']),1,rtol=0,atol=1e-10):raise ValueError('reflected_camera_rotation')


def assemble(packets,contract):
    if contract!=CONTRACT:raise ValueError('explicit_nominal_group_contract_required')
    if not packets:raise ValueError('no_available_camera')
    for p in packets:validate(p)
    return {k:np.concatenate([p[k] for p in packets]) for k in ['points','observations','camera_R','camera_t','Kb','covariance']}


def solve(packets,weighting,contract,iterations=12,damping=1e-8,rank_ratio=1e-10):
    p=assemble(packets,contract)
    if weighting not in ['fixed','cov']:raise ValueError('invalid_weighting')
    L=np.linalg.cholesky(p['covariance']) if weighting=='cov' else np.broadcast_to(np.eye(3),(len(p['points']),3,3))
    T=np.eye(4);costs=[];first_ratio=None;last_step=None
    def system(T):
        uv,J=project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],T)
        r=np.linalg.solve(L,(uv-p['observations'])[...,None]).reshape(-1)
        A=np.linalg.solve(L,J).reshape(-1,6)
        return r,A
    for _ in range(iterations):
        r,A=system(T);costs.append(float(r@r));sv=np.linalg.svd(A,compute_uv=False)
        ratio=float(sv[-1]/sv[0])
        if first_ratio is None:first_ratio=ratio
        if ratio<rank_ratio:raise ValueError('degenerate_geometry')
        H=A.T@A;g=A.T@r
        delta=np.linalg.solve(H+damping*np.diag(np.diag(H)),-g)
        if not np.isfinite(delta).all():raise ValueError('nonfinite_step')
        dR=Rotation.from_rotvec(delta[3:]).as_matrix()
        T[:3,3]=dR@T[:3,3]+delta[:3];T[:3,:3]=dR@T[:3,:3]
        last_step=float(np.linalg.norm(delta))
    r,A=system(T);final=float(r@r)
    if final>costs[0]+1e-9*max(1,costs[0]):raise ValueError('objective_increased')
    return {'status':'FIXED_BUDGET_COMPLETE','source_to_target_body':T.tolist(),'transform':np.linalg.inv(T).tolist(),'points':len(p['points']),'weighting':weighting,'cost_history':costs,'final_cost':final,'last_step_norm':last_step,'initial_jacobian_singular_ratio':first_ratio,'gradient_inf':float(np.max(np.abs(A.T@r))),'convergence_claim':False}


def geometry(cal):
    Rr=np.asarray(cal['R_nir2rgb']['values'],float);Rt=np.asarray(cal['R_nir2thr']['values'],float)
    tr=np.asarray(cal['T_nir2rgb']['values'],float).reshape(3)/1000
    tt=np.asarray(cal['T_nir2thr']['values'],float).reshape(3)/1000
    E=np.eye(4);E[:3,:3]=Rt@Rr.T;E[:3,3]=tt-E[:3,:3]@tr
    return {'rgb':np.eye(4),'thermal':E}


def bind_camera(saved,event,parent,source_rows,sensor,E):
    if event is None or event.get('original_need_interp',True):raise ValueError('no_native_supported_first_write')
    times=[source_rows[j]['cameras'][sensor]['left']['recorded_time_ns'] for j in [parent,parent+1]]
    if [event['time0_ns'],event['time1_ns']]!=times:raise ValueError('native_interval_mismatch')
    if event['stage'] not in ['pair_processing','termination']:raise ValueError('unexpected_native_stage')
    mask=(saved['edge/match2frame1/mapping']==parent)&(saved['edge/match2frame2/mapping']==parent+1)
    uv=saved['match//pixel1_uv'][mask].astype(float);depth=saved['match//pixel1_d'][mask].reshape(-1).astype(float)
    if not np.isfinite(depth).all() or np.any(depth<=0):raise ValueError('invalid_source_depth')
    k0=saved['frames//K'][parent].astype(float);k=saved['frames//K'][parent+1].astype(float)
    pc=np.column_stack(((uv[:,0]-k0[0,2])*depth/k0[0,0],(uv[:,1]-k0[1,2])*depth/k0[1,1],depth))
    points=(pc-E[:3,3])@E[:3,:3]
    uvc=saved['match//pixel2_uv_cov'][mask].astype(float);dc=saved['match//pixel2_disp_cov'][mask].reshape(-1).astype(float)
    C=np.zeros((len(depth),3,3));C[:,0,0]=uvc[:,0];C[:,1,1]=uvc[:,1];C[:,0,1]=C[:,1,0]=uvc[:,2];C[:,2,2]=dc
    p={'points':points,'observations':np.column_stack((saved['match//pixel2_uv'][mask],saved['match//pixel2_disp'][mask].reshape(-1))).astype(float),'camera_R':np.repeat(E[None,:3,:3],len(depth),axis=0),'camera_t':np.repeat(E[None,:3,3],len(depth),axis=0),'Kb':np.tile([k[0,0],k[1,1],k[0,2],k[1,2],float(saved['frames//baseline'][parent+1])],(len(depth),1)),'covariance':C}
    validate(p)
    return p,{'actual_left_times_ns':times,'native_factor_indices':np.flatnonzero(mask).tolist(),'native_first_write_sequence':event['sequence'],'native_first_write_stage':event['stage'],'all_native_factors_retained':True,'source_points_conditioned_on':True}
