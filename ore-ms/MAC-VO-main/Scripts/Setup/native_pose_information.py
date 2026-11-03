"""Copied fixed-landmark disparity information, not pose covariance."""
import numpy as np
from scipy.spatial.transform import Rotation


def copied(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.array(value, dtype=np.float64, copy=True)


def skew(v):
    x,y,z = v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def disparity_jacobian(points_camera, K, baseline):
    """Residual derivative for T_wc Exp(delta_camera), order translation/rotation."""
    p = copied(points_camera); k = copied(K)
    if (p.ndim!=2 or p.shape[1]!=3 or k.shape!=(3,3) or not np.isfinite(p).all()
            or not np.isfinite(k).all() or np.any(p[:,0]<=0) or k[0,1]!=0
            or k[0,0]<=0 or k[1,1]<=0 or not np.isfinite(baseline) or baseline<=0):
        raise ValueError('finite forward NED points, positive focal/baseline and zero skew required')
    j=[]
    for x,y,z in p:
        projection=np.array([[-k[0,0]*y/x**2,k[0,0]/x,0.],
                             [-k[1,1]*z/x**2,0.,k[1,1]/x],
                             [-k[0,0]*baseline/x**2,0.,0.]])
        j.append(projection@np.column_stack((-np.eye(3),skew([x,y,z]))))
    return np.asarray(j).reshape(-1,3,6)


def information(points_camera, K, baseline, covariance):
    p=copied(points_camera);c=copied(covariance)
    if c.shape!=(len(p),3,3) or not len(p) or not np.isfinite(c).all():
        raise ValueError('nonempty aligned finite covariance required')
    if not np.allclose(c,c.transpose(0,2,1),rtol=0,atol=1e-12):
        raise ValueError('symmetric covariance required')
    eig=np.linalg.eigvalsh(c)
    if np.any(eig < -1e-12*np.maximum(1,np.max(np.abs(eig),axis=1))[:,None]):
        raise ValueError('positive semidefinite covariance required')
    j=disparity_jacobian(p,K,baseline)
    precision=np.linalg.pinv(c,rcond=1e-15)
    h=np.einsum('nki,nkl,nlj->ij',j,precision,j)
    h=(h+h.T)/2
    return {'information_6x6':h.tolist(),'factors':len(p),
        'minimum_covariance_rank':int(min(np.linalg.matrix_rank(c))),
        'tangent':'right perturbation of supplied current camera pose; translation metres then rotation radians',
        'model':'fixed-landmark Gaussian J^T pinv(Sigma_uv_disp) J',
        'robust_kernel_included':False,'damping_included':False,
        'marginal_pose_covariance':False,'common_body_information':False,
        'physical_accuracy_verified':False}


def from_graph_data(data):
    """Read copies only; applies equally to native global/local optimization frames."""
    pose=copied(data.init_motion).reshape(-1)
    edges=copied(data.edges_index).reshape(-1)
    if pose.shape!=(7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1)>1e-5:
        raise ValueError('one finite unit-quaternion initial pose required')
    if np.any(edges!=0): raise ValueError('single optimized pose required')
    world=copied(data.points.data['pos_Tw'])
    p=(world-pose[:3])@Rotation.from_quat(pose[3:]).as_matrix()
    obs=data.observations.data
    uv=copied(obs['pixel2_uv_cov']);disp=copied(obs['pixel2_disp_cov']).reshape(-1)
    if uv.shape!=(len(p),3) or disp.shape!=(len(p),) or len(edges)!=len(p):
        raise ValueError('factor binding mismatch')
    # Native ReprojDisp constructs its covariance buffer as float32 before .double().
    c=np.zeros((len(p),3,3),np.float32)
    c[:,0,0]=uv[:,0];c[:,1,1]=uv[:,1];c[:,0,1]=c[:,1,0]=uv[:,2];c[:,2,2]=disp
    baseline=copied(data.baseline).reshape(-1)
    if len(baseline)!=1:raise ValueError('one stereo baseline required')
    return information(p,copied(data.images_intrinsic),float(baseline[0]),c)


def to_body_information(h_camera, body_from_camera):
    """At the same instant, body right twist -> camera right twist via Ad(T_cb)."""
    h=copied(h_camera);t=copied(body_from_camera)
    if h.shape!=(6,6) or t.shape!=(4,4) or not np.isfinite(h).all() or not np.isfinite(t).all():
        raise ValueError('finite information and rigid transform required')
    r=t[:3,:3]
    if not np.allclose(t[3],[0,0,0,1]) or not np.allclose(r.T@r,np.eye(3),atol=1e-10) or not np.isclose(np.linalg.det(r),1):
        raise ValueError('proper rigid body-from-camera transform required')
    invr=r.T;invt=-invr@t[:3,3]
    adj=np.block([[invr,skew(invt)@invr],[np.zeros((3,3)),invr]])
    return adj.T@h@adj


def project_information(h_camera, native_camera_delta, body_from_camera,
                        native_interval_ns, target_interval_ns):
    """Push local information through the existing stationary-twist motion model.

    Conditional on a fixed parent/landmarks/extrinsic. Includes no clock, model,
    extrinsic, map or cross-camera uncertainty. Not an independent pose covariance.
    """
    from .causal_interval_model import project, se3, jacobian
    h=copied(h_camera);delta=se3(native_camera_delta);t=se3(body_from_camera)
    if h.shape!=(6,6) or not np.isfinite(h).all():raise ValueError('finite camera information required')
    body_delta=t@delta@np.linalg.inv(t)
    result=project(body_delta,native_interval_ns,target_interval_ns)
    if not result['valid_model_projection']:raise ValueError('motion projection horizon exceeded')
    nominal=np.asarray(result['delta_model']);invnominal=np.linalg.inv(nominal)
    if np.linalg.norm(Rotation.from_matrix(body_delta[:3,:3]).as_rotvec())>np.pi-.001:
        raise ValueError('principal rotation branch too close to pi')
    def exp(v):
        out=np.eye(4);out[:3,:3]=Rotation.from_rotvec(v[3:]).as_matrix();out[:3,3]=jacobian(v[3:])@v[:3];return out
    def log(m):
        w=Rotation.from_matrix(m[:3,:3]).as_rotvec();return np.r_[np.linalg.solve(jacobian(w),m[:3,3]),w]
    columns=[];eps=1e-6
    for k in range(6):
        e=np.eye(6)[k]*eps;values=[]
        for v in [e,-e]:
            moved=t@delta@exp(v)@np.linalg.inv(t)
            projected=project(moved,native_interval_ns,target_interval_ns)
            values.append(log(invnominal@np.asarray(projected['delta_model'])))
        columns.append((values[0]-values[1])/(2*eps))
    a=np.column_stack(columns)
    if not np.isfinite(a).all() or np.linalg.cond(a)>1e10:raise ValueError('ill-conditioned information transport')
    inverse=np.linalg.inv(a);out=inverse.T@h@inverse;out=(out+out.T)/2
    return {'information_body_6x6':out.tolist(),'delta_model':result['delta_model'],
        'native_interval_ns':list(native_interval_ns),'target_interval_ns':list(target_interval_ns),
        'model':result['model'],'transport_jacobian':a.tolist(),'finite_difference_step':eps,
        'conditional_fixed_parent_landmarks_extrinsic':True,'additional_model_uncertainty_included':False,
        'physical_accuracy_verified':False,'admission_ready':False}
