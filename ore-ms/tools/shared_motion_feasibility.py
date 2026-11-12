"""Synthetic shared stereo motion prototype. Optical XYZ, float64 CPU only.

C maps body coordinates into a camera. T maps source body to target body.
Source landmarks are conditioned on, not jointly optimized. Log variances are
residual UV/disparity task weights, not calibrated sensor uncertainties.
"""
from dataclasses import dataclass
import torch


@dataclass
class Packet:
    points: torch.Tensor       # N,3 in source body
    observations: torch.Tensor # N,3 target u,v,disparity
    camera_R: torch.Tensor     # N,3,3 body -> camera
    camera_t: torch.Tensor     # N,3
    intrinsics: torch.Tensor   # N,5 fx,fy,cx,cy,stereo baseline metres
    modality: torch.Tensor     # N integers 0 or 1
    intervals: torch.Tensor    # 2,2 actual source/target times


def skew(x):
    a,b,c = x.unbind(-1)
    z = torch.zeros_like(a)
    return torch.stack((z,-c,b,c,z,-a,-b,a,z),-1).reshape(*x.shape[:-1],3,3)


def validate(p, log_variance):
    n = len(p.points)
    shapes = [(p.points,(n,3)),(p.observations,(n,3)),
              (p.camera_R,(n,3,3)),(p.camera_t,(n,3)),
              (p.intrinsics,(n,5)),(log_variance,(n,3))]
    for x,shape in shapes:
        if x.shape != shape or x.dtype != torch.float64 or x.device.type != 'cpu':
            raise ValueError('shape/dtype/device')
        if not torch.isfinite(x).all(): raise ValueError('nonfinite input')
    if n < 6 or p.modality.shape != (n,) or set(p.modality.tolist()) != {0,1}:
        raise ValueError('need both modalities and at least six observations')
    if (p.intrinsics[:,[0,1,4]] <= 0).any(): raise ValueError('camera geometry')
    if p.intervals.shape != (2,2) or not torch.isfinite(p.intervals).all():
        raise ValueError('invalid intervals')
    if not torch.equal(p.intervals[0],p.intervals[1]):
        raise ValueError('asynchronous intervals require trajectory model')
    if p.intervals[0,1] <= p.intervals[0,0]: raise ValueError('nonpositive interval')
    eye = torch.eye(3,dtype=torch.float64)
    if not torch.allclose(p.camera_R @ p.camera_R.transpose(-1,-2),eye.expand(n,3,3),atol=1e-10,rtol=0):
        raise ValueError('nonrigid camera rotation')
    if not torch.allclose(torch.linalg.det(p.camera_R),torch.ones(n,dtype=torch.float64),atol=1e-10,rtol=0):
        raise ValueError('reflected camera rotation')


def project(p,R,t):
    q = p.points @ R.T + t
    c = (p.camera_R @ q.unsqueeze(-1)).squeeze(-1) + p.camera_t
    if not torch.isfinite(c).all() or (c[:,2] <= 0).any():
        raise ValueError('invalid target depth')
    x,y,z = c.unbind(-1)
    fx,fy,cx,cy,bl = p.intrinsics.unbind(-1)
    predicted = torch.stack((fx*x/z+cx,fy*y/z+cy,fx*bl/z),-1)
    o = torch.zeros_like(z)
    D = torch.stack((fx/z,o,-fx*x/z**2,o,fy/z,-fy*y/z**2,
                     o,o,-fx*bl/z**2),-1).reshape(-1,3,3)
    B = torch.cat((torch.eye(3,dtype=q.dtype).expand(len(q),3,3),-skew(q)),dim=-1)
    J = D @ p.camera_R @ B
    return predicted,J


def solve(p, log_variance, iterations=12, damping=1e-8, rank_ratio=1e-10):
    validate(p,log_variance)
    # One gauge for both cameras preserves their relative precision.
    precision = torch.exp(-(log_variance-log_variance.mean())).reshape(-1)
    if not torch.isfinite(precision).all() or (precision <= 0).any():
        raise ValueError('unrepresentable precision')
    R = torch.eye(3,dtype=torch.float64)
    t = torch.zeros(3,dtype=torch.float64)
    history = []
    for _ in range(iterations):
        pred,J = project(p,R,t)
        J = J.reshape(-1,6)
        residual = (pred-p.observations).reshape(-1)
        A = J * precision.sqrt()[:,None]
        sv = torch.linalg.svdvals(A.detach())
        if sv[-1] / sv[0] < rank_ratio:
            raise ValueError('degenerate measurement geometry')
        H = J.T @ (precision[:,None]*J)
        g = J.T @ (precision*residual)
        delta = torch.linalg.solve(H+damping*torch.diag(torch.diag(H)), -g)
        dR = torch.matrix_exp(skew(delta[3:]))
        R,t = dR @ R, dR @ t+delta[:3]
        history.append(float((precision*residual.square()).sum().detach()))
    return R,t,history


def pose_loss(R,t,Rref,tref):
    # Smooth synthetic loss only; this is not the future real-data training loss.
    return (R-Rref).square().mean() + (t-tref).square().sum()
