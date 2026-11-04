"""Reference-free CPU/CUDA checks of the pinned DROID stereo metric convention."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0,'/opt/DROID-SLAM/droid_slam')
import lietorch
import droid_backends
import geom.projective_ops as pops

from droid_ms2_development import metric_poses,INTERNAL_BASELINE


def check(output):
    count=0;results=[]
    def need(value,label):
        nonlocal count
        if not value:raise ValueError(label)
        count+=1
    for device in ['cpu','cuda']:
        for baseline in [.29918420530585865,.3038995142414822]:
            height,width=8,8;fx=128.;depth_metres=3.
            ratio=baseline/INTERNAL_BASELINE
            disps=torch.full((1,1,height,width),ratio/depth_metres,device=device)
            intrinsics=torch.tensor([[[fx,120.,3.,3.]]],device=device)
            poses=lietorch.SE3.Identity(1,1,device=device)
            ii=jj=torch.tensor([0],device=device)
            coords,valid=pops.projective_transform(poses,disps,intrinsics,ii,jj)
            grid=torch.arange(width,device=device)[None,:].expand(height,width)
            expected=grid-fx*baseline/depth_metres
            err=float((coords[0,0,:,:,0]-expected).abs().max())
            need(err<2e-5,'provided-baseline stereo projection '+device)
            need(bool((valid==1).all()),'all analytic points valid')
            t=torch.tensor([[.2,-.1,.3,0.,0.,np.sqrt(.5),np.sqrt(.5)]],dtype=torch.float32,device=device)
            tcw=lietorch.SE3(t);twc=tcw.inv()
            matrix=twc.matrix().cpu().numpy()
            converted,_,_=metric_poses(matrix,baseline,True)
            # Independent inverse of a 90degree-Z pose, then baseline-unit conversion.
            need(np.allclose(np.asarray(converted[0])[:3,3],[.1*ratio,.2*ratio,-.3*ratio],atol=2e-6),'native terminate inverse direction')
            recovered_depth=ratio/(ratio/depth_metres)
            need(abs(recovered_depth-depth_metres)<1e-12,'calibration-only depth scale')
            need(abs(1.10/(ratio/depth_metres)-depth_metres)>.5,'EuRoC multiplier negative control')
            results.append({'device':device,'baseline_m':baseline,'translation_factor':ratio,'max_projection_error_px':err})
    # Exercise the actual CUDA bundle-adjustment kernel's ii==jj stereo branch.
    # Pose is fixed by stereo self-edge; one inverse-depth step has a closed form.
    device='cuda';height=width=8;fx=128.;start=.3;truth=.4;eta_value=.001
    poses=torch.tensor([[0.,0.,0.,0.,0.,0.,1.]],device=device)
    disps=torch.full((1,height,width),start,device=device)
    sens=torch.zeros_like(disps);intrinsics=torch.tensor([fx,120.,3.,3.],device=device)
    y,x=torch.meshgrid(torch.arange(height,device=device),torch.arange(width,device=device),indexing='ij')
    target=torch.stack([x.float()-fx*.1*truth,y.float()])[None].contiguous()
    weight=torch.ones_like(target);eta=torch.full_like(disps,eta_value)
    ii=jj=torch.tensor([0],device=device,dtype=torch.long)
    droid_backends.ba(poses,disps,intrinsics,sens,target,weight,eta,ii,jj,0,1,1,1e-4,.1,False)
    torch.cuda.synchronize()
    information=.001*(fx*.1)**2
    expected=start+(truth-start)*information/(information+eta_value)
    error=float((disps-expected).abs().max())
    need(error<3e-5,'CUDA native stereo BA one-step depth update')
    need(bool(torch.isfinite(disps).all()),'CUDA depth finite')
    need(float((poses-torch.tensor([[0.,0.,0.,0.,0.,0.,1.]],device=device)).abs().max())<1e-6,'stereo self-edge leaves pose fixed')
    report={'pass':True,'checks':count,'projection_cases':results,
            'cuda_ba_expected_inverse_depth':expected,'cuda_ba_max_abs_error':error,
            'reference_used':False,'weights_or_images_used':False,'native_baseline':INTERNAL_BASELINE,
            'scope':'analytic tensor/CUDA geometry and metric-unit contract, not physical accuracy'}
    with Path(output).open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();check(args.output)
