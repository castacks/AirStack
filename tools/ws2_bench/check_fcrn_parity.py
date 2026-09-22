"""Compare deployed TF outputs with Rui's PyTorch FCRN on identical input tensors.

First run inspect_patch_depth.py in the Kim image. Run this script in a CPU torch
environment, with the unmodified Humanflow checkout and converted FCRN weights.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
import torch

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('captures',type=Path);p.add_argument('--humanflow',required=True,type=Path)
p.add_argument('--weights',required=True,type=Path)
p.add_argument('--diagnose-pooling',action='store_true',help='Temporary diagnostic only: TensorFlow SAME max-pool padding')
a=p.parse_args()
sys.path.insert(0,str(a.humanflow/'attk/src'))
from fcrn_pytorch import load_fcrn
torch.set_num_threads(2)
model=load_fcrn(str(a.weights),'cpu');rows={}
if a.diagnose_pooling:
    class SamePool(torch.nn.Module):
        def forward(self,x):
            h,w=x.shape[-2:]
            ph=max((h+1)//2*2+1-h,0);pw=max((w+1)//2*2+1-w,0)
            padded=torch.nn.functional.pad(x,(pw//2,pw-pw//2,ph//2,ph-ph//2),value=float('-inf'))
            return torch.nn.functional.max_pool2d(padded,3,2)
    model.pool=SamePool()
with torch.no_grad():
    for source in sorted(a.captures.glob('*_input.npy')):
        name=source.name.removesuffix('_input.npy');x=np.load(source)
        ref=np.load(a.captures/(name+'_fcrn.npy'))
        y=model(torch.from_numpy(x.transpose(2,0,1).copy())[None]/255).numpy().squeeze()
        rows[name]={'shape':list(y.shape),'mean_absolute_difference':float(np.abs(y-ref).mean()),
                    'max_absolute_difference':float(np.abs(y-ref).max()),
                    'tf_mean':float(ref.mean()),'pytorch_mean':float(y.mean())}
(a.captures/('pooling_diagnostic.json' if a.diagnose_pooling else 'fcrn_parity.json')).write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
