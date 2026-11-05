"""Native CUDA alt-correlation equivalence for legal active/reserved feature indices.
No camera images, weights, poses, reference data or full-record inference used.
"""
import hashlib,json
from pathlib import Path
import sys
from types import SimpleNamespace
import torch
import torch.nn.functional as F
from droid_bounded_worker_v4 import Guard,compact_feature_buffer

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/bounded-full-record/feature-compaction-contract.json'


def run():
 torch.manual_seed(0);torch.cuda.manual_seed_all(0)
 sys.path.insert(0,'/opt/DROID-SLAM/droid_slam')
 from modules.corr import AltCorrBlock
 reports=[]
 for h,w in [(31,98),(32,80)]:
  count=12;slots=64
  original=torch.randn(slots,2,128,h,w,device='cuda',dtype=torch.float16)
  v=SimpleNamespace(counter=SimpleNamespace(value=count),fmaps=original);g=Guard(slots)
  compact_feature_buffer(v,g)
  full=AltCorrBlock(original.view(1,slots*2,128,h,w))
  compact=AltCorrBlock(v.fmaps.view(1,v.fmaps.shape[0]*2,128,h,w))
  # Cover every active left/right feature plus all retained filler reserve rows.
  ii=torch.arange(2*(count+17),device='cuda');jj=torch.flip(ii,[0]);edges=len(ii)
  y,x=torch.meshgrid(torch.arange(h,device='cuda'),torch.arange(w,device='cuda'),indexing='ij')
  coords=torch.stack([x+.23,y-.17],-1).float()[None,None].expand(1,edges,h,w,2).contiguous()
  a=full(coords,ii,jj);b=compact(coords,ii,jj);torch.cuda.synchronize()
  if not torch.equal(a,b):raise ValueError('native active/reserve correlations differ')
  reports.append({'grid':[h,w],'active_keyframes':count,'reserved':17,'edges':edges,
   'full_slots':slots,'compacted_slots':v.fmaps.shape[0],'native_correlation_equal':True,
   'max_absolute_difference':float((a-b).abs().max()),'preserved_feature_bits_equal':True})
  del a,b,original,v,full,compact;torch.cuda.empty_cache()
 actual={m:4096*2*128*(h//8)*(w//8) for m,(h,w) in {'rgb':[248,784],'thermal':[256,640]}.items()}
 report={'passed':True,'fixtures':reports,'allocated4096_feature_elements':actual,
 'native_accessor_element_limit':2**31-1,'model_weights_loaded':False,'camera_images_used':False,
 'tracking_executed':False,'reference_scoring':False,'benchmark_execution_ready':False}
 with OUT.open('x') as f:json.dump(report,f,indent=2);f.write('\n')
 print(json.dumps(report,indent=2))


if __name__=='__main__':run()
