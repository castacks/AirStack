"""Run unchanged author depth evaluator against locked predictions."""
import json
import os
import runpy
import sys
from thermal_sfm_reproduction import AUTHOR,ASSETS,OUT,verify,compatibility,sha

scene=sys.argv[1]
assert scene in ('outdoor_robust_night1','outdoor_robust_night2')
verify();lock=json.loads((OUT/'prediction-lock.json').read_text())
for n,d in lock['pins'].items():assert sha(OUT/n)==d,n
compatibility();os.chdir(AUTHOR)
sys.argv=['eval_depth.py','--dataset','VIVID','--scene','outdoor','--input','T',
          '--pred_depth',str(OUT/scene/'depth/predictions.npy'),
          '--gt_depth',str(ASSETS/'prepared'/scene/'Depth_T'),
          '--ratio_name',str(OUT/scene/'author-depth-ratios.txt')]
runpy.run_path(str(AUTHOR/'eval_vivid/eval_depth.py'),run_name='__main__')
verify()
