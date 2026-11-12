"""Declared supplementary case: near-coincident landmarks, not exact duplicates."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from run_shared_motion_feasibility import fixture,tensor
from shared_motion_feasibility import solve,project

ap=argparse.ArgumentParser(); ap.add_argument('--protocol',required=True); ap.add_argument('--output',required=True); args=ap.parse_args()
pin=json.loads(Path(args.protocol).read_text())
root=Path(__file__).resolve().parents[1]
for name,h in pin['source_sha256'].items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==h
p,_,_=fixture(); p.points=tensor(np.array([0,0,6.])+np.random.default_rng(23).normal(size=(24,3))*pin['landmark_spread_m'])
_,J=project(p,torch.eye(3,dtype=torch.float64),torch.zeros(3,dtype=torch.float64)); sv=torch.linalg.svdvals(J.reshape(-1,6))
ratio=float(sv[-1]/sv[0]); assert ratio<pin['rank_ratio']
try: solve(p,torch.zeros((24,3),dtype=torch.float64),rank_ratio=pin['rank_ratio'])
except ValueError as e:
 assert 'degenerate' in str(e)
 result={'pass':True,'singular_value_ratio':ratio,'landmark_spread_m':pin['landmark_spread_m'],'rejected':str(e)}
else: raise AssertionError('near-degenerate geometry accepted')
with Path(args.output).open('x') as f: json.dump(result,f,indent=2); f.write('\n')
print(json.dumps(result))
