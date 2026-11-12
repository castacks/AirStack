"""Pinned author-source reproduction on a separately recorded modern runtime."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import runpy
import sys

ROOT=Path(__file__).resolve().parents[1]
AUTHOR=Path(os.environ.get('THERMAL_SFM_AUTHOR','/author'))
ASSETS=Path(os.environ.get('THERMAL_SFM_ASSETS','/assets'))
OUT=ROOT/'results/thermal-sfm-reproduction/run-001'
PLAN=ROOT/'experiments/thermal-sfm-reproduction-freeze-2026-11-12.json'
SEQUENCES=('outdoor_robust_night1','outdoor_robust_night2')


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def write(p,j):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(j,f,indent=2,allow_nan=False)


def compatibility():
    import numpy as np
    import torch
    # API-only compatibility: no author file, model or mathematical setting changed.
    np.float=float
    np.int=int
    original=torch.load
    def legacy_load(*args,**kwargs):
        kwargs.setdefault('weights_only',False)
        return original(*args,**kwargs)
    torch.load=legacy_load
    torch.manual_seed(0);np.random.seed(0)
    torch.backends.cudnn.benchmark=False
    torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False


def prepare():
    """Call original loader and dump routine only for official outdoor test scenes."""
    compatibility()
    from path import Path as AuthorPath
    os.chdir(AUTHOR)
    sys.path.insert(0,str(AUTHOR/'common/data_prepare'))
    sys.argv=['prepare_train_data_VIVID.py',str(ASSETS/'raw'),'--dump-root',str(ASSETS/'prepared'),
              '--width','320','--height','256','--with-depth','--with-pose','--num-threads','1']
    spec=importlib.util.spec_from_file_location('author_prepare',AUTHOR/'common/data_prepare/prepare_train_data_VIVID.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from VIVID_raw_loader import VIVIDRawLoader
    loader=VIVIDRawLoader(AuthorPath(ASSETS/'raw'),img_height=256,img_width=320,get_depth=True,get_pose=True)
    assert tuple(loader.outdoor_test_list)==SEQUENCES
    module.args.dump_root=AuthorPath(ASSETS/'prepared')
    assert not (ASSETS/'prepared').exists(),'Refuse to overwrite prepared data'
    module.args.dump_root.makedirs_p();module.data_loader=loader
    for scene in SEQUENCES:module.dump_example(module.args,AuthorPath(ASSETS/'raw'/scene))
    write(ROOT/'results/thermal-sfm-reproduction/intake/preparation.json',{
        'source_functions':['VIVIDRawLoader','dump_example'], 'selected_scenes':SEQUENCES,
        'selection':'author cumulative-speed threshold 0.15; unchanged',
        'omissions':'indoor-only well-lit extraction and training split publication not invoked',
        'no_inference_or_accuracy_scoring':True})


def verify():
    plan=json.loads(PLAN.read_text())
    roots={'author':AUTHOR,'assets':ASSETS,'workspace':ROOT}
    for key,digest in plan['pins'].items():
        kind,name=key.split('/',1)
        assert sha(roots[kind]/name)==digest,key


def run(kind,scene):
    verify();compatibility();os.chdir(AUTHOR)
    dest=OUT/scene/kind;assert not dest.exists(),'Refuse existing scientific output'
    dest.mkdir(parents=True)
    if kind=='depth':
        script=AUTHOR/'test_disp.py'
        args=['--pretrained-dispnet',str(ASSETS/'dispnet_disp_model_best.pth.tar'),
              '--dataset-dir',str(ASSETS/'prepared'/scene)+'/']
    else:
        script=AUTHOR/'test_pose.py'
        args=['--pretrained-posenet',str(ASSETS/'exp_pose_pose_model_best.pth.tar'),
              '--dataset-dir',str(ASSETS/'prepared'),'--sequences',scene]
    sys.argv=[str(script),*args,'--resnet-layers','18','--img-height','256','--img-width','320',
              '--input','T','--scene_type','outdoor','--output-dir',str(dest)]
    runpy.run_path(str(script),run_name='__main__')
    verify()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','depth','pose']);p.add_argument('--scene',choices=SEQUENCES);a=p.parse_args()
    if a.action=='prepare':prepare()
    else:assert a.scene;run(a.action,a.scene)
