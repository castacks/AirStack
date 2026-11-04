"""Launch the image's selected worker with its packaged model/source cache."""
import json
import os
from pathlib import Path
import runpy
import sys

ROOT=Path(__file__).resolve().parents[3]


def main():
    # A direct --check-model invocation needs the same CUDA wheel libraries as
    # a worker launched by runtime_process. Re-exec before importing frameworks.
    libraries = sorted(str(p) for p in Path(sys.prefix).glob('lib/python*/site-packages/nvidia/*/lib'))
    current = os.environ.get('LD_LIBRARY_PATH', '')
    if libraries and current.split(':')[:len(libraries)] != libraries:
        env = dict(os.environ, LD_LIBRARY_PATH=':'.join([*libraries, current]).rstrip(':'))
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], env)
    manifest=json.loads((ROOT/'image_manifest.json').read_text())
    planner=manifest['planner']
    device=manifest.get('inference_device', 'cpu' if planner=='pulp_dronet' else 'cuda')
    if device not in ('cpu', 'cuda'):
        raise ValueError('Unknown inference device in image manifest')
    if planner=='kim' and device=='cpu':
        # A fixed, recorded image choice, never an automatic GPU-error fallback.
        # This environment applies only to this worker; Isaac keeps its GPU.
        os.environ['CUDA_VISIBLE_DEVICES']='-1'
    print(json.dumps({'planner':planner,'inference_device':device,
                      'runtime_selection':'fixed_image_manifest'}),flush=True)
    entries={'mononav':('MonoNav','mononav_airstack.py'),
             'kim':('Collision-avoidance','collision_avoidance_airstack.py'),
             'pulp_dronet':('pulp-dronet','pulp_dronet_airstack.py')}
    repository,entry=entries[planner]
    if planner=='mononav':
        import torch
        if '--help' not in sys.argv and not torch.cuda.is_available():
            raise RuntimeError('MonoNav image requires working CUDA; CPU fallback is not a validated OSMO run')
        cache=Path(os.environ['TORCH_HOME'])/'hub'
        sources=list(cache.glob('*MiDaS*/hubconf.py'))
        if len(sources)!=1:raise RuntimeError('Package exactly one pinned MiDaS hub source')
        original=torch.hub.load
        def load(repo_or_dir,model,*args,**kwargs):
            if repo_or_dir!='intel-isl/MiDaS':raise ValueError('Unpackaged Torch Hub source: '+str(repo_or_dir))
            kwargs.pop('force_reload',None);kwargs.pop('trust_repo',None);kwargs['source']='local'
            return original(str(sources[0].parent),model,*args,**kwargs)
        torch.hub.load=load
    if planner=='kim' and device=='cuda' and '--help' not in sys.argv:
        import tensorflow as tf
        if not tf.config.list_physical_devices('GPU'):
            raise RuntimeError('Kim image requires working CUDA; CPU fallback is not a validated OSMO run')
    script=ROOT.parent/repository/entry
    os.chdir(script.parent);sys.path.insert(0,str(script.parent))
    if '--check-model' in sys.argv:
        from check_model import check
        print(json.dumps(check(planner), allow_nan=False), flush=True)
        return
    sys.argv=[str(script),*sys.argv[1:]]
    runpy.run_path(str(script),run_name='__main__')


if __name__=='__main__':main()
