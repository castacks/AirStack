"""Fresh native parity gate before any radiometric accuracy arm."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'MAC-VO-main'))
OUT = ROOT/'results/radiometric-native/run-001'
INPUT = ROOT/'results/unweighted-longer/run-001/inputs'


def verify():
    freeze=json.loads((ROOT/"experiments/radiometric-native-parity-freeze-2026-11-12.json").read_text())
    for name,digest in freeze["pins"].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name


def capture(condition, arm):
    verify()
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    from radiometric_native_hook import install, passthrough, raw_provider, tensor_hook
    args = SimpleNamespace(dataset=ROOT, manifest=INPUT/condition/'paired-manifest.json',
        output=OUT/condition/arm, frames=100, start=0, acknowledge_provisional_geometry=True,
        record_uncertainty=True, record_arbiter_events=True, record_root_initialization=False)
    receipts = []
    with adapted_runner() as runner:
        if arm in ('passthrough', 'refined'):
            callback = passthrough if arm == 'passthrough' else tensor_hook(
                raw_provider(ROOT, INPUT/condition/'source-manifest.json'), receipts)
            with install(runner, callback): runner.run(args, 'thermal')
        else:
            runner.run(args, 'rgb' if arm == 'rgb' else 'thermal')
    verify()
    if arm == 'refined':
        (OUT/condition/arm/'refinement-receipts.json').write_text(json.dumps(receipts, allow_nan=False))


def parity(condition):
    base = OUT/condition/'original/thermal'
    other = OUT/condition/'passthrough/thermal'
    files = sorted(p.relative_to(base) for p in base.rglob('*.npz'))
    assert files and set(files)=={p.relative_to(other) for p in other.rglob('*.npz')}
    count = 0
    for name in files:
        with np.load(base/name, allow_pickle=False) as a, np.load(other/name, allow_pickle=False) as b:
            assert set(a.files)==set(b.files),str(name)
            for key in a.files:
                assert a[key].dtype==b[key].dtype and a[key].shape==b[key].shape,(name,key)
                assert a[key].tobytes()==b[key].tobytes(),(name,key,'bitwise mismatch')
                count+=1
    for path in [base, other]:
        assert json.loads((path/'report.json').read_text())['status']=='PASS_EXECUTION_ONLY'
    result={'status':'PASS_ALL_EXPORTED_NPZ_BITWISE','arrays':count,'files':len(files),'condition':condition}
    (OUT/condition/'parity.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['capture','parity']);p.add_argument('condition',choices=['day','night','rain']);p.add_argument('--arm',choices=['original','passthrough','refined','rgb'])
    args=p.parse_args()
    if args.action=='capture':
        if args.arm is None:p.error('--arm required')
        if args.arm in ('refined','rgb'):
            for c in ['day','night','rain']:
                assert json.loads((OUT/c/'parity.json').read_text())['status']=='PASS_ALL_EXPORTED_NPZ_BITWISE'
            raise RuntimeError('Accuracy execution freeze required before these arms')
        capture(args.condition,args.arm)
    else:parity(args.condition)
