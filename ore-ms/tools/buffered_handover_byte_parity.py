"""Strict logical-cell bit-pattern verification, independent of value/NaN equality."""
import json
from pathlib import Path
import numpy as np
from buffered_handover_study import ROOT,OUT,sha,write

def main():
    pairs=[]
    for name,old in [('unperturbed','unperturbed'),('fixture_off','readiness_probe')]:
        for camera in ['rgb','thermal']:
            pairs.append((name+'/'+camera,OUT/'captures'/name/'experts'/camera,
                          ROOT/'results/readiness-continuation/run-001/captures'/old/'experts'/camera))
    results={};failures=[];count=0
    for name,a,b in pairs:
        aa={str(p.relative_to(a)):p for p in a.rglob('*.npz')}
        bb={str(p.relative_to(b)):p for p in b.rglob('*.npz')}
        assert aa.keys()==bb.keys();arrays=0
        for path in aa:
            with np.load(aa[path],allow_pickle=False) as x,np.load(bb[path],allow_pickle=False) as y:
                assert set(x.files)==set(y.files)
                for key in x.files:
                    arrays+=1
                    if x[key].dtype!=y[key].dtype or x[key].shape!=y[key].shape or x[key].tobytes(order='C')!=y[key].tobytes(order='C'):
                        failures.append({'case':name,'file':path,'key':key})
        results[name]={'compared_arrays':arrays};count+=arrays
    result={'status':'PASS_STRICT_ARRAY_BIT_PATTERNS' if not failures else 'FAIL_STRICT_ARRAY_BIT_PATTERNS',
            'compared_arrays':count,'cases':results,'mismatches':failures,
            'scope':'dtype, shape and every logical-cell byte; independent of NPZ container timestamps/compression',
            'source_sha256':sha(__file__)}
    write(OUT/'byte-parity.json',result);print(json.dumps(result,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
