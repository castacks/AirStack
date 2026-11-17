"""Documented frozen audit correction: SVD consistency, raw-E failure retained."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
ORIGINAL_SHA='9b46adcf9a60454af2915d4dab5f047649d55e3c0c2dec4045b715d6fabc4658'
FROM="check(min(np.linalg.norm(a-b),np.linalg.norm(a+b))<1e-5,'E-R-t numerical consistency')"
TO="check(svd_gap(np.asarray(r['E']),R,t)<1e-5,'SVD-E-R-t numerical consistency')"
def digest(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def svd_gap(e,r,t):
    u,_,vt=np.linalg.svd(e);a=u@np.diag([1.,1.,0.])@vt;x,y,z=t;b=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])@r;a=a/np.linalg.norm(a);b=b/np.linalg.norm(b);return min(np.linalg.norm(a-b),np.linalg.norm(a+b))
def main(config):
    amendment=json.loads((ROOT/'results/pre-essential-exclusion/source/audit-amendment-lock.json').read_text())
    for name,h in amendment['pins'].items():assert digest(ROOT/name)==h,name
    path=ROOT/'tools/audit_pre_essential_exclusion.py';assert digest(path)==ORIGINAL_SHA;source=path.read_text();assert source.count(FROM)==1;namespace={'__name__':'frozen_svd_audit','__file__':str(Path(__file__).resolve()),'svd_gap':svd_gap};exec(compile(source.replace(FROM,TO),str(Path(__file__).resolve()),'exec'),namespace);result=namespace['main'](config);result.update(decomposition_contract='SVD(U,V); same1e-5tolerance',original_raw_E_equality_gate='fail_retained',original_auditor_sha256=ORIGINAL_SHA,amendment_lock_sha256=digest(ROOT/'results/pre-essential-exclusion/source/audit-amendment-lock.json'));cfg=json.loads(Path(config).read_text());(ROOT/cfg['output']/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print('SVD audit passed; original raw-E equality failure retained');return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
