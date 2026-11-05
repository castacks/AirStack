"""Supplemental independent EXP142 TAR-to-extracted-reference identity closure.

Does not use the intake helper or its generated member hashes as authority.
Added after the geometric score audit; scoring and frozen predictions stay intact.
"""
import hashlib
import json
from pathlib import Path
import tarfile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/provided-reference-full-record/run-001'
PLAN=ROOT/'experiments/provided-reference-full-record-2026-11-05.json'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def same_member(name,data,expected):
    if name not in expected:raise ValueError('undeclared camera/recording/stem')
    stored=OUT/'reference'/name
    if stored.read_bytes()!=data:raise ValueError('TAR/source member mismatch')

def audit():
    declaration=read(OUT/'archive-audit-declaration.json');p=read(PLAN);lock=read(OUT/'reference-lock.json')
    assert declaration['auditor_sha256']==sha(__file__) and declaration['plan_sha256']==sha(PLAN)
    assert declaration['score_summary_sha256']==sha(OUT/'summary.json')
    checks=0;count=0;reports=[];mutations=0;all_expected=set()
    for seq in sorted({r['sequence'] for r in p['runs']}):
        runs=[r for r in p['runs'] if r['sequence']==seq]
        expected={f"{seq}/{r['modality']}/{i:06}.txt" for r in runs for i in range(r['source_frames'])}
        all_expected|={str((OUT/'reference'/n).relative_to(ROOT)) for n in expected}
        archive=OUT/'archives'/(seq+'.tar.bz2')
        assert sha(archive)==runs[0]['reference_archive_sha256'];checks+=1
        seen=set();example=None
        with tarfile.open(archive) as t:
            for m in t:
                if m.name not in expected:continue
                assert m.isfile() and m.name not in seen and m.size<=10000;checks+=1
                data=t.extractfile(m).read();same_member(m.name,data,expected);checks+=1
                rel=str((OUT/'reference'/m.name).relative_to(ROOT))
                assert hashlib.sha256(data).hexdigest()==lock['member_sha256'][rel];checks+=1
                seen.add(m.name);example=(m.name,data)
        assert seen==expected;checks+=1;count+=len(seen)
        name,data=example
        for bad_name,bad_data in [(name,data+b'0'),('reserved/rgb/000000.txt',data),(seq+'/nir/000000.txt',data)]:
            try:same_member(bad_name,bad_data,expected)
            except ValueError:mutations+=1
            else:raise ValueError('undetected source mutation')
        reports.append(dict(sequence=seq,raw_TAR_members_equal=len(seen)))
    assert all_expected==set(lock['member_sha256']) and count==55618;checks+=1
    result=dict(passed=True,independent_checks=checks,reference_members_compared_to_raw_TAR=count,
                mutations_rejected=mutations,runs=reports,auditor_sha256=sha(__file__),
                declaration_sha256=sha(OUT/'archive-audit-declaration.json'),score_or_prediction_changed=False)
    with (OUT/'archive-source-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':audit()
