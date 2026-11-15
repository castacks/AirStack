"""Recover fixed0..1199 reference rows; previous0..599 must match byte-for-byte."""
from pathlib import Path
import json
import tarfile
import hashlib

ROOT=Path(__file__).resolve().parents[1]
TMP=Path('/tmp/thermal-intake')
OUT=ROOT/'results/fixed-rotation-continuation/recovery'


def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((TMP/'decisive-manifest.json').read_text())
    archive=TMP/'decisive.tar.gz';assert sha(archive)==manifest['archive']['sha256']
    wanted={f'ore-ms/results/decisive-uncertainty-evaluation/run-001/references/{c}/{i:06}.txt':
            (c,i) for c in ['day','night'] for i in range(1200)}
    restored=[]
    with tarfile.open(archive) as tf:
        for member in tf:
            if not member.isfile() or member.name not in wanted:continue
            c,i=wanted.pop(member.name);b=tf.extractfile(member).read();digest=hashlib.sha256(b).hexdigest()
            assert digest==manifest['member_sha256'][member.name]
            if i<600:
                old=ROOT/'results/longer-fixed-thermal/recovery/references'/c/f'{i:06}.txt'
                assert b==old.read_bytes()
            dest=OUT/'references'/c/f'{i:06}.txt';dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(b)
            restored.append({'path':str(dest.relative_to(ROOT)),'sha256':digest,'member':member.name})
    assert not wanted
    (OUT/'proof.json').write_text(json.dumps({'archive_sha256':manifest['archive']['sha256'],
        'files':restored,'previous600_reference_rows_match':True,'references_parsed_for_scoring':False},indent=2)+'\n')
    print('Recovered2400pinnedreferences;1200priorreferencebytes match.')


if __name__=='__main__':main()
