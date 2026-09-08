"""Compare accepted production iterations on pod 58 with the mounted NAS."""
import hashlib
import json
from pathlib import Path
import subprocess

REMOTE = r'''
import hashlib,json
from pathlib import Path
root=Path('/root/AirStack/osmo/results')
out=[]
for p in sorted(root.glob('*/*/iter_*/iteration.json')):
    if 'failed_attempt' in str(p): continue
    if not (p.relative_to(root).parts[0].startswith('remaining58_') or
            p.relative_to(root).parts[0]=='urban_fire_remaining_8robot_optimized_pod58'): continue
    if json.loads(p.read_text()).get('status')!='passed': continue
    files={str(f.relative_to(p.parent)):f.stat().st_size for f in p.parent.rglob('*') if f.is_file()}
    out.append(dict(iteration=str(p.parent.relative_to(root)),files=files,
                    metadata_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
print(json.dumps(out))
'''


def main():
    result = subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',
        '-p','2206','root@localhost','python3 -'],input=REMOTE,text=True,
        capture_output=True,check=True)
    rows=json.loads(result.stdout)
    root=Path('/media/share/coa-sei')
    for row in rows:
        base=root/row['iteration']
        row['missing_or_size_mismatch']=[]
        for name,size in row['files'].items():
            p=base/name
            if not p.is_file() or p.stat().st_size!=size:
                row['missing_or_size_mismatch'].append(name)
        meta=base/'iteration.json'
        row['metadata_matches']=meta.is_file() and hashlib.sha256(meta.read_bytes()).hexdigest()==row['metadata_sha256']
        row['verified']=not row['missing_or_size_mismatch'] and row['metadata_matches']
        print(row['iteration'], 'MATCH' if row['verified'] else 'UPLOAD_NEEDED',
              len(row['missing_or_size_mismatch']),flush=True)
    Path('osmo/results/pod58_upload_audit.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('matched',sum(r['verified'] for r in rows),'/',len(rows),flush=True)


if __name__=='__main__':
    main()
