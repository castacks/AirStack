"""Byte-preserving layout adapter for original night EXP043/044/045 payloads."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def stage(root,recovery_root,output):
    if output.exists():raise FileExistsError(output)
    snapshots={name:json.loads((recovery_root/name/'manifest.json').read_text()) for name in ['2026-10-23-exp-043','2026-10-24-exp-044-complete','2026-10-25-exp-045']}
    original=root/'results/ms2-night-pilot';manifest=root/'results/ms2-timestamp-semantics/source-protocol-input-manifest.json'
    seq=json.loads(manifest.read_text())['sequence'];assert seq=='_2021-08-13-21-18-04'
    mapping=[('source-manifest.json',manifest,'2026-10-23-exp-043'),('calibration.json',original/'calibration.json','2026-10-25-exp-045')]
    for mod in ['rgb','thr']:
        mapping.append((f'pilot/sync_data/{seq}/{mod}/img_left_timestamp.txt',original/f'pilot/sync_data/{seq}/{mod}/img_left_timestamp.txt','2026-10-25-exp-045'))
        for i in range(20):
            stem=f'{i:06d}'
            for kind,sub in [('sync_data','img_left'),('proj_depth','depth')]:
                relative=f'pilot/{kind}/{seq}/{mod}/{sub}/{stem}.png'
                mapping.append((relative,original/relative,'2026-10-23-exp-043'))
            source=root/'results/ms2-frontend-smoke'/('rgb-run-001' if mod=='rgb' else 'thermal-run-001')/f'{mod}-stereo-{stem}-{stem}.npz'
            mapping.append((f'frontend-run-001/{source.name}',source,'2026-10-24-exp-044-complete'))
    checked=[]
    for name,source,snapshot in mapping:
        origin='ore-ms/'+str(source.relative_to(root));value=digest(source);assert snapshots[snapshot]['member_sha256'][origin]==value,origin
        checked.append((name,source,snapshot,origin,value))
    output.mkdir(parents=True)
    origins={};members={}
    for name,source,snapshot,origin,value in checked:
        target=output/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target);assert digest(source)==digest(target)==value
        member='ore-ms/'+str(target.relative_to(root));members[member]=value;origins[member]={'original_snapshot':snapshot,'original_member':origin,'original_sha256':value}
    report={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'run_start_commit':'fc1434c2d','script_sha256':digest(__file__),'scope':'byte-for-byte layout normalization; composite integrity manifest, not a new original-source archive','source_manifest_semantics':'original byte copy; embedded original paths intentionally preserved as provenance; runner uses explicit staged file locations','original_manifest_sha256':{name:digest(recovery_root/name/'manifest.json') for name in snapshots},'member_sha256':members,'origin_by_member':origins}
    (output/'layout-provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Night layout staged:',len(checked),'byte-identical files, all original archive-member hashes verified.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['root','recovery-root','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();stage(a.root.resolve(),a.recovery_root,a.output)
