"""Evidence-backed project exposure and read-only proposed-partition audit.

Never opens data payloads or assigns a test set. Project exposure is distinct
from checkpoint membership; reported-reserved does not certify non-access.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_ledger(ledger,root):
    if ledger['schema_version']!=1:raise ValueError('unknown ledger version')
    text={}
    for name,source in ledger['sources'].items():
        path=(root/source['snapshot_relative']).resolve()
        if not path.is_relative_to(root.resolve()):raise ValueError('snapshot outside root')
        if sha(path)!=source['sha256']:raise ValueError('source snapshot hash mismatch: '+name)
        text[name]=path.read_text()
    aliases={}
    for r in ledger['records']:
        if r['exposure_status'] not in ('development_exposed','reported_reserved'):raise ValueError('unclassified record must not be assumed unused')
        for identity in [r['id'],*r['aliases']]:
            if not isinstance(identity,str) or not identity.strip() or identity in aliases:raise ValueError('ambiguous/duplicate recording identity')
            aliases[identity]=r['id']
        if not r['evidence']:raise ValueError('record lacks evidence')
        for evidence in r['evidence']:
            if not evidence['quote'] or evidence['quote'] not in text[evidence['source']]:raise ValueError('evidence quote differs from source')
        if r['exposure_status']=='development_exposed':
            if r['image_observation']!='observed' and not r['reference_observation'].startswith('observed'):raise ValueError('development exposure lacks observed payload evidence')
        else:
            if r['image_observation']!='reported_unexamined' or r['reference_observation']!='not_established_by_recording':raise ValueError('reserved exposure cannot be overstated')
            if not any('remain unexamined' in e['quote'] and r['aliases'][0] in e['quote'] for e in r['evidence']):raise ValueError('reserved status lacks explicit recording-specific source')
        if r['MACT_Offroad_training_membership']!='unknown':raise ValueError('checkpoint membership needs new source evidence, not relabelling')
    for evidence in ledger['checkpoint']['evidence']:
        if evidence['quote'] not in text[evidence['source']]:raise ValueError('checkpoint evidence mismatch')
    return aliases


def partition_audit(ledger,assignments):
    """Diagnostic only: validates identities/whole-recording split independence."""
    records={r['id']:r for r in ledger['records']};aliases={}
    for r in records.values():
        for identity in [r['id'],*r['aliases']]:
            if identity in aliases:raise ValueError('ambiguous identity')
            aliases[identity]=r['id']
    seen={};issues=[];candidates=[]
    for a in assignments:
        given,partition=a['recording_id'],a['partition']
        if partition not in ('development','validation','test'):raise ValueError('unknown partition')
        identity=aliases.get(given)
        if identity is None:
            issues.append({'recording_id':given,'reason':'unknown_recording_or_clip_alias'});continue
        if identity in seen:
            issues.append({'recording_id':identity,'reason':'recording_crosses_partitions' if seen[identity]!=partition else 'duplicate_assignment'})
        seen[identity]=partition
        r=records[identity]
        if partition in ('validation','test'):
            if r['exposure_status']=='development_exposed':issues.append({'recording_id':identity,'reason':'development_exposure_in_'+partition})
            elif r['exposure_status']=='reported_reserved':candidates.append({'recording_id':identity,'partition':partition})
            else:issues.append({'recording_id':identity,'reason':'unclassified_exposure'})
            if r['unit_kind']!='temporal_recording':issues.append({'recording_id':identity,'reason':'static_scene_cannot_validate_temporal_arbiter'})
    return {'partition_structure_valid':not issues,'issues':issues,'canonical_assignments':seen,
        'reported_reserved_candidates':candidates,'checkpoint_training_heldout_supported':False,
        'complete_prospective_evaluation_freeze':False,
        'scope':'read-only assignment diagnostic; candidates are not an executed or fully declared test set'}


def audit(ledger,root,box_metadata):
    aliases=validate_ledger(ledger,root)
    actual={r['Name'] for r in box_metadata if r['Name'].endswith('.zip')}
    known={a for r in ledger['records'] if r['dataset']=='td3' for a in r['aliases'] if a.endswith('.zip')}
    if actual!=known:raise ValueError('Box archive inventory changed; unlisted archives remain unknown')
    return {'status':'PASS_DATED_LEDGER','records':len(ledger['records']),'pinned_note_sources':len(ledger['sources']),
        'by_dataset':dict(Counter(r['dataset'] for r in ledger['records'])),
        'exposure_categories':dict(Counter(r['exposure_status'] for r in ledger['records'])),
        'td3_archive_names':sorted(actual),'td3_unexposed_in_listed_pool':0,
        'reported_reserved_ms2':[r['id'] for r in ledger['records'] if r['exposure_status']=='reported_reserved'],
        'checkpoint_training_heldout_supported':False,'scientific_payloads_opened':False,
        'unlisted_recording_status':'unknown','complete_prospective_evaluation_freeze':False}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--ledger',type=Path,required=True);p.add_argument('--box-metadata',type=Path,required=True);p.add_argument('--assignments',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();ledger=read(a.ledger);result=audit(ledger,a.root,read(a.box_metadata))
    result.update(ledger_sha256=sha(a.ledger),box_metadata_sha256=sha(a.box_metadata),auditor_sha256=sha(__file__))
    if a.assignments:result['partition_diagnostic']=partition_audit(ledger,read(a.assignments))
    with a.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))
