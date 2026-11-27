#!/usr/bin/env python3
"""Read-only offline package/integrity validator; Git reads only, no runtime attach."""
import ast
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'RRM/evidence/M00-E1A';V=ROOT/'RRM/verification/reports/M00-E1A'
PARENT='31e33893ef2fa0f724862a980f6b5db0d62509d2'
def must(ok,reason):
    if not ok:raise ValueError(reason)
def sha(data):return hashlib.sha256(data).hexdigest()
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT)
def load(p):return json.loads(p.read_text())
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def module(name,path):
    s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def validate():
    checks=[]
    def record(name,detail):checks.append(dict(check=name,result='PASS',detail=detail))
    before=load(E/'repository-before.json')
    must(before['head']==PARENT and before['branch']=='ore_proj' and before['status_porcelain']=='','Starting identity discrepancy')
    must(git('branch','--show-current').decode().strip()=='ore_proj','Wrong branch')
    head=git('rev-parse','HEAD').decode().strip();committed=head!=PARENT
    if committed:
        must(git('rev-parse','HEAD^').decode().strip()==PARENT,'Unexpected parent')
        must(git('log','-1','--format=%s').decode().strip()=='RRM-F00 M00-E1A: close Tier B authorization prerequisites','Unexpected commit subject')
    record('repository_start_and_branch','Pinned clean E1 parent and ore_proj; no reset or branch change')
    for name,identity in before['tracked_entries'].items():
        if name=='RRM/PROJECT_STATUS.md':continue
        p=ROOT/name
        if identity['type']=='file':must(p.is_file() and not p.is_symlink() and sha(p.read_bytes())==identity['sha256'] and p.stat().st_mode&0o777==identity['mode'],'Protected file changed: '+name)
        elif identity['type']=='symlink':must(p.is_symlink() and os.readlink(p)==identity['target'],'Protected symlink changed: '+name)
        else:
            must(subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD']).decode().strip()==identity['head'],'Submodule changed: '+name)
            must(not subprocess.check_output(['git','-C',str(p),'status','--porcelain']),'Submodule dirty: '+name)
    for manifest in ['source-manifest.json','input-inventory.json']:
        for item in load(E/manifest)['files']:
            data=(ROOT/item['path']).read_bytes()
            must(sha(data)==item['sha256'] and len(data)==item['bytes'],'Source/input changed: '+item['path'])
    inventory=load(E/'input-inventory.json')['files']
    must({i['path'] for i in inventory}=={str(p.relative_to(ROOT)) for p in (ROOT/'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()},'Input file set changed')
    record('protected_repository_and_inputs',f"{len(before['tracked_entries'])-1} prior tracked entries unchanged except authorized status; symlinks/submodules/modes, sources and full controlled input preserved")
    pins={'M00':'e772605ccfb6c3bfe515e8e07961cdddc0831968','M00-B':'93b8196e835effc3da34cac5677eab4d30c1378a','M00-C':'b4ac9467b073b6ae7927a47b939f46e9407cb6c0','M00-D':'5b11d4d348db399e0e3ae638e318da9d7079ace6','M00-E1':PARENT}
    total=0
    for milestone,pin in pins.items():
        path=ROOT/f'RRM/evidence/{milestone}/SHA256SUMS'
        if not path.exists():continue
        for line in path.read_text().splitlines():
            digest,name=line.split('  ',1);must(sha(git('show',pin+':'+name))==digest,'Pinned historical mismatch: '+name);total+=1
    must(not load(E/'starting-integrity.json')['starting_discrepancies'],'Starting checksum discrepancy')
    record('historical_evidence_integrity',f'{total} sealed manifest entries pass at pinned milestone Git commits including historical status snapshots')
    pre=rows(E/'M00E1A_PREREQUISITE_REGISTER.csv');cfg=rows(V/'M00E1A_CONFIGURATION_CLOSURE.csv')
    must({r['prerequisite_id'] for r in pre}=={'CHK%02d'%i for i in range(1,21)} and len(pre)==20,'Prerequisite IDs incomplete/duplicate')
    must({r['status'] for r in pre}<={'SATISFIED','UNSATISFIED','UNVERIFIED'},'Invalid prerequisite status')
    must({r['prerequisite_id'] for r in pre if r['status']=='SATISFIED'}=={'CHK17','CHK18','CHK20'},'Unsupported satisfied control')
    for r in pre:
        must(all(r[k] for k in ['observed_evidence','evidence_source','owner','closure_condition','active_verification_authority_needed']),'Missing evidence/owner/closure: '+r['prerequisite_id'])
        must(r['execution_authorized']=='NO','Prerequisite implies authority')
    must({r['status'] for r in cfg}=={'VERIFIED','UNVERIFIED','MISMATCH','NOT APPLICABLE'},'Configuration classifications incomplete')
    for r in cfg:
        must(all(r[k] for k in ['scope','expected_value','observed_value','evidence_source','closure_action','owner']),'Missing configuration traceability')
        must(not(r['scope']=='TARGET' and r['status']=='VERIFIED'),'Absent target falsely verified')
    record('prerequisite_and_configuration_dispositions',f'{len(cfg)} configuration items with scope/source/expected/observed/action;20 prerequisites, counts '+json.dumps(dict(Counter(r['status'] for r in pre)))+'; only document checks satisfied')
    instrument=rows(V/'M00E1A_INSTRUMENTATION_READINESS.csv')
    must(len(instrument)==9 and len({r['measurement_id'] for r in instrument})==9,'Missing measurements')
    must({x for r in instrument for x in r['instrumentation_ids'].split(';')}=={'I%02d'%i for i in range(1,7)},'Missing I01-I06')
    must({x for r in instrument for x in r['hypothesis_ids'].split(';')}=={'HD%02d'%i for i in range(1,9)},'Missing HD01-HD08')
    for r in instrument:
        must(all(r[k] for k in ['source','clock_domain','collection_method','expected_artifact','known_limitation','closure_condition','separate_authorization']),'Measurement trace incomplete')
        must(r['live_acquisition_status']=='UNVERIFIED_NOT_INSTALLED_OR_VALIDATED_ON_TARGET','False acquisition readiness')
    replay=module('e1areplay',ROOT/'RRM/scripts/m00e1a_analyze.py');actual=replay.analyze(load(replay.SOURCE))
    must(actual==load(E/'offline-timing-replay.json'),'Offline replay not reproducible')
    must({r['measurement_id']:r['seconds'] for r in actual['measurements']}=={'L11':'19.276162179','L03':'74.228771713'},'Historical interval mismatch')
    must(not actual['execution_authorized'] and not actual['acquisition_readiness_verified'] and actual['new_startup_trials']==0,'False readiness/authority inference')
    record('instrumentation_and_independent_timing','Nine measurement contracts map to all six instruments/eight hypotheses; independent raw Decimal replay agrees exactly; no live acquisition/clock/overhead proof inferred')
    old=rows(ROOT/'RRM/verification/reports/M00-E1/M00E1_TRIAL_MATRIX.csv');new=rows(V/'M00E1A_FINAL_TRIAL_MATRIX.csv')
    must(len(old)==len(new)==24,'Trial count changed')
    config_ids={r['configuration_id'] for r in cfg}
    for a,b in zip(old,new):
        must(all(b[k]==v for k,v in a.items()),'Original E1 schedule/protocol field changed')
        must(set(b['configuration_closure_ids'].split(';'))<=config_ids,'Unknown trial configuration')
        for k in ['controlled_variables','observed_variables','startup_operations','shutdown_operations','e1a_safety_preconditions','abort_conditions','abort_reference','evidence_artifacts','interpretation_limit']:
            must(b[k],'Missing trial field '+k)
        must(set(b['abort_conditions'].split(';'))=={'AB%02d'%i for i in range(1,9)},'Incomplete abort mapping')
        must(b['execution_status']=='NOT_EXECUTED' and b['authorization']=='SEPARATE_PD_AUTHORIZATION_REQUIRED','Trial falsely authorized/executed')
    must([r['trial_id'] for r in new if r['initial_set_role']=='MINIMAL_INITIAL_PAIR_CHECKPOINT_PROPOSED']==['M00E2-TB-001','M00E2-TB-002'],'Initial checkpoint not original pair prefix')
    record('trial_matrix_continuity','Every original field in24 E1 rows preserved;2x2x6 and order unchanged; added per-trial configuration/control/start-stop/abort/artifact/limit fields; first pair proposed checkpoint only')
    decision=load(E/'pending-decision.json');sheet=(V/'M00E1A_PD_DECISION_SHEET.md').read_text()
    must(decision['package_state']=='PREPARED — NOT AUTHORIZED FOR EXECUTION' and decision['recommendation']=='NOT READY — TECHNICAL' and decision['execution_authorized'] is False,'Invalid authority/recommendation')
    must(all(v is None for k,v in decision.items() if k.startswith(('pd_','approved_'))),'Fabricated decision fields')
    must(sheet.count('**NOT READY — TECHNICAL**')==1,'Decision recommendation not singular')
    for token in ['PREPARED — NOT AUTHORIZED FOR EXECUTION','External blockers','UNVERIFIED ISOLATION','Approval record','prerequisite-closure']:
        must(token.lower() in sheet.lower(),'Decision missing '+token)
    safety=(V/'M00E1A_SAFETY_READINESS.md').read_text();isolation=(V/'M00E1A_ISOLATION_ASSURANCE.md').read_text()
    for token in ['AB%02d'%i for i in range(1,9)]+['AUTH-PREP-01','AUTH-INST-01','AUTH-REC-01','Owner','closure','Recovery','failed/aborted/incomplete']:
        must(token.lower() in safety.lower(),'Missing safety control '+token)
    must('UNVERIFIED ISOLATION' in isolation and 'Internal=false' in isolation and 'privileged=true' in isolation,'Unsupported isolation statement')
    record('safety_and_pd_decision','UNVERIFIED target isolation with explicit physical-path blockers; eight abort conditions/owners/recovery; one NOT READY technical recommendation plus external blockers, blank PD fields and separate narrow future authority requests')
    old=rows(ROOT/'RRM/evidence/M00-E1/M00E1_BLOCKER_DISPOSITION.csv');new=rows(E/'M00E1A_BLOCKER_DISPOSITION.csv')
    must(len(new)==len(old)==10,'Original blockers lost')
    for a,b in zip(old,new):must(all(b[k]==v for k,v in a.items()) and b['closed']=='NO','Original blocker fields modified/closed')
    status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
    for token in ['M00-E1A',PARENT,'PENDING PD REVIEW','WP01 NOT AUTHORIZED','BLOCKED BY EXTERNAL INPUT','28 in-scope NOT_TESTED','1 OUT_OF_SCOPE','NOT READY — TECHNICAL','prerequisite-closure','M00-E2 is not recommended as immediate next milestone']:
        must(token in status,'Status continuity missing '+token)
    must(sha((ROOT/'RRM/ROADMAP_MANDATE.md').read_bytes())==before['tracked_entries']['RRM/ROADMAP_MANDATE.md']['sha256'],'Roadmap changed')
    record('g0_roadmap_and_status','All ten original blockers and fields remain open; RA01 pending; WP01 unauthorized; coverage unchanged29/28/1; immediate next proposal is prerequisite closure; roadmap unchanged')
    a=load(E/'passive-before.json');b=load(E/'passive-after.json')
    must(a['containers']==b['containers'] and a['images']==b['images'] and a['networks']==b['networks'],'Passive reference metadata changed')
    for capture in [a,b]:must(all(c['exit_code']==0 for c in capture['commands']),'Passive query failed')
    audit=load(E/'scope-audit.json');must(audit['active_operations_performed']==[] and audit['tier_b_starts']==0 and audit['tier_c_operations']==0,'Active operation recorded')
    tree=ast.parse((ROOT/'RRM/scripts/m00e1a_analyze.py').read_text());imports={n.names[0].name for n in ast.walk(tree) if isinstance(n,ast.Import)}|{n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    must(imports<={'argparse','datetime','decimal','json','pathlib','re'},'Replay tool includes runtime API')
    record('passive_only_scope','Reference lifecycle/config/image/network metadata unchanged before/after; inspect command exit codes0; operator audit no active operations. Does not prove absence of unobserved external activity or certify isolation')
    paths=(E/'M00E1A_CHANGED_FILES.txt').read_text().splitlines();must(paths==sorted(set(paths)),'Changed paths not unique/sorted')
    must(all(p.startswith('RRM/') and not p.startswith('RRM/RRM-C001-F00-EXT/') for p in paths),'Out-of-scope path')
    must(set(paths)&set(before['tracked_entries'])=={'RRM/PROJECT_STATUS.md'},'Prior file improperly included')
    if committed:
        must(set(git('diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines())==set(paths),'Committed file list mismatch')
        must(not git('status','--porcelain'),'Postcommit worktree dirty')
    else:
        tracked=set(git('ls-files').decode().splitlines());untracked=set(git('ls-files','--others','--exclude-standard').decode().splitlines())
        must((tracked-set(before['tracked_entries']))|untracked==set(paths)-{'RRM/PROJECT_STATUS.md'},'Unexpected added/missing files')
    staged=set(git('diff','--cached','--name-only').decode().splitlines());must(not staged or staged==set(paths),'Unexpected staged paths')
    excluded={str((E/f).relative_to(ROOT)) for f in ['M00E1A_EVIDENCE_MANIFEST.json','SHA256SUMS','verification-results.json']}
    manifest=load(E/'M00E1A_EVIDENCE_MANIFEST.json');must({r['path'] for r in manifest['files']}==set(paths)-excluded,'Manifest coverage mismatch')
    for r in manifest['files']:
        data=(ROOT/r['path']).read_bytes();must(sha(data)==r['sha256'] and len(data)==r['bytes'],'Manifest mismatch: '+r['path'])
    hashed=set()
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,p=line.split('  ',1);hashed.add(p);must(sha((ROOT/p).read_bytes())==digest,'Checksum mismatch: '+p)
    must(hashed==set(paths)-{str((E/f).relative_to(ROOT)) for f in ['SHA256SUMS','verification-results.json']},'Checksum coverage mismatch')
    for p in paths:
        if p.endswith(('.md','.py')):
            must(all(line==line.rstrip() for line in (ROOT/p).read_text().splitlines()),'Trailing whitespace in '+p)
    for args in [('diff','--check'),('diff','--cached','--check')]:subprocess.run(['git',*args],cwd=ROOT,check=True)
    record('package_hashes_file_set_and_staging',f'{len(paths)} exact RRM-only changes; prior status sole mutation; explicit manifests/hashes/file sets and whitespace checks pass; result digest excluded to avoid recursion, final Git protects bytes')
    return dict(milestone='M00-E1A',result='PASS',recommendation=decision['recommendation'],execution_authorized=False,trials_executed=0,checks=checks,limitations=['Passive reference metadata is not an assigned target, complete physical/network/mission isolation proof or evidence of unobserved external actions.','Offline parser/schema/design validation does not establish live acquisition, clock mapping, overhead, supervisor/stop survival or qualification.','Active simulator/service/ROS/network/isolation/fault/start-stop checks not run because this directive authorizes none; see command-results.json for skipped checks.','Original frozen records/approval chains and exact future configuration/operator/limits remain external dependencies; no push or remote verification.'])
if __name__=='__main__':print(json.dumps(validate(),indent=2))
