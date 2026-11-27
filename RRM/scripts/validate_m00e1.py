#!/usr/bin/env python3
"""Read-only offline E1 package validator. Never attaches to a runtime."""
import ast
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'RRM/evidence/M00-E1'
V=ROOT/'RRM/verification/reports/M00-E1'
PARENT='5b11d4d348db399e0e3ae638e318da9d7079ace6'

def sha(data):return hashlib.sha256(data).hexdigest()
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT)
def readcsv(path):return list(csv.DictReader(path.open()))
def load(path):return json.loads(path.read_text())
def must(condition,message):
    if not condition:raise ValueError(message)

def validate():
    results=[]
    def record(name,detail):results.append(dict(check=name,result='PASS',detail=detail))
    before=load(E/'repository-before.json')
    must(before['head']==PARENT and before['branch']=='ore_proj' and before['status_porcelain']=='','Incorrect/dirty starting checkout')
    must(git('branch','--show-current').decode().strip()=='ore_proj','Wrong branch')
    head=git('rev-parse','HEAD').decode().strip()
    committed=head!=PARENT
    if committed:
        must(git('rev-parse','HEAD^').decode().strip()==PARENT,'Unexpected commit parent')
        must(git('log','-1','--format=%s').decode().strip()=='RRM-F00 M00-E1: prepare Tier B reproduction authorization','Unexpected commit subject')
    record('starting_checkout_and_branch','Clean pinned M00-D start; current branch ore_proj; precommit parent or direct E1 child')
    for path,identity in before['tracked_entries'].items():
        if path=='RRM/PROJECT_STATUS.md':continue
        p=ROOT/path
        if identity['type']=='file':
            must(p.is_file() and not p.is_symlink() and sha(p.read_bytes())==identity['sha256'] and p.stat().st_mode&0o777==identity['mode'],'Protected file changed: '+path)
        elif identity['type']=='symlink':must(p.is_symlink() and os.readlink(p)==identity['target'],'Protected symlink changed: '+path)
        else:
            actual=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD']).decode().strip()
            must(actual==identity['head'],'Submodule HEAD changed: '+path)
            must(not subprocess.check_output(['git','-C',str(p),'status','--porcelain']),'Submodule dirty: '+path)
    record('protected_repository_identities','All 1997 prior tracked entries other than authorized PROJECT_STATUS retain bytes/mode/symlink/submodule identity; submodules clean')
    for manifest,key in [(E/'source-manifest.json','sources'),(E/'input-inventory.json','files')]:
        for item in load(manifest)[key]:
            data=(ROOT/item['path']).read_bytes()
            must(len(data)==item['bytes'] and sha(data)==item['sha256'],'Source/input changed: '+item['path'])
    record('reviewed_sources_and_controlled_input','Source identities and complete supplied input inventory match; input remains unstaged/ignored')
    # Each sealed milestone gets its own status snapshot, rather than mutable working-tree status.
    pins={'M00':'e772605ccfb6c3bfe515e8e07961cdddc0831968','M00-B':'93b8196e835effc3da34cac5677eab4d30c1378a','M00-C':'b4ac9467b073b6ae7927a47b939f46e9407cb6c0','M00-D':PARENT}
    count=0
    for milestone,pin in pins.items():
        manifest=ROOT/f'RRM/evidence/{milestone}/SHA256SUMS'
        if not manifest.exists():continue
        for line in manifest.read_text().splitlines():
            digest,path=line.split('  ',1)
            must(sha(git('show',pin+':'+path))==digest,'Historical pinned checksum mismatch: '+path)
            count+=1
    record('historical_pinned_manifests',f'{count} prior manifest entries verified at their milestone Git commits, including original status snapshots')
    integrity=load(E/'starting-integrity.json')
    must(not integrity['starting_discrepancies'],'Starting integrity discrepancy')
    spec=importlib.util.spec_from_file_location('plan',ROOT/'RRM/scripts/m00e1_plan.py');plan=importlib.util.module_from_spec(spec);spec.loader.exec_module(plan)
    must(plan.REPORT.read_text()==plan.render(),'Schedule differs from deterministic proposal')
    decision=plan.prepared_decision_valid(load(E/'M00E1_PENDING_DECISION.json'))
    must(decision['execution_authorized'] is False,'Inferred authorization')
    rows=readcsv(plan.REPORT)
    checks=readcsv(E/'M00E1_READINESS_CHECKLIST.csv');ops=readcsv(V/'M00E1_ACTIVE_OPERATIONS.csv')
    must(len(checks)==18 and len(ops)==7,'Incomplete registers')
    for row in rows:
        for column,key,items in [('required_preconditions','check_id',checks),('active_operation_ids','operation_id',ops)]:
            must(set(row[column].split(';'))=={x[key] for x in items},'Broken trial register reference')
        must(row['objective'] and row['required_observations'] and row['execution_status']=='NOT_EXECUTED','Missing objective/observation or executed trial')
    must(all(x['authorization']=='SEPARATE_PD_AUTHORIZATION_REQUIRED' and x['performed_in_m00e1']=='NO' for x in ops),'Active operation implies approval/performance')
    must(all(x['execution_pass']=='NOT_ASSESSED' for x in checks),'Prerequisite falsely execution passed')
    must({x['check_id'] for x in checks if x['status']=='VERIFIED_DOCUMENT'}=={'CHK17','CHK18'},'Target controls falsely verified')
    cfg=readcsv(V/'M00E1_CONFIGURATION_REGISTER.csv')
    must({x['status'] for x in cfg}=={'VERIFIED','PROPOSED','UNKNOWN','REQUIRES_APPROVAL'},'Configuration status distinction missing')
    must(not any(x['status']=='VERIFIED' and x['scope']=='TARGET' for x in cfg),'Uninspected target incorrectly verified')
    record('design_authority_and_register_links','24 frozen exploratory trials; objectives/observations, all 18 prerequisites/7 separately unauthorized operations; explicit reference/target uncertainty; pending decision cannot grant authority')
    old=readcsv(ROOT/'RRM/evidence/M00-D/M00D_BLOCKER_DISPOSITION.csv');new=readcsv(E/'M00E1_BLOCKER_DISPOSITION.csv')
    must(len(old)==len(new)==10,'Original blocker denominator changed')
    for a,b in zip(old,new):
        must(all(b[k]==v for k,v in a.items()) and b['closed']=='NO','Prior blocker fields altered or closed')
    status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
    for token in ['M00-E1','M00-D',PARENT,'BLOCKED BY EXTERNAL INPUT','RA01','PENDING PD REVIEW','WP01 NOT AUTHORIZED','28 in-scope NOT_TESTED','1 OUT_OF_SCOPE']:
        must(token in status,'Status missing continuity: '+token)
    must(sha((ROOT/'RRM/ROADMAP_MANDATE.md').read_bytes())==before['tracked_entries']['RRM/ROADMAP_MANDATE.md']['sha256'],'Roadmap changed')
    record('g0_and_roadmap_continuity','All original blocker fields/closures preserved; RA01 pending; WP01 unauthorized; roadmap unchanged; coverage 29/28/1 with zero qualification')
    documents=[V/'M00E1_INSTRUMENTATION_SPEC.md',V/'M00E1_SAFETY_ISOLATION_REVIEW.md',V/'M00E1_AUTHORIZATION_REQUEST.md',ROOT/'RRM/verification/procedures/M00E1_TIER_B_EXECUTION_PROCEDURE.md',ROOT/'RRM/handoffs/M00E1_HANDOFF.md']
    for path in documents:
        text=path.read_text()
        must('PREPARED — NOT AUTHORIZED FOR EXECUTION' in text,'Missing package authority boundary: '+str(path))
    instrument=documents[0].read_text()
    for x in ['I%02d'%n for n in range(1,7)]+['HD%02d'%n for n in range(1,9)]+['19.276162179','74.228771713','clock','drop']:
        must(x in instrument,'Instrumentation gap '+x)
    procedure=documents[3].read_text().lower();safety=documents[1].read_text().lower()
    for token in ['failed','aborted','incomplete','no replacement','sha-256','operator','authorization','clock','cache','no numerical acceptance threshold','medians','six','24']:
        must(token in procedure,'Procedure missing '+token)
    for token in ['abort','physical','privileged','unverified','interruption','recovery','mission','arming','motion']:
        must(token in safety,'Safety review missing '+token)
    record('instrumentation_safety_retention_and_criteria','I01-I06/HD01-HD08 clock/event/resource coverage, explicit missing target proof, abort/recovery, unsuccessful-attempt retention and exploratory analysis; document coverage checks plus engineering review, not runtime validation')
    tree=ast.parse((ROOT/'RRM/scripts/m00e1_plan.py').read_text())
    imports={n.names[0].name.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.Import)}|{n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    must(imports<={'argparse','csv','io','json','pathlib','random'},'Schedule tool includes runtime API')
    audit=load(E/'scope-audit.json')
    must(audit['active_operations_performed']==[] and audit['tier_b_starts']==0 and audit['tier_c_operations']==0,'Active operation recorded')
    start=load(E/'passive-environment.json');end=load(E/'passive-environment-after.json')
    a=[json.loads(x) for x in start['commands'][0]['stdout'].splitlines()]
    b=[json.loads(x) for x in end['command']['stdout'].splitlines()]
    must(a==b,'Reference metadata changed; review scope and environment')
    record('no_active_execution_evidence','Offline tool has no runtime API; operator action audit records no active operations; passive reference IDs/start times/restart counts/settings unchanged. Not proof of unobserved external actions or target isolation')
    changed=(E/'M00E1_CHANGED_FILES.txt').read_text().splitlines()
    must(len(changed)==len(set(changed)) and changed==sorted(changed),'Changed list not unique/sorted')
    must(all(p.startswith('RRM/') and not p.startswith('RRM/RRM-C001-F00-EXT/') for p in changed),'Out-of-scope paths')
    expected_old=set(before['tracked_entries'])
    must({p for p in changed if p in expected_old}=={'RRM/PROJECT_STATUS.md'},'Unauthorized existing file in package')
    tracked=set(git('ls-files').decode().splitlines())
    untracked=set(git('ls-files','--others','--exclude-standard').decode().splitlines())
    if committed:
        must(set(git('diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines())==set(changed),'Committed file set mismatch')
        must(not git('status','--porcelain'),'Postcommit tree dirty')
    else:
        must((tracked-expected_old)|untracked==set(changed)-{'RRM/PROJECT_STATUS.md'},'Unexpected new tracked/untracked file')
    staged=set(git('diff','--cached','--name-only').decode().splitlines())
    must(not staged or staged==set(changed),'Index includes missing/extra paths')
    for item in load(E/'M00E1_EVIDENCE_MANIFEST.json')['files']:
        data=(ROOT/item['path']).read_bytes()
        must(len(data)==item['bytes'] and sha(data)==item['sha256'],'E1 manifest mismatch: '+item['path'])
    manifest=load(E/'M00E1_EVIDENCE_MANIFEST.json')
    excluded={str(E.relative_to(ROOT)/x) for x in ['M00E1_EVIDENCE_MANIFEST.json','SHA256SUMS','verification-results.json']}
    must({i['path'] for i in manifest['files']}==set(changed)-excluded,'Manifest coverage mismatch')
    sums={}
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,path=line.split('  ',1);sums[path]=digest
        must(sha((ROOT/path).read_bytes())==digest,'E1 checksum mismatch: '+path)
    must(set(sums)==set(changed)-{str(E.relative_to(ROOT)/'SHA256SUMS'),str(E.relative_to(ROOT)/'verification-results.json')},'Checksum coverage mismatch')
    subprocess.run(['git','diff','--check'],cwd=ROOT,check=True)
    subprocess.run(['git','diff','--cached','--check'],cwd=ROOT,check=True)
    record('package_manifest_and_git_scope',f'{len(changed)} exact RRM-only paths; explicit file-set coverage, hashes and diff whitespace checks; verification-results is detached from hashes to avoid recursive digest')
    return dict(milestone='M00-E1',result='PASS',execution_authorized=False,trials_executed=0,checks=results,limitations=['Offline design/integrity checks do not establish target safety, instrumentation acquisition, runtime behavior or qualification.','No live trials or synthetic ROS tests run. Pending PD decision and unresolved target prerequisites bar execution.','No external push performed; remote state not fetched.'])

if __name__=='__main__':print(json.dumps(validate(),indent=2))
