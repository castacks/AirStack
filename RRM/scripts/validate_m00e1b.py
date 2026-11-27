#!/usr/bin/env python3
"""Read-only E1B package validation: local files and Git only; no runtime query."""
import ast
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1B';V=ROOT/'RRM/verification/reports/M00-E1B'
PARENT='df8ebd18a6e16909cca461e18dfba882b53c8120'
def must(ok,reason):
 if not ok:raise ValueError(reason)
def sha(data):return hashlib.sha256(data).hexdigest()
def load(p):return json.loads(p.read_text())
def rows(p):
 with p.open() as f:return list(csv.DictReader(f))
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT)
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def validate():
 checks=[]
 def record(name,detail):checks.append(dict(check=name,result='PASS',detail=detail))
 before=load(E/'repository-before.json');head=git('rev-parse','HEAD').decode().strip();committed=head!=PARENT
 must(before['head']==PARENT and before['branch']=='ore_proj' and before['status_porcelain']=='','Starting checkout mismatch')
 must(git('branch','--show-current').decode().strip()=='ore_proj','Branch changed')
 if committed:
  must(git('rev-parse','HEAD^').decode().strip()==PARENT,'Unexpected commit parent')
  must(git('log','-1','--format=%s').decode().strip()=='RRM-F00 M00-E1B: verify target environment and passive readiness','Unexpected commit subject')
 record('pinned_start_and_branch','Clean required E1A parent on ore_proj; no concealed branch/history change')
 for name,i in before['tracked_entries'].items():
  if name=='RRM/PROJECT_STATUS.md':continue
  p=ROOT/name
  if i['type']=='file':must(p.is_file() and not p.is_symlink() and sha(p.read_bytes())==i['sha256'] and p.stat().st_mode&0o777==i['mode'],'Protected file changed: '+name)
  elif i['type']=='symlink':must(p.is_symlink() and os.readlink(p)==i['target'],'Symlink changed: '+name)
  else:
   must(subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD']).decode().strip()==i['head'],'Submodule changed: '+name)
   must(not subprocess.check_output(['git','-C',str(p),'status','--porcelain']),'Dirty submodule: '+name)
 for name in ['source-manifest.json','input-inventory.json']:
  for i in load(E/name)['files']:
   data=(ROOT/i['path']).read_bytes();must(len(data)==i['bytes'] and sha(data)==i['sha256'],'Source/input changed: '+i['path'])
 must({i['path'] for i in load(E/'input-inventory.json')['files']}=={str(p.relative_to(ROOT)) for p in (ROOT/'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()},'Controlled input membership changed')
 pins={'M00':'e772605ccfb6c3bfe515e8e07961cdddc0831968','M00-B':'93b8196e835effc3da34cac5677eab4d30c1378a','M00-C':'b4ac9467b073b6ae7927a47b939f46e9407cb6c0','M00-D':'5b11d4d348db399e0e3ae638e318da9d7079ace6','M00-E1':'31e33893ef2fa0f724862a980f6b5db0d62509d2','M00-E1A':PARENT};total=0
 for milestone,pin in pins.items():
  path=ROOT/f'RRM/evidence/{milestone}/SHA256SUMS'
  if not path.exists():continue
  for line in path.read_text().splitlines():
   digest,name=line.split('  ',1);must(sha(git('show',pin+':'+name))==digest,'Pinned historical mismatch: '+name);total+=1
 record('protected_and_historical_integrity',f"{len(before['tracked_entries'])-1} prior tracked entries preserved except authorized status; full controlled input/source identities;{total} sealed checksum entries at pinned Git commits")
 old=rows(ROOT/'RRM/evidence/M00-E1A/M00E1A_PREREQUISITE_REGISTER.csv');new=rows(E/'M00E1B_PREREQUISITE_DELTA.csv')
 must(len(old)==len(new)==20,'Lost original prerequisites')
 must({r['prerequisite_id'] for r in new}=={'CHK%02d'%i for i in range(1,21)},'Wrong/duplicate original IDs')
 unresolved=0
 for a,b in zip(old,new):
  must(a['prerequisite_id']==b['prerequisite_id'] and a['prerequisite']==b['original_requirement'] and a['status']==b['previous_disposition'],'Prior requirement/disposition changed')
  must(b['result'] in {'SATISFIED','UNSATISFIED','UNVERIFIED','NOT APPLICABLE'},'Invalid disposition')
  if a['status']=='UNSATISFIED':must(b['result']=='UNSATISFIED','Unsupported downgrade/closure')
  if a['status']!='SATISFIED':unresolved+=1
  for key in ['evidence_required_for_closure','inherited_evidence','inherited_source','new_evidence','new_evidence_source','verification_method','remaining_uncertainty','responsible_party','next_closure_action']:
   must(b[key],'Incomplete delta '+key)
  classes=set(b['gap_classes'].split(';'));must(classes<={'A','B','C','D'} and classes,'Invalid gap classes')
  if 'D' in classes:must(b['safety_precedence']=='D_PRECEDES_ANY_CLASS_C_ACTIVITY','D safety restriction absent')
  must(b['whole_prerequisite_closed_in_e1b']=='NO' and b['target_designation']=='UNVERIFIED','Unsupported closure/designation')
 must(unresolved==17 and Counter(r['result'] for r in new)==Counter({'SATISFIED':3,'UNSATISFIED':8,'UNVERIFIED':9}),'Disposition count mismatch')
 record('all20_prerequisite_deltas','All17 unresolved reassessed with distinct new/inherited evidence, owner/method/uncertainty/action and overlapping A/B/C/D; no whole closure or UNSATISFIED downgrade')
 evidence=rows(E/'M00E1B_PASSIVE_EVIDENCE_INDEX.csv')
 must(len(evidence)>=9,'Missing evidence items')
 for r in evidence:
  data=(ROOT/r['path']).read_bytes();must(sha(data)==r['sha256'] and len(data)==int(r['bytes']),'Evidence index mismatch: '+r['path'])
  must(all(r[k] for k in ['evidence_kind','provenance','acquisition_method','acquisition_timestamp_utc','limitations','credentials']),'Missing provenance')
 must({'NEW_ACQUIRED_SANITIZED_PASSIVE','NEW_REPLAY_OF_INHERITED_RAW','INHERITED_RAW'}<={r['evidence_kind'] for r in evidence},'Evidence age/acquisition not distinguished')
 d=load(E/'passive-discovery.json');a=load(E/'installed-artifacts.json');h=load(E/'host-and-lifecycle.json')
 must(d['target_designation']=='UNVERIFIED' and len(d['containers'])==3,'Candidate scope mismatch')
 must(len(a['static_workflow_sources'])==4 and all(r['state']=='SOURCE_DEFINITION_ONLY_NOT_DEPLOYMENT_EVIDENCE' for r in a['static_workflow_sources']),'Workflow definition/deployment conflation')
 must(all(c['exit_code']==0 for c in d['commands']),'Passive command failure not dispositioned')
 must('not yet been formally designated' in (E/'TARGET_CLARIFICATION.md').read_text(),'Missing user clarification')
 record('new_passive_evidence_provenance','New current proc/artifact/config/device/socket/log observations plus user clarification and replay outputs indexed with byte count/hash/provenance; four source workflow definitions never counted as deployments')
 ex=rows(V/'M00E1B_EXPOSURE_REGISTER.csv');must(len(ex)==12 and len({r['exposure_id'] for r in ex})==12,'Exposure IDs missing')
 for r in ex:
  for key in ['source','destination','protocol_or_interface','pathway_state','evidence_description','evidence_source','potential_actuation_or_operational_consequence','required_mitigation','evidence_limit']:must(r[key],'Exposure trace missing '+key)
 boundary=(V/'M00E1B_ISOLATION_BOUNDARY.md').read_text()
 for token in ['```mermaid','UNVERIFIED ISOLATION','ttyS0..31','configured','possible','Class D','without opening them','no packet']:
  must(token.lower() in boundary.lower(),'Boundary missing '+token)
 trace=rows(V/'M00E1B_CONFIGURATION_TRACEABILITY.csv');names={r['subsystem'] for r in trace}
 must({'AirStack','OSMO','Simulator image','Robot image','Isaac Sim','PX4','MAVROS','ROS2','DDS','MAVLink','Scene and profile','Vehicle / Pegasus'}<=names,'Missing critical subsystem')
 for r in trace:
  must(all(r.values()) and r['f00_equivalence']=='NOT_ESTABLISHED' and r['target_designation']=='UNVERIFIED','Missing trace or false equivalence/designation')
 record('configuration_and_exposure_boundaries','Critical subsystem trace separates installed/current/intended/historical/F00 excerpts;12 exposure paths and Mermaid diagram distinguish observed/configured/possible; no isolation/equivalence overclaim')
 replay=module('e1b',ROOT/'RRM/scripts/m00e1b_analyze.py');findings=replay.derive(d,a,h);must(findings==load(E/'discovery-findings.json'),'Offline derived metadata not reproducible')
 must(findings['observed_process_birth_separation_s']=='21.62' and not findings['execution_authorized'] and not findings['live_instrumentation_ready'],'False readiness/authority')
 m=module('e1a',ROOT/'RRM/scripts/m00e1a_analyze.py');historical=m.analyze(load(m.SOURCE));must({r['measurement_id']:r['seconds'] for r in historical['measurements']}=={'L11':'19.276162179','L03':'74.228771713'},'Historical timing changed')
 for name in ['historical-d-replay.json','historical-e1a-replay.json']:
  r=load(E/name);must(r['exit_code']==0,'Retained replay failed')
  output=json.loads(r['stdout'])
  must(output.get('new_startup_trials',0)==0,'Replay claims new trial')
 instrument=(V/'M00E1B_INSTRUMENTATION_ASSESSMENT.md').read_text()
 for token in ['DDS discovery','Deadline evaluation','Readiness/transaction/latch','Resource/measurement','19.276162179','74.228771713','not demonstrated ready','offline parse success']:
  must(token in instrument,'Instrumentation evidence gap '+token)
 record('instrumentation_and_timing','Current native proc/socket parsing and inherited raw Decimal replay reproducible;19.276162179s/74.228771713s preserved; complete live acquisition/clock/overhead/latch/supervision gaps explicitly retained')
 proposal=(ROOT/'RRM/verification/procedures/M00E1B_ACTIVE_VERIFICATION_PROPOSAL.md').read_text()
 for token in ['AV01','AV02','AV03','AV04','AV05','Class D','Side effects','Abort','Evidence','Separate written PD scope required','New specific Tier B directive required','not an active test']:
  must(token.lower() in proposal.lower(),'Proposal missing '+token)
 readiness=(V/'M00E1B_READINESS_DISPOSITION.md').read_text();handoff=(ROOT/'RRM/handoffs/M00E1B_HANDOFF.md').read_text()
 for token in ['none','CAND04','not designated','Narrowest safe next step','3 SATISFIED','8 UNSATISFIED','9 UNVERIFIED','UNVERIFIED']:
  must(token.lower() in readiness.lower(),'Readiness missing '+token)
 must('user clarification' in readiness.lower() and 'Which original17' in handoff and 'preferred target' in handoff.lower(),'Missing handoff/target recommendation')
 record('separate_active_proposals_and_target_recommendation','AV01-AV05 separate narrow future activities with prerequisites/operation/side effects/oversight/evidence/abort/recovery and PD authority; D precedes C; preferred new startup-only concept not assigned or activated')
 oldb=rows(ROOT/'RRM/evidence/M00-E1A/M00E1A_BLOCKER_DISPOSITION.csv');newb=rows(E/'M00E1B_BLOCKER_DISPOSITION.csv')
 must(len(newb)==len(oldb)==10,'Original G0 blockers lost')
 for x,y in zip(oldb,newb):must(all(y[k]==v for k,v in x.items()) and y['closed']=='NO','Original blocker field changed/closed')
 status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
 for token in ['M00-E1B',PARENT,'PENDING PD REVIEW','WP01 NOT AUTHORIZED','BLOCKED BY EXTERNAL INPUT','28 in-scope NOT_TESTED','1 OUT_OF_SCOPE','Target designation UNVERIFIED','zero whole closures']:
  must(token.lower() in status.lower(),'Status missing '+token)
 record('g0_roadmap_and_status','All original G0 blocker fields/IDs retained open, RA01 pending/WP01 unauthorized and29/28/1 coverage unchanged; roadmap hash preserved; status advances only investigation')
 after=[json.loads(s) for s in h['after_lifecycle']['stdout'].splitlines()]
 must(h['after_lifecycle']['exit_code']==0,'Lifecycle check failed')
 for c,r in zip(d['containers'],after):must(c['id']==r['id'] and c['started']==r['started'] and c['restart_count']==r['restarts'] and c['init_host_pid']==r['pid'],'Lifecycle changed during collection')
 audit=load(E/'scope-audit.json');must(audit['active_operations_performed']==[] and audit['tier_b_starts']==0 and audit['tier_c_operations']==0,'Active operation recorded')
 tree=ast.parse((ROOT/'RRM/scripts/m00e1b_analyze.py').read_text());imports={n.names[0].name for n in ast.walk(tree) if isinstance(n,ast.Import)}|{n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
 must(imports<={'decimal','datetime','argparse','json','pathlib','re'},'Offline analyzer imports runtime API')
 record('passive_scope_and_lifecycle','No exec/probe/ROS/device opens/start-restart/vehicle action in performed-action audit; reference start/restart/init PID unchanged during collection; not proof against unobserved external activity')
 paths=(E/'M00E1B_CHANGED_FILES.txt').read_text().splitlines();must(paths==sorted(set(paths)),'Changed-file list not unique/sorted')
 must(all(p.startswith('RRM/') and not p.startswith('RRM/RRM-C001-F00-EXT/') for p in paths),'Protected/outside path')
 must(set(paths)&set(before['tracked_entries'])=={'RRM/PROJECT_STATUS.md'},'Prior protected path in changes')
 if committed:
  must(set(git('diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines())==set(paths),'Committed file set mismatch')
  must(not git('status','--porcelain'),'Postcommit tree dirty')
 else:
  newpaths=set(git('ls-files').decode().splitlines())-set(before['tracked_entries'])
  newpaths|=set(git('ls-files','--others','--exclude-standard').decode().splitlines());must(newpaths==set(paths)-{'RRM/PROJECT_STATUS.md'},'New file set mismatch')
 staged=set(git('diff','--cached','--name-only').decode().splitlines());must(not staged or staged==set(paths),'Staged scope mismatch')
 excluded={str((E/f).relative_to(ROOT)) for f in ['M00E1B_EVIDENCE_MANIFEST.json','SHA256SUMS','verification-results.json']}
 manifest=load(E/'M00E1B_EVIDENCE_MANIFEST.json');must({r['path'] for r in manifest['files']}==set(paths)-excluded,'Manifest coverage mismatch')
 for r in manifest['files']:
  data=(ROOT/r['path']).read_bytes();must(sha(data)==r['sha256'] and len(data)==r['bytes'],'Package hash mismatch: '+r['path'])
 hashed=set()
 for line in (E/'SHA256SUMS').read_text().splitlines():
  digest,p=line.split('  ',1);hashed.add(p);must(sha((ROOT/p).read_bytes())==digest,'Checksum mismatch: '+p)
 must(hashed==set(paths)-{str((E/f).relative_to(ROOT)) for f in ['SHA256SUMS','verification-results.json']},'Checksum coverage mismatch')
 for p in paths:
  if p.endswith(('.md','.py')):must(all(line==line.rstrip() for line in (ROOT/p).read_text().splitlines()),'Trailing whitespace: '+p)
 for args in [('diff','--check'),('diff','--cached','--check')]:subprocess.run(['git',*args],cwd=ROOT,check=True)
 record('package_manifest_and_staging',f'{len(paths)} exact RRM-only files, PROJECT_STATUS sole prior mutation; byte/hash/file-set and diff/whitespace checks pass; final result hash excluded to avoid recursion and bound by final Git object')
 return dict(milestone='M00-E1B',result='PASS',target_designation='UNVERIFIED',simulator_isolation='UNVERIFIED',live_instrumentation_ready=False,execution_authorized=False,trials_executed=0,whole_original_unresolved_prerequisites_closed=0,prerequisite_counts=dict(Counter(r['result'] for r in new)),checks=checks,limitations=['Local daemon/proc/file discovery is bounded, not a global workflow inventory, target assignment or physical safety proof.','Selected startup files and backing executable hashes do not attest effective runtime parameter state, in-memory binaries, loaded scene or reproducible build.','Existing log tails incomplete; historical native clocks/calibration/coverage limits remain. Offline replay does not establish live acquisition/overhead or qualification.','No active operational verification, network/device/vehicle probe or credential login performed; skipped/expected access/size limits in command-results.json. No push/remote fetch.'])
if __name__=='__main__':print(json.dumps(validate(),indent=2))
