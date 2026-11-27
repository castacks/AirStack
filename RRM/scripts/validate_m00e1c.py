#!/usr/bin/env python3
"""Offline E1C design-package validator: existing local files and read-only Git only."""
import ast
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1C';V=ROOT/'RRM/verification/reports/M00-E1C';PARENT='4326dd079a2d80518983a5b793b9bab7f751a5db'
def must(ok,reason):
 if not ok:raise ValueError(reason)
def sha(b):return hashlib.sha256(b).hexdigest()
def load(p):return json.loads(p.read_text())
def rows(p):
 with p.open() as f:return list(csv.DictReader(f))
def git(*a):return subprocess.check_output(['git',*a],cwd=ROOT)
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def validate():
 checks=[]
 def record(name,detail):checks.append(dict(check=name,result='PASS',detail=detail))
 before=load(E/'repository-before.json');head=git('rev-parse','HEAD').decode().strip();committed=head!=PARENT
 must(before['head']==PARENT and before['branch']=='ore_proj' and not before['status_porcelain'],'Starting repository discrepancy')
 must(git('branch','--show-current').decode().strip()=='ore_proj','Branch changed')
 if committed:
  must(git('rev-parse','HEAD^').decode().strip()==PARENT,'Unexpected commit parent')
  must(git('log','-1','--format=%s').decode().strip()=='RRM-F00 M00-E1C: design isolated startup rig and prepare PD decision','Unexpected commit subject')
 record('starting_checkout','Clean pinned E1B parent on ore_proj; direct successor only; prior full validator passed before successor writes')
 for name,i in before['tracked_entries'].items():
  if name=='RRM/PROJECT_STATUS.md':continue
  p=ROOT/name
  if i['type']=='file':must(p.is_file() and not p.is_symlink() and sha(p.read_bytes())==i['sha256'] and p.stat().st_mode&0o777==i['mode'],'Protected file changed: '+name)
  elif i['type']=='symlink':must(p.is_symlink() and os.readlink(p)==i['target'],'Protected symlink changed: '+name)
  else:
   must(subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD']).decode().strip()==i['head'],'Submodule changed: '+name)
   must(not subprocess.check_output(['git','-C',str(p),'status','--porcelain']),'Dirty submodule: '+name)
 for name in ['source-manifest.json','input-inventory.json']:
  for r in load(E/name)['files']:
   b=(ROOT/r['path']).read_bytes();must(len(b)==r['bytes'] and sha(b)==r['sha256'],'Source/input changed: '+r['path'])
 must({r['path'] for r in load(E/'input-inventory.json')['files']}=={str(p.relative_to(ROOT)) for p in (ROOT/'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()},'Controlled input file set changed')
 pins={'M00':'e772605ccfb6c3bfe515e8e07961cdddc0831968','M00-B':'93b8196e835effc3da34cac5677eab4d30c1378a','M00-C':'b4ac9467b073b6ae7927a47b939f46e9407cb6c0','M00-D':'5b11d4d348db399e0e3ae638e318da9d7079ace6','M00-E1':'31e33893ef2fa0f724862a980f6b5db0d62509d2','M00-E1A':'df8ebd18a6e16909cca461e18dfba882b53c8120','M00-E1B':PARENT};n=0
 for milestone,pin in pins.items():
  p=ROOT/f'RRM/evidence/{milestone}/SHA256SUMS'
  if not p.exists():continue
  for line in p.read_text().splitlines():
   digest,name=line.split('  ',1);must(sha(git('show',pin+':'+name))==digest,'Historical pinned mismatch: '+name);n+=1
 record('protected_sources_and_sealed_history',f"{len(before['tracked_entries'])-1} prior tracked entries unchanged except authorized status; controlled input and sources intact;{n} checksum entries at pinned milestone commits")
 design=module('e1c',ROOT/'RRM/scripts/m00e1c_design_check.py')
 actual=dict(model=design.check_model(load(E/'M00E1C_DESIGN_MODEL.json')),manifest=design.check_manifest(load(E/'M00E1C_MANIFEST_PROPOSAL.json')),decision=design.check_pending(load(E/'M00E1C_PENDING_DECISION.json')))
 must(actual==load(E/'design-check-results.json'),'Design checker output not reproducible')
 must(all(not r['execution_authorized'] for r in actual.values()),'False authority inference')
 model=load(E/'M00E1C_DESIGN_MODEL.json');manifest=load(E/'M00E1C_MANIFEST_PROPOSAL.json')
 must(len(manifest['known_reference_records'])==12 and len(manifest['required_target_fields'])==17,'Manifest counts wrong')
 record('architecture_dependencies_and_non_executable_manifest','Both26-node P0/P1 projections acyclic, fresh independent P1 and original transaction path enforced;12 inherited reference facts/17 null target groups; all operational flags false; no invented target identity')
 owners=rows(V/'M00E1C_ENVIRONMENT_OWNER_MATRIX.csv');requests=rows(V/'M00E1C_OWNER_EVIDENCE_ITEMS.csv');ownerids={r['owner_id'] for r in owners};requestids={r['request_id'] for r in requests}
 must(ownerids=={'OWN%02d'%i for i in range(1,10)} and len(owners)==9,'Owner roles incomplete')
 must(requestids=={'RQ%02d'%i for i in range(1,15)} and len(requests)==14,'RFI items incomplete')
 for r in owners:
  must(r['identity_status']=='UNVERIFIED' and r['verified_named_owner_or_team']=='NONE_ESTABLISHED','Unsupported owner identity')
  must(r['request_status']=='PREPARED_FOR_PD_ROUTING_NOT_SENT_TO_EXTERNAL_OWNERS','Unsupported delivery claim')
  must(set(r['requested_evidence_ids'].split(';'))<=requestids and r['source_evidence'] and r['closure_condition'],'Owner trace missing')
 for r in requests:
  must(set(r['owner_role_ids'].split(';'))<=ownerids and r['requested_information'] and r['acceptable_response'],'RFI routing/response incomplete')
  must(r['response_status']=='AWAITING_OWNER' and r['external_delivery_status']=='NOT_SENT','Response/delivery fabricated')
 search=load(E/'owner-record-search.json');must(search['named_environment_owners_verified']==0 and search['external_owner_messages_sent']==0,'Unsupported owner acquisition')
 record('owner_identity_and_formal_request','Nine role requests/14 specific RFI items, source versus proposed contact distinguished; no verified named owner invented and no external delivery/response claimed')
 controls=rows(V/'M00E1C_ISOLATION_CONTROL_MATRIX.csv');ids={r['control_id'] for r in controls};exposures=rows(ROOT/'RRM/verification/reports/M00-E1B/M00E1B_EXPOSURE_REGISTER.csv')
 must(ids=={'IC%02d'%i for i in range(1,13)} and len(controls)==12,'Missing control')
 must({x for r in controls for x in r['e1b_exposure_ids'].split(';')}=={r['exposure_id'] for r in exposures},'E1B exposure left without control disposition')
 for r in controls:
  for key in ['hazard_or_exposure','proposed_mitigation','enforcement_mechanism','verification_method','required_evidence','failure_mode','residual_risk','responsible_owner_role_ids','source']:must(r[key],'Control missing '+key)
  must(r['implementation_status']=='PROPOSED_NOT_IMPLEMENTED' and r['owner_assignment']=='UNVERIFIED_PD_TO_ASSIGN','Control/owner falsely implemented/assigned')
  must(set(r['responsible_owner_role_ids'].split(';'))<=ownerids,'Control owner unknown')
 comparison=rows(V/'M00E1C_EXISTING_VS_INDEPENDENT.csv');must(len(comparison)==11,'Comparison dimensions incomplete')
 for r in comparison:
  must(r['evidence_freshness']=='HISTORICAL_E1B_NOT_REOBSERVED_IN_E1C' and r['proposal_status']=='NOT_IMPLEMENTED_NOT_VERIFIED','Current runtime/actual isolation overclaim')
  must(set(r['control_ids'].split(';'))<=ids and all(r.values()),'Comparison evidence/limitations incomplete')
 architecture=(V/'M00E1C_RIG_ARCHITECTURE.md').read_text()
 for token in ['```mermaid','material diagnostic configuration difference','new diagnostic-only host','unverified platform assumptions','No automatic restart','No stub service']:
  must(token.lower() in architecture.lower(),'Architecture missing limitation: '+token)
 record('isolation_and_comparison','All12 E1B exposure paths map to12 proposed defense-in-depth controls with enforcement/evidence/failure/residual/owner;11 comparison dimensions label E1B evidence as historical; guard-host material context delta explicit')
 catalog=rows(V/'M00E1C_EVENT_CATALOG.csv');must(len(catalog)==11 and {r['event_group_id'] for r in catalog}=={'EV%02d'%i for i in range(1,12)},'Event groups incomplete')
 must({s for r in catalog for s in r['instrumentation_ids'].split(';')}=={'I%02d'%i for i in range(1,7)},'Missing I instrument')
 must({s for r in catalog for s in r['hypothesis_ids'].split(';')}=={'HD%02d'%i for i in range(1,9)},'Missing HD hypothesis')
 for r in catalog:
  must(all(r.values()) and r['implementation_status']=='PROPOSED_NOT_IMPLEMENTED','Missing event/clock/source/artifact or false acquisition')
 timing=module('hist',ROOT/'RRM/scripts/m00e1a_analyze.py').analyze(load(ROOT/'RRM/evidence/M00-C/startup-source-records.json'))
 must({r['measurement_id']:r['seconds'] for r in timing['measurements']}=={'L11':'19.276162179','L03':'74.228771713'},'Historical timing changed')
 accept=rows(V/'M00E1C_DESIGN_ACCEPTANCE_CHECKLIST.csv');must(len(accept)==14,'Implementation acceptance checks incomplete')
 for r in accept:
  must(r['status']=='NOT_PERFORMED_PROPOSED_FUTURE_CHECK' and r['criterion_authority']=='PROPOSED_RIG_ACCEPTANCE_NOT_APPROVED_F00_REQUIREMENT' and r['execution_authorized']=='NO','False acceptance/requirement adoption')
  must(set(r['control_ids'].split(';'))<=ids and set(r['owner_request_ids'].split(';'))<=requestids,'Broken acceptance trace')
 must(any('OFFLINE' in r['verification_tier'] for r in accept) and any('ACTIVE' in r['verification_tier'] for r in accept) and any('TIER_B' in r['verification_tier'] for r in accept),'Verification tiers not separated')
 record('instrumentation_historical_timing_and_future_acceptance','EV01–EV11 cover all I01–I06/HD01–HD08 with native clocks/source/artifacts/limits; exact historical intervals replay;14 future offline/passive/active checks all unperformed and proposed, not F00 requirements')
 old=rows(ROOT/'RRM/evidence/M00-E1B/M00E1B_PREREQUISITE_DELTA.csv');impact=rows(E/'M00E1C_PREREQUISITE_IMPACT.csv');must(len(old)==len(impact)==20,'Lost original prerequisite')
 for a,b in zip(old,impact):
  must(a['prerequisite_id']==b['prerequisite_id'] and a['original_requirement']==b['original_requirement'] and a['result']==b['prior_e1b_disposition']==b['current_disposition'],'Prerequisite identity/disposition changed')
  must(b['prerequisite_closed_by_e1c']=='NO' and b['target_designation']=='UNVERIFIED','Design closes prerequisite or assigns target')
  for key in ['design_could_define_mitigation','requires_implementation_evidence','requires_authorized_active_verification','requires_authoritative_external_input','outside_rig_scope','remaining_closure_evidence','responsible_party']:must(b[key],'Impact incomplete: '+key)
 must(Counter(r['current_disposition'] for r in impact)==Counter({'SATISFIED':3,'UNSATISFIED':8,'UNVERIFIED':9}),'Prerequisite count changed')
 oldb=rows(ROOT/'RRM/evidence/M00-E1B/M00E1B_BLOCKER_DISPOSITION.csv');newb=rows(E/'M00E1C_BLOCKER_DISPOSITION.csv');must(len(oldb)==len(newb)==10,'Original G0 register incomplete')
 for a,b in zip(oldb,newb):must(all(b[k]==v for k,v in a.items()) and b['closed']=='NO','Original blocker field/closure changed')
 status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
 for token in ['M00-E1C',PARENT,'America/New_York','PENDING PD REVIEW','WP01 NOT AUTHORIZED','BLOCKED BY EXTERNAL INPUT','28 in-scope NOT_TESTED','1 OUT_OF_SCOPE','zero whole closures','Target designation','UNVERIFIED']:
  must(token in status,'Status missing '+token)
 record('prerequisite_g0_and_roadmap_continuity','All20 original IDs/dispositions with17 unresolved impacts; ten original blocker fields remain open, RA01 pending/WP01 unauthorized,29/28/1 coverage and roadmap/source criteria unchanged; status uses client display timezone')
 decision=(V/'M00E1C_PD_ARCHITECTURE_DECISION.md').read_text()
 must(decision.count('- [ ] ')==3 and '- [x]' not in decision.lower(),'PD decision preselected')
 for token in [design.BOUNDARY,'does not authorize construction','guard-host','Minimum implementation scope','all options UNSELECTED']:
  must(token.lower() in decision.lower(),'PD packet missing '+token)
 evidence=rows(E/'M00E1C_EVIDENCE_INDEX.csv')
 for r in evidence:
  b=(ROOT/r['path']).read_bytes();must(len(b)==int(r['bytes']) and sha(b)==r['sha256'],'Evidence-index identity mismatch: '+r['path'])
  must(all(r.values()),'Missing evidence provenance')
 must(any(r['evidence_kind'].startswith('INHERITED') for r in evidence) and any(r['evidence_kind'].startswith('NEW') for r in evidence),'Inherited/new design evidence conflated')
 audit=load(E/'scope-audit.json');must(not audit['runtime_commands_performed'] and not audit['construction_or_deployment_actions'] and not audit['active_tests_or_service_operations'] and audit['external_owner_messages_sent']==0,'Out-of-scope action recorded')
 tree=ast.parse((ROOT/'RRM/scripts/m00e1c_design_check.py').read_text());imports={n.names[0].name for n in ast.walk(tree) if isinstance(n,ast.Import)}|{n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
 must(imports<={'argparse','json','pathlib'},'Design checker contains runtime API')
 record('pending_decision_evidence_provenance_and_scope','Three architecture options unselected, no operational flags or messages; inherited/new artifacts indexed; pure design checker has no deployment/runtime API; skipped owner/runtime/build/active/qualification checks explicitly recorded')
 paths=(E/'M00E1C_CHANGED_FILES.txt').read_text().splitlines();must(paths==sorted(set(paths)),'Changed list duplicate/unsorted')
 must(all(p.startswith('RRM/') and not p.startswith('RRM/RRM-C001-F00-EXT/') for p in paths),'Out-of-scope change')
 must(set(paths)&set(before['tracked_entries'])=={'RRM/PROJECT_STATUS.md'},'Protected prior file in package')
 must(not any(p.endswith(('.sh','.yaml','.yml','.service')) for p in paths),'Executable deployment/service configuration not permitted')
 if committed:
  must(set(git('diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines())==set(paths),'Commit file set mismatch')
  must(not git('status','--porcelain'),'Postcommit working tree dirty')
 else:
  newpaths=set(git('ls-files').decode().splitlines())-set(before['tracked_entries']);newpaths|=set(git('ls-files','--others','--exclude-standard').decode().splitlines())
  must(newpaths==set(paths)-{'RRM/PROJECT_STATUS.md'},'Unexpected new/missing file')
 staged=set(git('diff','--cached','--name-only').decode().splitlines());must(not staged or staged==set(paths),'Unexpected staged file set')
 excluded={str((E/f).relative_to(ROOT)) for f in ['M00E1C_EVIDENCE_MANIFEST.json','SHA256SUMS','verification-results.json']};manifest=load(E/'M00E1C_EVIDENCE_MANIFEST.json')
 must({r['path'] for r in manifest['files']}==set(paths)-excluded,'Package manifest coverage mismatch')
 for r in manifest['files']:
  b=(ROOT/r['path']).read_bytes();must(sha(b)==r['sha256'] and len(b)==r['bytes'],'Package manifest mismatch: '+r['path'])
 hashed=set()
 for line in (E/'SHA256SUMS').read_text().splitlines():
  digest,p=line.split('  ',1);hashed.add(p);must(sha((ROOT/p).read_bytes())==digest,'Package checksum mismatch: '+p)
 must(hashed==set(paths)-{str((E/f).relative_to(ROOT)) for f in ['SHA256SUMS','verification-results.json']},'Checksum coverage mismatch')
 for p in paths:
  if p.endswith(('.md','.py')):must(all(line==line.rstrip() for line in (ROOT/p).read_text().splitlines()),'Trailing whitespace: '+p)
 for args in [('diff','--check'),('diff','--cached','--check')]:subprocess.run(['git',*args],cwd=ROOT,check=True)
 record('exact_file_set_hashes_and_staging',f'{len(paths)} exact RRM-only changes, PROJECT_STATUS sole prior mutation; no deployment launch files; complete manifest/hash/whitespace checks. Result excluded from recursive digests and bound by final Git commit')
 return dict(milestone='M00-E1C',result='PASS',recommendation='PREFER_INDEPENDENT_FOR_DESIGN_REVIEW_ONLY',target_designation='UNVERIFIED',design_decision=None,construction_authorized=False,deployment_authorized=False,tier_b_authorized=False,external_owner_requests_sent=0,whole_prerequisite_closures=0,checks=checks,limitations=['No named environment owner, target, platform feasibility or runtime condition independently verified in E1C; RFI prepared for PD routing, not externally delivered.','Graph/schema/offline review does not establish actual device/network isolation, acquisition completeness, clock accuracy, overhead, recorder survival or qualification.','Diagnostic guard-host integration is a material candidate delta requiring explicit review and exact future scope; original24-row schedule unchanged.','Current runtime inspection/build/construction/deployment/active tests skipped by design boundary; original frozen records and owner approvals remain external; see command-results.json. No push.'])
if __name__=='__main__':print(json.dumps(validate(),indent=2))
