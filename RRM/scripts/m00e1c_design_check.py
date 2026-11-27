#!/usr/bin/env python3
"""Offline consistency checks for non-executable E1C design data. Cannot grant authority."""
import argparse
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1C'
BOUNDARY='DESIGN PROPOSAL ONLY — NO DEPLOYMENT OR TIER B EXECUTION AUTHORIZED'
REQUIRED_NODES=set('GATE SUP REC ORCH SIM APP SCENE PLAY PX4 TCP UDP DDS MAVROS HB OBS FRESH PARAM P0PERMIT P1PERMIT GUARD CALLBACK TX READBACK READY EVAL LATCH'.split())
TARGET_FIELDS={'repository_source_and_overlay_manifest','workflow_uid_pool_host_boot_owner','simulator_registry_digest_and_installed_manifest','diagnostic_guard_host_registry_digest_and_build_manifest','mavros_registry_digest_and_plugin_dependency_manifest','isaac_scene_asset_closure_units_dynamics_seed_epoch','px4_source_executable_rcs_airframe_full_parameters','mavros_fcu_url_sysid_compid_parameter_profile','ros_distribution_rmw_packages_dds_qos_identity_permissions','private_network_peers_ports_route_firewall_manifest','gpu_only_device_uid_capability_mount_runtime_policy','cpu_memory_gpu_disk_io_duration_capture_storage_limits','instrumentation_schema_tool_versions_clock_maps_overhead','p0_p1_launcher_predicate_and_shutdown_identity_manifest','private_cache_snapshot_manifests_and_attempt_evidence_root','independent_supervisor_recorder_exit_durability_manifest','operator_stop_owner_and_exact_authority_scope_expiry'}
def require(ok,reason):
 if not ok:raise ValueError(reason)
def projection(model,policy):
 ids=[r['id'] for r in model['components']];require(set(ids)==REQUIRED_NODES and len(ids)==len(set(ids)),'Missing, extra or duplicate design component')
 require(all(r['implementation_status']=='PROPOSED_NOT_IMPLEMENTED' for r in model['components']),'Design falsely implemented')
 dependencies={key:set() for key in ids};seen=set()
 for edge in model['dependencies']:
  require(edge['policy'] in {'P0','P1','BOTH'},'Unknown policy')
  require(edge['source'] in dependencies and edge['target'] in dependencies,'Unknown dependency endpoint')
  ident=(edge['source'],edge['target'],edge['policy']);require(ident not in seen,'Duplicate dependency');seen.add(ident)
  if edge['policy'] in {'BOTH',policy}:dependencies[edge['target']].add(edge['source'])
 remaining={k:set(v) for k,v in dependencies.items()};order=[]
 while remaining:
  ready=sorted(k for k,v in remaining.items() if not v);require(bool(ready),'Dependency cycle')
  order+=ready
  for k in ready:del remaining[k]
  for v in remaining.values():v.difference_update(ready)
 def ancestors(node):
  result=set();todo=list(dependencies[node])
  while todo:
   key=todo.pop()
   if key not in result:result.add(key);todo.extend(dependencies[key])
  return result
 require({'GATE','SUP','REC','ORCH'}<=ancestors('GUARD'),'Missing authority/evidence prerequisite')
 require({'APP','SCENE','PLAY'}<=ancestors('PX4'),'PX4 launch semantic prerequisites missing')
 require('TCP' not in ancestors('UDP'),'Invented mandatory TCP-to-companion-UDP ordering')
 require({'GUARD','CALLBACK','PARAM','TX','READBACK'}<=ancestors('READY'),'Readiness transaction path missing')
 require('GUARD' not in ancestors('P1PERMIT') and 'READY' not in ancestors('P1PERMIT'),'Circular dependency observer')
 if policy=='P1':require({'FRESH','PARAM','P1PERMIT'}<=ancestors('GUARD'),'P1 must gate actual dependencies before guard construction')
 else:require('FRESH' not in ancestors('GUARD') and 'P1PERMIT' not in ancestors('GUARD'),'P0 silently changed to dependency-gated startup')
 return dict(policy=policy,nodes=len(ids),edges=sum(r['policy'] in {'BOTH',policy} for r in model['dependencies']),topological_order=order)
def check_model(model):
 require(model['model_kind']=='NON_EXECUTABLE_DESIGN_ONLY' and model['package_state']==BOUNDARY,'Wrong design boundary')
 require(model['target_designation']=='UNVERIFIED' and model['design_decision'] is None,'Fabricated target/design decision')
 for key in ['construction_authorized','deployment_authorized','active_verification_authorized','tier_b_authorized']:require(model[key] is False,'Design must not grant '+key)
 require(set(model['controls'])=={'IC%02d'%n for n in range(1,13)},'Missing isolation controls')
 require(set(model['source_hypotheses'])=={'HD%02d'%n for n in range(1,9)},'Missing hypotheses')
 require(set(model['instrumentation_ids'])=={'I%02d'%n for n in range(1,7)},'Missing instrumentation')
 require(len(model['excluded_components'])>=10,'Exclusions incomplete')
 return dict(design_consistent=True,projections=[projection(model,p) for p in ['P0','P1']],implementation_accepted=False,execution_authorized=False)
def check_manifest(manifest):
 require(manifest['kind']=='NON_EXECUTABLE_MANIFEST_SPEC_PROPOSAL' and manifest['package_state']==BOUNDARY,'Wrong manifest boundary')
 require(manifest['target_designation']=='UNVERIFIED' and manifest['design_decision'] is None,'Manifest fabricated target/decision')
 for k in ['construction_authorized','deployment_authorized','tier_b_authorized']:require(manifest[k] is False,'Manifest authority inferred')
 require(set(manifest['required_target_fields'])==TARGET_FIELDS,'Target schema incomplete')
 require(all(v is None for v in manifest['required_target_fields'].values()),'E1C cannot invent populated target values')
 require(all(r['status']=='INHERITED_REFERENCE_NOT_TARGET_ATTESTATION' and r['source'] and r['limitation'] for r in manifest['known_reference_records']),'Reference fact falsely attested')
 return dict(schema_consistent=True,missing_target_field_groups=sorted(TARGET_FIELDS),complete_for_execution=False,execution_authorized=False)
def check_pending(decision):
 require(decision['package_state']==BOUNDARY and decision['target_designation']=='UNVERIFIED','Decision boundary mismatch')
 require(decision['offered_options']==['APPROVE DESIGN FOR FURTHER ENGINEERING','RETURN FOR REVISION','REJECT PROPOSED DESIGN'],'Decision options changed')
 for k in ['selected_option','pd_name','pd_signature','decision_date','decision_reference','conditions','scope']:require(decision[k] is None,'No PD decision supplied')
 for k in ['construction_authorized','deployment_authorized','activation_authorized','active_verification_authorized','tier_b_authorized']:require(decision[k] is False,'Design approval cannot infer operations')
 require(decision['external_owner_requests_sent']==0,'No external request delivery supported')
 return dict(record_consistent=True,selected_option=None,execution_authorized=False)
def main():
 argparse.ArgumentParser(description=__doc__).parse_args()
 result=dict(model=check_model(json.loads((E/'M00E1C_DESIGN_MODEL.json').read_text())),manifest=check_manifest(json.loads((E/'M00E1C_MANIFEST_PROPOSAL.json').read_text())),decision=check_pending(json.loads((E/'M00E1C_PENDING_DECISION.json').read_text())))
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
