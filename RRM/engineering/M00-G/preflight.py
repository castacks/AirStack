"""Offline preflight: no process launch, hardware interaction or network access."""
from pathlib import Path
from common import load
from config_validator import check_package
import csv
import json


def evaluate(package,repository):
    package,repository=Path(package),Path(repository)
    static=check_package(package,repository)
    gate_path=repository/'RRM/engineering/M00-F/M00F_ISOLATION_ACCEPTANCE_GATE.csv'
    try:
        with gate_path.open() as f:gate=list(csv.DictReader(f))
        if len(gate)!=14:raise ValueError('Original IG coverage missing')
        manifest=load(package/'PROVENANCE.json')
        facts=manifest.get('runtime_evidence',{})
        fields=['host_device_isolation','network_confinement','gpu_least_privilege','recorder_reliability',
                'independent_stop','command_exclusion','clock_maps','host_resource_permissions']
        # No API in this offline prototype can attest/approve runtime evidence or execution.
        runtime=[{'property':key,'status':'NOT VERIFIED','evidence':facts.get(key)} for key in fields]
        unresolved=manifest.get('unresolved_dependencies',[])
        if not isinstance(unresolved,list):raise ValueError('Malformed unresolved dependencies')
        execution=manifest.get('execution_authorized')
        extra_fail=[] if execution is False else ['Execution authorization cannot be granted by manifest edit']
        if any(facts.get(key) not in (None,'NOT VERIFIED') for key in fields):
            extra_fail.append('Runtime attestation cannot be asserted through offline manifest')
        failed=static['status']=='FAIL' or bool(extra_fail)
        return {'overall':'FAIL' if failed else 'BLOCKED','execution_ready':False,
                'static_configuration':static,'runtime_containment':{'status':'NOT VERIFIED','properties':runtime},
                'acceptance_gate':[{'gate_id':row['gate_id'],'criterion':row['criterion'],
                                    'status':'NOT VERIFIED','result':'NOT PASSED'} for row in gate],
                'implementation_blockers':unresolved,'additional_failures':extra_fail,
                'authorization':'NOT AUTHORIZED','tier_b_target':'UNDESIGNATED',
                'note':'Static PASS and synthetic tests never constitute overall execution readiness.'}
    except (OSError,ValueError,TypeError) as exc:
        return {'overall':'FAIL','execution_ready':False,'static_configuration':static,
                'runtime_containment':{'status':'NOT VERIFIED'},'errors':[str(exc)]}


if __name__=='__main__':
    p=Path(__file__).resolve().parent
    result=evaluate(p,p.parents[2])
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['overall']=='PASS' else 2)
