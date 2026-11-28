"""Successor to sealed G preflight; imports G policy read-only, never edits it."""
from pathlib import Path
import importlib.util
import sys
from h_support import decode,file_sha
from check_build_dependencies import GUARD_SHA,HEADER


def source_correspondence(repository):
    p=Path(repository)/HEADER
    anchors=['using Clock = std::chrono::steady_clock','started_ = Clock::now()',
             'if (!query_lock.owns_lock())','config_valid_ && !attempted_',
             'attempted_ = true;', 'startup_reason_ = initialized_ ? "INITIALIZED" : "STARTUP_NOT_VERIFIED"',
             'message->armed','LANDED_STATE_IN_AIR','PARAMETER_DOUBLE','double_value = 1.0']
    text=p.read_text()
    ok=file_sha(p)==GUARD_SHA and all(x in text for x in anchors)
    return {'source_identity':'PASS' if ok else 'FAIL','synthetic_source_correspondence':'PASS' if ok else 'FAIL',
            'full_guard_compatibility':'NOT VERIFIED','guard_sha256':file_sha(p),
            'limitation':'Anchors/hash and synthetic tests do not compile/execute the original C++ guard.'}


def inherited_preflight(repository):
    directory=Path(repository)/'RRM/engineering/M00-G'
    if str(directory) not in sys.path:sys.path.append(str(directory))
    spec=importlib.util.spec_from_file_location('_rrm_g_preflight',directory/'preflight.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.evaluate(directory,repository)


def evaluate(repository,build_result,test_result):
    root=Path(repository)
    try:
        g=inherited_preflight(root);compatibility=source_correspondence(root)
        test_bound=bool(test_result.get('source_sha256')) and all((root/path).is_file() and file_sha(root/path)==value
                                                                 for path,value in test_result.get('source_sha256',{}).items())
        test_status='PASS' if test_bound and test_result.get('status')=='PASS' and test_result.get('test_count',0)>=20 else 'FAIL'
        build_bound=all((root/x['path']).is_file() and file_sha(root/x['path'])==x['sha256'] for x in build_result.get('python_sources',[]))
        build_status=build_result.get('build_status','BLOCKED') if build_bound and build_result.get('python_sources') else 'BLOCKED'
        failed=test_status=='FAIL' or compatibility['source_identity']=='FAIL' or g['static_configuration'].get('static_policy')!='PASS' or build_status=='FAIL'
        return {'BUILD STATUS':build_status,'OFFLINE TEST STATUS':test_status,
                'STATIC ISOLATION STATUS':g['static_configuration'].get('static_policy','FAIL'),
                'RUNTIME CONTAINMENT STATUS':'NOT VERIFIED',
                'OVERALL EXECUTION READINESS':'FAIL' if failed else 'BLOCKED','execution_ready':False,
                'dependency_free_python':build_result.get('dependency_free_python','NOT VERIFIED'),
                'guard_compatibility':compatibility,'adapters':'OFFLINE_IMPLEMENTED; ROS_BINDING_NOT_VERIFIED',
                'recorder':'LOCAL_SYNTHETIC_FAILURE_TESTS_ONLY; RUNTIME_RELIABILITY_NOT_VERIFIED',
                'dependency_identity_completeness':'PARTIAL; HOST_TOOLCHAIN_MISSING; RUNTIME_IDENTITIES_UNRESOLVED',
                'missing_build_dependencies':build_result.get('dependencies',{}).get('missing',[]),
                'original_isolation_gate':g.get('acceptance_gate',[]),'target':'UNDESIGNATED',
                'execution_authorized':False,'wp01_authorized':False,
                'limits':'Static/test/source correspondence cannot establish full guard ABI/runtime behavior or physical containment.'}
    except (OSError,ValueError,KeyError,TypeError,AttributeError) as exc:
        return {'BUILD STATUS':'BLOCKED','OFFLINE TEST STATUS':'FAIL','STATIC ISOLATION STATUS':'NOT VERIFIED',
                'RUNTIME CONTAINMENT STATUS':'NOT VERIFIED','OVERALL EXECUTION READINESS':'FAIL',
                'execution_ready':False,'error':str(exc)}


if __name__=='__main__':
    import json
    root=Path(__file__).resolve().parents[3];e=root/'RRM/evidence/M00-H'
    try:result=evaluate(root,decode((e/'build/BUILD_RESULT.json').read_bytes()),decode((e/'M00H_TEST_RESULTS.json').read_bytes()))
    except (OSError,ValueError) as exc:result={'OVERALL EXECUTION READINESS':'BLOCKED','error':str(exc),'execution_ready':False}
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result.get('OVERALL EXECUTION READINESS')=='PASS' else 2)
