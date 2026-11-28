"""Conditional original-header native configure/compile/link, never executes node.

No container invocation, package install/fetch, ROS setup or shell hooks. Reports
blocked stages once when genuine current-host dependencies are absent.
"""
from pathlib import Path
import json
import os
import subprocess
from native_environment import inspect,sha


def verify(repository,output):
    root=Path(repository).resolve();directory=Path(output).resolve()
    if not directory.is_relative_to(root/'RRM'):raise ValueError('All outputs must remain under RRM')
    directory.mkdir(parents=True,exist_ok=True)
    here=Path(__file__).resolve().parent
    environment=inspect(root)
    sources=[{'path':str(p.relative_to(root)),'sha256':sha(p)} for p in sorted(here.rglob('*.py'))
             if not set(p.relative_to(here).parts)&{'notebook','__pycache__','.tmp','native-output'}]
    for item in sources:compile((root/item['path']).read_bytes(),item['path'],'exec')
    stages={'SOURCE CHECK':'PASS' if environment['original_guard_identity']=='PASS' else 'FAIL',
            'CONFIGURATION':'BLOCKED','COMPILATION':'BLOCKED','LINKING':'BLOCKED',
            'OFFLINE UNIT TESTING':'NOT RUN BY BUILD TOOL',
            'PRODUCTION RUNTIME INTEGRATION':'NOT VERIFIED'}
    result={'outcome':'I2','stages':stages,'environment':environment,'commands':[],
            'dependency_free_python_compilation':'PASS','python_source_identities':sources,
            'genuine_native_build':False,'guard_executed':False,'execution_ready':False}
    if stages['SOURCE CHECK']=='FAIL':
        result['outcome']='I3'
        result['build_attempt_stopped']='Original guard identity failed; no native invocation'
    elif environment['readiness']=='READY_FOR_NATIVE_ATTEMPT':
        prefixes=[p for p in environment['prefixes'] if Path(p).is_dir()]
        command=[environment['tools']['cmake'],'-S',str(here/'guard_host'),'-B',str(directory/'native-output'),
                 '-DRRM_UPSTREAM_INCLUDE_DIR='+str(root/'robot/ros_ws/src/interface/mavros_interface/include'),
                 '-DCMAKE_PREFIX_PATH='+';'.join(prefixes),'-DCMAKE_CXX_COMPILER='+environment['tools']['cxx']]
        # Avoid inheriting conflicting ROS/colcon discovery paths. Never source .bashrc.
        process_env={k:v for k,v in os.environ.items() if k not in {'CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH'}}
        for stage,args in [('CONFIGURATION',command),('COMPILATION',[environment['tools']['cmake'],'--build',str(directory/'native-output')])]:
            result['commands'].append(args)
            done=subprocess.run(args,cwd=root,env=process_env,capture_output=True,text=True,timeout=120)
            (directory/(stage.lower()+'.log')).write_text(done.stdout+done.stderr)
            stages[stage]='PASS' if done.returncode==0 else 'FAIL'
            if done.returncode:
                result['outcome']='I3';break
        else:
            binary=directory/'native-output/rrm_startup_guard'
            if binary.is_file():
                stages['LINKING']='PASS';result['genuine_native_build']=True;result['outcome']='I1'
                result['binary_sha256']=sha(binary)
            else:
                stages['LINKING']='FAIL';result['outcome']='I3'
    else:
        result['build_attempt_stopped']='Existing host lacks mandatory native tools/dependencies; supported container build path documented; no further attempts'
    (directory/'M00I_NATIVE_BUILD_RESULTS.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    return result


if __name__=='__main__':
    root=Path(__file__).resolve().parents[3]
    result=verify(root,Path(__file__).resolve().parent)
    print(json.dumps({'outcome':result['outcome'],'stages':result['stages'],
                      'missing':result['environment']['native_missing']},indent=2))
    raise SystemExit(0 if result['genuine_native_build'] else 2)
