"""Explicit offline-only build procedure. Never executes the guard binary.

No stubs, package installation, image operations or network operations. Compiles
Python modules with compile() separately without creating bytecode outside RRM.
"""
from pathlib import Path
import json
import subprocess
from check_build_dependencies import check
from h_support import file_sha


def build(repository,output_dir):
    root=Path(repository).resolve();output=Path(output_dir).resolve()
    if not output.is_relative_to(root/'RRM'):raise ValueError('Build output must remain under RRM')
    output.mkdir(parents=True,exist_ok=True)
    here=Path(__file__).resolve().parent
    compiled=[]
    for p in sorted(here.glob('*.py')):
        compile(p.read_bytes(),str(p),'exec');compiled.append({'path':str(p.relative_to(root)),'sha256':file_sha(p)})
    deps=check(root);commands=[];result={'build_status':'BLOCKED','dependency_free_python':'PASS',
        'python_sources':compiled,'dependencies':deps,'commands':commands,'guard_executed':False,
        'production_build_verified':False}
    if deps['status']=='READY_FOR_COMPILE_ATTEMPT':
        build_dir=output/'native-build'
        commands += [[deps['tools']['cmake'],'-S',str(root/'RRM/engineering/M00-G/guard_host'),'-B',str(build_dir),
                      '-DRRM_UPSTREAM_INCLUDE_DIR='+str(root/'robot/ros_ws/src/interface/mavros_interface/include')],
                     [deps['tools']['cmake'],'--build',str(build_dir)]]
        for index,command in enumerate(commands):
            done=subprocess.run(command,cwd=root,capture_output=True,text=True,timeout=120)
            (output/f'compile-{index}.log').write_text(done.stdout+done.stderr)
            if done.returncode:
                result['build_status']='FAIL';result['failed_command']=index;break
        else:
            result['build_status']='PASS';result['production_build_verified']=True
            result['limitation']='Full original-header host compile/link only; no runtime semantics/ABI/isolation qualification.'
    (output/'BUILD_RESULT.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    return result


if __name__=='__main__':
    root=Path(__file__).resolve().parents[3]
    outcome=build(root,root/'RRM/evidence/M00-H/build')
    print(json.dumps({'build_status':outcome['build_status'],'python':outcome['dependency_free_python'],
                      'missing':outcome['dependencies']['missing']},indent=2))
    raise SystemExit(0 if outcome['build_status']=='PASS' else 2)
