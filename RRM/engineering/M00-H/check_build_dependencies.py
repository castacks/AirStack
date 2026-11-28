"""Deterministic, read-only host dependency inventory; no Docker or probes.

Absence is scoped to the checked prefixes/PATH, not a claim about other images.
No dependency discovery by invoking ROS, pkg-config, compiler or runtime tools.
"""
from pathlib import Path
import shutil
import sys
import sysconfig
from h_support import file_sha

HEADER='robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp'
GUARD_SHA='e522a5f6ac10e293e6985b1aaca52d59fad7d80f91a5d5eed0aca0876f785ea2'
PACKAGES=['ament_cmake','rclcpp','ament_index_cpp','mavros_msgs','rcl_interfaces','std_msgs','interface_bringup','yaml-cpp']


def check(repository,prefixes=None,which=shutil.which):
    root=Path(repository)
    prefixes=[Path(x) for x in (prefixes if prefixes is not None else ['/opt/ros/jazzy','/usr','/usr/local'])]
    tools={'cxx':next((x for x in [which('c++'),which('g++'),which('clang++')] if x),None),
           'cmake':which('cmake')}
    deps=[]
    for package in PACKAGES:
        candidates=[]
        for prefix in prefixes:
            candidates += [prefix/'share'/package/'cmake'/(package+'Config.cmake'),
                           prefix/'lib/cmake'/package/(package+'-config.cmake'),
                           prefix/'lib/x86_64-linux-gnu/cmake'/package/(package+'-config.cmake')]
        found=[dict(path=str(p),sha256=file_sha(p)) for p in candidates if p.is_file()]
        header_rel={'rclcpp':'rclcpp/rclcpp.hpp','ament_index_cpp':'ament_index_cpp/get_package_share_directory.hpp',
                    'mavros_msgs':'mavros_msgs/msg/state.hpp','rcl_interfaces':'rcl_interfaces/srv/get_parameters.hpp',
                    'std_msgs':'std_msgs/msg/bool.hpp','yaml-cpp':'yaml-cpp/yaml.h'}.get(package)
        headers=[] if header_rel is None else [prefix/'include'/extra/header_rel for prefix in prefixes for extra in ['',package]]
        libraries=[]
        for prefix in prefixes:
            for directory in [prefix/'lib',prefix/'lib/x86_64-linux-gnu']:
                libraries.extend(sorted(directory.glob('lib'+package+'*.so*')))
        deps.append(dict(package=package,header_candidates=[str(x) for x in headers],
                         header_evidence=[dict(path=str(x),sha256=file_sha(x)) for x in headers if x.is_file()],
                         library_evidence=[dict(path=str(x),sha256=file_sha(x)) for x in libraries if x.is_file()],
                         cmake_config_candidates=[str(x) for x in candidates],
                         config_evidence=found,status='FOUND_CONFIG_ONLY' if found else 'MISSING'))
    header=root/HEADER
    identity_ok=header.is_file() and file_sha(header)==GUARD_SHA
    missing=[key for key,value in tools.items() if value is None]+[x['package'] for x in deps if x['status']=='MISSING']
    if not identity_ok:missing.append('original_guard_header_identity')
    return dict(status='BLOCKED' if missing else 'READY_FOR_COMPILE_ATTEMPT',tools=tools,
                required_language='C++17',minimum_cmake='3.16',prefixes=[str(x) for x in prefixes],
                dependencies=deps,missing=missing,original_guard_identity='PASS' if identity_ok else 'FAIL',
                python_version=sys.version.split()[0],python_compiler_setting=sysconfig.get_config_var('CC'),
                limitation='Config-file presence is not compile/link/ABI proof; no installed library is substituted.')


if __name__=='__main__':
    import json
    print(json.dumps(check(Path(__file__).resolve().parents[3]),indent=2))
