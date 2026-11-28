"""Offline dependency resolution; local files only, no source/setup execution.

Includes the generated workspace prefix omitted by H. Resource availability is
separate from toolchain and generated ROS C++/typesupport availability.
"""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import hashlib
import re

PACKAGES={
    'ament_cmake':{'headers':[],'libraries':[]},
    'rclcpp':{'headers':['rclcpp/rclcpp.hpp'],'libraries':['librclcpp.so*']},
    'ament_index_cpp':{'headers':['ament_index_cpp/get_package_share_directory.hpp'],'libraries':['libament_index_cpp.so*']},
    'mavros_msgs':{'headers':['mavros_msgs/msg/state.hpp','mavros_msgs/msg/extended_state.hpp'],'libraries':['libmavros_msgs__rosidl_typesupport_cpp.so*']},
    'rcl_interfaces':{'headers':['rcl_interfaces/srv/get_parameters.hpp','rcl_interfaces/srv/set_parameters.hpp','rcl_interfaces/msg/parameter_type.hpp'], 'libraries':['librcl_interfaces__rosidl_typesupport_cpp.so*']},
    'std_msgs':{'headers':['std_msgs/msg/bool.hpp','std_msgs/msg/string.hpp'],'libraries':['libstd_msgs__rosidl_typesupport_cpp.so*']},
    'yaml-cpp':{'headers':['yaml-cpp/yaml.h'],'libraries':['libyaml-cpp.so*']}}
GUARD='robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp'
GUARD_SHA='e522a5f6ac10e293e6985b1aaca52d59fad7d80f91a5d5eed0aca0876f785ea2'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prefix_candidates(repository,extra=()):
    root=Path(repository);base=[Path('/opt/ros/jazzy'),Path('/usr'),Path('/usr/local')]
    install=root/'robot/ros_ws/install'
    if install.is_dir():
        base.append(install)
        base.extend(p for p in sorted(install.iterdir()) if p.is_dir())
    return list(dict.fromkeys([*base,*map(Path,extra)]))


def inspect_package(package,prefixes):
    configs=[];versions=[];headers={};libraries={}
    spec=PACKAGES.get(package,{'headers':[],'libraries':[]})
    for prefix in prefixes:
        for directory in [prefix/'share'/package/'cmake',prefix/'lib/cmake'/package,
                          prefix/'lib/x86_64-linux-gnu/cmake'/package,prefix/'cmake']:
            for name in [package+'Config.cmake',package+'-config.cmake']:
                path=directory/name
                if path.is_file():configs.append(path)
        package_xml=prefix/'share'/package/'package.xml'
        if package_xml.is_file():
            try:versions.append({'path':str(package_xml),'version':ET.parse(package_xml).getroot().findtext('version'),'sha256':sha(package_xml)})
            except ET.ParseError:pass
    for config in configs:
        for version_file in config.parent.glob('*[Vv]ersion.cmake'):
            match=re.search(r'set\(PACKAGE_VERSION\s+"?([0-9][0-9A-Za-z_.+-]*)',version_file.read_text())
            if match:
                versions.append({'path':str(version_file),'version':match.group(1),'sha256':sha(version_file)})
    for header in spec['headers']:
        candidates=[prefix/'include'/extra/header for prefix in prefixes for extra in ['',package]]
        headers[header]=[str(p) for p in candidates if p.is_file()]
    for pattern in spec['libraries']:
        libraries[pattern]=[str(p) for prefix in prefixes for directory in [prefix/'lib',prefix/'lib/x86_64-linux-gnu']
                            for p in sorted(directory.glob(pattern)) if p.is_file()]
    if not configs:status='DECLARED BUT NOT LOCALLY AVAILABLE'
    elif not all(headers.values()) or not all(libraries.values()):
        status='GENERATED ARTIFACT NOT AVAILABLE' if package in {'mavros_msgs','rcl_interfaces','std_msgs'} else 'DECLARED BUT NOT LOCALLY AVAILABLE'
    elif not versions:status='VERSION UNRESOLVED'
    else:status='AVAILABLE AND VERIFIED'
    return {'package':package,'availability':status,'configs':[{'path':str(p),'sha256':sha(p),
             'symlink_target':str(p.readlink()) if p.is_symlink() else None} for p in dict.fromkeys(configs)],
            'versions':versions,'required_headers':headers,'required_libraries':libraries,
            'verification_limit':'Read-only identity evidence; no configure/link/ABI/runtime proof'}


def inspect(repository,extra_prefixes=(),which=shutil.which):
    root=Path(repository);prefixes=prefix_candidates(root,extra_prefixes)
    packages={name:inspect_package(name,prefixes) for name in PACKAGES}
    tools={'cxx':next((p for p in [which('c++'),which('g++'),which('clang++')] if p),None),
           'cmake':which('cmake'),'colcon_optional_for_direct_cmake':which('colcon')}
    resource=inspect_package('interface_bringup',prefixes)
    resource_file=root/'robot/ros_ws/src/interface/interface_bringup/config/px4_config.yaml'
    guard=root/GUARD
    guard_ok=guard.is_file() and sha(guard)==GUARD_SHA
    missing=[name for name in ['cxx','cmake'] if not tools[name]]
    missing += [name for name,item in packages.items() if item['availability']!='AVAILABLE AND VERIFIED']
    if not guard_ok:missing.append('original_guard_identity')
    return {'intended_environment':'AirStack robot desktop development container; ROS2 Jazzy underlay',
            'preferred_path':'B_CONTAINER_BUILD; narrow RRM-only CMake build using existing underlay',
            'tools':tools,'prefixes':[str(p) for p in prefixes],'packages':packages,
            'runtime_resource':{**resource,'native_build_requirement':False,
                                'explicit_config_file':str(resource_file),'explicit_config_sha256':sha(resource_file) if resource_file.is_file() else None},
            'original_guard_identity':'PASS' if guard_ok else 'FAIL',
            'native_missing':missing,'readiness':'BLOCKED' if missing else 'READY_FOR_NATIVE_ATTEMPT',
            'installed_message_definitions':'NOT LOCALLY AVAILABLE; generated headers/typesupport cannot be supplied by documentation',
            'runtime_containment':'NOT VERIFIED'}
