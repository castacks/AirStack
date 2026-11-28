#!/usr/bin/env python3
"""Validate the closed RRM design schema, never execute or resolve Compose.

PASS means policy intent only. Does not inspect Docker, devices, sockets or ROS.
Unknown keys and inherited inputs fail closed. stdlib only.
"""
import json
import sys
from pathlib import Path

TOP = {'kind', 'runtime_verified', 'components', 'network', 'environment', 'mounts',
       'privileged', 'pid', 'ipc', 'cap_add', 'cap_drop', 'security_opt', 'devices',
       'device_cgroup_rules', 'ports', 'entrypoint_policy', 'overlays', 'gpu_policy'}
COMPONENTS = ['isaac_pegasus_px4_sitl', 'mavros', 'original_guard_host', 'independent_recorder']
ENV = {'AUTOLAUNCH': 'false', 'SIM_TYPE': 'isaac', 'FCU_URL': 'udp://:14541@172.30.50.2:14581',
       'GCS_URL': '', 'ROS_DOMAIN_ID': '1', 'AIRSTACK_STACK_ENTRY': 'startup_only',
       'AIRSTACK_STACK_DIR': '/rrm/design-stack', 'SIM_IP': '172.30.50.2',
       'DDS_POLICY': 'explicit-confined-peers-only', 'PLAY_SIM_ON_START': 'false'}
NETWORK = {'mode': 'private_bridge', 'internal': True, 'external_routes': [],
           'peers': ['172.30.50.2', '172.30.50.3'], 'dds_external_discovery': False}


def validate(data):
    errors = []
    if not isinstance(data, dict):
        return ['root must be object']
    if set(data) != TOP:
        errors.append('unknown or missing policy keys')
    exact = {'kind': 'NON_EXECUTABLE_UNVALIDATED_DESIGN', 'runtime_verified': False,
             'components': COMPONENTS, 'network': NETWORK, 'environment': ENV,
             'privileged': False, 'pid': 'private', 'ipc': 'private', 'cap_add': [],
             'cap_drop': ['ALL'], 'security_opt': ['no-new-privileges:true'],
             'devices': [], 'device_cgroup_rules': [], 'ports': [],
             'entrypoint_policy': 'reviewed-explicit-SITL-only-no-fallback', 'overlays': [],
             'gpu_policy': 'UNVERIFIED_MINIMUM_COMPUTE_GRANT_NOT_APPROVED'}
    for key, value in exact.items():
        # JSON equality alone treats 0 as False; require exact JSON scalar types too.
        if json.dumps(data.get(key), sort_keys=True) != json.dumps(value, sort_keys=True):
            errors.append('unsafe or unresolved ' + key)
    mounts = data.get('mounts')
    approved = [dict(source='pinned_source', target='/rrm/source', type='named_design_volume', read_only=True),
                dict(source='pinned_assets', target='/rrm/assets', type='named_design_volume', read_only=True),
                dict(source='private_cache', target='/rrm/cache', type='named_design_volume', read_only=False),
                dict(source='private_evidence', target='/rrm/evidence', type='named_design_volume', read_only=False)]
    if json.dumps(mounts, sort_keys=True) != json.dumps(approved, sort_keys=True):
        errors.append('unapproved mount or writable source; device/socket/host binds forbidden')
    return errors


def read_design(path):
    def unique(pairs):
        d = {}
        for key, value in pairs:
            if key in d:
                raise ValueError('duplicate JSON key: ' + key)
            d[key] = value
        return d
    return json.loads(Path(path).read_text(), object_pairs_hook=unique,
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError('non-JSON constant')))


if __name__ == '__main__':
    try:
        issues = validate(read_design(sys.argv[1]))
    except (ValueError, OSError, IndexError) as exc:
        issues = [str(exc)]
    print(json.dumps({'result': 'FAIL' if issues else 'PASS_STATIC_INTENT_ONLY',
                      'runtime_isolation': 'NOT YET VERIFIED', 'errors': issues}, indent=2))
    sys.exit(bool(issues))
