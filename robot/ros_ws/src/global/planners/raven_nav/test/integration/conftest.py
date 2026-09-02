"""Collection rules for raven_nav's integration tests.

`test_e2e_shared_rayfronts.py` is the full-chain smoke: it forks the REAL
`rayfronts.encoder_server` and `rayfronts.multi_robot_mapping_server`
processes plus the REAL `raven_nav_node`, drives them with fake robots on two
ROS domains and takes ~1-3 minutes.  It must NOT run as part of the ordinary
`python3 -m pytest test -q` sweep (which is expected to finish in seconds and
whose pass/skip counts are tracked), so it is not even collected unless
``RAVEN_E2E`` is set — that is what ``scripts/raven_rayfronts_tests.sh --e2e``
does.
"""
import os

collect_ignore = []

if os.environ.get('RAVEN_E2E', '').strip().lower() not in (
        '1', 'true', 'yes', 'on'):
    collect_ignore.append('test_e2e_shared_rayfronts.py')


def pytest_configure(config):
    config.addinivalue_line(
        'markers',
        'e2e: full-chain smoke — real encoder_server + shared '
        'multi_robot_mapping_server + real raven_nav_node against fake '
        'robots. Only collected when RAVEN_E2E=1.')
