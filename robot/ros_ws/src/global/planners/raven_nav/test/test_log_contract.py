"""raven's stdout is a wire format.

`semantic_search_task._filter_raven` runs over every line raven_nav prints and
drops everything that does not match one of a handful of literal substrings or
the bracketed mode-tag regex; whatever survives becomes the operator-facing
action feedback. Reword a log line and the feedback silently goes blank.

`_filter_raven` is loaded by file path — importing the module would pull in
rclpy, which the host does not have.
"""
import pathlib
import re
import types

import pytest

from raven_nav import params as P

HERE = pathlib.Path(__file__).resolve().parent
SST_NODE = (HERE.parents[1] / 'semantic_search_task' / 'semantic_search_task'
            / 'node.py')


def _load_filter_raven():
    """Extract `_clean` + `_filter_raven` from semantic_search_task/node.py and
    exec just those two functions, so no ROS import happens."""
    src = SST_NODE.read_text()
    import ast
    tree = ast.parse(src)
    wanted = {'_clean', '_filter_raven', '_ANSI_RE', '_ROS_PREFIX_RE'}
    keep = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef,)) and node.name in wanted:
            keep.append(node)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id in wanted:
                    keep.append(node)
    mod = types.ModuleType('sst_filter')
    mod.__dict__['re'] = re
    exec(compile(ast.Module(body=keep, type_ignores=[]), '<sst>', 'exec'),
         mod.__dict__)
    return mod.__dict__['_filter_raven']


pytestmark = pytest.mark.skipif(not SST_NODE.exists(),
                                reason='semantic_search_task absent')


@pytest.fixture(scope='module')
def filter_raven():
    return _load_filter_raven()


def test_startup_line_survives_the_filter(filter_raven):
    line = (f'{P.LOG_STARTED} | robot=robot_1 | timer=0.50s | '
            "query_labels=['person'] | targets=['person']")
    assert filter_raven(line) == 'raven_nav started'


def test_waiting_for_odometry_survives(filter_raven):
    assert filter_raven(P.LOG_WAITING_ODOM) == 'Waiting for odometry...'


def test_boot_gps_line_survives(filter_raven):
    line = f'{P.LOG_BOOT_GPS}: alt_ground=3.00m, boot_enu=(1.0, 2.0, 3.0)'
    assert filter_raven(line) == line


def test_search_area_line_survives(filter_raven):
    line = f'{P.LOG_SEARCH_AREA} updated: 4 vertices.'
    assert filter_raven(line) == line


@pytest.mark.parametrize('mode', ['Frontier-based', 'Ray-based', 'Voxel-based',
                                  'LVLM-guided'])
def test_every_status_line_survives(filter_raven, mode):
    line = P.format_status_line(mode, 'person', ['person'])
    assert filter_raven(line) == line


def test_status_line_starts_with_the_bracketed_mode(filter_raven):
    assert P.format_status_line('Ray-based').startswith('[Ray-based]')
    assert P.format_status_line('Ray-based', 'person') == \
        '[Ray-based] target=person'
    assert P.format_status_line('Ray-based', '', ['a', 'b']) == \
        "[Ray-based] completed=['a', 'b']"


def test_lvlm_guided_is_in_the_upstream_regex():
    """WP-C adds LVLM-guided to `_filter_raven`; without it the whole
    LVLM-mode feedback disappears."""
    src = SST_NODE.read_text()
    m = re.search(r"re\.search\(r'\\\[\((.+?)\)\\\]'", src)
    assert m, 'the mode-tag regex moved'
    tags = set(m.group(1).split('|'))
    assert tags == set(P.STATUS_TAGS), \
        f'raven publishes {sorted(P.STATUS_TAGS)}, the filter matches {sorted(tags)}'


def test_nav_mode_tags_are_the_documented_vocabulary():
    assert set(P.NAV_MODE_TAG.values()) == {
        'frontier', 'ray', 'voxel', 'lvlm', 'complete'}
    # semantic_search_task ends a run on this exact literal.
    assert P.NAV_MODE_TAG['Complete'] == 'complete'
    assert P.NAV_MODE_IDLE == 'idle'


def test_complete_is_the_string_semantic_search_task_watches_for():
    src = SST_NODE.read_text()
    assert "nav_mode == 'complete'" in src


def test_an_unrelated_line_is_dropped(filter_raven):
    assert filter_raven('some incidental chatter') is None


def test_error_lines_are_escalated(filter_raven):
    assert filter_raven('Traceback (most recent call last)').startswith('ERROR:')


def test_the_node_actually_logs_those_literals():
    src = (HERE.parent / 'raven_nav' / 'raven_nav_node.py').read_text()
    for const in ('P.LOG_STARTED', 'P.LOG_WAITING_ODOM', 'P.LOG_BOOT_GPS',
                  'P.LOG_SEARCH_AREA', 'P.format_status_line'):
        assert const in src, f'{const} is no longer used by the node'
