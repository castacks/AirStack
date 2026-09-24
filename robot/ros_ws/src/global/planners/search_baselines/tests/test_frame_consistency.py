#!/usr/bin/env python3
"""Markers, frontier cloud and the search-area test share ONE transform.

The search-area outline is drawn from CONFIG coordinates and renders in the
right place; markers were drawn from raw grid coordinates and rendered offset
from the drone. Both must go through the same grid->map transform the
published path uses (`_build_path` subtracts `offset`), or the picture
disagrees with the plan and the point-in-polygon test judges the robot out of
bounds while it is inside its sector.

Pure logic — no ROS, no sim.
"""
import os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, '..', 'search_baselines', 'planner_node.py')).read()
FAILS = []


def check(c, m):
    print(('  PASS  ' if c else '  FAIL  ') + m)
    if not c:
        FAILS.append(m)


class S:
    """Stub carrying only what the transform helpers touch."""
    def __init__(self, off):
        self._marker_offset = off


def _bind(name):
    i = SRC.index(f'    def {name}(self')
    j = SRC.index('\n    def ', i + 10)
    import textwrap
    ns = {'np': np}
    exec('class T:\n' + textwrap.indent(textwrap.dedent(SRC[i:j]), '    '), ns)
    return getattr(ns['T'], name)


to_map = _bind('_to_map')
to_map_arr = _bind('_to_map_arr')


def main():
    print('=' * 66)
    print('FRAME CONSISTENCY')
    print('=' * 66)

    print('\n[1] the transform matches what _build_path applies')
    # _build_path does: position.x = x - offset[0]
    off = np.array([12.0, -7.0, 0.0])
    s = S(off)
    gx, gy = 100.0, 50.0
    mx, my = to_map(s, gx, gy)
    check((mx, my) == (gx - off[0], gy - off[1]),
          f'grid ({gx}, {gy}) -> map ({mx}, {my}) == grid - offset')

    print('\n[2] no offset yet -> identity, never a crash')
    s0 = S(None)
    check(to_map(s0, 3.0, 4.0) == (3.0, 4.0), 'identity before the first tick')
    a = np.array([[1.0, 2.0], [3.0, 4.0]])
    check(np.array_equal(to_map_arr(s0, a), a), 'array form is identity too')
    check(to_map_arr(s0, np.zeros((0, 2))).size == 0, 'empty candidate set is safe')

    print('\n[3] array and scalar forms agree')
    pts = np.array([[100.0, 50.0], [-20.0, 8.0], [0.0, 0.0]])
    got = to_map_arr(s, pts)
    want = np.array([to_map(s, p[0], p[1]) for p in pts])
    check(np.allclose(got, want), 'vectorised transform == per-point transform')

    print('\n[4] every VIZ coordinate goes through it')
    for pat, what in (
        (r'cx, cy = self\._to_map\(\*agent\.grid_to_map_xy', 'frontier centroids'),
        (r'gx, gy = self\._to_map\(\*agent\.grid_to_map_xy', 'frontier region cells'),
        (r'here = self\._to_map\(\*self\._agent_xy', 'robot marker + trail'),
        (r'mx, my = self\._to_map\(\*o3d_xz_to_map_xy', 'raw detection cloud'),
        (r'_tx, _ty = self\._to_map\(', 'target discs'),
        (r'self\._points_in_polygon\(self\._to_map_arr', 'search-area test'),
    ):
        check(re.search(pat, SRC) is not None, what)

    print('\n[5] internal grid math is NOT transformed (would double-apply)')
    # the goal handed to _build_path must stay in grid frame
    check(re.search(r'return agent\.grid_to_map_xy\(\*self\._goal_points\[i\]\)', SRC)
          is not None, '_goal_xy still returns grid coords for _build_path')

    print('\n' + '=' * 66)
    if FAILS:
        print(f'{len(FAILS)} FAILED')
        for f in FAILS:
            print('   -', f)
        return 1
    print('all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
