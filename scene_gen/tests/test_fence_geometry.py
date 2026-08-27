#!/usr/bin/env python3
"""2D fence placement: does a run get covered, and do neighbours meet?

Exercises the REAL `_fence_run` tiler sliced out of suburb_scene.py (that
module imports pxr, so it cannot be imported here), against measured module
lengths, over the run lengths a real plat produces.
"""
import math, os, re, sys, textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
SRC = open(os.path.join(HERE, '..', 'suburb_scene.py')).read()

_i = SRC.index('def _fence_run')
_j = SRC.index('\ndef ', _i + 10)
_ns = {'math': math}
exec(textwrap.dedent(SRC[_i:_j]), _ns)
fence_run = _ns['_fence_run']

FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


def lay(length, mod_len):
    """Run the tiler along +X and return (n, fit, covered_span)."""
    run = fence_run((0.0, 0.0), (length, 0.0), mod_len)
    if not run:
        return 0, 0.0, 0.0
    fit = run[0][3]
    panel = mod_len * fit
    xs = sorted(r[0] for r in run)
    covered = (xs[-1] + panel / 2.0) - (xs[0] - panel / 2.0)
    return len(run), fit, covered


def test_coverage():
    print('\n[1] a laid run is covered end to end, no gaps, no overlap')
    bad_cov = bad_lap = 0
    for mod_len in (1.83, 2.44, 3.05, 5.95):        # 6/8/10/19.5 ft panels
        for length in [x * 0.5 for x in range(2, 121)]:   # 1 .. 60 m
            run = fence_run((0.0, 0.0), (length, 0.0), mod_len)
            if not run:
                continue
            panel = mod_len * run[0][3]
            xs = sorted(r[0] for r in run)
            if abs(((xs[-1] + panel / 2) - (xs[0] - panel / 2)) - length) > 0.05:
                bad_cov += 1
            for a, b in zip(xs, xs[1:]):
                if abs((b - a) - panel) > 0.05:      # abut exactly
                    bad_lap += 1
    check(bad_cov == 0, f'every laid run spans its boundary exactly ({bad_cov} bad)')
    check(bad_lap == 0, f'modules abut with no gap or overlap ({bad_lap} bad)')


def test_dropped_runs():
    print('\n[2] which boundaries get NO fence at all')
    rows = []
    for mod_len in (1.83, 2.44, 3.05, 5.95):
        dropped = [round(x * 0.5, 1) for x in range(2, 121)
                   if not fence_run((0.0, 0.0), (x * 0.5, 0.0), mod_len)]
        longest = max(dropped) if dropped else 0.0
        rows.append((mod_len, len(dropped), longest))
        print(f'        module {mod_len:4.2f} m -> {len(dropped):3d} of 119 '
              f'lengths unfenced, longest dropped = {longest:4.1f} m')
    # A boundary up to a full module long getting nothing is the hole the
    # renders show. Anything longer than one module MUST be fenced.
    worst = max(longest for _m, _n, longest in rows)
    worst_mod = max(m for m, _n, _l in rows)
    check(worst <= worst_mod,
          f'no boundary longer than one module is left bare '
          f'(worst dropped {worst:.1f} m vs module {worst_mod:.2f} m)')


def test_short_boundaries():
    print('\n[3] short boundaries specifically')
    for mod_len in (2.44, 5.95):
        for length in (1.5, 2.0, 3.0, 4.0, 6.0):
            n, fit, cov = lay(length, mod_len)
            state = 'BARE' if n == 0 else f'{n} x {mod_len * fit:.2f} m'
            print(f'        {length:4.1f} m boundary, {mod_len:4.2f} m module -> {state}')


def main():
    print('=' * 68)
    print('FENCE GEOMETRY (2D)')
    print('=' * 68)
    test_coverage()
    test_dropped_runs()
    test_short_boundaries()
    print('\n' + '=' * 68)
    if FAILS:
        print(f'{len(FAILS)} FAILED')
        for f in FAILS:
            print('   -', f)
        return 1
    print('all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
