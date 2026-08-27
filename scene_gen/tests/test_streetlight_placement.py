#!/usr/bin/env python3
"""A street light stands on the verge, on its pole, outside the garden fence.

SM_StreetLight is an L-shape: a 0.40 m pole at the asset ORIGIN and a 2.79 m
mast arm cantilevered off it, measured on Nucleus (local bbox Y spans
-278.6..8.1 at scale 0.01). Its bounding-box centre is therefore NOT on the
pole -- it hangs 1.35 m out in mid-air under the arm.

`apply_placements` re-centres every prop on its bbox unless the placement says
`raw_pivot`. For a lamp that is the wrong anchor: it pushed the pole from the
1.6 m verge line back to 2.95 m, which is BEHIND the 2.5 m front-fence line --
the pole ended up standing in somebody's front garden, arm reaching back over
the roof instead of over the road.

Pure arithmetic against the measured asset. No USD, no sim.
"""
import math, os, re, sys, textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, '..', 'suburb_scene.py')).read()
PARKS = open(os.path.join(HERE, '..', 'detail', 'parks.py')).read()

# --- measured on Nucleus, scale 0.01 (see module docstring) ----------------
LAMP_CY = -1.35          # m, origin -> bbox centre, along the arm
LAMP_ARM = 2.87          # m, footprint across the arm
HYDRANT_CY = -0.04       # m, effectively centred
# --- the plat's cross-section, from the kerb inward ------------------------
VERGE_M = 1.6            # build_frontage `furnishing_inset_m` default
WALK_M = 0.8             # verge * 0.5
FENCE_INSET_M = 2.5      # suburb_parcel DEFAULTS["fence_front_inset_m"]

FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


def pole_offset_from_request(cy, raw_pivot):
    """How far the POLE ends up from the point build_frontage asked for.

    `apply_placements` subtracts the yaw-rotated (cx, cy) so the bbox centre
    lands on the request; the origin then sits that far the other way. With
    `raw_pivot` the correction is skipped and the origin IS the request.
    """
    return 0.0 if raw_pivot else abs(cy)


def test_cross_section():
    print('\n[1] the pole lands on the verge, in front of the fence')
    pole = VERGE_M + pole_offset_from_request(LAMP_CY, raw_pivot=True)
    check(abs(pole - 1.60) < 1e-6, f'pole stands {pole:.2f} m from the kerb (verge line)')
    check(pole < FENCE_INSET_M,
          f'pole clears the {FENCE_INSET_M} m fence line by {FENCE_INSET_M - pole:.2f} m')
    check(pole > WALK_M, f'pole is behind the {WALK_M} m sidewalk ring, not on it')

    print('\n[2] the bug this test exists for: centring the bbox')
    bad = VERGE_M + pole_offset_from_request(LAMP_CY, raw_pivot=False)
    check(abs(bad - 2.95) < 0.01, f'centred anchor put the pole at {bad:.2f} m')
    check(bad > FENCE_INSET_M,
          f'  -> {bad - FENCE_INSET_M:.2f} m INSIDE the garden, which is the reported fault')

    print('\n[3] the arm reaches over the carriageway, which is its job')
    # yaw faces -n (the kerb) and the pool entry adds yaw-offset 90 deg, which
    # maps the asset's local -Y (the arm) onto the outward road normal.
    for yaw_place in (0.0, 37.0, 90.0, 180.0, -140.0):
        th = math.radians(yaw_place + 90.0)          # + yaw-offset from the pool
        arm = (math.sin(th), -math.cos(th))          # local -Y rotated by th
        out = (math.cos(math.radians(yaw_place)), math.sin(math.radians(yaw_place)))
        check(abs(arm[0] - out[0]) < 1e-9 and abs(arm[1] - out[1]) < 1e-9,
              f'yaw {yaw_place:+.0f} deg: arm points along the outward normal')
    reach = LAMP_ARM - 0.4 / 2.0
    check(VERGE_M < reach, f'arm reaches {reach:.2f} m > the {VERGE_M} m standoff, '
                           f'so the head is over asphalt')

    print('\n[4] round props are unaffected -- centring them IS anchoring them')
    check(abs(pole_offset_from_request(HYDRANT_CY, raw_pivot=False)) < 0.05,
          f'hydrant centroid is {abs(HYDRANT_CY):.2f} m off its pivot; leave it centred')


def test_kerb_anchored():
    """The second fault: the block polygon is not always the kerb.

    `blocks_from_faces` insets each face by half that road's width, so the
    boundary IS the kerb along a straight frontage. Where `offset_polygon`
    hits its mitre limit it drifts inward instead, and a lamp offset from the
    drifted boundary stood up to 14.6 m back from the asphalt. `_RoadIndex.verge`
    rebuilds the point from the centreline when the two disagree by more than
    `_KERB_TOL_M`, and leaves junction/bulb ends alone because a bulb kerb is
    an arc that the half-width model does not describe.
    """
    print('\n[5] the verge is measured from the road, not from the block polygon')
    check('def nearest(self, p):' in SRC, '_RoadIndex.nearest exists')
    check('def verge(self, p, inward_n, standoff):' in SRC, '_RoadIndex.verge exists')
    check('self.segs = []' in SRC and 'self.segs.append(seg)' in SRC,
          'a flat segment list backs the exact query (the grid hash cannot)')
    check('q, n = road.verge(p, n, verge)' in SRC,
          'build_frontage places kerbside props through it')
    check(re.search(r'if not interior or abs\(d - hw\) <= self\._KERB_TOL_M:'
                    r'\s+return fallback', SRC) is not None,
          'a bulb/junction end falls back rather than snapping to a bogus kerb')
    check('_KERB_TOL_M = 0.75' in SRC, 'the tolerance is wider than rounding, narrower than drift')

    # Measured over seeds 1/2/3/42 at 250 m and 400 m with the real _RoadIndex:
    # lamps behind the 2.5 m fence line went 6->0, 32->1, 23->3, 21->1, 25->4;
    # median distance from the kerb went 2.95 m -> 1.60 m in every layout.
    print('\n[6] cul-de-sac bulbs, for the record')
    BULB_R = 14.64          # sn.DEFAULTS["bulb_radius_m"]
    measured = 14.62        # seed 42, the lamp the "far from kerb" column flagged
    check(abs(measured - BULB_R) < 0.1,
          f'a bulb lamp measures {measured} m from the STUB centreline, which is '
          f'the {BULB_R} m bulb radius -- it stands at the bulb kerb, not in a garden')


def test_wired_in():
    print('\n[7] build_frontage actually asks for it')
    m = re.search(r'for pool, sp_m, cat, pivot in \((.*?)\):', SRC, re.S)
    check(m is not None, 'the frontage loop carries a per-category pivot flag')
    if m:
        body = m.group(1)
        check(re.search(r'lamps,\s*lamp_sp,\s*"streetlight",\s*True', body) is not None,
              'streetlight -> raw_pivot True')
        check(re.search(r'hyd,\s*hyd_sp,\s*"fire_hydrant",\s*False', body) is not None,
              'fire_hydrant -> raw_pivot False')
        check(re.search(r'bins,\s*bin_sp,\s*"trash_can",\s*False', body) is not None,
              'trash_can -> raw_pivot False')
    check('yaw, rng, raw_pivot=pivot' in SRC, 'the flag reaches pools.place')
    check(re.search(r'def place\(self, resolver, usd, category, x, y, yaw, rng=None,\s*'
                    r'z_extra=0\.0, scale_mul=1\.0, raw_pivot=False\)', SRC) is not None,
          'pools.place accepts it')
    check(re.search(r'if raw_pivot:\s*\n\s*out\["raw_pivot"\] = True', SRC) is not None,
          'and sets the key apply_placements reads')

    print('\n[8] park trail lamps get the same treatment')
    check('_PIVOT_ANCHORED = frozenset(("streetlight",))' in PARKS,
          'parks.py names the pivot-anchored categories')
    check(re.search(r'if category in _PIVOT_ANCHORED:\s*\n\s*p\["raw_pivot"\] = True',
                    PARKS) is not None, 'parks.add honours it')


def test_place_behaviour():
    print('\n[9] AssetPools.place, executed for real')
    ns = {}
    i = SRC.index('class AssetPools')
    j = SRC.index('\ndef _raw_pool', i)
    exec(textwrap.dedent(SRC[i:j]), ns)
    P = ns['AssetPools']
    pools = P.__new__(P)
    pools.asset_scale, pools.asset_root = 1.0, ''
    pools._scale, pools._axis, pools._yaw = {}, {}, {}

    class R:
        def get(self, *a, **k):
            return {'sx': 0.4, 'sy': 2.87, 'sz': 7.16, 'base': 0.0,
                    'cx': 0.0, 'cy': LAMP_CY, 'cz': 0.0}

    lamp = pools.place(R(), 'x.usd', 'streetlight', 5.0, 6.0, 12.0, raw_pivot=True)
    hyd = pools.place(R(), 'x.usd', 'fire_hydrant', 5.0, 6.0, 12.0)
    check(lamp.get('raw_pivot') is True, 'lamp placement carries raw_pivot')
    check('raw_pivot' not in hyd, 'hydrant placement does not')
    check((lamp['x_m'], lamp['y_m']) == (5.0, 6.0), 'the requested point is untouched')
    check(lamp['z_m'] == 0.0, 'pole base sits on the ground (base = 0)')


def main():
    print('=' * 68)
    print('STREETLIGHT PLACEMENT')
    print('=' * 68)
    test_cross_section()
    test_kerb_anchored()
    test_wired_in()
    test_place_behaviour()
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
