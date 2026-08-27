#!/usr/bin/env python3
"""Fence rules on generated suburb lots.

1. Cul-de-sac (wedge) lots carry NO fence — a perimeter fence on a pie-slice
   lot narrows to a point at the kerb and does not read as a fence.
2. A BOUNDARY BETWEEN TWO FENCED NEIGHBOURS IS CONTINUOUSLY FENCED —
   measured as coverage along the shared ground, not as the distance between
   two runs' endpoints. See `_boundary_holes` for why the endpoint proxy this
   replaces was reporting 34 of 61 pairs on seed 3 as failures while every one
   of those boundaries was fully fenced.
3. NO FENCE RUN STANDS ACROSS ITS OWN LOT'S DRIVEWAY — front, side or rear.
4. EVERY LOT THAT SHOWS A FENCE ENCLOSES ITS BACK YARD WITH IT.

Offline: no pxr, no Isaac Sim. Tests 1-3 need the plat only and run on
`parcel_blocks` directly; test 4 cannot, and the reason is the whole point of
it. `fence_segs` is what the PLAT PROPOSED, and three cuts stand between it and
the ground — `_trim_to_building_line`, `_trim_offroad` and the `_FenceGrid`
clash trim — so a rule stated over the proposal is a rule about a fence nobody
can see. It is stated over `h["fence_drawn"]`, which `build_placements` writes
from the modules that actually went down, and reached through `tools/fence_png`
because that module stubs `pxr` before importing `suburb_scene`.
"""
import inspect, math, os, random, sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
sys.path.insert(0, os.path.join(_HERE, '..', 'tools'))
from layout import suburb_net as sn
from detail import suburb_parcel as sp
# Installs the pxr stubs on import; `suburb_scene` has to be imported after it.
import fence_png as fp                                             # noqa: E402
import suburb_scene as ss                                          # noqa: E402

FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


WALK_HALF_M = 1.5


def lots(region=250.0, seed=3, pcfg=None):
    net = sn.generate(region, region, random.Random(seed))[0]
    blocks = sn.blocks_from_faces(net, sn.faces(net))
    return sp.parcel_blocks(blocks, random.Random(seed), pcfg)


def test_no_fence_on_cul_de_sac():
    print('\n[1] cul-de-sac lots carry no fence')
    total = wedge_n = wedge_fenced = 0
    for region, seed in ((250.0, 3), (400.0, 7), (250.0, 11)):
        for b in lots(region, seed):
            for h in (b.get('houses') or []):
                total += 1
                if h.get('wedge_lot'):
                    wedge_n += 1
                    if h.get('fence_segs') or h.get('has_fence'):
                        wedge_fenced += 1
    print(f'        houses={total}  wedge/cul-de-sac={wedge_n}  of which fenced={wedge_fenced}')
    if wedge_n == 0:
        print('        (no wedge lots in these seeds — rule not exercised here)')
    check(wedge_fenced == 0, 'no cul-de-sac lot carries a fence')


# -- what "two neighbours share a fence" actually means -----------------------
# `sp._line_dupe`'s OWN THRESHOLDS, read off the function rather than copied.
# It is the plat's definition of "these two lines are one boundary" — 1.2 m
# apart at the middle of the overlap, near-parallel, overlapping by more than
# half a metre — and this test has to mean the same thing by it or it is asking
# about different ground. Read through `inspect` so a change over there lands
# here without anybody remembering to look.
_DUPE = inspect.signature(sp._line_dupe).parameters
_SAME_LINE_M = float(_DUPE['tol'].default)            # 1.20
_SAME_LINE_COS = float(_DUPE['cos_tol'].default)      # 0.92
_SAME_LINE_OV = float(_DUPE['min_ov'].default)        # 0.50

# How finely a boundary is walked when looking for a hole in it. `_edge_cover`
# samples at `_YARD_SAMPLE_M` (1.0 m) because its answer is only ever compared
# against 0.85 and a metre already decides that; a HOLE is a length and wants
# better than metre resolution, so this is 0.25 m — a quarter of the shortest
# fence module in the pool (2.0 m).
_HOLE_STEP_M = 0.25

# THE BAR. Measured over the shipped preset, seeds 1/3/5/7: 236 boundaries
# shared by two fenced neighbours and the worst hole in any of them is 0.00 m.
# So this is not a tolerance the pass is spending — it is headroom, set at half
# a metre because that is the width of a missing post rather than of a missing
# panel (the pool's modules are 2.00, 3.51 and 5.28 m long), and because
# `fence_check`'s run-continuity test already calls 0.25 m between consecutive
# modules a break.
#
# AND THE BAR THIS IS *NOT*. `suburb_scene._YARD_FENCE_COVER` is 0.85, which on
# a 30 m boundary tolerates 4.5 m of hole, and the measurement above looks like
# an argument for raising it — 94% of the enclosure edges on seed 3 have no
# hole over 0.5 m. It is not, and the reason is that the measurement is
# CONDITIONED ON SURVIVAL: it is taken over the lots that already passed the
# 0.85 gate. Measured by rebuilding seeds 1/3/5/7 with the bar moved:
#
#     bar    fenced lots (4 seeds)    modules    seating props
#     0.85          458                13,482        1,437
#     0.90          428  (-6.6%)       12,666        1,389
#     0.95          362 (-21.0%)       11,340        1,288
#     1.00          310 (-32.3%)       10,145        1,187
#
# Only 48 of the 458 lots (10.5%) have a `fence_cover` in [0.85, 0.95) — but
# 96 are lost at 0.95, exactly double, because the all-or-nothing sweep
# ITERATES: every lot it drops takes its shared side line with it and un-closes
# a neighbour. A fifth of the visible fence in the suburb is not headroom
# nobody is using, so 0.85 stays where it is.
MAX_BOUNDARY_HOLE_M = 0.5


def _shared_stretch(e0, e1):
    """The ground two enclosure edges have in common, or None.

    WHY THE EDGES AND NOT `fence_segs`. The obvious way to ask "do these two
    lots share a boundary" is to look for a platted line in both — and it never
    fires: `_relay` moves a shared boundary onto ONE lot's list, so the other
    lot has no seg on that line at all. Measured on seed 3, zero of the 121
    fenced lots pair up that way. `h["rear_edges"]` is struck from the numbers
    each lot was ISSUED on, so both neighbours publish their own view of the
    same ground and the two views can be matched.

    THE OVERLAP, NOT EITHER EDGE. The two edges start at their own lots'
    building lines and stop at their own rear lines, which are different
    depths; the stretch that is genuinely SHARED is where they shadow each
    other, and that is what a claim about "the boundary between them" can be
    made over.
    """
    a, b = e0
    q0, q1 = e1
    d = sp._unit(sp._sub(b, a))
    e = sp._unit(sp._sub(q1, q0))
    if abs(sp._dot(d, e)) < _SAME_LINE_COS:
        return None
    ln = sp._dist(a, b)
    t0, t1 = sp._dot(sp._sub(q0, a), d), sp._dot(sp._sub(q1, a), d)
    lo, hi = max(min(t0, t1), 0.0), min(max(t0, t1), ln)
    if hi - lo <= _SAME_LINE_OV:
        return None
    mid = sp._add(a, sp._mul(d, (lo + hi) * 0.5))
    k = max(0.0, min(sp._dist(q0, q1), sp._dot(sp._sub(mid, q0), e)))
    if sp._dist(mid, sp._add(q0, sp._mul(e, k))) > _SAME_LINE_M:
        return None
    return (sp._add(a, sp._mul(d, lo)), sp._add(a, sp._mul(d, hi)))


def _boundary_holes(p0, p1, fences):
    """Stretches of p0->p1 with no fence on them, as ``(start, end)`` metres.

    COVERAGE, NOT ENDPOINT PROXIMITY, and that is the entire correction this
    test exists for. The rule it replaces measured the distance between the
    nearest pair of run ENDPOINTS belonging to two neighbours, and `_line_dupe`
    made that meaningless: a shared boundary is fenced ONCE, by the lot issued
    first, so the other lot has no run on the line and its nearest endpoint is
    its own rear or far-side run, legitimately metres away. It scored 34 of 61
    pairs as failures on seed 3 while every one of those boundaries was
    continuously fenced. The metric only meant anything when both neighbours
    laid their own run down the shared line — which is the doubled-fence defect
    `_FenceGrid` was written to stop.

    THE SAME COVERED TEST `_edge_cover` USES: a sample is covered when a drawn
    span passes within `_YARD_FENCE_REACH_M` of it. Written out rather than
    called because `_edge_cover` returns a FRACTION and a fraction cannot
    distinguish one 4 m hole from eight half-metre ones — which is exactly the
    difference between a gate and a fence that is falling down.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return L, []
    reach = ss._YARD_FENCE_REACH_M
    x0, x1 = min(p0[0], p1[0]) - reach, max(p0[0], p1[0]) + reach
    y0, y1 = min(p0[1], p1[1]) - reach, max(p0[1], p1[1]) + reach
    near = []
    for (a, b) in fences:
        if (max(a[0], b[0]) < x0 or min(a[0], b[0]) > x1
                or max(a[1], b[1]) < y0 or min(a[1], b[1]) > y1):
            continue
        ex, ey = b[0] - a[0], b[1] - a[1]
        near.append((a[0], a[1], ex, ey, max(ex * ex + ey * ey, 1e-12)))
    n = max(2, int(math.ceil(L / _HOLE_STEP_M)) + 1)
    rr = reach * reach
    mask = []
    for i in range(n):
        s = i / float(n - 1)
        qx, qy = p0[0] + dx * s, p0[1] + dy * s
        hit = False
        for (ax, ay, ex, ey, e2) in near:
            t = ((qx - ax) * ex + (qy - ay) * ey) / e2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            if (qx - ax - ex * t) ** 2 + (qy - ay - ey * t) ** 2 <= rr:
                hit = True
                break
        mask.append(hit)
    out, run, ri = [], 0, 0
    for i, c in enumerate(mask + [True]):
        if c:
            if run:
                out.append((ri * L / (n - 1.0), (ri + run) * L / (n - 1.0)))
            run = 0
        else:
            if run == 0:
                ri = i
            run += 1
    return L, out


def _openings(h):
    """This lot's legitimate breaks, as ``(p, u, n, offset, half, max_depth)``.

    `front_gaps` IS BOTH CARVE-OUTS AT ONCE. The old comment on this test
    excused a drive running along the shared line, and the plat now derives ONE
    `drive_off` that the front opening, the apron cuts and the paved ribbon all
    read — it is in `front_gaps` alongside the kit's own door and garage-door
    gaps. `suburb_scene._gate_returns` breaks the gate return on that same
    list, so the drive opening and the gate opening are one test rather than
    two, and a change to where the plat sites a drive moves both.

    The depth ceiling is `_gate_returns`' own: it refuses a side run whose
    front end is past the back wall, so no `front_gaps` cut reaches deeper than
    that. Below it a hole on the drive's bearing is a gate; past it, it is a
    hole in the back garden.
    """
    p, u, n, d = h.get("frontage"), h.get("u"), h.get("n"), h.get("d")
    stop = ss._building_line_depth(h)
    if stop is None or not (p and u and n) or not d:
        return []
    return [(p, u, n, float(o), float(hw), stop + float(d) + 0.5)
            for (o, hw) in (h.get("front_gaps") or ())]


def _is_opening(q0, q1, lots, pave, tol=0.6):
    """Is the hole q0->q1 an opening somebody put there on purpose?

    THE WHOLE HOLE, NOT ITS MIDPOINT. A 20 m hole whose centre happens to fall
    on a 3 m drive is not a drive; both ends have to sit inside the opening for
    the hole to BE the opening.
    """
    for h in lots:
        for (p, u, n, o, hw, dmax) in _openings(h):
            ok = True
            for q in (q0, q1):
                al = (q[0] - p[0]) * u[0] + (q[1] - p[1]) * u[1]
                dp = (q[0] - p[0]) * n[0] + (q[1] - p[1]) * n[1]
                if abs(al - o) > hw + tol or not (-tol <= dp <= dmax + tol):
                    ok = False
                    break
            if ok:
                return True
    # ...and the paving itself, which is what `apply_ground` will actually lay.
    # Kept as a second test because a kit plan's drive can sit where the plat's
    # `front_gaps` does not, and `fence_png.dangling` already excuses an end on
    # exactly these ribbons.
    mid = ((q0[0] + q1[0]) / 2.0, (q0[1] + q1[1]) / 2.0)
    probe = sp._corners(mid[0], mid[1], 0.02, 0.02, 1.0, 0.0)
    return any(sp._obb_overlap(probe, b, pad=-0.30) for b in pave)


def _shared_boundaries(scene):
    """Every stretch of ground two FENCED neighbours have in common."""
    fenced = [h for h in fp.houses(scene) if h.get('fence_drawn')
              and h.get('rear_edges')]
    out = []
    for i, h in enumerate(fenced):
        for g in fenced[i + 1:]:
            # A CULL, NOT A RULE. Two lots that share a boundary have their
            # house centres one lot width or one lot depth apart; measured over
            # the 236 sharing pairs on seeds 1/3/5/7 the widest is 58.1 m, so
            # 130 m is better than two times the real maximum and exists only
            # to keep this from being 121 x 121 edge comparisons.
            if math.dist(h['c'], g['c']) > 130.0:
                continue
            for e0 in h['rear_edges']:
                for e1 in g['rear_edges']:
                    sh = _shared_stretch(e0, e1)
                    if sh is not None:
                        out.append((sh, h, g))
    return fenced, out


def test_shared_boundary_is_continuously_fenced():
    """A boundary between two fenced neighbours has no hole in it.

    WHAT THIS ASKS THAT ITS PREDECESSOR DID NOT. The old rule compared run
    ENDPOINTS and read 34 of 61 pairs as failures on seed 3 — against a
    coverage measurement over the same lots that finds no hole wider than
    0.00 m in any of the 236 shared boundaries on seeds 1/3/5/7. Both cannot be
    describing the ground, and the endpoint one is the broken proxy: see
    `_boundary_holes` for why `_relay` killed it, and `_shared_stretch` for why
    the shared line cannot be found in `fence_segs` at all any more.

    IT ALSO READS `fence_drawn`, NOT `fence_segs`. The old rule asked about the
    plat's PROPOSAL, of which ~72% ever becomes a module, and since the
    all-or-nothing sweep it can name two lots whose fence was removed
    altogether — a "fenced pair" with no fence between them.
    """
    print('\n[2] a boundary between two fenced neighbours is continuously fenced')
    _mine = len(FAILS)
    seen = worst = 0
    excused = 0
    for seed in (1, 3, 7):
        sc = _scene(seed)
        drawn = fp.drawn_fences(sc)
        pave = [b for b in (fp._ribbon_box(r) for r in fp.drives(sc))
                if b is not None]
        fenced, bounds = _shared_boundaries(sc)
        bad, holed = [], 0
        for (sh, h, g) in bounds:
            L, gs = _boundary_holes(sh[0], sh[1], drawn)
            ux = (sh[1][0] - sh[0][0]) / L
            uy = (sh[1][1] - sh[0][1]) / L
            for (a, b) in gs:
                if b - a <= MAX_BOUNDARY_HOLE_M:
                    continue
                q0 = (sh[0][0] + ux * a, sh[0][1] + uy * a)
                q1 = (sh[0][0] + ux * b, sh[0][1] + uy * b)
                if _is_opening(q0, q1, (h, g), pave):
                    excused += 1
                    continue
                holed += 1
                worst = max(worst, b - a)
                if len(bad) < 5:
                    bad.append((b - a, L, q0))
        seen += len(bounds)
        print(f'        seed {seed}: {len(fenced)} fenced lots, '
              f'{len(bounds)} shared boundaries, {holed} with a hole over '
              f'{MAX_BOUNDARY_HOLE_M} m')
        for (w, L, q) in bad:
            print(f'          {w:.2f} m of a {L:.1f} m boundary at '
                  f'({q[0]:.1f}, {q[1]:.1f})')
        check(len(bounds) >= 5,
              f'seed {seed} produced shared boundaries to test ({len(bounds)})')
        check(holed == 0,
              f'seed {seed}: {holed} shared boundaries have a hole in them')
    print(f'        {seen} boundaries checked, worst unexcused hole '
          f'{worst:.2f} m, {excused} holes excused as an opening')

    # -- and the rule has teeth, both ways ------------------------------
    # PUNCHED, NOT HOPED FOR. A test whose measured answer is 0.00 m everywhere
    # proves nothing about whether it can see a hole, so one is made: every
    # drawn span within reach of the middle third of a real shared boundary is
    # withheld, and the checker has to report a hole about a third of the
    # boundary long.
    sc = _scene(3)
    drawn = fp.drawn_fences(sc)
    _fenced, bounds = _shared_boundaries(sc)
    (sh, h, g) = max(bounds, key=lambda t: math.dist(t[0][0], t[0][1]))
    L = math.dist(sh[0], sh[1])
    ux, uy = (sh[1][0] - sh[0][0]) / L, (sh[1][1] - sh[0][1]) / L
    # WITHHELD BY WHAT THEY COVER, NOT BY WHERE THEY END. The first version of
    # this dropped spans with an ENDPOINT in the middle third and punched a
    # 0.00 m hole: a shared boundary is fenced by ONE relayed run that spans the
    # whole of it, so both its endpoints are outside the middle third and
    # nothing was withheld at all. That is the same endpoint-versus-coverage
    # confusion the test itself exists to correct, reproduced in its own
    # scaffolding — so the withhold now asks the coverage question too.
    lo, hi = L / 3.0, 2.0 * L / 3.0
    probe = [(sh[0][0] + ux * t, sh[0][1] + uy * t)
             for t in [lo + 0.25 * k for k in range(int((hi - lo) / 0.25) + 1)]]
    rr = ss._YARD_FENCE_REACH_M ** 2

    def _covers_middle(span):
        (a, b) = span
        ex, ey = b[0] - a[0], b[1] - a[1]
        e2 = max(ex * ex + ey * ey, 1e-12)
        for (qx, qy) in probe:
            t = ((qx - a[0]) * ex + (qy - a[1]) * ey) / e2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            if (qx - a[0] - ex * t) ** 2 + (qy - a[1] - ey * t) ** 2 <= rr:
                return True
        return False

    cut = [s for s in drawn if not _covers_middle(s)]
    _L, gs = _boundary_holes(sh[0], sh[1], cut)
    punched = max((b - a for (a, b) in gs), default=0.0)
    # 21.56 m of a 23.3 m boundary, not 7.8 — and the overshoot is the point.
    # Withholding whatever covers the MIDDLE of a shared boundary takes out the
    # single relayed run that covers all of it, because that is how a shared
    # boundary is fenced. The endpoint proxy this test replaces would have
    # scored the remainder as two runs meeting perfectly.
    print(f'        withheld what covers the middle third of a {L:.1f} m '
          f'shared boundary: hole measured {punched:.2f} m')
    check(punched > MAX_BOUNDARY_HOLE_M and punched > L / 4.0,
          f'a hole punched in a shared boundary is measured ({punched:.2f} m)')
    check(max((b - a for (a, b) in _boundary_holes(sh[0], sh[1], drawn)[1]),
              default=0.0) <= MAX_BOUNDARY_HOLE_M,
          'and the untouched boundary measures clean')

    # ...AND AN OPENING IS STILL EXCUSED. The carve-out never fires on the
    # shipped preset — 0 excused above, because the preset stamps no attached
    # garages, so no drive ever reaches a side lot line to cut it — and a
    # carve-out that is never exercised is a carve-out nobody knows is broken.
    # So it is exercised here, on a lot frame with one 2 m opening in it.
    fix = {'frontage': (0.0, 0.0), 'u': (1.0, 0.0), 'n': (0.0, 1.0),
           'c': (0.0, 13.0), 'd': 10.0, 'lot_width': 20.0, 'lot_depth': 40.0,
           'front_gaps': [(4.0, 2.0)]}
    check(_is_opening((2.2, 6.0), (5.8, 6.0), (fix,), ()),
          'a hole sitting inside a front_gaps opening is excused')
    check(not _is_opening((2.2, 6.0), (12.0, 6.0), (fix,), ()),
          'a hole running well past that opening is NOT')
    check(not _is_opening((2.2, 34.0), (5.8, 34.0), (fix,), ()),
          'nor is one on the same bearing out in the back garden')
    assert seen > 0 and len(FAILS) == _mine, FAILS[_mine:]


def test_no_fence_across_a_driveway():
    """A drive is an opening in the perimeter, wherever it crosses it.

    THE DEFECT THIS PINS. The front run has always broken for the drive, but
    it broke at `art_gaps` — the KIT's front openings — while the paving was
    laid at the plat's own `drives[]` entry, and for every garage-less style
    (cottage, two_storey, wide_house, terrace) those are different places: the
    kit contributes a door gap only, so the drive got none. Meanwhile no cut
    ever looked at the SIDE and REAR runs, which a drive up a narrow side yard
    crosses outright. Measured before the fix, on the shipped preset: 32 (seed
    3), 36 (seed 1), 19 (seed 5) and 39 (seed 7) modules standing on asphalt.

    `suburb_parcel` now derives ONE `drive_off` that the front opening, the
    apron cuts in `_cut_run` and the paved ribbon all read, and re-cuts
    standing fences when a later lot's drive lands on them. This asserts the
    result on the RUNS, offline — no pxr, no module tiling, no Isaac.

    WHAT THIS TEST DOES AND DOES NOT GATE. Reverting the `front_gaps` union
    puts 8 runs back across a drive here, so the FRONT half is genuinely
    pinned. The SIDE and REAR half is not: a drive only reaches a side line
    once the kit's own garage sets `drive_off` out at the lot edge, and this
    fixture has no kit. That half is gated on the shipped preset by
    `tools/fence_check.py`, where `on_drive` is a hard failure — run it before
    trusting a change to this pass.
    """
    print('\n[3] no fence run stands across a driveway')
    # A GARAGE-LESS KIT STYLE, which is the case that was broken. `cottage`,
    # `two_storey`, `wide_house` and `terrace` have a front door and no garage
    # door, so `front_openings` returns ONE gap — the walk — and `art_gaps` is
    # non-empty. That non-emptiness is what used to suppress the plat's own
    # drive opening entirely. Without this fixture the plat falls back to its
    # own single gap and the bug cannot reproduce, so the test would pass on
    # broken code.
    def _door_only(size_index, house_w):
        return [(-house_w * 0.22, WALK_HALF_M, 'door')]

    pcfg = {'house_sizes': [(11.0, 8.5), (13.0, 9.5), (15.0, 11.0)],
            'front_openings': _door_only}
    n_lots = n_drives = hits = 0
    worst = []
    for region, seed in ((250.0, 3), (400.0, 7), (250.0, 11)):
        for b in lots(region, seed, pcfg):
            hs = b.get('houses') or []
            ds = b.get('drives') or []
            for di, d in enumerate(ds):
                n_drives += 1
                h = hs[di] if di < len(hs) else None
                if h is None:
                    continue
                n_lots += 1
                box = sp._drive_box(d)
                for (a, bb, _tag) in (h.get('fence_segs') or ()):
                    # The run's hairline core, so a fence MEETING the apron
                    # edge is not read as standing on it — the same 2 cm
                    # ribbon `_FenceGrid` and `tools/fence_png` reason about.
                    if sp._obb_overlap(sp._seg_box(a, bb, t=0.02), box,
                                       pad=-0.25):
                        hits += 1
                        if len(worst) < 6:
                            worst.append((round(a[0], 1), round(a[1], 1),
                                          round(bb[0], 1), round(bb[1], 1)))
    print(f'        lots with a drive={n_lots}  drives={n_drives}  '
          f'runs crossing one={hits}')
    for w in worst:
        print(f'          ({w[0]}, {w[1]}) -> ({w[2]}, {w[3]})')
    check(n_drives > 0, 'the fixture produced drives to test against')
    check(hits == 0, f'{hits} fence runs stand across a driveway')
    assert n_drives > 0 and hits == 0, f'{hits} fence runs across a driveway'


# 1000 x 750 m of the shipped preset rather than its full 1600 x 1200 m. The
# street network is 14.4 s of the 16 s a full build costs and it is the only
# expensive part of it, so this is the same passes, the same records and the
# same invariant for 2.3 s a seed. The full preset over seeds 1/3/5/7 is gated
# by `tools/fence_check.py`; what pytest buys is that this is gated on every
# commit rather than when somebody remembers to run the tool.
_REGION = (1000.0, 750.0)
_SCENES = {}


def _scene(seed):
    if seed not in _SCENES:
        _SCENES[seed] = fp.build(seed, region_m=_REGION)
    return _SCENES[seed]


def test_every_fenced_lot_encloses_its_back_yard():
    """The headline guarantee of the enclosure rework, stated as an invariant.

    WHAT IT REPLACES. A fenced lot used to be a U OPEN AT BOTH SIDE YARDS: the
    side runs stopped on the lot line at the building line while the house's
    front corner sat 3-17 m inboard of them, so you walked straight in. 79 of
    155 fenced lots on seed 3 closed; the other 76 were fence enclosing
    nothing, which is what the eye reads as "the fences are broken". Three
    changes fixed it — a gate return from each side run to the house corner,
    `front_open` decided after the cuts rather than before, and an
    ALL-OR-NOTHING sweep that strips a lot which still cannot close.

    WHY IT IS AN INVARIANT AND NOT A RATE. The sweep only ever deletes, so
    there is no tolerance to spend: a lot either keeps a fence that closes its
    yard or keeps no fence at all. Anything above zero here means the sweep
    stopped running to a fixed point, and one sweep alone leaves 38 fenced-open
    lots on seed 3 — a quarter of the visible fence.

    THE GARDENLESS LOTS ARE IN, NOT EXCEPTED. A lot with under 4 m of garden
    behind its back wall can never close, and while `_FENCE_ON_GARDENLESS_LOT`
    was true this rule needed a carve-out for them — an exception that hides
    exactly the defect the rule is stated over. They are stripped too now, so
    `_rear_yard_edges` returning `[]` makes `_yard_enclosed` answer False and
    such a lot fails here like any other.
    """
    print('\n[4] every lot that shows a fence encloses its back yard')
    # OWN FAILURES ONLY. `FAILS` is shared by every test in this module and
    # `test_neighbour_fences_meet` currently records one into it — a `check()`
    # with no `assert` behind it, so pytest has never seen it. Asserting on the
    # whole list would make this test fail for somebody else's reason and
    # report the wrong cause; asserting on what this function added says what
    # this function found.
    _mine = len(FAILS)
    total_fenced = 0
    for seed in (1, 3, 7):
        sc = _scene(seed)
        hs = fp.houses(sc)
        drawn = fp.drawn_fences(sc)
        fenced = [h for h in hs if h.get('fence_drawn')]
        total_fenced += len(fenced)
        bad = fp.fence_fragment(sc)
        gardenless = [h for h in fenced if not h.get('rear_edges')]
        print(f'        seed {seed}: {len(hs)} lots, {len(fenced)} show fence '
              f'({len(drawn)} spans), {len(bad)} of them enclose nothing, '
              f'{len(gardenless)} have no back garden at all')
        check(len(fenced) >= 8,
              f'seed {seed} platted fence to test ({len(fenced)} lots)')
        check(not bad, f'seed {seed}: {len(bad)} lots carry fence that does '
                       f'not enclose their back yard')
        # The invariant restated through the shipped predicate rather than
        # through `h["enclosure"]["closed"]`, which is written by the same pass
        # that does the stripping: a check that read the flag would be
        # asserting the pass agrees with itself.
        check(all(ss._yard_enclosed(h, fences=drawn)[0] for h in fenced),
              f'seed {seed}: `_yard_enclosed` agrees, lot by lot')

    # ...AND THE RULE HAS TEETH. The old behaviour cannot be restored from
    # inside a test, so it is reproduced as an INPUT: take a lot that closes,
    # delete the spans covering its rear boundary, and the checker must name
    # that lot and no other. That three-sided remainder IS what 76 lots on
    # seed 3 looked like before the sweep existed.
    sc = _scene(3)
    victim = next(h for h in fp.houses(sc) if h.get('fence_drawn')
                  and h.get('rear_edges'))
    rear = victim['rear_edges'][2]
    keep = [sp_ for sp_ in victim['fence_drawn']
            if ss._edge_cover(rear[0], rear[1], fences=[(sp_[0], sp_[1])]) < 0.05]
    saved = victim['fence_drawn']
    victim['fence_drawn'] = keep
    try:
        broken = fp.fence_fragment(sc)
        print(f'        cut {len(saved) - len(keep)} spans off one lot\'s rear '
              f'boundary: {len(broken)} lot(s) now enclose nothing')
        check(victim in broken,
              'a lot whose rear run is deleted is caught by the checker')
        # AND SO IS THE LOT BEHIND IT, WHICH IS THE POINT. A rear lot line is
        # SHARED — drawn once, by whichever of the two lots was issued first —
        # so deleting it opens two gardens, not one. That is exactly why
        # `build_placements` iterates its all-or-nothing sweep to a fixed point
        # instead of sweeping once: on the full seed 3 it settles in four
        # rounds (144, 29, 7, 2 lots) and a single round would leave 38 fenced
        # lots standing open.
        print(f'        of those {len(broken)}, '
              f'{len(broken) - 1} are neighbours the shared boundary took '
              f'down with it')
        check(1 <= len(broken) <= 2,
              f'the damage is confined to that lot and the one across its rear '
              f'line ({len(broken)})')
    finally:
        victim['fence_drawn'] = saved
    check(not fp.fence_fragment(sc), 'and the suburb is clean again once it is '
                                     'put back')
    assert total_fenced > 0 and len(FAILS) == _mine, FAILS[_mine:]


def main():
    print('=' * 66)
    print('FENCE RULES')
    print('=' * 66)
    test_no_fence_on_cul_de_sac()
    test_shared_boundary_is_continuously_fenced()
    test_no_fence_across_a_driveway()
    test_every_fenced_lot_encloses_its_back_yard()
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
