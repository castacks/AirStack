#!/usr/bin/env python3
"""Fence rules on generated suburb lots.

1. Cul-de-sac (wedge) lots carry NO fence — a perimeter fence on a pie-slice
   lot narrows to a point at the kerb and does not read as a fence.
2. Two neighbours that both fence MEET on their shared boundary instead of
   stopping short of each other.
3. NO FENCE RUN STANDS ACROSS ITS OWN LOT'S DRIVEWAY — front, side or rear.

Offline: layout + parcel only, no pxr, no Isaac Sim.
"""
import math, os, random, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from layout import suburb_net as sn
from detail import suburb_parcel as sp

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


def test_neighbour_fences_meet():
    print('\n[2] adjacent fenced neighbours meet on the shared boundary')
    worst = 0.0
    pairs = 0
    gaps = []
    for region, seed in ((250.0, 3), (400.0, 7)):
        for b in lots(region, seed):
            hs = [h for h in (b.get('houses') or []) if h.get('fence_segs')]
            for i, h in enumerate(hs):
                for g in hs[i + 1:]:
                    # SHARED BOUNDARY, not "nearby": two lot corners in the
                    # same place. Diagonal neighbours are metres apart by
                    # design and must not be counted as failures.
                    lot_gap = min(math.dist(p, q)
                                  for p in h['lot_corners']
                                  for q in g['lot_corners'])
                    if lot_gap > 0.5:
                        continue
                    best = min(
                        math.dist(pa, pb)
                        for sa in h['fence_segs'] for pa in (sa[0], sa[1])
                        for sb in g['fence_segs'] for pb in (sb[0], sb[1]))
                    pairs += 1
                    gaps.append(best)
                    worst = max(worst, best)
    touching = sum(1 for g in gaps if g <= 0.5)
    print(f'        shared-boundary fenced pairs={pairs}  touching={touching}  '
          f'worst={worst:.2f} m')
    print(f'        gaps: {sorted(round(g, 2) for g in gaps)}')
    if pairs == 0:
        print('        (no adjacent fenced pairs in these seeds)')
    # MOST must touch, not all: a perimeter is deliberately CUT where a
    # driveway crosses it (`_cut_run`), so a neighbour whose drive runs along
    # the shared line legitimately leaves an opening. Before the fix the gaps
    # were metres wide across the board; the bar is that sharing a boundary
    # now means sharing a fence line.
    check(pairs == 0 or touching >= 0.75 * pairs,
          'at least 3/4 of shared-boundary fences meet (<= 0.5 m)')


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


def main():
    print('=' * 66)
    print('FENCE RULES')
    print('=' * 66)
    test_no_fence_on_cul_de_sac()
    test_neighbour_fences_meet()
    test_no_fence_across_a_driveway()
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
