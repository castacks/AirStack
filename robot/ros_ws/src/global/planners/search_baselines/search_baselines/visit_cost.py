"""Visit cost: ground the drone has already flown over, as a scalar field.

WHY THIS EXISTS. The usual way to stop a frontier explorer re-selecting the
place it just came from is a blacklist: reach a frontier, mark it, never offer
it again. That has an ending built into it — every frontier eventually lands on
the blacklist, the candidate set empties, and the search stops. Correct for an
object-goal benchmark ("find the sofa, then you are done"), wrong for
search-and-rescue, where the detector is imperfect and the operator WANTS a
second and a third pass over a sector. A casualty missed on pass 1 (occluded by
a roof line, in shadow, a bad viewing angle, motion blur on the turn) is only
found on pass 2 if the drone is still ALLOWED to go back there.

So visited ground is never removed from the candidate set. It is made
temporarily EXPENSIVE:

    score = info_gain - weight * visit_cost

THE DYNAMIC THIS BUYS. Early on the field is lumpy — the ground under the
take-off point carries cost, everything else is free — so the cost term
dominates and pushes the drone outward, which is exactly the behaviour a
blacklist gives. As coverage completes the lumps fill in, every frontier ends
up carrying a SIMILAR visit cost, and a term that is near-identical across all
candidates cannot change an argmax. The differences wash out, selection reverts
to information gain, and the drone sweeps the region again of its own accord.
Nothing has to detect "coverage is done" and switch modes; the switch is a
consequence of the field flattening. The `__main__` self-test measures exactly
this: the spread (max - min) of cost across a set of frontiers, at 10 %, 50 %
and 100 % coverage.

Frame and grid geometry are the planner's occupancy grid, not a private one, so
a frontier cell index reads straight across with no resampling:

    x = (i - ox + 0.5) * res
    y = -((j - oy + 0.5) * res)

The MINUS on y is this stack's convention (rows run south as j increases, the
same sign `ValueMap` and `Map_Extraction` use). Get it wrong and nothing throws
— the field is silently mirrored about y = 0, the drone is repelled from ground
it has never seen and drawn back over ground it just swept, and the failure
looks like a tuning problem rather than a bug. The self-test pins the sign.
"""

import numpy as np


class VisitCost:
    """A (size, size) float32 cost field on the planner's grid geometry.

    Dense float32 rather than a sparse set of visited (x, y) points because
    every read is "what does THIS frontier cell cost", one array lookup, and
    every write is a small stamp. A point list would make the read O(visits),
    which grows without bound over a long episode — and the whole reason this
    class exists is to support long episodes.
    """

    def __init__(self, size, resolution_m, origin_cells, radius_m,
                 decay_per_s=0.0, weight=1.0):
        """
        size          cells per side, matching the occupancy grid.
        resolution_m  metres per cell, matching the occupancy grid.
        origin_cells  (ox, oy), the cell the map-frame origin sits in.
        radius_m      radius of the deposit stamp. Set it to the sensor's
                      ground footprint radius, not to the vehicle radius: the
                      claim being recorded is "this ground has been LOOKED at",
                      not "the drone was here".
        decay_per_s   exponential forgetting rate, 1/s. 0 disables it.
        weight        cost weight in `score`.

        Degenerate inputs: size < 1 or resolution_m <= 0 raise ValueError —
        there is no safe field to build and carrying on would silently produce
        a 0-cell grid whose every lookup returns 0.0, i.e. a visit cost that
        does nothing. radius_m is clamped up to half a cell instead of raising:
        a stamp smaller than a cell is physically meaningless on this grid but
        it is a plausible config value (someone converts a 0.3 m vehicle radius
        on a 1 m grid), and clamping degrades to "stamp one cell" rather than
        dividing by zero.
        """
        self.size = int(size)
        self.res = float(resolution_m)
        if self.size < 1 or self.res <= 0.0:
            raise ValueError(
                f'VisitCost needs size >= 1 and resolution_m > 0, '
                f'got size={size} resolution_m={resolution_m}')
        self.ox, self.oy = float(origin_cells[0]), float(origin_cells[1])
        self.radius = max(float(radius_m), 0.5 * self.res)
        self.decay_per_s = max(float(decay_per_s), 0.0)
        self.weight = float(weight)
        self.field = np.zeros((self.size, self.size), dtype=np.float32)
        # Peak is cached because `score` is called once per frontier per tick
        # and a full-array max on every call is pure waste; `visit` and `decay`
        # are called once per tick each.
        self._peak = 0.0
        self._dirty = False

    # ── grid <-> map frame ────────────────────────────────────────────────────

    def cell_of(self, x, y):
        """Map-frame metres -> (i, j). Identical to `ValueMap.cell_of`; the two
        must agree cell-for-cell or a frontier's value and its visit cost come
        from different places on the ground."""
        return (int(np.floor(x / self.res) + self.ox),
                int(np.floor(-y / self.res) + self.oy))

    def xy_of(self, i, j):
        """Cell centre in map-frame metres. The inverse of `cell_of` up to the
        half-cell rounding, and the thing the self-test round-trips."""
        return ((i - self.ox + 0.5) * self.res,
                -((j - self.oy + 0.5) * self.res))

    # ── deposit and forget ────────────────────────────────────────────────────

    def visit(self, xy, amount=1.0):
        """Deposit visit mass centred at map-frame `xy`. Returns cells touched.

        The stamp is a RAISED COSINE, w(d) = 0.5 * (1 + cos(pi * d / R)), which
        is 1 at the centre, 0 at d = R, and has zero derivative at both ends.
        The obvious alternative — a hard disc, w = 1 inside R and 0 outside —
        is wrong here for a specific reason: two frontiers a single cell apart,
        one just inside the disc and one just outside, would score a full
        `weight` apart despite describing the same piece of ground. The planner
        then commits to whichever side of that arbitrary edge it lands on, and
        the choice is an artefact of the stamp rather than of the search. A
        Gaussian would do as well; the raised cosine is preferred only because
        it has compact support, so the stamp is a bounded window rather than a
        whole-array update once per tick.

        Non-finite inputs are ignored (return 0) rather than raising: this is
        fed from a pose stream, and one NaN pose during a TF dropout should not
        take down the planner. A stamp entirely off the grid also returns 0 —
        the drone is outside the mapped area and there is nothing to record.
        """
        try:
            x, y, amt = float(xy[0]), float(xy[1]), float(amount)
        except (TypeError, IndexError, ValueError):
            return 0
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(amt)):
            return 0

        r_cells = int(np.ceil(self.radius / self.res))
        ic, jc = self.cell_of(x, y)
        i0, i1 = max(0, ic - r_cells - 1), min(self.size, ic + r_cells + 2)
        j0, j1 = max(0, jc - r_cells - 1), min(self.size, jc + r_cells + 2)
        if i0 >= i1 or j0 >= j1:
            return 0

        ii = np.arange(i0, i1, dtype=np.float64)[:, None]
        jj = np.arange(j0, j1, dtype=np.float64)[None, :]
        dx = (ii - self.ox + 0.5) * self.res - x
        dy = -((jj - self.oy + 0.5) * self.res) - y
        d = np.hypot(dx, dy)

        inside = d < self.radius
        if not inside.any():
            return 0
        w = np.zeros_like(d)
        w[inside] = 0.5 * (1.0 + np.cos(np.pi * d[inside] / self.radius))
        self.field[i0:i1, j0:j1] += (amt * w).astype(np.float32)
        self._dirty = True
        return int(inside.sum())

    def decay(self, dt_s):
        """Exponential forgetting: field *= exp(-decay_per_s * dt_s).

        WHY THIS IS OPTIONAL RATHER THAN ALWAYS-ON. Forgetting encodes a claim
        about the world: that what was true of a patch of ground when it was
        last seen may no longer be true, so an old observation is worth less
        than a new one. That claim holds for a moving target — a walking
        survivor, a drifting boat, a fire front — where ground cleared an hour
        ago is genuinely worth re-checking, and it holds for a long endurance
        mission where an operator wants continuous re-patrol rather than a
        one-off sweep.

        It is FALSE for the case this baseline is mostly run on: a fixed set of
        static casualties inside a bounded sector, over an episode of minutes.
        There, decay makes the drone drift back toward the take-off point
        (visited earliest, therefore forgotten most) instead of pressing on into
        unseen ground, which is a slower sweep for no gain in what is found.

        Worse, decay and re-sweep are two mechanisms for the same behaviour, and
        turning both on double-counts. The flattening described in the module
        docstring already returns the drone to old ground once coverage is
        complete, on the honest trigger (coverage is done) rather than on a
        clock. Decay makes it go back on a timer, whether or not the rest of the
        sector has been seen yet. Default 0: earn the re-visit, do not schedule
        it.

        dt_s <= 0 is a no-op, so a caller that computes dt from a wall clock and
        occasionally gets a zero or a backwards step cannot amplify the field.
        """
        if self.decay_per_s <= 0.0 or dt_s <= 0.0:
            return
        k = float(np.exp(-self.decay_per_s * float(dt_s)))
        self.field *= np.float32(k)
        self._peak *= k

    # ── read ──────────────────────────────────────────────────────────────────

    def cost_at(self, i, j):
        """Raw accumulated visit mass at cell (i, j); 0.0 off the grid.

        Off-grid reads return 0 (= never visited) rather than raising because
        frontier cells arrive from an upstream extractor that has its own idea
        of the grid bounds, and "outside the map" and "not yet visited" want the
        same answer here: nothing is known against this cell.
        """
        if 0 <= i < self.size and 0 <= j < self.size:
            return float(self.field[i, j])
        return 0.0

    def cost_at_xy(self, x, y):
        """Raw accumulated visit mass at a map-frame position."""
        i, j = self.cell_of(x, y)
        return self.cost_at(i, j)

    def score(self, info_gain, i, j):
        """Selection score for a frontier at cell (i, j): info_gain - w * cost.

        UNITS. Subtracting cost from information gain only means anything if
        the two are commensurate; otherwise `weight` is not a preference, it is
        a unit conversion with an arbitrary constant folded in, and a value
        tuned on one map is meaningless on the next.

        The convention here:

        * `info_gain` is assumed to be NORMALISED TO [0, 1] — the caller's gain
          divided by the best gain among the current candidates, or by a fixed
          reference (frontier size / max frontier size, unobserved cells a view
          would reveal / most any view could reveal). It is NOT checked, because
          a caller with a genuinely calibrated gain in other units may want to
          set `weight` to match; but if it is raw cell counts in the hundreds,
          any sane `weight` is swamped and the visit cost does nothing.
        * `cost` is the field NORMALISED BY ITS OWN CURRENT PEAK, so it too is
          in [0, 1]: 0 = never visited, 1 = the most-visited ground on the map
          right now.

        So `weight` reads directly: weight = 1 means the most-visited ground on
        the map must offer a full unit more information gain than never-visited
        ground to win. weight = 0 is pure frontier exploration.

        The trade-off accepted by normalising against a RUNNING PEAK: a single
        pathological hotspot — the drone hovering in place waiting for a
        clearance, stamping the same cells hundreds of times — inflates the peak
        and compresses every real difference toward 0, weakening the cost term
        just when the map is otherwise uniform. That is tolerable and
        self-correcting (the hotspot is one stamp radius wide, the sweep
        continues to raise everything else), and it is strictly better than the
        alternative of subtracting raw mass, where the cost term's magnitude
        depends on episode length and stamp rate and therefore has to be
        re-tuned per platform and per mission duration.
        """
        return float(info_gain) - self.weight * self._normalised_at(i, j)

    def _peak_value(self):
        if self._dirty:
            self._peak = float(self.field.max()) if self.field.size else 0.0
            self._dirty = False
        return self._peak

    def _normalised_at(self, i, j):
        pk = self._peak_value()
        if pk <= 0.0:
            return 0.0
        return self.cost_at(i, j) / pk

    def normalised(self):
        """The whole field scaled to 0..1, for publishing as a grid overlay.

        All-zero field returns all zeros rather than dividing by zero, so a
        visualiser subscribing before the first visit gets a blank layer instead
        of NaNs (which most viewers render as garbage rather than as nothing).
        """
        pk = self._peak_value()
        if pk <= 0.0:
            return np.zeros_like(self.field)
        return self.field / np.float32(pk)

    def stats(self):
        """Cheap summary for logging: how much of the grid has been touched at
        all, and how hard. `coverage` counts cells above a small epsilon rather
        than strictly > 0 because the raised-cosine tail deposits values around
        1e-7 at the stamp edge, and counting those would report near-total
        coverage from the first few visits."""
        eps = 1e-4
        visited = int((self.field > eps).sum())
        total = int(self.field.size)
        return {
            'coverage': (visited / total) if total else 0.0,
            'visited_cells': visited,
            'cells': total,
            'max': float(self.field.max()) if total else 0.0,
            'mean': float(self.field.mean()) if total else 0.0,
        }


# ── self-test ────────────────────────────────────────────────────────────────

def _selftest():
    rng = np.random.default_rng(0)
    res, size, ox, oy = 0.5, 96, 48.0, 48.0

    # 1. A visit raises cost near it and not far away, and the y sign round-trips.
    vc = VisitCost(size, res, (ox, oy), radius_m=3.0)
    px, py = 4.3, -6.7                      # generic point, not on a cell edge
    n = vc.visit((px, py))
    assert n > 0, 'stamp touched no cells'
    here = vc.cost_at_xy(px, py)
    far = vc.cost_at_xy(px + 8.0, py + 8.0)
    assert here > 0.9, f'cost at the visit should be ~1, got {here}'
    assert far == 0.0, f'cost 11 m away should be 0, got {far}'
    # THE SIGN TEST: the cell holding the deposit must be the global argmax. If
    # the y sign were flipped the peak would land at -py and this fails.
    pk_i, pk_j = np.unravel_index(int(np.argmax(vc.field)), vc.field.shape)
    assert (pk_i, pk_j) == vc.cell_of(px, py), (
        f'peak at cell {(pk_i, pk_j)}, deposit cell {vc.cell_of(px, py)}')
    assert abs(here - vc.field.max()) < 1e-6, 'cost_at_xy is not the max'
    mx, my = vc.xy_of(pk_i, pk_j)
    assert abs(mx - px) <= res and abs(my - py) <= res, (
        f'cell centre {(mx, my)} is not within a cell of {(px, py)}')
    print(f'[1] visit at ({px}, {py}): {n} cells, cost here={here:.4f} '
          f'far={far:.4f}, peak cell={vc.cell_of(px, py)} '
          f'centre=({mx:.2f}, {my:.2f})  OK')

    # 2. decay(0) is a no-op; decay(t) with decay_per_s > 0 strictly reduces.
    vd = VisitCost(size, res, (ox, oy), radius_m=3.0, decay_per_s=0.0)
    vd.visit((0.0, 0.0))
    before = vd.field.copy()
    vd.decay(0.0)
    assert np.array_equal(vd.field, before), 'decay(0) changed the field'
    vd.decay(10.0)
    assert np.array_equal(vd.field, before), 'decay_per_s=0 must disable decay'
    ve = VisitCost(size, res, (ox, oy), radius_m=3.0, decay_per_s=0.05)
    ve.visit((0.0, 0.0))
    b = ve.cost_at_xy(0.0, 0.0)
    ve.decay(0.0)
    assert ve.cost_at_xy(0.0, 0.0) == b, 'decay(0) is not a no-op'
    ve.decay(20.0)
    a = ve.cost_at_xy(0.0, 0.0)
    nz = before > 0
    assert a < b, f'decay did not reduce cost: {b} -> {a}'
    assert np.all(ve.field[nz] < b + 1e-9), 'some cell grew under decay'
    print(f'[2] decay: no-op at dt=0 and at rate=0; '
          f'rate=0.05 over 20 s: {b:.4f} -> {a:.4f} '
          f'(expected x{np.exp(-0.05 * 20):.4f})  OK')

    # 3. THE KEY PROPERTY. Sweep a small region in raster order (a lawnmower:
    #    coverage is one-sided until it is complete) and watch the spread of
    #    cost across a fixed set of frontier probes.
    #
    #    Two things this test deliberately does NOT do, because either would
    #    make it pass for the wrong reason:
    #      * The sweep does not land on cell centres. Real flight does not, and
    #        a sweep of exact cell centres makes every interior cell accumulate
    #        the identical lattice sum, so the final spread is 0 by symmetry
    #        rather than because the field flattened. Each visit is jittered by
    #        up to 0.4 of a cell.
    #      * Probes sit at least one stamp radius inside the swept region. The
    #        field only flattens where the stamp support is fully covered; a
    #        probe on the rim keeps a permanently lower cost, which is correct
    #        (its neighbourhood really is less swept) and would mask what is
    #        being measured.
    #
    #    The spread RISES before it falls, and that is the honest shape: at 10 %
    #    nothing near the probes has been swept, so every probe reads ~0, the
    #    cost term is uniform and selection is already on information gain. The
    #    lumpy phase is the middle, where half the probes sit on swept ground
    #    and half do not. What the baseline needs is that the lump does not
    #    persist — that it is a transient of incomplete coverage.
    radius = 3.0
    vk = VisitCost(size, res, (ox, oy), radius_m=radius, weight=1.0)
    half = 9.0
    axis = np.arange(-half, half + 1e-9, res) + 0.5 * res
    sweep = np.array([(x, y) for y in axis[::-1] for x in axis])
    sweep = sweep + rng.uniform(-0.4 * res, 0.4 * res, size=sweep.shape)
    inset = half - radius - res
    probes = rng.uniform(-inset, inset, size=(32, 2))
    probe_cells = [vk.cell_of(x, y) for x, y in probes]

    marks = [0.10, 0.25, 0.50, 0.75, 0.90, 1.00]
    spreads, rel_spreads, done = [], [], 0
    print(f'[3] {len(sweep)} jittered visits over a {2 * half:.0f} x {2 * half:.0f} m '
          f'region, stamp radius {radius:.0f} m, {len(probe_cells)} frontier probes')
    for frac in marks:
        upto = int(round(frac * len(sweep)))
        for x, y in sweep[done:upto]:
            vk.visit((x, y))
        done = upto
        c = np.array([vk.cost_at(i, j) for i, j in probe_cells])
        nc = np.array([vk._normalised_at(i, j) for i, j in probe_cells])
        spread = float(c.max() - c.min())
        rel = spread / c.mean() if c.mean() > 0 else 0.0
        spreads.append(spread)
        rel_spreads.append(rel)
        print(f'[3] coverage {frac:5.0%}:  cost spread(max-min) = {spread:8.4f}   '
              f'mean = {c.mean():8.4f}   spread/mean = {rel:8.5f}   '
              f'spread of the normalised cost that enters score() = {nc.max() - nc.min():.5f}')

    peak_spread = max(spreads)
    assert peak_spread > 1.0, f'the field never got lumpy: peak spread {peak_spread}'
    assert spreads[-1] < 0.10 * peak_spread, (
        f'spread did not collapse: peak {peak_spread:.4f} -> '
        f'final {spreads[-1]:.4f}')
    assert rel_spreads[-1] < 0.10, (
        f'residual relative spread too large: {rel_spreads[-1]:.4f}')
    assert (rel_spreads[2] > rel_spreads[3] > rel_spreads[4] > rel_spreads[5]), (
        f'relative spread not monotonically falling once the sweep is past '
        f'half done: {rel_spreads}')

    # The residual does NOT go to zero, and the reason is physical rather than
    # numerical: the sweep is jittered, so deposit density fluctuates by a few
    # percent across the region and a probe that happened to sit under a denser
    # patch of passes keeps a slightly higher cost forever. A perfectly regular
    # sweep of exact cell centres would give exactly 0. So the claim to make is
    # not "cost differences vanish" but the weaker and true one: at full
    # coverage the cost term can only decide between frontiers whose
    # information gains are within the residual normalised spread of each
    # other, and above that gap it is information gain that decides.
    resid = max(vk._normalised_at(i, j) for i, j in probe_cells) \
        - min(vk._normalised_at(i, j) for i, j in probe_cells)
    agree, forced, trials = 0, 0, 1000
    for _ in range(trials):
        ig = rng.uniform(0.0, 1.0, size=len(probe_cells))
        sc = np.array([vk.score(g, i, j) for g, (i, j) in zip(ig, probe_cells)])
        top = np.sort(ig)[::-1]
        if int(np.argmax(sc)) == int(np.argmax(ig)):
            agree += 1
        if top[0] - top[1] > resid:
            forced += 1
            assert int(np.argmax(sc)) == int(np.argmax(ig)), (
                'info-gain argmax won by more than the residual cost spread '
                'and still lost the score argmax')
    print(f'[3] spread peaked at {peak_spread:.4f} and collapsed to '
          f'{spreads[-1]:.4f} ({spreads[-1] / peak_spread:.2%} of the peak); '
          f'residual normalised spread {resid:.4f}')
    print(f'[3] over {trials} random info-gain draws across the {len(probe_cells)} '
          f'probes, argmax(score) == argmax(info_gain) in {agree / trials:.1%} of '
          f'them, and in 100% of the {forced} draws where the info-gain margin '
          f'beat the residual  OK')
    assert agree / trials > 0.75, f'argmax agreement only {agree / trials:.2f}'

    # 4. A second full sweep must not re-lump the field: re-visiting uniformly
    #    scales every probe alike, so selection stays on information gain and
    #    the drone keeps sweeping instead of latching onto one corner.
    cf = np.array([vk.cost_at(i, j) for i, j in probe_cells])
    for x, y in sweep:
        vk.visit((x, y))
    c2 = np.array([vk.cost_at(i, j) for i, j in probe_cells])
    rel2 = (c2.max() - c2.min()) / c2.mean()
    assert rel2 <= rel_spreads[-1] + 1e-6, (
        f'the second pass re-lumped the field: {rel_spreads[-1]:.5f} -> {rel2:.5f}')
    st = vk.stats()
    print(f'[4] second pass: mean cost {cf.mean():.3f} -> {c2.mean():.3f}, '
          f'spread/mean {rel_spreads[-1]:.5f} -> {rel2:.5f} (no re-lumping); '
          f'stats coverage={st["coverage"]:.3f} max={st["max"]:.2f} '
          f'mean={st["mean"]:.3f}  OK')

    # 5. Degenerate inputs.
    nm = vk.normalised()
    assert nm.max() <= 1.0 + 1e-6 and nm.min() >= 0.0, 'normalised out of range'
    assert VisitCost(8, 1.0, (4, 4), radius_m=2.0).normalised().max() == 0.0
    v0 = VisitCost(8, 1.0, (4, 4), radius_m=0.0)
    assert v0.radius == 0.5, 'radius should clamp to half a cell'
    assert v0.visit((float('nan'), 0.0)) == 0, 'NaN visit must be ignored'
    assert v0.visit((1e6, 1e6)) == 0, 'off-grid visit must be ignored'
    assert v0.cost_at(-1, 0) == 0.0 and v0.cost_at(999, 0) == 0.0
    for bad in ((0, 1.0), (8, 0.0), (8, -1.0)):
        try:
            VisitCost(bad[0], bad[1], (0, 0), 1.0)
        except ValueError:
            pass
        else:
            raise AssertionError(f'VisitCost{bad} should have raised')
    print('[5] degenerate inputs: radius clamp, NaN/off-grid visits ignored, '
          'off-grid reads = 0, bad size/resolution raise  OK')
    print('visit_cost.py: all self-tests passed')


if __name__ == '__main__':
    _selftest()
