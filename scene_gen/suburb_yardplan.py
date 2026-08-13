"""
suburb_yardplan.py — yard planting for lots that are not axis-aligned.

WHY `suburb_yards` COULD NOT SIMPLY BE CALLED
---------------------------------------------
`suburb_yards` is a good pass and this module reuses the hard half of it. What
it cannot do is this layout's geometry, and the reason is measured rather than
aesthetic. Its yard records are world AABBs — `x0, y0, x1, y1 = rect` — and its
`_house_side` picks the wall a foundation row follows by comparing AXIS-ALIGNED
outward normals. On a curved street the lots turn with the street:

    lots within 5 deg of axis-aligned      9%   of 1,523
    AABB area / true footprint             median 1.74x, p90 2.00x, max 2.16x
    overlapping house AABB pairs           132   (true oriented boxes: 0)

So handing it AABBs would inflate every yard by about three quarters, spill
planting across the kerb and into next door, produce 132 overlapping yards, and
put the foundation row against a wall the house does not have — which is
precisely the failure `_house_side` was written to avoid.

WHAT IS REUSED, AND WHAT IS REPLACED
------------------------------------
Reused verbatim, because neither touches geometry and both encode work that
would be silly to redo:

    _Lib      species pools carrying a MEASURED point cost and a sampling
              weight, so an expensive handsome species can be carried as an
              occasional accent instead of being dropped or bankrupting the run
    _Budget   two purses, trees and everything else, allocated per unit area.
              This is the binding constraint of the whole scene: the library's
              cheapest bound green tree is 48.7k points, and the urban scene
              OOM-killed at 89.1M.

Replaced: all of the geometry.

THE LOCAL FRAME IS THE WHOLE IDEA
---------------------------------
Every lot already knows its own frame — `suburb_parcel` solved it when it
placed the house: a frontage point, the tangent ``u`` along the street, and the
inward normal ``n``. So planting is composed in ``(along, deep)`` coordinates
and transformed to the world at the end:

    world = house_centre + u * along + n * deep

which means the composition is written once, in a frame where the house front
is simply ``deep < 0``, and it follows the street round its curve for free. No
case analysis, no nearest-wall test, nothing to get wrong when the lot rotates.

    deep
      ^   . . . . . . . . .   rear boundary planting (massed)
      |   .                .
      |   .   [ HOUSE ]    .
      |   .   ###row###    .   foundation row along the true front wall
      |   .      T         .   specimen tree, offset to one side
      0 --+------o---------+-- kerb        o = mailbox
          |<---- lot ----->|
                along
"""

import math
import random

from suburb_yards import _Lib, _Budget          # geometry-free, reused as-is
import suburb_net as sn

DEFAULTS = {
    "point_budget": 30_000_000,
    "tree_budget_frac": 0.62,
    # Front yard: composed, not scattered.
    "foundation_row": [3, 7],        # shrubs along the house front wall
    "foundation_gap_m": 1.7,
    "foundation_off_m": 1.1,         # stand-off from the wall
    "specimen_tree_chance": 0.55,
    "mailbox_chance": 0.7,
    # Rear yard: massed along the boundary, canopy over the middle.
    "rear_boundary_step_m": 4.5,
    "rear_tree_chance": 0.75,
    "rear_prop_chance": 0.45,
    "clear_house_m": 0.6,
    "clear_drive_m": 0.4,
    "tree_min_separation_m": 3.0,
}


def _rng_pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _pick(pool, rng):
    """Weighted draw. Species is chosen BEFORE price is checked, deliberately:
    drawing only from what is affordable silently deletes every expensive
    species, which is the failure `suburb_yards` documents."""
    if not pool:
        return None
    total = sum(max(0.0, float(e.get("weight", 1.0))) for e in pool)
    if total <= 0.0:
        return pool[rng.randrange(len(pool))]
    r = rng.random() * total
    for e in pool:
        r -= max(0.0, float(e.get("weight", 1.0)))
        if r <= 0.0:
            return e
    return pool[-1]


def plan(config, parcels, rng, resolver=None):
    """Plant every lot in *parcels*. Returns ``(placements, stats)``.

    *parcels* is `suburb_parcel.parcel_blocks` output, so each house carries its
    own frame: centre ``c``, along-street unit ``u``, footprint ``w`` x ``d``.
    """
    cfg = dict(DEFAULTS)
    cfg.update(config.get("suburb_yardplan") or config.get("suburb_yards") or {})

    lib = _Lib(config)
    shrubs = lib.pool("yard_shrubs") or lib.pool("plants")
    trees = lib.pool("yard_trees") or lib.pool("trees")
    props = lib.pool("yard_props")

    houses = [h for p in parcels for h in p["houses"]]
    if not houses:
        return [], {"lots": 0}
    # Budget is allocated per unit area, so a big lot gets proportionally more
    # and the spend is uniform in points per square metre across the suburb.
    # SHUFFLED, and that is load-bearing rather than tidy. A purse carries its
    # under-spend forward until a yard can afford a 48.7k-point tree, so in
    # build order the trees all land in the first streets laid and the rest of
    # the suburb comes out bare. Visiting lots in random order spreads them.
    rng.shuffle(houses)
    lot_area = [max(60.0, h["w"] * (h["d"] + 26.0)) for h in houses]
    budget = _Budget(float(cfg["point_budget"]), sum(lot_area),
                     float(cfg["tree_budget_frac"]))

    row_lo, row_hi = _rng_pair(cfg["foundation_row"], (3.0, 7.0))
    gap = float(cfg["foundation_gap_m"])
    stand = float(cfg["foundation_off_m"])
    tree_sep = float(cfg["tree_min_separation_m"])

    out = []
    placed_trees = []
    tally = {}

    def emit(entry, x, y, yaw, category, purse):
        """Charge the purse, then place. Returns False if unaffordable."""
        if entry is None:
            return False
        pts = float(entry.get("points", 0.0) or 0.0)
        if not purse.can(pts):
            return False
        purse.charge(pts)
        fp = None
        if resolver is not None:
            fp = resolver.get(entry["usd"], category,
                              scale=entry.get("scale", 1.0),
                              axis_up=entry.get("axis_up", "Z"))
        out.append({
            "usd": entry["usd"], "x_m": x, "y_m": y,
            "z_m": (fp or {}).get("base", 0.0),
            "yaw_deg": yaw + float(entry.get("yaw_offset", 0.0)),
            "roll_deg": 90.0 if entry.get("axis_up") == "Y" else 0.0,
            "pitch_deg": 0.0, "scale": entry.get("scale", 1.0),
            "category": category, "axis_up": entry.get("axis_up", "Z"),
        })
        tally[category] = tally.get(category, 0) + 1
        return True

    for h, area in zip(houses, lot_area):
        budget.open(area)
        cx, cy = h["c"]
        u = h["u"]                      # along the street
        # The INWARD normal, taken from the lot rather than recomputed. perp(u)
        # is perpendicular to u by construction, so no dot product against u can
        # say which of the two normals faces away from the kerb -- the first
        # version of this tried exactly that and the test was identically zero,
        # leaving `deep` pointing at the street on about half the lots.
        n = h.get("n") or sn._perp(u)

        def world(along, deep):
            return (cx + u[0] * along + n[0] * deep,
                    cy + u[1] * along + n[1] * deep)

        half_w, half_d = h["w"] / 2.0, h["d"] / 2.0
        yaw_deg = h["yaw_deg"]

        # -- front: foundation row along the TRUE front wall ---------------
        n_row = int(rng.uniform(row_lo, row_hi + 0.999))
        span = min(h["w"] - 0.8, max(0.0, (n_row - 1) * gap))
        for k in range(n_row):
            a = (-span / 2.0 + k * gap) if n_row > 1 else 0.0
            x, y = world(a, -half_d - stand)
            emit(_pick(shrubs, rng), x, y, rng.uniform(0, 360), "plant",
                 budget.other)

        # -- front: one specimen tree, offset to a side, never centred -----
        if trees and rng.random() < float(cfg["specimen_tree_chance"]):
            side = 1.0 if rng.random() < 0.5 else -1.0
            x, y = world(side * (half_w * 0.75 + 1.6), -half_d - 5.0)
            if all(sn._dist((x, y), t) >= tree_sep for t in placed_trees):
                if emit(_pick(trees, rng), x, y, rng.uniform(0, 360), "tree",
                        budget.tree):
                    placed_trees.append((x, y))

        # -- front: mailbox at the kerb, facing the street -----------------
        if props and rng.random() < float(cfg["mailbox_chance"]):
            x, y = world(half_w * 0.9, -half_d - 8.5)
            emit(_pick(props, rng), x, y, yaw_deg + 90.0, "plant", budget.other)

        # -- rear: massed along the boundary, canopy over the middle -------
        step = float(cfg.get("rear_boundary_step_m", 4.5))
        depth = 16.0
        k = 0
        a = -half_w
        while a <= half_w and shrubs:
            x, y = world(a, half_d + depth)
            emit(_pick(shrubs, rng), x, y, rng.uniform(0, 360), "plant",
                 budget.other)
            a += step
            k += 1
        if trees and rng.random() < float(cfg["rear_tree_chance"]):
            x, y = world(rng.uniform(-half_w, half_w), half_d + rng.uniform(5.0, 12.0))
            if all(sn._dist((x, y), t) >= tree_sep for t in placed_trees):
                if emit(_pick(trees, rng), x, y, rng.uniform(0, 360), "tree",
                        budget.tree):
                    placed_trees.append((x, y))

    stats = {"lots": len(houses), "placed": len(out), "tally": tally,
             "points": budget.spent, "budget": budget.total,
             "refused": budget.refused}
    return out, stats


def report(stats):
    t = "  ".join(f"{k}={v}" for k, v in sorted(stats["tally"].items()))
    pct = 100.0 * stats["points"] / max(1.0, stats["budget"])
    print(f"[yardplan] {stats['placed']} placed across {stats['lots']} lots\n"
          f"[yardplan]   {t}\n"
          f"[yardplan]   {stats['points']:,} of {int(stats['budget']):,} points "
          f"({pct:.0f}%), {stats['refused']} refused on budget")
