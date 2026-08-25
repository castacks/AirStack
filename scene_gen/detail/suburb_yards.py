"""
suburb_yards.py — plant the front and rear yard of every lot.

A FRONT YARD AND A REAR YARD ARE DIFFERENT PLACES
-------------------------------------------------
A front yard is kept and seen from the street, so it is composed, not
scattered: foundation shrubs in a row against the house face, ONE specimen
tree set off to a side, a mailbox at the kerb, and an open lawn that is
deliberately left empty. Scattering plants evenly over a front yard is exactly
what makes it read as rough ground rather than as somebody's garden.

A rear yard is private and working: the bigger shrubs, a shed, a patio set, a
bin, and the canopy. It is massed rather than composed — planting along the
boundary, which is what a fence gives you something to plant against.

WHICH YARDS EXIST is not decided here. `suburb_lots.py` lots each block and
emits the FRONT and REAR rectangles; a neighbourhood that has no back gardens
simply emits no rear rects. This pass plants whatever it is handed, and can
additionally suppress a kind per neighbourhood (`skip: [rear]`) so the style
and the lotting can disagree without either having to know about the other.

THE POINT BUDGET IS THE BINDING CONSTRAINT, NOT THE LAYOUT
----------------------------------------------------------
`apply_placements` instances only the categories it is explicitly handed, and
this pipeline hands it none, so N copies of a tree cost N x its points. AEC
vegetation is archviz-grade — the CHEAPEST bound, green tree in the library is
48.7k points and the cheapest shrub-scale plant that is not a grass tuft is
34.7k — and the urban scene OOM-killed at 89.1M. A tree in every yard of a
500-lot suburb is therefore ~25M points before a single shrub goes in.

So the pass is budget-driven rather than density-driven. `point_budget` is
divided across the yards IN PROPORTION TO AREA, which makes the spend uniform
in points per square metre, and under-spend carries forward so the budget is
used rather than stranded. When it is tight the pass thins out evenly across
the whole suburb instead of planting the first half lushly and the second bare.
Two things that look like details and are not:

* TREES GET THEIR OWN PURSE (`tree_budget_frac`). Sharing one with the ground
  cover means the cheap end always wins — a grass tuft is 1,220 points against
  a tree's 48,701 — and the first version of this pass duly produced 151 trees
  across 875 yards with 85% of its budget spent on tufts.
* SPECIES IS DRAWN BEFORE PRICE IS CHECKED, and an unaffordable draw plants
  nothing rather than falling back to something cheaper. Drawing from only
  what is affordable silently deletes every expensive species: sweeping the
  accent weight over a factor of twelve moved the accent count from 1 to 3.

EVERY CATEGORY RESERVES ITS FOOTPRINT, AND SO DOES EVERY OBSTACLE
------------------------------------------------------------------
One `_Occ` (parks') for the WHOLE pass, not one per yard: yards abut, and a
front-yard tree and the neighbour's shed are a metre apart. Before anything is
planted, the houses, driveways, walks, fence lines AND every placement another
pass already put inside a yard are claimed in that same grid, so "nothing is
planted on the house, the driveway or the fence" is a property of the
reservation rather than a rule each routine has to remember — and it holds even
for obstacles the yard record does not describe.

    python suburb_yards.py --selftest        # no Isaac Sim, no Nucleus
"""

# Running as a script is the verification path and must work on a bare host, so
# the stub goes in before `scene_generator` is imported for its `pxr`.
if __name__ == "__main__":
    import sys as _sys
    import types as _types
    for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
               "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
        _sys.modules.setdefault(_m, _types.ModuleType(_m))

import math

# One implementation of oriented-footprint reservation for the whole generator.
# `city_detail._Occupancy` reserves an AABB, which is exact on a kerb where
# everything is square to the street; a yard yaws its planting to the house
# face and to the fence line, so it needs the oriented test parks already has.
from detail.parks import _Occ, _box, _cells
from scene_generator import _in_exclusion, _normalize_usd_list

_GRASS_TOP = 0.01

# Distinct from parks' `tree`/`plant`: a re-run replaces this pass's own work
# and nothing else, and the v2 tally separates yard planting from park planting.
_OWNED = ("yard_tree", "yard_shrub", "yard_prop")

# Yard-rect keys tried in order when the config names none. The key the lots
# pass emits is brokered between the two modules; `layout_key` overrides this.
_LAYOUT_KEYS = ("lot_yards", "yards", "suburb_yards", "yard_rects", "lots")


# ---------------------------------------------------------------------------
# config plumbing
# ---------------------------------------------------------------------------

def _pair(v, fallback):
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _merge(base, over):
    """Deep-merge *over* onto *base*; a neighbourhood style is a sparse patch."""
    out = dict(base or {})
    for k, v in (over or {}).items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(
            out.get(k), dict) else v
    return out


def _rect_of(obj, *keys):
    """A 4-tuple rect from a bare sequence or from any of *keys* on a dict."""
    if obj is None:
        return None
    if isinstance(obj, dict):
        for k in keys:
            r = obj.get(k)
            if r is not None:
                return _rect_of(r)
        return None
    if len(obj) < 4:
        return None
    x0, y0, x1, y1 = (float(v) for v in obj[:4])
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def _shrink(rect, m):
    """*rect* inset by *m* on every side, or None if that closes it."""
    x0, y0, x1, y1 = rect
    x0, y0, x1, y1 = x0 + m, y0 + m, x1 - m, y1 - m
    return (x0, y0, x1, y1) if x1 - x0 > 0.25 and y1 - y0 > 0.25 else None


def _rect_box(rect, pad=0.0):
    return _box((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0, 0.0,
                rect[2] - rect[0] + 2.0 * pad, rect[3] - rect[1] + 2.0 * pad)


def _box_in_rect(b, rect):
    """Is the oriented box wholly inside *rect*? (b is axis-aligned or yawed)"""
    ex = abs(b[2] * b[4]) + abs(b[3] * b[5])
    ey = abs(b[2] * b[5]) + abs(b[3] * b[4])
    return (rect[0] <= b[0] - ex + 1e-6 and b[0] + ex <= rect[2] + 1e-6
            and rect[1] <= b[1] - ey + 1e-6 and b[1] + ey <= rect[3] + 1e-6)


# ---------------------------------------------------------------------------
# the pools: scale / axis / yaw / tags, plus the two fields this pass adds
# ---------------------------------------------------------------------------

class _Lib:
    """`usds` pools with a measured `points` cost and a sampling `weight`.

    `_parse_usd_entry` ignores keys it does not know, so both live in the same
    YAML entry as the geometry they describe — the point cost of an asset is a
    property OF that asset and belongs beside its path, not in a second table
    that goes stale the moment the pool changes.

    `weight` is what makes an unaffordable asset usable at all: the library's
    handsome species cost 4-6x the cheap ones, so they are carried at a low
    weight as occasional accents rather than dropped for being expensive or
    included at a rate that eats the whole budget.
    """

    def __init__(self, config: dict):
        self._default = float(config.get("asset_scale", 1.0))
        self._root = str(config.get("asset_root", "") or "")
        self._usds = config.get("usds") or {}

    def pool(self, key):
        raw = self._usds.get(key) or []
        if isinstance(raw, (str, dict)):
            raw = [raw]
        paths, sc, au, yo, tags = _normalize_usd_list(raw, self._default,
                                                      self._root)
        out = []
        for entry, path in zip(raw, paths):
            meta = entry if isinstance(entry, dict) else {}
            out.append({
                "usd": path,
                "scale": sc.get(path, self._default),
                "axis_up": au.get(path, "Z"),
                "yaw": yo.get(path, 0.0),
                "tags": tags.get(path, frozenset()),
                # No default cost: an unmeasured asset must not be free, or the
                # budget silently stops binding the moment somebody adds one.
                "points": int(meta.get("points", 0) or 0),
                "weight": float(meta.get("weight", 1.0)),
            })
        return out


def _tagged(pool, tag):
    """Entries carrying *tag*, or the whole pool if none is tagged that way."""
    hit = [e for e in pool if tag in e["tags"]]
    return hit or pool


def _weighted(rng, entries):
    total = sum(e["weight"] for e in entries)
    if total <= 0.0:
        return entries[rng.randrange(len(entries))]
    t = rng.random() * total
    for e in entries:
        t -= e["weight"]
        if t <= 0.0:
            return e
    return entries[-1]


# ---------------------------------------------------------------------------
# the point budget
# ---------------------------------------------------------------------------

class _Purse:
    """Points spent against a ceiling, shared out by yard AREA.

    Per-yard shares rather than one running total, because a single pool would
    be spent by the first lots reached and the rest of the suburb would come out
    bare. Area-proportional shares make the spend uniform in points per square
    metre, so a deep lot is planted more heavily than a narrow one without
    either being special-cased.

    Under-spend CARRIES FORWARD, and that is the load-bearing part. The cheapest
    tree in the library is 48.7k points; at 500 lots a yard's whole share is a
    fraction of that, so a strict per-yard cap would plant no tree anywhere.
    Carrying it means the cap climbs until a yard can afford one, and because
    the yards are visited in shuffled order those trees land evenly across the
    suburb rather than in the first streets built.
    """

    INSTANCE_PTS = 400          # a transform and a prototype pointer, not a mesh

    def __init__(self, total: float, area_total: float, instanced: bool = False):
        self.total = max(0.0, float(total))
        self.area_total = max(1e-6, float(area_total))
        self.spent = 0
        self.cap = 0.0
        self.refused = 0
        # Charging every placement its asset's full point count is right only
        # when each one composes as its own reference. When the caller opts the
        # category into `apply_placements(instance_categories=...)` the tenth
        # copy of an oak costs a transform, not another 48.7k points, and
        # full-price accounting is what kept the suburb thin: the purse refused
        # trees to stay under a ceiling it was nowhere near.
        #
        # Opt-in, because this pipeline instances nothing. Off, this is
        # byte-for-byte the old behaviour.
        self.instanced = bool(instanced)
        self._seen = set()

    def _price(self, pts: float, usd=None) -> float:
        """What this placement ACTUALLY costs, given what is already on stage."""
        if self.instanced and usd is not None and usd in self._seen:
            return float(self.INSTANCE_PTS)
        return float(pts)

    def open(self, area: float) -> None:
        self.cap = min(self.total,
                       self.cap + self.total * max(0.0, area) / self.area_total)

    def can(self, pts: int, usd=None) -> bool:
        if self.spent + self._price(pts, usd) > self.cap:
            self.refused += 1
            return False
        return True

    def charge(self, pts: int, usd=None) -> None:
        self.spent += int(self._price(pts, usd))
        if self.instanced and usd is not None:
            self._seen.add(usd)

    def affordable(self, entries):
        room = self.cap - self.spent
        return [e for e in entries
                if self._price(e["points"], e.get("usd")) <= room]


class _Budget:
    """Two purses: one for trees, one for everything else.

    Split because they compete on wildly different unit costs and the cheap
    side always wins a shared purse. A grass tuft is 1.2k points and a tree is
    48.7k, so a single budget spends itself on ground cover and the suburb
    comes out treeless — which is what the first run of this pass actually did:
    151 trees across 875 yards, with 85% of the budget spent.

    `tree_budget_frac` is therefore a composition decision, not a tuning knob:
    it is the share of the scene's geometry that goes into canopy.
    """

    def __init__(self, total: float, area_total: float, tree_frac: float,
                 instanced: bool = False):
        f = min(1.0, max(0.0, float(tree_frac)))
        self.tree = _Purse(total * f, area_total, instanced)
        self.other = _Purse(total * (1.0 - f), area_total, instanced)
        self.total = float(total)

    def open(self, area: float) -> None:
        self.tree.open(area)
        self.other.open(area)

    @property
    def spent(self) -> int:
        return self.tree.spent + self.other.spent

    @property
    def refused(self) -> int:
        return self.tree.refused + self.other.refused


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

class _RectIndex:
    """Bucketed rect lookup — which yard, if any, contains a point.

    Pre-seeding the occupancy from the placements another pass already made
    means testing tens of thousands of props against up to a thousand rects;
    the buckets keep that linear in the props rather than quadratic.
    """

    def __init__(self, rects, cell=20.0):
        self.cell = max(4.0, float(cell))
        self.rects = list(rects)
        self.g = {}
        for i, r in enumerate(self.rects):
            for gx in range(int(math.floor(r[0] / self.cell)),
                            int(math.floor(r[2] / self.cell)) + 1):
                for gy in range(int(math.floor(r[1] / self.cell)),
                                int(math.floor(r[3] / self.cell)) + 1):
                    self.g.setdefault((gx, gy), []).append(i)

    def hit(self, x, y) -> bool:
        for i in self.g.get((int(math.floor(x / self.cell)),
                             int(math.floor(y / self.cell))), ()):
            r = self.rects[i]
            if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                return True
        return False


def _face_toward(rect, px, py):
    """The side of *rect* that faces (px, py): its two ends and outward normal.

    The foundation row has to follow the house wall the yard actually looks at.
    Picking the nearest side by centre distance is what puts a front-yard row
    along the gable end when the lot is deeper than it is wide, so the side is
    chosen by which outward normal the point lies beyond.
    """
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    dx, dy = px - cx, py - cy
    # Normalised by the half-extent, so "beyond the short side" and "beyond the
    # long side" are compared on the same scale.
    hx, hy = max(1e-3, (x1 - x0) / 2.0), max(1e-3, (y1 - y0) / 2.0)
    if abs(dx) / hx >= abs(dy) / hy:
        return ((x1, y0), (x1, y1), 1.0, 0.0) if dx >= 0 else \
               ((x0, y0), (x0, y1), -1.0, 0.0)
    return ((x0, y1), (x1, y1), 0.0, 1.0) if dy >= 0 else \
           ((x0, y0), (x1, y0), 0.0, -1.0)


def _far_point(rect, rng, avoid, darts=80, margin=1.0):
    """The point in *rect* furthest from everything in *avoid*.

    Best-candidate rather than rejection: an open lawn is wherever the yard is
    emptiest, and there is no threshold that means "empty enough" across yards
    that differ by an order of magnitude in area.
    """
    x0, y0, x1, y1 = rect
    if x1 - x0 < 2.0 * margin or y1 - y0 < 2.0 * margin:
        margin = 0.0
    best = None
    for _ in range(darts):
        px = rng.uniform(x0 + margin, x1 - margin)
        py = rng.uniform(y0 + margin, y1 - margin)
        d = min([(px - ax) ** 2 + (py - ay) ** 2 for ax, ay in avoid] or [0.0])
        if best is None or d > best[0]:
            best = (d, px, py)
    return best[1], best[2]


# ---------------------------------------------------------------------------
# the yard record, and where it comes from
# ---------------------------------------------------------------------------

def yards_from_layout(config: dict, layout: dict) -> list:
    """Pull the per-lot yard rectangles out of the layout dict.

    Deliberately tolerant. The key `suburb_lots.py` publishes under is brokered
    between two modules built in parallel, so this tries the configured name
    first and then a short list of candidates, and accepts either a flat list of
    yard records or a list of LOT records carrying front/rear rects. Planting
    itself takes an explicit list (`plant`); this is wiring only, and `selftest`
    checks it against a record in the shape the lots pass publishes.
    """
    cfg = config.get("suburb_yards") or {}
    keys = ([cfg["layout_key"]] if cfg.get("layout_key") else list(_LAYOUT_KEYS))
    raw = None
    for k in keys:
        if layout.get(k):
            raw = layout[k]
            break
    if not raw:
        return []

    out = []
    for rec in raw:
        if not isinstance(rec, dict):
            r = _rect_of(rec)
            if r:
                out.append({"rect": r, "kind": "front"})
            continue
        direct = _rect_of(rec, "rect", "bounds", "rect_m", "xyxy")
        if direct is not None and rec.get("kind"):
            out.append(dict(rec, rect=direct))
            continue
        # A LOT record: one entry per yard rect it carries. `suburb_lots.py`
        # emits `front_yard` / `rear_yard` as world AABBs and sets either to
        # None for a neighbourhood that has only one of them, which is why this
        # iterates the rects present rather than assuming a pair.
        for kind, *names in (("front", "front_yard", "front", "front_rect"),
                             ("rear", "rear_yard", "rear", "rear_rect",
                              "back", "back_yard")):
            r = _rect_of(rec, *names)
            if r is None:
                continue
            out.append({
                "rect": r, "kind": kind,
                "fenced": bool(rec.get("fenced", False)),
                "neighbourhood": rec.get("neighbourhood", ""),
                # `facing_deg` is the lots pass's name for the direction the
                # lot fronts the street; the mailbox is the only thing that
                # cares which way that is.
                "street_yaw_deg": float(rec.get("street_yaw_deg",
                                                rec.get("facing_deg", 0.0))),
                "house": _rect_of(rec, "house", "house_rect", "building"),
                "driveway": _rect_of(rec, "driveway", "drive", "drive_rect"),
                "walk": _rect_of(rec, "walk", "path", "walk_rect")})
    return out


# ---------------------------------------------------------------------------
# planting
# ---------------------------------------------------------------------------

def plant(config: dict, yards: list, placements: list, resolver, rng) -> dict:
    """Plant *yards*; append to *placements*. Returns a stats dict.

    The real entry point. `build` is a two-line adapter over it, so the pass is
    driven from a hand-built list of rectangles in the selftest without any of
    the layout machinery being present.
    """
    cfg = config.get("suburb_yards") or {}
    lib = _Lib(config)
    shrubs = lib.pool("yard_shrubs")
    trees = lib.pool("yard_trees")
    props = lib.pool("yard_props")
    styles = cfg.get("neighbourhoods") or {}
    exclusions = config.get("exclusions") or []

    occ = _Occ(float(cfg.get("occupancy_cell_m", 4.0)),
               float(cfg.get("occupancy_pad_m", 0.15)))
    pad = {k: float(cfg.get(f"{k}_clear_m", d)) for k, d in
           (("house", 0.5), ("drive", 0.35), ("walk", 0.25), ("fence", 0.5))}
    boundary = float(cfg.get("boundary_inset_m", 0.4))
    tree_sep = float(cfg.get("tree_min_separation_m", 3.0))
    canopy_frac = float(cfg.get("canopy_footprint_frac", 0.35))

    yards = [y for y in yards if _rect_of(y, "rect") is not None]
    for y in yards:
        y["rect"] = _rect_of(y, "rect")
    for y in yards:
        x0, y0, x1, y1 = y["rect"]
        y["_area"] = max(0.0, (x1 - x0) * (y1 - y0))

    stats = {"tally": {}, "refused_overlap": 0, "boxes": [], "yards": 0,
             "front": 0, "rear": 0, "skipped": 0, "cleared": 0,
             "points": 0, "by_kind": {}, "scatter_cells": 0,
             "scatter_filled": 0, "seeded": 0}
    if not yards:
        return stats

    budget = _Budget(float(cfg.get("point_budget", 30_000_000)),
                     sum(y["_area"] for y in yards),
                     float(cfg.get("tree_budget_frac", 0.62)))

    # ---- a re-run replaces its own work rather than doubling it
    index = _RectIndex([y["rect"] for y in yards])
    n_before = len(placements)
    placements[:] = [p for p in placements
                     if not (p.get("category") in _OWNED
                             and index.hit(float(p["x_m"]), float(p["y_m"])))]
    stats["cleared"] = n_before - len(placements)

    def fp_of(entry, cat, scale=None):
        return resolver.get(entry["usd"], cat,
                            scale=entry["scale"] if scale is None else scale,
                            axis_up=entry["axis_up"])

    # ---- claim the obstacles BEFORE planting anything -----------------------
    # Two sources, because neither is complete on its own: the yard record
    # describes the house and driveway of ITS lot, and the placement list
    # carries whatever another pass actually put on the ground — fence modules,
    # the house art, a bin.
    for y in yards:
        for key, cat in (("house", "house"), ("driveway", "driveway"),
                         ("walk", "walk")):
            r = _rect_of(y, key)
            if r:
                occ.take(_rect_box(r, pad[{"house": "house", "driveway": "drive",
                                           "walk": "walk"}[key]]), cat)
    n_seed = 0
    for p in placements:
        if p.get("category") in _OWNED:
            continue
        px, py = float(p["x_m"]), float(p["y_m"])
        if not index.hit(px, py):
            continue
        try:
            fp = resolver.get(p["usd"], p.get("category", "asset"),
                              scale=float(p.get("scale", 1.0)),
                              axis_up=p.get("axis_up", "Z"))
        except Exception:
            continue
        occ.take(_box(px, py, float(p.get("yaw_deg", 0.0)), fp["sx"], fp["sy"]),
                 str(p.get("category", "asset")))
        n_seed += 1
    stats["seeded"] = n_seed

    # ---- per-yard planting --------------------------------------------------
    rng.shuffle(yards)
    for y in yards:
        kind = "rear" if str(y.get("kind", "front")).startswith(("rear", "back")) \
            else "front"
        style = _merge(cfg.get(kind) or {},
                       (styles.get(str(y.get("neighbourhood", ""))) or {}
                        ).get(kind) or {})
        nb = styles.get(str(y.get("neighbourhood", ""))) or {}
        if kind in [str(s) for s in (nb.get("skip") or ())]:
            stats["skipped"] += 1
            continue

        rect = y["rect"]
        fenced = bool(y.get("fenced", False))
        # An unfenced lot's planting is held off the property line so the lawns
        # read as one continuous sweep, which is what an open-plan suburb looks
        # like. A fence gives you something to plant AGAINST, so a fenced lot
        # only clears the fence itself.
        plot = _shrink(rect, pad["fence"] if fenced else boundary)
        if plot is None:
            stats["skipped"] += 1
            continue
        budget.open(y["_area"])
        stats["yards"] += 1
        stats[kind] += 1
        spent0 = budget.spent

        house = _rect_of(y, "house")
        drive = _rect_of(y, "driveway")
        placed_here = []

        def place(entry, cat, role, purse, x, y_, yaw, scale=None, keep=None):
            """The ONLY way anything enters a yard: afford, measure, reserve.

            *keep* overrides the measured footprint with a square of that side.
            Planting reserves the trunk and its pit, not the crown that hangs
            over it — massed canopy is what a mature suburb looks like from
            above, and what may not overlap is the ground the trunk stands on.

            *role* is carried for verification only: "even coverage" is a claim
            about the SCATTER, and a mean nearest-neighbour distance taken over
            a 1.5 m foundation row and a scattered lawn together says nothing
            about either.
            """
            if entry is None or not purse.can(entry["points"]):
                return None
            fp = fp_of(entry, cat, scale)
            yaw_ = yaw + entry["yaw"]
            box = (_box(x, y_, yaw_, keep, keep) if keep
                   else _box(x, y_, yaw_, fp["sx"], fp["sy"]))
            if not _box_in_rect(box, plot):
                return None
            if exclusions and _in_exclusion(x, y_, exclusions):
                return None
            if not occ.reserve(box, cat):
                stats["refused_overlap"] += 1
                return None
            purse.charge(entry["points"])
            placements.append({
                "usd": entry["usd"], "x_m": x, "y_m": y_,
                "z_m": _GRASS_TOP + fp["base"], "yaw_deg": yaw_,
                "roll_deg": 90.0 if entry["axis_up"] == "Y" else 0.0,
                "pitch_deg": 0.0,
                "scale": entry["scale"] if scale is None else scale,
                "category": cat, "axis_up": entry["axis_up"]})
            stats["tally"][cat] = stats["tally"].get(cat, 0) + 1
            stats["boxes"].append((box, cat, rect, role))
            placed_here.append((x, y_))
            return fp

        def by_height(pool, lo, hi):
            """Entries whose MEASURED height falls in the band.

            Selecting by size rather than rescaling to fit: an 0.24 m grass
            tuft stretched to 1.2 m is a stretched grass tuft, not a shrub.
            Trees are the exception below — an oak at 0.55x is still an oak.

            No fallback to the whole pool when the band is empty. A foundation
            row is a row of SHRUBS; falling back would line the wall with 0.1 m
            grass patches, which is worse than the empty wall it replaces.
            """
            return [e for e in pool if lo <= fp_of(e, "yard_shrub")["sz"] <= hi]

        def tree_pick(pool, h_lo, h_hi, max_keep):
            """An affordable tree plus the scale that puts it in the band.

            Derived per asset from its own MEASURED height, never a blanket
            multiplier: the pool spans a 6.0 m fir and a 19.7 m oak, so one
            shared scale gives either a bonsai fir or an oak over the roof.

            *max_keep* is what the yard can actually hold. Without it a small
            lot gets no tree at all rather than a small one — an 11 x 7 m rear
            yard was refusing every candidate because the pool's target height
            implied a 5 m trunk reservation. A suburb of narrow lots is still a
            suburb with trees in it; they are just smaller trees.

            SPECIES FIRST, PRICE SECOND, AND NO SUBSTITUTION. The purse refills
            by about half a tree per yard, so a cheap draw is always payable and
            an accent at 4-6x never is: drawing from only what is affordable
            deletes the expensive species outright. Sweeping the accent weight
            from 0.04 to 0.50 moved the accent count from 1 to 3 in 500 lots —
            the filter, not the weight, was choosing. An unaffordable draw now
            plants nothing and the purse keeps accumulating instead.
            """
            if not pool:
                return None, None
            e = _weighted(rng, pool)
            if not budget.tree.can(e["points"]):
                return None, None
            fp0 = fp_of(e, "yard_tree")
            h = fp0["sz"]
            if h <= 0.1:
                return e, None
            crown = max(fp0["sx"], fp0["sy"]) * canopy_frac
            k = rng.uniform(h_lo, h_hi) / h
            if crown > 1e-6:
                k = min(k, max_keep / crown)
            # Floored so a species far outside the band is shrunk toward it
            # rather than mangled — better a slightly tall fir than a 0.2x one.
            k = max(0.35, min(1.4, k))
            return e, e["scale"] * k

        # -- the protected lawn: an open middle, and the one thing here that is
        # a deliberate ABSENCE. A front lawn scattered with plants stops being
        # a lawn; parks protects its greensward for the same reason.
        anchors = [((house[0] + house[2]) / 2.0, (house[1] + house[3]) / 2.0)] \
            if house else []
        if drive:
            anchors.append(((drive[0] + drive[2]) / 2.0,
                            (drive[1] + drive[3]) / 2.0))
        lawn_frac = float(style.get("lawn_frac", 0.4))
        lawn_r = math.sqrt(max(0.0, y["_area"] * lawn_frac) / math.pi)
        lawn = _far_point(plot, rng, anchors or [(plot[0], plot[1])], margin=0.5)

        def open_ground(px, py, r=0.0):
            return math.hypot(px - lawn[0], py - lawn[1]) >= lawn_r + r

        # -- the specimen / canopy tree(s). FIRST, ahead of the shrubs: a tree
        # is the most expensive thing in the pass by a factor of ten, so placed
        # last it competes for the yard's remaining share against ground cover
        # it should never have been ranked beside — which is how the first
        # version of this pass produced a treeless suburb.
        tcfg = style.get("tree") or {}
        n_lo, n_hi = _pair(tcfg.get("count"), (1.0, 1.0))
        want = rng.randint(int(n_lo), int(n_hi)) \
            if rng.random() < float(tcfg.get("chance", 0.0)) else 0
        h_lo, h_hi = _pair(tcfg.get("height_m"), (5.0, 9.0))
        # Both setbacks are capped by the yard itself. A 2.5 m house setback in
        # a 4.2 m deep front yard excludes the whole yard, and the tree that
        # does not get planted is not a safer outcome than one standing a
        # metre closer to the wall than the style asked for.
        depth = min(plot[2] - plot[0], plot[3] - plot[1])
        from_house = min(float(tcfg.get("from_house_m", 3.0)),
                         max(1.0, depth * 0.35))
        max_keep = max(tree_sep * 0.6, depth * 0.5)
        # Front and rear draw from DIFFERENT species. An ornamental evergreen
        # belongs by the front door and a 25 m spreading oak does not; the
        # pool's `front`/`rear` tags say which is which, and `_tagged` falls
        # back to everything if a pool carries no tags at all.
        tree_pool = _tagged(trees, kind)
        for _ in range(want if tree_pool else 0):
            e, sc = tree_pick(tree_pool, h_lo, h_hi, max_keep)
            if e is None:
                break
            fp = fp_of(e, "yard_tree", sc)
            crown = max(fp["sx"], fp["sy"])
            keep = min(max(tree_sep, crown * canopy_frac), max_keep)
            # Darts scored on distance from the house, the drive and whatever
            # is already planted — but the score SATURATES. Taking the strict
            # maximum put every front tree in the same corner of every lot,
            # because with one anchor the furthest point from it is always a
            # corner. Past the setback one spot is as good as another, so the
            # choice is random.
            #
            # The tree does NOT avoid the lawn. A specimen tree standing in a
            # mown lawn is what a front garden IS; parks protects its greensward
            # because a greensward is an open meadow, and copying that rule here
            # left the small lots treeless.
            avoid = anchors + placed_here
            cap2 = (from_house + 2.0) ** 2
            good, best = [], None
            for _d in range(int(tcfg.get("darts", 40))):
                px = rng.uniform(plot[0] + keep / 2.0, plot[2] - keep / 2.0)
                py = rng.uniform(plot[1] + keep / 2.0, plot[3] - keep / 2.0)
                if house and (house[0] - from_house <= px <= house[2] + from_house
                              and house[1] - from_house <= py
                              <= house[3] + from_house):
                    continue
                d = min([(px - qx) ** 2 + (py - qy) ** 2
                         for qx, qy in avoid] or [cap2])
                if d >= cap2:
                    good.append((px, py))
                elif best is None or d > best[0]:
                    best = (d, px, py)
            if good:
                tx, ty = good[rng.randrange(len(good))]
            elif best is not None:
                tx, ty = best[1], best[2]
            else:
                break
            if not place(e, "yard_tree", "tree", budget.tree, tx, ty,
                         rng.uniform(0.0, 360.0), scale=sc, keep=keep):
                break

        # -- foundation shrubs: a ROW along the house wall the yard faces.
        fnd = style.get("foundation") or {}
        if house and shrubs and rng.random() < float(fnd.get("chance", 0.0)):
            (ax, ay), (bx, by), nx, ny = _face_toward(
                house, (plot[0] + plot[2]) / 2.0, (plot[1] + plot[3]) / 2.0)
            off = float(fnd.get("offset_m", 0.9))
            step = max(0.4, float(fnd.get("spacing_m", 1.5)))
            jit = float(fnd.get("jitter_m", 0.25))
            lo, hi = _pair(fnd.get("height_m"), (0.4, 1.8))
            band = by_height(shrubs, lo, hi)
            run = math.hypot(bx - ax, by - ay)
            n = int(run // step)
            yaw = math.degrees(math.atan2(ny, nx))
            for i in range(n):
                t = (i + 0.5) / max(1, n)
                px = ax + (bx - ax) * t + nx * off + rng.uniform(-jit, jit)
                py = ay + (by - ay) * t + ny * off + rng.uniform(-jit, jit)
                aff = budget.other.affordable(band)
                if aff:
                    place(_weighted(rng, aff), "yard_shrub", "foundation",
                          budget.other, px, py, yaw + rng.uniform(-20.0, 20.0))

        # -- boundary massing. A fenced yard is planted AGAINST the fence; an
        # open one is not, because a hedge along an invisible line reads as a
        # mistake. Open lots get the same material as free-standing clumps.
        hcfg = style.get("hedge") or {}
        if shrubs and fenced and rng.random() < float(hcfg.get("chance", 0.0)):
            off = float(hcfg.get("offset_m", 0.7))
            step = max(0.5, float(hcfg.get("spacing_m", 1.8)))
            lo, hi = _pair(hcfg.get("height_m"), (0.5, 2.6))
            band = by_height(shrubs, lo, hi)
            x0, y0_, x1, y1_ = plot
            edges = ((x0 + off, y0_, x0 + off, y1_, 90.0),
                     (x1 - off, y0_, x1 - off, y1_, 270.0),
                     (x0, y0_ + off, x1, y0_ + off, 0.0),
                     (x0, y1_ - off, x1, y1_ - off, 180.0))
            # Two of the four sides, not all four: a yard hedged on every
            # boundary is a hedge maze, and which sides are planted is exactly
            # what varies between neighbouring gardens.
            picks = list(edges)
            rng.shuffle(picks)
            for ex0, ey0, ex1, ey1, yaw in picks[:rng.randint(1, 2)]:
                run = math.hypot(ex1 - ex0, ey1 - ey0)
                n = int(run // step)
                for i in range(n):
                    t = (i + 0.5) / max(1, n)
                    aff = budget.other.affordable(band)
                    if not aff:
                        break
                    place(_weighted(rng, aff), "yard_shrub", "hedge",
                          budget.other,
                          ex0 + (ex1 - ex0) * t, ey0 + (ey1 - ey0) * t,
                          yaw + rng.uniform(-15.0, 15.0))

        # -- props: shed, patio set, bin, mailbox. Each is placed where it
        # belongs rather than scattered — a shed goes in a far corner, a patio
        # set against the house, a mailbox at the street edge.
        for name, cat_tag in (("shed", "shed"), ("patio", "patio"),
                              ("bin", "bin"), ("mailbox", "mailbox")):
            chance = float(style.get(f"{name}_chance", 0.0))
            if not props or rng.random() >= chance:
                continue
            pool = budget.other.affordable(_tagged(props, cat_tag))
            if not pool:
                continue
            e = _weighted(rng, pool)
            fp = fp_of(e, "yard_prop")
            m = max(fp["sx"], fp["sy"]) / 2.0 + 0.2
            spot = _shrink(plot, m)
            if spot is None:
                continue
            if name in ("shed",):
                px, py = _far_point(spot, rng, anchors or [lawn], darts=60)
                yaw = math.degrees(math.atan2(
                    (anchors[0][1] if anchors else lawn[1]) - py,
                    (anchors[0][0] if anchors else lawn[0]) - px))
            elif name in ("patio", "bin") and house:
                (ax, ay), (bx, by), nx, ny = _face_toward(
                    house, (plot[0] + plot[2]) / 2.0, (plot[1] + plot[3]) / 2.0)
                t = rng.uniform(0.2, 0.8)
                d = m + (2.0 if name == "patio" else 0.6)
                px = min(max(ax + (bx - ax) * t + nx * d, spot[0]), spot[2])
                py = min(max(ay + (by - ay) * t + ny * d, spot[1]), spot[3])
                yaw = math.degrees(math.atan2(-ny, -nx))
            else:
                # Mailbox: the street edge, which is the yard edge furthest
                # from the house.
                px, py = _far_point(spot, rng, anchors or [lawn], darts=40)
                yaw = float(y.get("street_yaw_deg", rng.uniform(0.0, 360.0)))
            place(e, "yard_prop", name, budget.other, px, py, yaw)

        # -- what is left: a low, even scatter on the jittered grid. One
        # candidate per cell is uniform BY CONSTRUCTION; rejection sampling
        # clumps, and the clumps eat the separation budget before the gaps are
        # found, which is what makes a yard look half bare and half overgrown.
        scfg = style.get("scatter") or {}
        den = float(scfg.get("density_per_100m2", 0.0))
        if shrubs and den > 0.0:
            lo, hi = _pair(scfg.get("height_m"), (0.05, 2.6))
            band = by_height(shrubs, lo, hi)
            cell = math.sqrt(100.0 / max(0.05, den))
            for cx_, cy_, cw, ch in _cells(plot, cell, rng):
                aff = budget.other.affordable(band)
                if not aff:
                    break
                # Coverage counts only cells that were actually OFFERED a
                # planting. A cell inside the protected lawn is meant to be
                # empty and a cell reached after the purse ran dry was never
                # tried, so scoring either as a miss would report a budget
                # decision or a deliberate lawn as patchy planting.
                hit = False
                for _try in range(3):
                    px = cx_ + rng.uniform(0.1, 0.9) * cw
                    py = cy_ + rng.uniform(0.1, 0.9) * ch
                    if not open_ground(px, py):
                        hit = None
                        break
                    if place(_weighted(rng, aff), "yard_shrub", "scatter",
                             budget.other, px, py, rng.uniform(0.0, 360.0)):
                        hit = True
                        break
                if hit is not None:
                    stats["scatter_cells"] += 1
                    stats["scatter_filled"] += int(hit)

        k = stats["by_kind"].setdefault(kind, {"yards": 0, "points": 0})
        k["yards"] += 1
        k["points"] += budget.spent - spent0

    stats["points"] = budget.spent
    stats["budget"] = budget.total
    stats["budget_refused"] = budget.refused
    stats["tree_points"] = budget.tree.spent
    stats["tree_budget"] = budget.tree.total
    return stats


def build(config: dict, layout: dict, placements: list, resolver, rng) -> int:
    """Plant every yard the lots pass published. Returns the number planted."""
    yards = yards_from_layout(config, layout)
    if not yards:
        print("[yards] no yard rectangles in layout — pass skipped")
        return 0
    s = plant(config, yards, placements, resolver, rng)
    report(s)
    return s["yards"]


def report(s: dict) -> None:
    tally = "  ".join(f"{k}={v}" for k, v in sorted(s["tally"].items()))
    n = max(1, s["yards"])
    cov = (100.0 * s["scatter_filled"] / s["scatter_cells"]
           if s["scatter_cells"] else 0.0)
    print(f"[yards] planted {s['yards']} yard(s) "
          f"({s['front']} front / {s['rear']} rear), "
          f"{s['skipped']} skipped, cleared {s['cleared']}, "
          f"seeded {s.get('seeded', 0)} obstacle footprint(s)\n"
          f"[yards]   {tally}\n"
          f"[yards]   {s['refused_overlap']} refused on footprint overlap, "
          f"{s.get('budget_refused', 0)} refused on point budget; "
          f"scatter cover {cov:.0f}% of cells\n"
          f"[yards]   {s['points']:,} points of {int(s.get('budget', 0)):,} "
          f"budget ({100.0 * s['points'] / max(1.0, s.get('budget', 1)):.0f}%), "
          f"{s['points'] // n:,} per yard — "
          f"{s.get('tree_points', 0):,} of it canopy "
          f"({s['tally'].get('yard_tree', 0)} trees in {n} yards)")


# ---------------------------------------------------------------------------
# selftest — host-side, no Isaac Sim, no Nucleus, no layout pass
# ---------------------------------------------------------------------------

# The measured library, as a resolver stands in for it. These are the real
# numbers from tools/measure_assets.py at the authored scale 0.01 (every AEC
# asset is centimetre-authored), so the selftest exercises the same size
# relationships the scene will.
_MEASURED = {
    # tree                       sx     sy     sz     points
    "Black_Oak":              (25.42, 24.07, 19.74,  48_701),
    "Shumard_Oak":            (10.29, 10.47, 11.43,  54_766),
    "Douglas_Fir":            (3.02,  2.77,  6.03,   56_262),
    "Largetooth_Aspen":       (5.09,  5.20,  7.89,  197_775),
    "Common_Apple":           (5.86,  5.23,  5.59,  237_356),
    "American_Beech":         (4.09,  4.01,  5.97,  305_126),
    # shrub
    "Grass_Trimmed_C":        (0.24,  0.26,  0.09,   1_220),
    "Grass_Short_C":          (0.28,  0.30,  0.13,   2_208),
    "Grass_Trimmed_B":        (0.62,  0.64,  0.10,   8_476),
    "Grass_Short_B":          (0.66,  0.68,  0.16,  15_375),
    "Lupin":                  (1.65,  1.38,  1.37,  34_685),
    "Grass_Short_A":          (1.29,  1.29,  0.16,  44_922),
    "Meadowlark":             (0.94,  1.02,  0.92,  83_329),
    "Fountain_Grass_Short":   (2.02,  2.05,  1.47, 173_997),
    "Hibiscus":               (2.31,  2.13,  1.64, 195_688),
    "Yew":                    (1.23,  1.24,  0.73, 324_028),
    # prop
    "MobilaShelter":          (2.14,  1.95,  2.33,  11_698),
    "Kalmar_TblChr":          (2.43,  2.42,  1.01,  60_806),
    "ParkBench01":            (2.62,  0.79,  0.39,   1_463),
    "TrashCan":               (0.51,  0.51,  0.73,     630),
    "MailboxType01":          (0.65,  0.52,  1.34,     264),
    "MailboxType02":          (0.76,  0.61,  1.14,     264),
    "house":                  (12.0,  10.0,  6.0,   20_000),
}


class _FakeResolver:
    """`SizeResolver` over the measured table — the sizes are the real ones."""

    def __init__(self):
        self.scale = 0.01

    def get(self, usd_path, category, scale=None, axis_up="Z"):
        name = usd_path.rsplit("/", 1)[-1].rsplit(".", 1)[0]
        sx, sy, sz, _pts = _MEASURED.get(name, (1.0, 1.0, 1.0, 0))
        k = (float(scale) if scale is not None else 0.01) / 0.01
        return {"sx": sx * k, "sy": sy * k, "sz": sz * k,
                "base": 0.0, "cx": 0.0, "cy": 0.0, "cz": 0.0}


def _entry(name, points, weight=1.0, tags=None):
    e = {"usd": f"aec://{name}.usd", "scale": 0.01, "points": points,
         "weight": weight}
    if tags:
        e["tags"] = tags
    return e


def _selftest_config(budget=25_000_000):
    """The shipped pool and the shipped defaults, in dict form."""
    return {
        "asset_scale": 1.0,
        "usds": {
            "yard_trees": [
                _entry("Black_Oak", 48_701, 1.0, ["rear"]),
                _entry("Shumard_Oak", 54_766, 1.0, ["front", "rear"]),
                _entry("Douglas_Fir", 56_262, 0.8, ["front"]),
                _entry("Largetooth_Aspen", 197_775, 0.30, ["rear"]),
                _entry("Common_Apple", 237_356, 0.30, ["rear"]),
                _entry("American_Beech", 305_126, 0.20, ["front", "rear"]),
            ],
            "yard_shrubs": [
                _entry("Grass_Trimmed_C", 1_220, 1.0),
                _entry("Grass_Short_C", 2_208, 1.0),
                _entry("Grass_Trimmed_B", 8_476, 0.8),
                _entry("Grass_Short_B", 15_375, 0.6),
                _entry("Lupin", 34_685, 0.5),
                _entry("Grass_Short_A", 44_922, 0.2),
                _entry("Meadowlark", 83_329, 0.25),
                _entry("Fountain_Grass_Short", 173_997, 0.05),
                _entry("Hibiscus", 195_688, 0.05),
                _entry("Yew", 324_028, 0.03),
            ],
            "yard_props": [
                _entry("MobilaShelter", 11_698, 1.0, ["shed"]),
                _entry("Kalmar_TblChr", 60_806, 1.0, ["patio"]),
                _entry("ParkBench01", 1_463, 1.0, ["patio"]),
                _entry("TrashCan", 630, 1.0, ["bin"]),
                _entry("MailboxType01", 264, 1.0, ["mailbox"]),
                _entry("MailboxType02", 264, 1.0, ["mailbox"]),
            ],
        },
        "suburb_yards": {
            "point_budget": budget,
            "occupancy_cell_m": 4.0, "occupancy_pad_m": 0.15,
            "house_clear_m": 0.5, "drive_clear_m": 0.35, "walk_clear_m": 0.25,
            "fence_clear_m": 0.5, "boundary_inset_m": 0.4,
            "tree_min_separation_m": 3.0, "canopy_footprint_frac": 0.35,
            "front": {
                "lawn_frac": 0.5,
                "foundation": {"chance": 0.85, "offset_m": 0.9,
                               "spacing_m": 1.5, "jitter_m": 0.25,
                               "height_m": [0.4, 1.8]},
                "tree": {"chance": 0.7, "count": [1, 1], "darts": 40,
                         "height_m": [4.5, 7.5], "from_house_m": 2.5},
                "scatter": {"density_per_100m2": 0.3, "height_m": [0.4, 1.8]},
                "mailbox_chance": 0.6,
            },
            "rear": {
                "lawn_frac": 0.35,
                "tree": {"chance": 0.6, "count": [1, 2], "darts": 40,
                         "height_m": [6.5, 11.0], "from_house_m": 3.5},
                "hedge": {"chance": 0.7, "offset_m": 0.7, "spacing_m": 1.8,
                          "height_m": [0.5, 2.6]},
                "scatter": {"density_per_100m2": 0.9, "height_m": [0.05, 2.6]},
                "shed_chance": 0.35, "patio_chance": 0.35, "bin_chance": 0.5,
            },
            "neighbourhoods": {
                "cul_de_sac": {"rear": {"tree": {"chance": 0.7}}},
                "starter": {"skip": ["rear"]},
            },
        },
    }


def _synthetic(n_lots, rng, lot_w=18.0, front_d=9.0, rear_d=13.0,
               house_d=10.0, per_row=20):
    """Hand-built lots on a grid: house between a front and a rear rectangle."""
    yards = []
    depth = front_d + house_d + rear_d + 6.0
    for i in range(n_lots):
        col, row = i % per_row, i // per_row
        x0 = col * (lot_w + 2.0)
        y0 = row * depth
        hx0, hx1 = x0 + 2.0, x0 + lot_w - 4.0
        hy0 = y0 + front_d
        hy1 = hy0 + house_d
        house = (hx0, hy0, hx1, hy1)
        drive = (x0 + lot_w - 4.0, y0, x0 + lot_w - 0.5, hy1)
        fenced = rng.random() < 0.5
        nb = rng.choice(["", "", "cul_de_sac", "starter"])
        common = {"lot": i, "house": house, "driveway": drive,
                  "fenced": fenced, "neighbourhood": nb,
                  "street_yaw_deg": 270.0}
        yards.append(dict(common, kind="front",
                          rect=(x0, y0, x0 + lot_w, hy0)))
        yards.append(dict(common, kind="rear",
                          rect=(x0, hy1, x0 + lot_w, hy1 + rear_d)))
    return yards


def _overlaps(boxes):
    """All-pairs SAT over the RESERVED footprints. The headline number."""
    from detail.parks import _box_hit
    hits = 0
    grid = {}
    for i, (b, _cat, _r, _role) in enumerate(boxes):
        ex = abs(b[2] * b[4]) + abs(b[3] * b[5])
        ey = abs(b[2] * b[5]) + abs(b[3] * b[4])
        for gx in range(int((b[0] - ex) // 8), int((b[0] + ex) // 8) + 1):
            for gy in range(int((b[1] - ey) // 8), int((b[1] + ey) // 8) + 1):
                grid.setdefault((gx, gy), []).append(i)
    seen = set()
    for cell in grid.values():
        for a in range(len(cell)):
            for b_ in range(a + 1, len(cell)):
                key = (min(cell[a], cell[b_]), max(cell[a], cell[b_]))
                if key in seen:
                    continue
                seen.add(key)
                if _box_hit(boxes[key[0]][0], boxes[key[1]][0]):
                    hits += 1
    return hits


def _nn_stats(pts):
    """Mean / stdev of nearest-neighbour distance — is the cover even?"""
    if len(pts) < 3:
        return 0.0, 0.0
    ds = []
    for i, (x, y) in enumerate(pts):
        d = min((x - qx) ** 2 + (y - qy) ** 2
                for j, (qx, qy) in enumerate(pts) if j != i)
        ds.append(math.sqrt(d))
    m = sum(ds) / len(ds)
    return m, math.sqrt(sum((d - m) ** 2 for d in ds) / len(ds))


def selftest() -> int:
    import random

    print("suburb_yards selftest — synthetic yard rectangles, "
          "measured sizes, no Isaac Sim\n"
          "overlap/out_rect/on_solid must be 0. nnd is the nearest-neighbour\n"
          "distance among SCATTER plantings only (mean+-sd, m); cover is the\n"
          "share of scatter grid cells outside the protected lawn that\n"
          "received one — evenness, where a mean alone would hide clumping.\n")
    hdr = (f"{'seed':>5} {'lots':>5} {'lot_w':>6} {'bud':>5} {'yards':>6} "
           f"{'props':>6} {'trees':>6} {'treed':>6} {'ovl':>4} {'orect':>6} "
           f"{'osolid':>7} {'nnd':>6} {'+-':>5} {'cover':>6} {'Mpts':>7} "
           f"{'%bud':>5}  R")
    print(hdr)
    print("-" * len(hdr))
    fails = []
    for seed in (1, 7, 42, 1234):
        for lots, lot_w, fd, rd, budget in (
                (60, 18.0, 9.0, 13.0, 30_000_000),
                (60, 11.0, 5.0, 7.0, 30_000_000),     # narrow, shallow lots
                (60, 30.0, 14.0, 22.0, 30_000_000),   # generous lots
                (250, 18.0, 9.0, 13.0, 30_000_000),
                (500, 18.0, 9.0, 13.0, 30_000_000),   # the budget case
                (500, 18.0, 9.0, 13.0, 4_000_000)):   # starved
            rng = random.Random(seed)
            yards = _synthetic(lots, rng, lot_w=lot_w, front_d=fd, rear_d=rd)
            cfg = _selftest_config(budget)
            res = _FakeResolver()
            placements = []
            s = plant(cfg, yards, placements, res, random.Random(seed))

            over = _overlaps(s["boxes"])
            out_rect = sum(1 for b, _c, r, _o in s["boxes"]
                           if not _box_in_rect(b, r))
            # Independent of the reservation: recheck against every house and
            # driveway rect in the input, not against what _Occ was told.
            solid = []
            for y in yards:
                for k in ("house", "driveway"):
                    r = _rect_of(y, k)
                    if r:
                        solid.append(_rect_box(r))
            from detail.parks import _box_hit
            on_solid = sum(1 for b, _c, _r, _o in s["boxes"]
                           if any(_box_hit(b, q) for q in solid))
            nnd, nnsd = _nn_stats([(b[0], b[1]) for b, _c, _r, role
                                   in s["boxes"] if role == "scatter"])
            cover = (100.0 * s["scatter_filled"] / s["scatter_cells"]
                     if s["scatter_cells"] else 0.0)

            trees = s["tally"].get("yard_tree", 0)
            ok = (over == 0 and out_rect == 0 and on_solid == 0
                  and s["points"] <= budget)
            if not ok:
                fails.append((seed, lots, lot_w, over, out_rect, on_solid))
            print(f"{seed:>5} {lots:>5} {lot_w:>6.1f} {budget // 1000000:>4}M "
                  f"{s['yards']:>6} {len(s['boxes']):>6} {trees:>6} "
                  f"{100 * trees // max(1, s['yards']):>5}% {over:>4} "
                  f"{out_rect:>6} {on_solid:>7} {nnd:>6.2f} {nnsd:>5.2f} "
                  f"{cover:>5.0f}% {s['points'] / 1e6:>7.2f} "
                  f"{100.0 * s['points'] / budget:>4.0f}%  "
                  f"{'PASS' if ok else 'FAIL'}")

    # Determinism: the same seed must give the same scene, or nothing above
    # means anything on a re-run.
    def run(seed):
        rng = random.Random(seed)
        yards = _synthetic(60, rng)
        pl = []
        plant(_selftest_config(), yards, pl, _FakeResolver(),
              random.Random(seed))
        return [(p["usd"], round(p["x_m"], 6), round(p["y_m"], 6),
                 round(p["yaw_deg"], 6)) for p in pl]
    same = run(42) == run(42)
    print(f"\ndeterminism (seed 42 twice): {'PASS' if same else 'FAIL'}")
    if not same:
        fails.append(("determinism",))

    # A yard whose neighbourhood skips its kind must plant nothing.
    rng = random.Random(3)
    yards = [y for y in _synthetic(40, rng) if y["neighbourhood"] == "starter"]
    pl = []
    s = plant(_selftest_config(), yards, pl, _FakeResolver(), random.Random(3))
    rear_skipped = all(str(p.get("category")) in _OWNED for p in pl) and \
        s["skipped"] == sum(1 for y in yards if y["kind"] == "rear")
    print(f"neighbourhood skip (starter has no rear yards): "
          f"{'PASS' if rear_skipped else 'FAIL'} "
          f"({s['skipped']} rear yards skipped, {s['front']} front planted)")
    if not rear_skipped:
        fails.append(("skip",))

    # Pre-existing placements inside a yard must be reserved, not planted over.
    rng = random.Random(9)
    yards = _synthetic(20, rng)
    pl = [{"usd": "aec://Kalmar_TblChr.usd", "x_m": y["rect"][0] + 3.0,
           "y_m": y["rect"][1] + 2.0, "z_m": 0.0, "yaw_deg": 0.0, "scale": 0.01,
           "category": "fence", "axis_up": "Z"}
          for y in yards if y["kind"] == "rear"]
    seeded = [(p["x_m"], p["y_m"]) for p in pl]
    s = plant(_selftest_config(), yards, pl, _FakeResolver(), random.Random(9))
    from detail.parks import _box_hit
    foreign = [_box(x, y_, 0.0, 2.43, 2.42) for x, y_ in seeded]
    clash = sum(1 for b, _c, _r, _o in s["boxes"]
                if any(_box_hit(b, q) for q in foreign))
    print(f"foreign placements reserved: {'PASS' if clash == 0 else 'FAIL'} "
          f"({s.get('seeded', 0)} seeded, {clash} clashes)")
    if clash:
        fails.append(("seeded",))

    # The wiring, against a record in exactly the shape `suburb_lots.py`
    # publishes at layout["lot_yards"] — including the one-yard case, which is
    # how a neighbourhood says it has no back gardens.
    lay = {"lot_yards": [
        {"block": (0, 0, 40, 40), "neighbourhood": "cul_de_sac",
         "lot": (0.0, 0.0, 18.0, 34.0), "street_side": "S", "facing_deg": 90.0,
         "house": (2.0, 9.0, 14.0, 19.0), "driveway": (14.0, 0.0, 17.5, 19.0),
         "front_yard": (0.0, 0.0, 14.0, 9.0),
         "rear_yard": (0.0, 19.0, 18.0, 34.0),
         "fenced": True, "fence_sides": ["rear"]},
        {"block": (0, 0, 40, 40), "neighbourhood": "starter",
         "lot": (20.0, 0.0, 38.0, 34.0), "street_side": "S", "facing_deg": 90.0,
         "house": (22.0, 9.0, 34.0, 19.0), "driveway": None,
         "front_yard": (20.0, 0.0, 38.0, 9.0), "rear_yard": None,
         "fenced": False, "fence_sides": []},
    ]}
    got = yards_from_layout({"suburb_yards": {}}, lay)
    wired = (len(got) == 3
             and [g["kind"] for g in got] == ["front", "rear", "front"]
             and got[0]["fenced"] is True and got[1]["neighbourhood"] == "cul_de_sac"
             and got[0]["house"] == (2.0, 9.0, 14.0, 19.0)
             and got[0]["street_yaw_deg"] == 90.0
             and got[2]["driveway"] is None)
    print(f"layout['lot_yards'] adapter: {'PASS' if wired else 'FAIL'} "
          f"({len(got)} yards from 2 lots, one with no rear)")
    if not wired:
        fails.append(("adapter", got))

    # Built outside the f-string: an expression spanning two lines INSIDE the
    # braces is PEP 701, i.e. Python 3.12 and up, and the Isaac Sim container
    # runs older than that — inline, this file raised SyntaxError there.
    summary = ("ALL PASS" if not fails
               else f"{len(fails)} FAILURE(S): {fails!r}")
    print(f"\n{summary}")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    raise SystemExit(selftest() if "--selftest" in sys.argv[1:]
                     else (print(__doc__) or 0))
