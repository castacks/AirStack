"""urban_fire_spread — how a fire gets from ONE building to the NEXT.

WHY THIS IS A SEPARATE MODEL FROM THE WILDFIRE'S
------------------------------------------------
`disaster.fire` solves an elliptical front over a continuous fuel bed: every
point on the plate has a fuel state, the front sweeps it, and "when does this
tree ignite" is a closed form in the plate's coordinates. A city block is not
a fuel bed. It is a set of DISCRETE, non-combustible containers separated by
gaps, and fire crosses those gaps by three mechanisms that have nothing to do
with each other and different reaches by an order of magnitude:

  ATTACHED / PARTY WALL   gap < ~1 m. Terraces, back-to-back blocks, anything
                          sharing a wall or a roof void. Fire passes by
                          conduction through the wall, through service
                          penetrations, and over the top through a common
                          roof space. FAST — minutes — and near-certain. This
                          is the mechanism behind every historical
                          conflagration in a terraced street.
  RADIATION               gap ~1-12 m. A fully-involved façade is a radiant
                          panel; the neighbour's window glass fails and its
                          contents ignite when the received flux passes
                          ~12.5 kW/m2 (the piloted-ignition figure used in
                          separation-distance codes; ~6 m is where codes start
                          restricting unprotected openings). Flux falls off
                          roughly as 1/d^2, so the delay grows steeply with
                          the gap and the reach is short.
  BRANDS / SPOTTING       12-60 m, DOWNWIND ONLY. Burning debris lifted off
                          the fire building lands on a roof downwind. Long,
                          erratic, and the only mechanism that crosses a
                          street. This is what makes a conflagration jump a
                          road instead of stopping at it.

So the solve is a SHORTEST-PATH over a building graph, not a field. It is
also why the three mechanisms are kept separate rather than collapsed into
one distance falloff: they set the ENTRY POINT differently, and the entry
point is most of what makes a spreading fire read as one.

THE ENTRY POINT IS THE POINT
-----------------------------
A building that caught from its neighbour is alight ON THE SIDE FACING THAT
NEIGHBOUR, at about the storey the neighbour's fire was on — because that is
where the radiant panel was. A building that caught from a brand is alight at
its TOP, because that is where the brand landed. Scatter the origin storey and
the burning elevation randomly and you get a street of unrelated fires that
happen to be near each other; derive them from the source and the same street
reads as one event moving through it. `plan.entry_side` and `plan.origin` are
what `urban_fire.burn_building` is handed.

THE CLOCK
---------
Per building, measured from its own ignition:

    0                 ignition, one compartment
    ~8 min            flashover: that compartment is fully alight
    ~20 min           fully involved: several storeys, the façade is a
                      radiant panel and it can light its neighbours
    ~75 min           burning down; the fuel load is going
    ~140 min          burnt out; smoke only
    ~200 min          cold, and a weakened URM shell may have dropped its
                      top floor

A building only RADIATES once it is fully involved, which is why
`T_FULL` appears in the edge relaxation rather than the ignition time — a
neighbour does not catch from a fire that is still in one room. That single
term is what spaces the ignitions out into a visible wave instead of lighting
the whole block at once.

READING THE WAVE — WHICH ELAPSED TIME PRODUCES WHICH GRADIENT
------------------------------------------------------------
The whole point of solving ignition TIMES rather than picking levels is that
one `elapsed_s` then paints the whole plate at once and the result reads as
ONE event moving through a city:

    the origin            F5 / F5c / F6   burnt out, shell gutted or dropped
    the ring behind it    F4              burning down, roof gone
    the front             F2 / F3         fully involved, flames out of it
    the just-caught edge  F1              one room, smoke out of two openings
    everything beyond     F0              untouched

That gradient is NOT tuned — it falls out of `level_for_age` reading each
building's own age. But it only appears if `elapsed_s` is deep enough into
the burn, and the arithmetic is worth writing down because it is easy to ask
for an epoch that cannot show it:

  * a building leaves F4 only at `T_OUT` (140 min) and only reaches F5/F6 at
    `T_COLD` (200 min). So `elapsed_s = duration_s * start_offset_frac` must
    be >= 8400 s before ANY building can be F5c, and >= 12000 s before the
    origin can be F5 or F6. At 0.55 of a 3-hour duration (99 min) the origin
    is still F4 and the top of the ladder is simply not reachable — the wave
    is there, but it tops out one rung low.
  * along a CHAIN (a terrace, or any 1-D run) consecutive ignitions are at
    least `T_FULL + D_ATTACHED[0]` = 24 min apart, because a building has to
    be fully involved before it lights the next one. The F1 window is 8 min
    wide and the F2 window 12 min, so a single line of buildings shows at
    most one F1 and one F2 at any instant, and often neither. A 2-D block
    does show every band at once, because one fully-involved building lights
    several neighbours within the same few minutes.
  * `level_for_age` is stochastic past `T_OUT` (the F5c / F6 draws), so the
    LEVEL gradient is only strictly monotone when it is called with
    `rng=None`. The AGE gradient is always monotone. Group F5/F5c/F6 into
    one "burnt out" rank (`RANK`) before asserting monotonicity on a solve
    that was given an rng.

Nothing here imports `pxr` or touches a stage: the whole plan is solved and
asserted host-side, the same contract `disaster.people` keeps. `urban_fire`
is imported LAZILY (for `_side_neighbors` only) so this module stays free of
even a host-side dependency until something actually asks for an entry point.
"""

import math

# --- the one optional dependency ------------------------------------------
# `urban_fire` is host-side importable (no `pxr` at module scope), but it
# drags in `soot_plume`, `fire_collapse` and numpy, so it is fetched ONLY
# when something asks for a side ring or a levels check. `_UF` is a
# three-state cache: None = not tried, False = tried and unavailable.
_UF = None


def _urban_fire():
    """`disaster.urban_fire` if it imports host-side, else None."""
    global _UF
    if _UF is None:
        _UF = False
        try:                                  # inside the `disaster` package
            from . import urban_fire as m
            _UF = m
        except Exception:
            try:                              # ...or on a bare sys.path
                import urban_fire as m
                _UF = m
            except Exception:
                _UF = False
    return _UF or None


# The four elevations of a rectangular footprint, in ring order, so that
# consecutive entries share a CORNER. `urban_fire._SIDE_RING` is the same
# tuple and the same convention (`quake_flow._side_of`: the building's front
# is -Y in its own frame and is called "S"); it is replicated rather than
# imported so this module keeps working with `urban_fire` absent.
_SIDE_RING = ("S", "E", "N", "W")


def side_neighbors(side):
    """The two elevations that share a corner with `side`.

    Delegates to `urban_fire._side_neighbors` when that module is available,
    so the two can never disagree about the ring order, and falls back to the
    local `_SIDE_RING` when it is not.
    """
    uf = _urban_fire()
    fn = getattr(uf, "_side_neighbors", None) if uf is not None else None
    if fn is not None:
        return tuple(fn(side))
    i = _SIDE_RING.index(side)
    return (_SIDE_RING[i - 1], _SIDE_RING[(i + 1) % len(_SIDE_RING)])


# --- the clock, in seconds -------------------------------------------------
T_FLASHOVER = 8 * 60
T_FULL = 20 * 60          # fully involved: only now can it light a neighbour
T_DECAY = 75 * 60
T_OUT = 140 * 60
T_COLD = 200 * 60

# --- spread delays, in seconds --------------------------------------------
# Attached: the party wall buys some time and not much. Historical terrace
# fires move at roughly a house every few minutes once established.
D_ATTACHED = (4 * 60, 11 * 60)
ATTACHED_GAP_M = 1.2
# Radiation: the delay at 2 m and the delay at the reach limit. Between them
# it scales with d^2, because the flux does.
D_RAD_NEAR = 7 * 60
D_RAD_FAR = 52 * 60
RAD_REACH_M = 13.0
# Brands: long, and only downwind.
D_SPOT = (28 * 60, 95 * 60)
SPOT_MIN_M = 8.0
SPOT_REACH_M = 55.0
SPOT_P = 0.55             # share of downwind pairs a brand actually takes

# Wind. A downwind neighbour catches much sooner: the plume leans onto it, the
# flame tilts, and the radiant view factor goes up. Upwind is not immune —
# radiation does not care about wind — but it is slower.
WIND_DOWN = 0.55          # delay multiplier straight downwind
WIND_UP = 1.7             # ...and straight upwind
# A TALLER FIRE BUILDING RADIATES MORE. The panel is bigger, so the flux at a
# given distance is higher and the neighbour lights sooner. Referenced to a
# 20 m block.
H_REF_M = 20.0

# THE LADDER, AND IT IS `urban_fire.LEVELS`. This tuple is a COPY rather than
# an import so the module stays dependency-free at import time, and
# `check_levels_sync()` (and `tests/test_urban_fire_spread.py`) assert the two
# never drift. F5c and F6 were missing here while `urban_fire` already had
# them, which meant a city solve could never ask for the two states the user
# specifically asked for ("I want some partial collapse buildings for fire in
# all sets", 2026-08-30) — a spread plan can only produce a level it names.
LEVELS = ("F0", "F1", "F2", "F3", "F4", "F5", "F5c", "F6")

# How BAD each level is, for ordering only. F5c ranks WITH F5, not above it:
# it is the same fire with a different structural outcome (`urban_fire`'s own
# comment), so a gradient that runs ... F4, F5c, F5 ... has not gone
# backwards. F6 is the one rung above — the cold, fully burnt-out shell.
RANK = {"F0": 0, "F1": 1, "F2": 2, "F3": 3, "F4": 4, "F5": 5, "F5c": 5,
        "F6": 6}

# Past `T_OUT` the fire is out and the building's fate is drawn once.
COLLAPSE_P = 0.35         # ...part of the shell comes down          -> F5c
BURNT_OUT_P = 0.30        # ...a cold URM shell is a gutted ruin     -> F6
# Origin draw: the same low-biased exponent `urban_fire.plan_fire` uses for
# the origin storey, reused here for "which building starts it".
ORIGIN_BIAS = 1.7


def level_for_age(age_s, btype="urm", rng=None, burnt_out_p=BURNT_OUT_P,
                  collapse_p=COLLAPSE_P):
    """Seconds since THIS building ignited -> an `urban_fire` level.

    The same shape as `damage.level_for_age` in the wildfire path and for the
    same reason: one number has to drive the structural level, the visual
    state and the fire state TOGETHER, or a scene reads as three unrelated
    effects that happen to be in the same place. Here that number is the age
    of this building's own fire, not the age of a front passing over it.

    PAST `T_OUT` THE FIRE IS OUT AND THE STRUCTURE DECIDES, NOT THE CLOCK.
    Three outcomes, and the construction type picks which are on the table:

      F5c  part of the shell has come down and the rest stands. Drawn at
           `collapse_p` for `urm` and `rc` from `T_OUT` on. THIS IS THE ONLY
           WAY F5c ENTERS A CITY SOLVE — `fire_collapse` registers the level
           in `soot_plume.DURATION_S` at import, and nothing else asks for it.
      F6   cold and completely gutted. `urm` only, and only past `T_COLD`,
           because it is a shell that has stood burnt-out long enough to lose
           its floors.
      F5   the default end state for a cold URM shell that neither dropped a
           wall nor went to F6: gutted, top storeys fallen in, four walls up.

    `rc_glass` NEVER RETURNS F5 OR F5c. A curtain-wall frame is a concrete
    cage with non-structural cladding: the glass goes, the spandrels spall,
    and the cage stands. Giving it a partial collapse would be the one
    construction type in the set where the mechanism is wrong.

    With `rng=None` no draw is made, so the mapping is deterministic and
    monotone in age — which is what the gradient assertions in
    `tests/test_urban_fire_spread.py` and `check()` rely on.
    """
    a = float(age_s)
    if a < 0:
        return "F0"
    if a < T_FLASHOVER:
        return "F1"                 # one room, smoke out of a few openings
    if a < T_FULL:
        return "F2"                 # that compartment fully alight
    if a < T_DECAY:
        return "F3"                 # fully involved, climbing
    if a < T_OUT:
        return "F4"                 # burning down / burnt out
    if btype == "rc_glass":
        return "F4"                 # the cage stands: never F5, never F5c
    if btype in ("urm", "rc") and rng is not None \
            and rng.random() < collapse_p:
        return "F5c"
    if a >= T_COLD and btype == "urm":
        if rng is not None and rng.random() < burnt_out_p:
            return "F6"
        return "F5"
    return "F4"


def check_levels_sync():
    """`LEVELS` here == `urban_fire.LEVELS` there, or a list of complaints.

    Returns [] when `urban_fire` cannot be imported at all — this module is
    allowed to be used without it.
    """
    uf = _urban_fire()
    if uf is None:
        return []
    bad = []
    if tuple(uf.LEVELS) != tuple(LEVELS):
        bad.append("LEVELS drifted from urban_fire: {0} vs {1}".format(
            LEVELS, tuple(uf.LEVELS)))
    for lv in LEVELS:
        if lv not in RANK:
            bad.append("no RANK for level {0}".format(lv))
    return bad


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def _corners(b):
    """The four world corners of a building's footprint."""
    a = math.radians(b["yaw"])
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = b["W"] / 2.0, b["D"] / 2.0
    return [(b["x"] + ca * dx - sa * dy, b["y"] + sa * dx + ca * dy)
            for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def _seg_dist(p, q, r, s):
    """Distance between segments pq and rs (0 if they cross)."""
    def _pt_seg(pt, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        L2 = ax * ax + ay * ay
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(
            1.0, ((pt[0] - a[0]) * ax + (pt[1] - a[1]) * ay) / L2))
        return math.hypot(pt[0] - (a[0] + t * ax), pt[1] - (a[1] + t * ay))

    d1 = (q[0] - p[0]) * (s[1] - r[1]) - (q[1] - p[1]) * (s[0] - r[0])
    if abs(d1) > 1e-12:
        t = ((r[0] - p[0]) * (s[1] - r[1]) - (r[1] - p[1]) * (s[0] - r[0])) / d1
        u = ((r[0] - p[0]) * (q[1] - p[1]) - (r[1] - p[1]) * (q[0] - p[0])) / d1
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0
    return min(_pt_seg(p, r, s), _pt_seg(q, r, s),
               _pt_seg(r, p, q), _pt_seg(s, p, q))


def gap_m(a, b):
    """Clear distance between two building footprints, 0 if they touch.

    EDGE TO EDGE, NOT CENTRE TO CENTRE. Two 40 m blocks 2 m apart have their
    centres 42 m apart, which every distance-based rule would score as "far"
    — and they are close enough to share a fire. The whole model turns on this
    being the clear gap between the walls.
    """
    ca, cb = _corners(a), _corners(b)
    best = 1e9
    for i in range(4):
        for j in range(4):
            best = min(best, _seg_dist(ca[i], ca[(i + 1) % 4],
                                       cb[j], cb[(j + 1) % 4]))
    return best


def bearing(a, b):
    """Compass bearing from a's centre to b's centre, radians, +X = 0."""
    return math.atan2(b["y"] - a["y"], b["x"] - a["x"])


def side_facing(b, wx, wy):
    """Which of b's elevations (S/E/N/W, in its own frame) faces (wx, wy).

    `urban_building`'s front is -Y in the building's local frame, which
    `quake_flow._side_of` calls "S"; the same convention has to be used here
    or every fire vents out of the back of its building.
    """
    a = math.radians(-b["yaw"])
    dx, dy = wx - b["x"], wy - b["y"]
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    # normalise by the half-extents so a long thin building still picks its
    # long face when the source is off its end
    nx = lx / max(1e-3, b["W"] / 2.0)
    ny = ly / max(1e-3, b["D"] / 2.0)
    if abs(nx) >= abs(ny):
        return "E" if nx > 0 else "W"
    return "N" if ny > 0 else "S"


def _wind_factor(brg, wind_dir, strength):
    """Delay multiplier for a neighbour lying at `brg` from the fire.

    `wind_dir` is the direction the wind BLOWS TOWARD, in radians.
    """
    if strength <= 0:
        return 1.0
    c = math.cos(brg - wind_dir)          # +1 straight downwind
    f = 1.0 + (1.0 - c) * 0.5 * (WIND_UP - WIND_DOWN) + (WIND_DOWN - 1.0)
    return max(WIND_DOWN, min(WIND_UP, 1.0 + (f - 1.0) * min(1.0, strength / 8.0)))


# ---------------------------------------------------------------------------
# The solve
# ---------------------------------------------------------------------------
def edges(buildings, wind_dir=0.0, wind_mps=5.0, rng=None,
          blocked=frozenset()):
    """Every ordered pair the fire could cross, with its delay and mechanism.

    Returns [(i, j, delay_s, mechanism)].

    A BLOCKED INDEX IS NOT IN THE GRAPH AT ALL — no edge in, no edge out.
    That is a firebreak, not a fireproof building: the fire has to route
    AROUND a blocked run, and if the only path to the far side went through
    one, the far side simply never catches. This is how a city solve keeps
    the tower/highrise districts out of a conflagration (the no-fire rule
    lives on the BLOCK, `layout["_typology_of"]`), and it is why the rule is
    expressed by deleting nodes rather than by discarding the result
    afterwards: a tower cannot be a stepping stone it was never on.
    """
    out = []
    n = len(buildings)
    blocked = frozenset(blocked or ())
    for i in range(n):
        if i in blocked:
            continue
        a = buildings[i]
        ha = max(6.0, float(a.get("H", H_REF_M)))
        for j in range(n):
            if i == j or j in blocked:
                continue
            b = buildings[j]
            g = gap_m(a, b)
            brg = bearing(a, b)
            wf = _wind_factor(brg, wind_dir, wind_mps)
            # a bigger radiant panel lights its neighbour sooner
            hf = (H_REF_M / ha) ** 0.45
            if g <= ATTACHED_GAP_M:
                lo, hi = D_ATTACHED
                d = (lo + (hi - lo) * (rng.random() if rng else 0.5))
                out.append((i, j, d * hf, "attached"))
                continue
            if g <= RAD_REACH_M:
                # flux ~ 1/d^2, so delay ~ d^2 between the two anchors
                t = (g - 2.0) / max(1e-3, RAD_REACH_M - 2.0)
                t = max(0.0, min(1.0, t))
                d = D_RAD_NEAR + (D_RAD_FAR - D_RAD_NEAR) * (t ** 2)
                jitter = (0.8 + 0.4 * rng.random()) if rng else 1.0
                out.append((i, j, d * wf * hf * jitter, "radiation"))
                continue
            if SPOT_MIN_M <= g <= SPOT_REACH_M:
                # BRANDS ONLY GO DOWNWIND. This is the only mechanism that
                # crosses a street, and gating it on the wind is what gives a
                # conflagration a DIRECTION — without it the fire grows as a
                # disc and reads as an explosion rather than a spread.
                if math.cos(brg - wind_dir) < 0.35 or wind_mps < 1.0:
                    continue
                if rng is not None and rng.random() > SPOT_P:
                    continue
                lo, hi = D_SPOT
                u = rng.random() if rng else 0.5
                d = lo + (hi - lo) * (g / SPOT_REACH_M) * (0.6 + 0.8 * u)
                out.append((i, j, d * wf, "spot"))
    return out


def solve(buildings, origin_idx, elapsed_s, wind_dir=0.0, wind_mps=5.0,
          rng=None, btype_of=None, collapse_p=COLLAPSE_P,
          blocked=frozenset(), max_burnt=None, burnt_out_p=BURNT_OUT_P):
    """Ignition time, level and entry point for every building.

    A Dijkstra relaxation from `origin_idx` over the edge set, with the
    building's own `T_FULL` added before it can light anyone — a fire in one
    room does not ignite the building opposite.

    `blocked` is a set of indices that are NOT IN THE GRAPH (see `edges`):
    they come back `t_ignite=None`, `level="F0"`, and can never appear as
    another building's `via`. `origin_idx` may not be blocked.

    `max_burnt`, if given, caps the burnt set to that many buildings by
    `cap_to_prefix` — see there for why a prefix of the Dijkstra order is
    still one connected fire.

    Returns a list of dicts, one per building, in the same order:
        t_ignite   seconds from t=0 (`None` = never caught)
        age        `elapsed_s - t_ignite`
        level      one of `LEVELS`
        via        which building lit it, or None for the origin
        how        attached / radiation / spot / origin
        entry_side S/E/N/W in the building's own frame
        origin_frac the height the fire entered at, as a fraction
    """
    n = len(buildings)
    blocked = frozenset(blocked or ())
    if origin_idx in blocked:
        raise ValueError(
            "solve: origin {0} is blocked — pick_origin() draws only from "
            "the unblocked set for exactly this reason".format(origin_idx))
    E = edges(buildings, wind_dir, wind_mps, rng, blocked=blocked)
    adj = {}
    for i, j, d, how in E:
        adj.setdefault(i, []).append((j, d, how))
    INF = float("inf")
    t = [INF] * n
    via = [None] * n
    how = [None] * n
    t[origin_idx] = 0.0
    how[origin_idx] = "origin"
    # small n, so an O(n^2) scan is simpler than a heap and just as fast.
    # A blocked node starts DONE, so the scan can never settle it and it can
    # never be relaxed out of (belt and braces: `edges` already dropped it).
    done = [i in blocked for i in range(n)]
    for _ in range(n):
        u, best = -1, INF
        for k in range(n):
            if not done[k] and t[k] < best:
                u, best = k, t[k]
        if u < 0:
            break
        done[u] = True
        # IT MUST BE FULLY INVOLVED BEFORE IT CAN LIGHT ANYTHING.
        ready = t[u] + T_FULL
        for v, d, hw in adj.get(u, ()):
            if done[v]:
                continue
            if ready + d < t[v]:
                t[v] = ready + d
                via[v] = u
                how[v] = hw
    out = []
    for k in range(n):
        b = buildings[k]
        ti = None if t[k] == INF else t[k]
        age = None if ti is None else (float(elapsed_s) - ti)
        bt = (btype_of(b) if btype_of else "urm")
        lvl = ("F0" if age is None
               else level_for_age(age, bt, rng, burnt_out_p, collapse_p))
        # --- where the fire got in ---------------------------------------
        side, frac = None, 0.25
        if via[k] is not None:
            src = buildings[via[k]]
            side = side_facing(b, src["x"], src["y"])
            if how[k] == "spot":
                # A BRAND LANDS ON THE ROOF. It is the one mechanism that
                # starts a fire at the TOP of a building, and that reads
                # instantly as a different mechanism from the wall-to-wall
                # spread happening down the street.
                frac = 0.88
            elif how[k] == "attached":
                # through the party wall, wherever the neighbour was worst —
                # low, because that is where its fire started
                frac = 0.22
            else:
                # radiation: at about the height of the radiant panel, which
                # is the middle of the neighbour's involved band
                frac = 0.45
        elif how[k] == "origin":
            side = None                    # drawn by the caller
            frac = 0.15                    # fires start low
        out.append({"i": k, "t_ignite": ti, "age": age, "level": lvl,
                    "via": via[k], "how": how[k], "entry_side": side,
                    "origin_frac": frac})
    if max_burnt is not None:
        out = cap_to_prefix(out, max_burnt)
    return out


# ---------------------------------------------------------------------------
# Choosing the origin, and capping the size of the fire
# ---------------------------------------------------------------------------
def pick_origin(buildings, blocked=frozenset(), rng=None, epicenter=None):
    """Which building starts it — biased toward the epicentre, never blocked.

    A city preset names an `epicenter` (`compile_disaster` compiles one for
    `fire` the same way it does for every other disaster type). The origin
    has to RESPECT it without being pinned to it: pinning puts the fire in
    whatever happens to stand at that coordinate — a tower, a car park, a
    building with no bake kind — and the whole plate is then decided by one
    lookup. So the unblocked candidates are RANKED by `1 / (1 + distance)`
    (nearest first) and one is drawn off that ranking with `u ** 1.7`, the
    same low-biased shape `urban_fire.plan_fire` uses to put a fire's origin
    storey near the ground: about 60 % of draws land in the nearest third,
    and the tail still reaches the far side of the plate.

    Deterministic for a given `rng` — exactly one `rng.random()` is consumed,
    so a caller can reproduce a plate from `(layout seed, FIRE_SEED)`.
    `epicenter` defaults to the plate centre, `(0, 0)`.
    """
    blocked = frozenset(blocked or ())
    cands = [i for i in range(len(buildings)) if i not in blocked]
    if not cands:
        raise ValueError("pick_origin: no unblocked building to ignite")
    ex, ey = ((0.0, 0.0) if epicenter is None
              else (float(epicenter[0]), float(epicenter[1])))

    def _key(i):
        b = buildings[i]
        d = math.hypot(float(b["x"]) - ex, float(b["y"]) - ey)
        # rank by the WEIGHT, descending; the index breaks ties so two
        # buildings the same distance out always rank the same way
        return (-(1.0 / (1.0 + d)), i)

    ranked = sorted(cands, key=_key)
    u = rng.random() if rng is not None else 0.0
    k = int(len(ranked) * (u ** ORIGIN_BIAS))
    return ranked[max(0, min(len(ranked) - 1, k))]


def cap_to_prefix(plan, n):
    """Keep the `n` earliest ignitions and send the rest back to F0.

    WHY A PREFIX IS ALWAYS ONE CONNECTED FIRE. `solve` is a Dijkstra, so
    every building's `via` is its parent in a shortest-path tree rooted at
    the origin, and the relaxation is `t[v] = t[via] + T_FULL + delay`. Both
    `T_FULL` (1200 s) and every `delay` are strictly positive, so

        t_ignite(via(v))  <  t_ignite(v)      for every non-origin v

    — a parent ALWAYS ignites strictly before its child. Sorting by
    `t_ignite` therefore lists every parent before its child, and a prefix of
    that order is closed under `via`: the kept set is a subtree containing
    the origin, which is to say one connected fire. No search, no repair
    pass, no "nearest N" heuristic that would leave islands of burnt
    buildings with untouched ones between them.

    That is asserted here rather than assumed, because it is the property the
    whole city plan rests on (`_plans/urban_fire_city_plan.md` §5 check 2).

    Returns a NEW plan (the dicts are copied); the input is untouched.
    """
    n = max(0, int(n))
    lit = sorted((p for p in plan if p["t_ignite"] is not None),
                 key=lambda p: (p["t_ignite"], p["i"]))
    keep = set(p["i"] for p in lit[:n])
    for p in lit[:n]:
        if p["via"] is not None and p["via"] not in keep:
            raise AssertionError(
                "cap_to_prefix: building {0} kept but its via {1} was not — "
                "the Dijkstra order is not parent-before-child".format(
                    p["i"], p["via"]))
    out = []
    for p in plan:
        q = dict(p)
        if q["t_ignite"] is not None and q["i"] not in keep:
            q.update({"t_ignite": None, "age": None, "level": "F0",
                      "via": None, "how": None, "entry_side": None,
                      "origin_frac": 0.25})
        out.append(q)
    return out


def entry_for_plan_fire(rec, n_storeys, rng=None):
    """One spread record + a building's storey count -> `plan_fire` arguments.

    Returns `(origin_storey, sides)` to be passed straight through as
    `urban_fire.plan_fire(info, level, rng, origin=..., sides=...)`.

    THIS IS THE JOIN BETWEEN THE TWO MODELS AND IT IS THE WHOLE POINT OF
    SOLVING SPREAD AT ALL. `plan_fire` left to itself DRAWS the origin storey
    and shuffles the elevations, which is right for a single building on a
    bench and wrong for a street: every fire then vents in a random
    direction and the row reads as unrelated fires that happen to be
    adjacent. Handing it the storey and the side the SOURCE implies is what
    makes a row read as one event travelling along it.

      * `origin_frac` is a fraction of the building's HEIGHT and `plan_fire`
        wants a storey INDEX, so it is scaled by `n_storeys - 1`. A brand
        (0.88) therefore lands in the top eighth of the block — the top
        storey of a 3-storey terrace, storey 21 of a 25-storey tower — and a
        party wall (0.22) low, at both heights. It is a fraction rather than
        "the top storey" because on anything tall those are different
        places, and a brand fire that starts four floors down and climbs is
        the correct picture.
      * F1/F2 vent through ONE elevation — the compartment that is alight
        opens onto exactly one face. From F3 the fire has gone through the
        floor plate, so it takes the elevation round the CORNER as well
        (`side_neighbors`); `plan_fire`'s own default does the same thing,
        this just makes the choice follow the source instead of a shuffle.

    The ORIGIN building has `entry_side=None` (nothing lit it), so a side is
    drawn from `rng` when one is supplied and falls back to "S" when it is
    not — the caller that wants a specific street elevation should overwrite
    it.
    """
    n = max(1, int(n_storeys))
    frac = rec.get("origin_frac")
    frac = 0.15 if frac is None else max(0.0, min(1.0, float(frac)))
    storey = max(0, min(n - 1, int(round(frac * (n - 1)))))
    side = rec.get("entry_side")
    if side not in _SIDE_RING:
        side = _SIDE_RING[rng.randrange(len(_SIDE_RING))] if rng else "S"
    if RANK.get(rec.get("level", "F0"), 0) >= RANK["F3"]:
        nb = side_neighbors(side)
        return storey, (side, nb[rng.randrange(len(nb))] if rng else nb[0])
    return storey, (side,)


def summarise(buildings, plan, elapsed_s):
    """One line per building, oldest fire first — the banner."""
    rows = []
    for p in plan:
        b = buildings[p["i"]]
        rows.append((-1e9 if p["t_ignite"] is None else -p["t_ignite"],
                     p["i"], p, b))
    # SORT ON THE KEY AND THE INDEX ONLY. A plain `rows.sort()` compares the
    # tuples elementwise, so ANY TIE on the first element falls through to
    # comparing the `plan` dicts and raises `TypeError: '<' not supported
    # between instances of 'dict' and 'dict'`. Every building the fire never
    # reached ties at -1e9, so this fires the moment a plate has two of them
    # — which a 100 m block never had and a 500 m city has eighteen of.
    rows.sort(key=lambda r: (r[0], r[1]))
    lines = []
    for _k, _i, p, b in rows:
        if p["t_ignite"] is None:
            lines.append("  {0:<18} {1}  not reached".format(
                b.get("style", "?"), p["level"]))
            continue
        lines.append(
            "  {0:<18} {1}  lit at T+{2:>3.0f} min ({3:<9}) burning {4:>3.0f} "
            "min, in on the {5} face at {6:.0%} height{7}".format(
                b.get("style", "?"), p["level"], p["t_ignite"] / 60.0,
                p["how"], max(0.0, p["age"]) / 60.0, p["entry_side"] or "-",
                p["origin_frac"],
                "" if p["via"] is None
                else "  <- {0}".format(buildings[p["via"]].get("style", "?"))))
    return lines


def check(verbose=True):
    """Host-side sanity: the mechanisms fire, and the wave has a direction."""
    import random
    bad = []
    # a terrace: five 20 x 15 m buildings 0.5 m apart, and one across a 14 m
    # street from the middle of it
    bs = [{"x": -40.0 + i * 20.5, "y": 0.0, "W": 20.0, "D": 15.0,
           "yaw": 0.0, "H": 18.0, "style": "t{0}".format(i)} for i in range(5)]
    bs.append({"x": 0.0, "y": -29.0, "W": 20.0, "D": 15.0, "yaw": 180.0,
               "H": 18.0, "style": "across"})
    rng = random.Random(3)
    pl = solve(bs, 0, 120 * 60, wind_dir=0.0, wind_mps=6.0, rng=rng)
    ts = [p["t_ignite"] for p in pl[:5]]
    if any(x is None for x in ts):
        bad.append("terrace: not every attached neighbour caught")
    elif ts != sorted(ts):
        bad.append("terrace: ignition order is not down the row: {0}".format(ts))
    if pl[0]["level"] not in ("F4", "F5"):
        bad.append("origin at T+120 min should be burnt out, got {0}"
                   .format(pl[0]["level"]))
    if pl[4]["t_ignite"] is not None and pl[4]["level"] in ("F4", "F5"):
        bad.append("the far end of the terrace should still be burning")
    # the entry side of a building lit from its west neighbour must be W
    if pl[1]["entry_side"] != "W":
        bad.append("entry side should face the source, got {0}"
                   .format(pl[1]["entry_side"]))
    # gap: touching rects
    if gap_m(bs[0], bs[0]) > 1e-6:
        bad.append("gap to self should be 0")
    if abs(gap_m(bs[0], bs[1]) - 0.5) > 0.01:
        bad.append("terrace gap should be 0.5 m, got {0:.2f}".format(
            gap_m(bs[0], bs[1])))
    # --- the ladder is the one `urban_fire` publishes ---------------------
    bad.extend(check_levels_sync())
    # --- a blocked building is a firebreak, not a fireproof house ---------
    blk = solve(bs, 0, 120 * 60, wind_dir=0.0, wind_mps=6.0,
                rng=random.Random(3), blocked=frozenset([2]))
    if blk[2]["t_ignite"] is not None or blk[2]["level"] != "F0":
        bad.append("blocked building ignited anyway: {0}".format(blk[2]))
    if any(p["via"] == 2 for p in blk):
        bad.append("blocked building was used as a via")
    # --- the prefix cap keeps a connected fire ----------------------------
    cap = cap_to_prefix(pl, 3)          # raises if the prefix is not a subtree
    if sum(1 for p in cap if p["t_ignite"] is not None) != 3:
        bad.append("cap_to_prefix kept the wrong number of buildings")
    if cap[0]["t_ignite"] != 0.0:
        bad.append("cap_to_prefix dropped the origin")
    if cap_to_prefix(pl, 999) != pl:
        bad.append("cap_to_prefix over-capped a plan smaller than N")
    # --- the origin is drawn, low-biased, off the epicentre ---------------
    o1 = pick_origin(bs, frozenset([0, 1]), random.Random(7), (0.0, 0.0))
    o2 = pick_origin(bs, frozenset([0, 1]), random.Random(7), (0.0, 0.0))
    if o1 != o2:
        bad.append("pick_origin is not deterministic for a given rng")
    if o1 in (0, 1):
        bad.append("pick_origin returned a blocked building")
    # --- the entry point `plan_fire` is handed ----------------------------
    st, sides = entry_for_plan_fire(pl[1], 6)
    if not 0 <= st <= 5 or sides[0] != "W":
        bad.append("entry_for_plan_fire: got storey {0} sides {1}".format(
            st, sides))
    if len(entry_for_plan_fire(pl[2], 6)[1]) != 2:
        bad.append("F3+ should vent through two elevations")
    if verbose:
        print("[urban_fire_spread] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
        for ln in summarise(bs, pl, 120 * 60):
            print(ln)
    return bad
