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

Nothing here imports `pxr` or touches a stage: the whole plan is solved and
asserted host-side, the same contract `disaster.people` keeps.
"""

import math

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

LEVELS = ("F0", "F1", "F2", "F3", "F4", "F5")


def level_for_age(age_s, btype="urm", rng=None, collapse_p=0.35):
    """Seconds since THIS building ignited -> an `urban_fire` level.

    The same shape as `damage.level_for_age` in the wildfire path and for the
    same reason: one number has to drive the structural level, the visual
    state and the fire state TOGETHER, or a scene reads as three unrelated
    effects that happen to be in the same place. Here that number is the age
    of this building's own fire, not the age of a front passing over it.
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
    # Past `T_OUT` the fire is out. A masonry shell that has stood gutted for
    # an hour is the only thing in this model that collapses, and most of them
    # still do not.
    if btype == "urm" and rng is not None and rng.random() < collapse_p:
        return "F5"
    return "F4"


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
def edges(buildings, wind_dir=0.0, wind_mps=5.0, rng=None):
    """Every ordered pair the fire could cross, with its delay and mechanism.

    Returns [(i, j, delay_s, mechanism)].
    """
    out = []
    n = len(buildings)
    for i in range(n):
        a = buildings[i]
        ha = max(6.0, float(a.get("H", H_REF_M)))
        for j in range(n):
            if i == j:
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
          rng=None, btype_of=None, collapse_p=0.35):
    """Ignition time, level and entry point for every building.

    A Dijkstra relaxation from `origin_idx` over the edge set, with the
    building's own `T_FULL` added before it can light anyone — a fire in one
    room does not ignite the building opposite.

    Returns a list of dicts, one per building, in the same order:
        t_ignite   seconds from t=0 (`None` = never caught)
        age        `elapsed_s - t_ignite`
        level      F0..F5
        via        which building lit it, or None for the origin
        how        attached / radiation / spot / origin
        entry_side S/E/N/W in the building's own frame
        origin_st  the storey the fire entered at, as a FRACTION of height
    """
    n = len(buildings)
    E = edges(buildings, wind_dir, wind_mps, rng)
    adj = {}
    for i, j, d, how in E:
        adj.setdefault(i, []).append((j, d, how))
    INF = float("inf")
    t = [INF] * n
    via = [None] * n
    how = [None] * n
    t[origin_idx] = 0.0
    how[origin_idx] = "origin"
    # small n, so an O(n^2) scan is simpler than a heap and just as fast
    done = [False] * n
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
        lvl = "F0" if age is None else level_for_age(age, bt, rng, collapse_p)
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
    return out


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
    if verbose:
        print("[urban_fire_spread] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
        for ln in summarise(bs, pl, 120 * 60):
            print(ln)
    return bad
