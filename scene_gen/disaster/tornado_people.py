"""tornado_people.py — where the people a tornado HIT actually are.

THIS IS NOT `disaster.people` WITH THE NAMES CHANGED, and the reason is worth
stating before any of the numbers: a wildfire and a tornado put people in
different places because one of them gives you hours and the other gives you
minutes.

    wildfire   hours of warning -> people MOVE. `disaster.people` is a model
               of EGRESS: refuge car parks, roadways, gridlocked queues, the
               cul-de-sac trap. Location is a function of the road network.

    tornado    ten to fifteen minutes -> nobody evacuates. People shelter
               where they already were, the building fails around them, and
               the survivors walk out of it. Location is a function of WHERE
               THEY LIVED and WHAT THE WIND DID TO IT.

So the scenarios here are not analogues of the wildfire ones. Four of that
module's six have no tornado counterpart at all — see WHAT IS DELIBERATELY
ABSENT at the bottom.

THE FACT THE WHOLE MODULE TURNS ON
----------------------------------
**Almost everybody survives, and almost all of them are on their feet.**

  * Joplin's catastrophic zone — total structural destruction — held 4,716
    people and killed 122. **97.4% survived.** Across the whole path, 13,547
    people, 161 dead (1.2%), ~1,371 injured (~10%): **~89% alive and mobile.**
    (NIST NCSTAR 3 geolocated all 161; Paul & Stimers 2014 mapped population
    by damage zone.)
  * Of the injured, **89% are MINOR** (ISS < 10), 6% moderate, 5% severe, and
    **86% are discharged home**. (Niederkrotenthaler et al. 2013, chart
    abstraction of 1,398 patients across 39 Alabama hospitals.)
  * Deaths are near-instantaneous: **86.6% died on scene** (Chiu et al. 2013),
    94% in May et al. 2000. There is no large population of "dying but
    savable" victims the way an earthquake collapse produces.

A scene built on the intuition that a levelled block is a morgue is therefore
wrong in the most consequential way available: it would be EMPTY of the
targets the benchmark exists to find. The corridor should be full of standing,
walking, digging people.

AND THEY CLUSTER
----------------
Chiu et al. recorded who each victim was found with:

    with other deceased AND survivors   26.3%
    with other deceased                 24.7%
    with other survivors                21.1%
    ALONE                                7.7%

Over ninety per cent of people are found with somebody else. Group placement
is the empirical default, not a rendering convenience — and a CLUSTER is
itself the strongest aerial indicator that someone is trapped underneath it.

WHAT IS VISIBLE FROM A DRONE, WHICH IS THE ONLY THING THAT COUNTS
-----------------------------------------------------------------
`disaster.people` retired its `exposed_interior` scenario and the reasoning
governs this module too: *a target that cannot be seen cannot be labelled, and
an annotation for one that is invisible is worse than no annotation at all.*

So nothing here is placed in a cellar, a safe room, or under an intact roof —
not because those are unrealistic (they are exactly where people shelter) but
because a benchmark cannot score them. `trapped_partial` is the deliberate
exception and the hardest thing in the file: a figure that is MOSTLY buried
and PARTLY visible. Fully buried is worthless; fully exposed is a lie. It is
the one scenario whose correctness has to be confirmed by looking at a render
rather than by reading the code — which is what
`tornado_people_preview_launch_script.py` exists for.

A PURE PLANNER
--------------
No stage, no Isaac imports, no USD. `plan_people` takes a context of things
somebody else built and returns placements plus ground truth, so the whole
plan can be run and asserted on the host before a container is started.
"""

import math
import os
import random

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
# Shares are of `total` and are normalised, so editing one does not silently
# change the head count.

DEFAULTS = {
    # DENSER THAN THE WILDFIRE SCENE, and that follows from the survival
    # numbers rather than from taste. ~39 damaged houses at ~2.5 residents
    # each is ~100 people present, of whom ~95 are physically uninjured and
    # most are outdoors within the hour. 90 visible figures over a 500 m
    # corridor is the honest reading of that; the wildfire plat's 60 over
    # 1600 x 1200 m is an order of magnitude sparser per hectare because an
    # evacuated suburb is genuinely empty.
    "total": 90,
    "seed": 91,
    # Nobody stands inside anybody else. 1.1 m is a shoulder-to-shoulder
    # crowd; below ~0.9 m two 0.5 m-wide figures interpenetrate.
    "min_separation_m": 1.1,

    # THE MOMENT THIS SCENE DEPICTS. T+30-60 min is the only window in which
    # aerial victim detection has any value, and it is also the peak of
    # visible human density:
    #
    #   T+0-15    self-extrication; roads impassable; nobody in uniform
    #   T+15-60   PEAK. neighbours digging, injured moved in private pickups,
    #             first patients reaching hospitals (Joplin: 100 arrivals in
    #             16 minutes, starting T+39)
    #   T+6-24 h  organised grid search, task forces, heavy equipment — and
    #             by then there is nobody left alive to find
    #
    # Joplin's professional rubble-rescue count was SEVENTEEN, against 1,371
    # injured; the earthquake literature puts civilian extrication at 60-100%.
    # So: no responders, no heavy equipment, no search markings. A scene with
    # those is depicting T+24 h and a different problem.
    "epoch_min": 45.0,

    "scenarios": {
        # Survivors on the wreckage of their own house. The commonest thing
        # in the scene and the most visible: with the structure gone there is
        # nothing left to occlude them, so a standing figure on a debris field
        # is a vertical object on a horizontal one.
        # Standing or walking on the pile. NOBODY WAVES: the raised-arm pose
        # read as a mannequin on review and was removed from the pose table
        # (scene_generator._HUMAN_POSES); a `wave_share` here is ignored.
        "on_the_rubble": {"share": 0.28, "per_wreck": [1, 3]},

        # THE ANCHOR SCENARIO. 3-6 people working at ONE collapsed house —
        # the shape Chiu's 92%-not-alone figure implies, and the shape that
        # tells a searcher something is under there. Sited where a wrecked
        # house adjoins a standing one, because that is where the diggers
        # come from.
        "neighbour_dig": {"share": 0.24, "group_size": [3, 6],
                          "cluster_r_m": [1.6, 4.5],
                          "crouch_share": 0.45},

        # Partially buried. See the module docstring — the hardest and most
        # valuable label here.
        "trapped_partial": {"share": 0.12, "sink_frac": [0.30, 0.55],
                            "planks_over": [3, 7]},

        # The road surface is the only navigable ground in a levelled
        # subdivision — Joplin's search doctrine was literally "house by
        # house, car by car, block by block".
        "street": {"share": 0.16, "groups": [1, 2]},

        # Two supporting a third. The visible signature of the 89%-minor
        # injury population, and of private vehicles being the ambulances
        # (38% of transport, against 31% by ground ambulance).
        "assisted": {"share": 0.08},

        # Occupants of displaced vehicles. UPRIGHT CARS ONLY — see
        # `_in_vehicle`.
        "in_vehicle": {"share": 0.08, "per_car": [1, 2]},

        # Thrown clear. CAPPED IN ABSOLUTE TERMS, not by share — see
        # `_thrown` for the arithmetic that says this is about one person in
        # a scene this size.
        "thrown": {"share": 0.04, "max_count": 2,
                   "range_m": [10.0, 40.0]},
    },
}

SCENARIOS = ("on_the_rubble", "neighbour_dig", "trapped_partial", "street",
             "assisted", "in_vehicle", "thrown")

# HOW DEEP THE DEBRIS IS, by damage level, in metres. Used to stand a figure
# ON the wreckage rather than through it, and to decide how far a trapped
# figure sinks.
#
# ESTIMATED, NOT MEASURED, and flagged as such because it is the one number
# here with no literature behind it. A levelled house leaves the deepest pile
# (everything is still on the lot); a swept slab leaves the shallowest
# (the material is downwind). The preview bench is what confirms these; if a
# figure floats or sinks, this table is the thing to correct.
DEBRIS_Z_M = {
    "pristine": 0.0,
    "roof_stripped": 0.15,
    "roof_collapsed": 0.55,
    "partial_collapse": 0.70,
    "leveled": 0.85,
    "swept": 0.30,
}

# Levels whose house is gone enough that survivors are standing ON it rather
# than beside it, and that a searcher would work.
_WRECKED = ("roof_collapsed", "partial_collapse", "leveled", "swept")
_FLATTENED = ("leveled", "swept")


def resolve_cfg(config):
    """`DEFAULTS` with the scene config's `people:` block merged over."""
    cfg = dict(DEFAULTS)
    user = (config or {}).get("people") or {}
    for k, v in user.items():
        if k == "scenarios" and isinstance(v, dict):
            sc = dict(cfg["scenarios"])
            for name, spec in v.items():
                sc[name] = dict(sc.get(name, {}), **(spec or {}))
            cfg["scenarios"] = sc
        else:
            cfg[k] = v
    return cfg


def _counts(cfg):
    """Head count per scenario, normalised, with absolute caps applied."""
    sc = cfg["scenarios"]
    tot = float(cfg.get("total", 90))
    raw = {k: float(sc.get(k, {}).get("share", 0.0)) for k in SCENARIOS}
    s = sum(raw.values()) or 1.0
    out = {}
    for k in SCENARIOS:
        n = int(round(tot * raw[k] / s))
        cap = sc.get(k, {}).get("max_count")
        if cap is not None:
            n = min(n, int(cap))
        out[k] = max(0, n)
    return out


# ---------------------------------------------------------------------------
# Placement helpers
# ---------------------------------------------------------------------------

class _Field(object):
    """Accumulates placements and enforces the no-interpenetration rule."""

    def __init__(self, cfg, ctx, rng):
        self.cfg = cfg
        self.ctx = ctx
        self.rng = rng
        self.sep = float(cfg.get("min_separation_m", 1.1))
        self.taken = []
        self.humans = []
        self.records = []
        self.debris = []

    def free(self, x, y):
        s2 = self.sep * self.sep
        for (px, py) in self.taken:
            if (px - x) ** 2 + (py - y) ** 2 < s2:
                return False
        return True

    def spot(self, cx, cy, r_lo, r_hi, tries=14):
        """A free point in an annulus about (cx, cy), or None."""
        for _ in range(tries):
            a = self.rng.uniform(0.0, 2.0 * math.pi)
            r = r_lo + (r_hi - r_lo) * math.sqrt(self.rng.random())
            x, y = cx + math.cos(a) * r, cy + math.sin(a) * r
            if self.free(x, y):
                return x, y
        return None

    def add(self, x, y, z_ground, yaw, pose, scenario, prone=False,
            note=None, sink_m=0.0):
        """Author one figure. Returns the placement dict, or None."""
        usd = self.pick_usd()
        if not usd:
            return None
        p = _human_placement(self.ctx, usd, x, y, z_ground, yaw, pose,
                             prone=prone)
        if p is None:
            return None
        if sink_m:
            # TRAPPED FIGURES ARE PUSHED DOWN INTO THE PILE, which is the
            # whole mechanism of `trapped_partial`. Applied after the pose
            # solve so the pose's own ground contact is still what is being
            # measured from.
            p["z_m"] = float(p["z_m"]) - float(sink_m)
        self.taken.append((x, y))
        self.humans.append(p)
        self.records.append({
            "x": round(float(x), 2), "y": round(float(y), 2),
            "z": round(float(p["z_m"]), 3),
            "scenario": scenario, "pose": pose,
            # EVERY FIGURE IN THIS MODULE IS ALIVE unless a caller turns
            # casualties on. The benchmark is about finding LIVE people, and
            # the epidemiology agrees: ~95% of the exposed are physically
            # uninjured and 89% of the injured are walking wounded.
            "alive": not prone or scenario != "thrown",
            "visibility": ("partial" if sink_m else "full"),
            "note": note,
        })
        return p

    def pick_usd(self):
        pool = list(self.ctx.get("humans") or ())
        return self.rng.choice(pool) if pool else None


def _human_placement(ctx, usd, x, y, z_ground, yaw, pose, prone=False):
    """Delegate to `disaster.people`, which owns the measured pose geometry.

    NOT REIMPLEMENTED HERE ON PURPOSE. That function carries per-character
    stature scaling, the ground-contact solve for seated and crouched poses,
    the RenderPeople yaw/roll corrections and the prone lift-by-half-body-
    depth rule — all of it derived by measurement against the rigs, and all of
    it equally true of a tornado survivor. Duplicating it would mean
    maintaining two copies of arithmetic that took a while to get right.
    """
    from . import people
    return people._human_placement(ctx, usd, x, y, z_ground, yaw, pose,
                                   prone=prone)


def _wreck_z(level):
    return float(DEBRIS_Z_M.get(str(level), 0.4))


def _bearing(ax, ay, bx, by):
    return math.degrees(math.atan2(by - ay, bx - ax))


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def _on_the_rubble(f, n):
    """Survivors standing and walking on the wreckage of their own house."""
    spec = f.cfg["scenarios"]["on_the_rubble"]
    wrecks = [w for w in f.ctx.get("wrecks") or ()
              if w.get("level") in _WRECKED]
    if not wrecks or n <= 0:
        return
    f.rng.shuffle(wrecks)
    lo, hi = spec.get("per_wreck", [1, 3])
    made = 0
    for w in wrecks:
        if made >= n:
            break
        z = _wreck_z(w["level"])
        for _ in range(f.rng.randint(int(lo), int(hi))):
            if made >= n:
                break
            # ON the footprint, not beside it — the debris IS the ground now.
            s = f.spot(w["x"], w["y"], 0.0, max(3.0, w["fp"] * 0.55))
            if s is None:
                continue
            pose = f.rng.choice(("idle", "walk", "walk"))
            if f.add(s[0], s[1], z, f.rng.uniform(0.0, 360.0), pose,
                     "on_the_rubble", note=w["level"]):
                made += 1


def _neighbour_dig(f, n):
    """3-6 people working at ONE collapsed house. The anchor scenario."""
    spec = f.cfg["scenarios"]["neighbour_dig"]
    wrecks = [w for w in f.ctx.get("wrecks") or ()
              if w.get("level") in _WRECKED]
    if not wrecks or n <= 0:
        return
    # PREFER A WRECK WITH A NEIGHBOUR LEFT STANDING. The diggers come from
    # somewhere, and a group working the one flattened house on an otherwise
    # intact street is both the commonest real case and the most legible one.
    intact = list(f.ctx.get("intact") or ())

    def nearest_intact(w):
        if not intact:
            return 1e9
        return min(math.hypot(w["x"] - p[0], w["y"] - p[1]) for p in intact)

    wrecks.sort(key=nearest_intact)
    lo, hi = spec.get("group_size", [3, 6])
    r_lo, r_hi = spec.get("cluster_r_m", [1.6, 4.5])
    made = 0
    for w in wrecks:
        if made >= n:
            break
        z = _wreck_z(w["level"])
        # The dig site: a point on the wreck, not its centre — people work an
        # edge, where the pile is shallow enough to move.
        a = f.rng.uniform(0.0, 2.0 * math.pi)
        rr = w["fp"] * f.rng.uniform(0.25, 0.45)
        cx, cy = w["x"] + math.cos(a) * rr, w["y"] + math.sin(a) * rr
        for _ in range(f.rng.randint(int(lo), int(hi))):
            if made >= n:
                break
            s = f.spot(cx, cy, r_lo * 0.4, r_hi)
            if s is None:
                continue
            pose = ("crouch" if f.rng.random() < spec.get("crouch_share", 0.45)
                    else "idle")
            # Everyone faces the dig point — a ring of figures all looking
            # inward is what makes a cluster read as purposeful rather than
            # as a crowd that happens to be standing together.
            yaw = _bearing(s[0], s[1], cx, cy)
            if f.add(s[0], s[1], z, yaw, pose, "neighbour_dig",
                     note="dig at %s" % w["level"]):
                made += 1


def _trapped_partial(f, n):
    """Partially buried figures: mostly under debris, head and torso showing.

    THE HARD ONE. Fully buried is an unlabellable target and fully exposed is
    not a trapped person, so the whole scenario lives or dies on getting a
    fraction right — which is why it emits its OWN debris rather than trusting
    whatever the wreck archetype happens to have left at that spot.

    Placed at the EDGE of the pile, on the argument that the middle of a
    collapsed house is where the material is deepest and a body there would be
    invisible from any angle. `sink_frac` pushes the figure down through the
    debris plane; `planks_over` lays boards across the lower body so the break
    between visible and hidden is geometry rather than luck.

    Over-represented relative to reality on purpose: Chiu et al. record
    "trapped in rubble" as the mechanism for 1.7% of fatalities and "crushed"
    for 18.7%, and Brown 2002 puts structural collapse behind 11-13% of
    injuries — so a true rate would put well under one of these in the scene.
    A dozen is a dataset decision, not a claim about the world, and it is
    recorded as such here so nobody later reads the share as an estimate.
    """
    spec = f.cfg["scenarios"]["trapped_partial"]
    wrecks = [w for w in f.ctx.get("wrecks") or ()
              if w.get("level") in _WRECKED]
    if not wrecks or n <= 0:
        return
    f.rng.shuffle(wrecks)
    s_lo, s_hi = spec.get("sink_frac", [0.30, 0.55])
    p_lo, p_hi = spec.get("planks_over", [3, 7])
    made = 0
    for w in wrecks:
        if made >= n:
            break
        z = _wreck_z(w["level"])
        # The pile EDGE: 60-95% of the way out from the footprint centre.
        a = f.rng.uniform(0.0, 2.0 * math.pi)
        rr = max(2.0, w["fp"] * f.rng.uniform(0.30, 0.48))
        x, y = w["x"] + math.cos(a) * rr, w["y"] + math.sin(a) * rr
        if not f.free(x, y):
            continue
        yaw = f.rng.uniform(0.0, 360.0)
        sink = z * f.rng.uniform(float(s_lo), float(s_hi))
        p = f.add(x, y, z, yaw, "idle", "trapped_partial", prone=True,
                  note="under %s debris" % w["level"], sink_m=sink)
        if p is None:
            continue
        made += 1
        # BOARDS ACROSS THE LOWER BODY. Authored along the body's own axis so
        # they lie across it rather than along it — a plank parallel to a
        # prone figure hides nothing and reads as a coincidence.
        br = math.radians(yaw)
        ux, uy = math.cos(br), math.sin(br)
        for k in range(f.rng.randint(int(p_lo), int(p_hi))):
            # From the hips (about +0.15 m along the body) to past the feet.
            t = f.rng.uniform(0.05, 1.05)
            jx = f.rng.uniform(-0.35, 0.35)
            f.debris.append({
                "x": x + ux * t - uy * jx,
                "y": y + uy * t + ux * jx,
                "z": z + f.rng.uniform(0.02, 0.16),
                "yaw": yaw + 90.0 + f.rng.gauss(0.0, 26.0),
                "len": f.rng.uniform(0.9, 2.4),
                "wide": f.rng.uniform(0.14, 0.55),
                "for": "trapped_partial",
            })


def _street(f, n):
    """Walking out along the road — the only navigable ground."""
    pts = list(f.ctx.get("road_pts") or ())
    if not pts or n <= 0:
        return
    f.rng.shuffle(pts)
    spec = f.cfg["scenarios"]["street"]
    lo, hi = spec.get("groups", [1, 2])
    made = 0
    for (px, py, tan) in pts:
        if made >= n:
            break
        for _ in range(f.rng.randint(int(lo), int(hi))):
            if made >= n:
                break
            s = f.spot(px, py, 0.0, 2.6)
            if s is None:
                continue
            # Walking ALONG the carriageway, either way — people are moving
            # out of the track and toward it in equal measure at this epoch.
            yaw = tan + (0.0 if f.rng.random() < 0.5 else 180.0)
            if f.add(s[0], s[1], 0.0, yaw + f.rng.gauss(0.0, 12.0), "walk",
                     "street"):
                made += 1


def _assisted(f, n):
    """Two people supporting a third, on the road, moving out."""
    pts = list(f.ctx.get("road_pts") or ())
    if not pts or n < 3:
        return
    f.rng.shuffle(pts)
    made = 0
    for (px, py, tan) in pts:
        if made + 3 > n:
            break
        s = f.spot(px, py, 0.0, 2.0)
        if s is None:
            continue
        yaw = tan + (0.0 if f.rng.random() < 0.5 else 180.0)
        ang = math.radians(yaw + 90.0)
        ox, oy = math.cos(ang), math.sin(ang)
        # Casualty in the middle, one either side, shoulder to shoulder. The
        # trio is tighter than `min_separation_m` allows by design — they are
        # touching — so it bypasses the spacing check and registers after.
        trio = ((0.0, "walk"), (-0.62, "walk"), (0.62, "walk"))
        n_ok = 0
        for (off, pose) in trio:
            hx, hy = s[0] + ox * off, s[1] + oy * off
            if f.add(hx, hy, 0.0, yaw, pose, "assisted",
                     note="walking wounded, assisted"):
                n_ok += 1
        made += n_ok


def _in_vehicle(f, n):
    """Occupants of displaced vehicles. UPRIGHT CARS ONLY.

    A car on its roof with somebody sitting in the driver's seat is not a
    scene anybody should train on: the pose rig seats a figure against a
    horizontal seat plane, so in a rolled car it comes out sideways in mid-
    air. Toppled cars in this scene are empty, which is also the likelier
    truth — being in a vehicle that rolls is how you stop being seated in it.
    """
    cars = [c for c in f.ctx.get("cars") or () if not c.get("toppled")]
    if not cars or n <= 0:
        return
    f.rng.shuffle(cars)
    lo, hi = f.cfg["scenarios"]["in_vehicle"].get("per_car", [1, 2])
    made = 0
    for c in cars:
        if made >= n:
            break
        for k in range(f.rng.randint(int(lo), int(hi))):
            if made >= n:
                break
            # Beside the car rather than in it: at this epoch the occupants
            # are out and standing by the door. `seated_car` needs a seat
            # plane the planner cannot see, and a standing figure at a car is
            # both easier to get right and more informative.
            a = f.rng.uniform(0.0, 2.0 * math.pi)
            s = f.spot(c["x"], c["y"], 1.6, 3.4)
            if s is None:
                continue
            del a
            if f.add(s[0], s[1], 0.0,
                     _bearing(s[0], s[1], c["x"], c["y"]),
                     "idle", "in_vehicle",
                     note="at a displaced vehicle"):
                made += 1


def _thrown(f, n):
    """Thrown clear of the structure. AT MOST ONE OR TWO IN THE WHOLE SCENE.

    THE ARITHMETIC, because the number looks too small to be deliberate:

      * CDC ran mortality surveillance on all 338 deaths in the April 2011
        outbreak and recorded location of INJURY and location of RECOVERY
        separately (MMWR 61(28)). 90.5% were injured indoors and 3.3% were
        outdoors when hit — but **37.0% of bodies were recovered outdoors**.
        So about a third of the DEAD end up off their own footprint.
      * Brown et al. 2002 (OKC 1999): "picked up / blown by tornado" was the
        mechanism for **43% of hospitalised** injuries but only **6% of
        treated-and-released** ones. Being thrown is a severity marker, not a
        common experience.

    Apply that to a corridor of ~39 damaged houses (~100 residents, Joplin
    path rates of 0.77% dead and 4.8% injured): ~0.8 dead x 37% recovered
    outdoors = 0.3 people, plus ~1.25 hospitalised x 43% thrown = 0.5 people.
    **Under one person in the whole 500 x 500 m scene.**

    Long-range lofting is record-book territory rather than a category at all
    — the documented survivals are 398 m (Matt Suter, an F2, GPS-measured,
    Guinness record) and 76 m (a mother and two children on a mattress at
    Dawson Springs). There is no published distribution of throw distances.
    `range_m` is therefore deliberately short: the modal case is "went out
    with the wall that failed and landed in the next lot".

    AND IT IS PRONE. Since being thrown is overwhelmingly a severe-injury or
    fatality outcome, a thrown figure does not stand up. Anyone walking in the
    open is somebody who emerged and walked there, which `street` and
    `on_the_rubble` already cover and which needs no throw model.
    """
    spec = f.cfg["scenarios"]["thrown"]
    wrecks = [w for w in f.ctx.get("wrecks") or ()
              if w.get("level") in _FLATTENED]
    if not wrecks or n <= 0:
        return
    f.rng.shuffle(wrecks)
    d_lo, d_hi = spec.get("range_m", [10.0, 40.0])
    th = math.radians(float(f.ctx.get("throw_deg", 0.0)))
    made = 0
    for w in wrecks:
        if made >= n:
            break
        # Downtrack, with a wide spread. NOT a tight fan: near-surface flow in
        # a tornado is CONVERGENT toward the centreline (Karstens et al.
        # digitised 10,300 tree falls at Joplin and 94,500 at Tuscaloosa and
        # found inward-pointing falls across most of the path), and the
        # 78%-left deposition statistic everybody quotes is for LIGHTWEIGHT
        # debris lofted into the parent storm, not for bodies. There is no
        # published azimuthal distribution for victim deposition, so a strong
        # directional bias here would be an invented claim.
        a = th + f.rng.gauss(0.0, math.radians(55.0))
        d = f.rng.uniform(float(d_lo), float(d_hi))
        x, y = w["x"] + math.cos(a) * d, w["y"] + math.sin(a) * d
        if not f.free(x, y):
            continue
        if f.add(x, y, 0.0, f.rng.uniform(0.0, 360.0), "idle", "thrown",
                 prone=True, note="thrown %.0f m from a %s house"
                 % (d, w["level"])):
            made += 1


_PLANNERS = {
    "on_the_rubble": _on_the_rubble,
    "neighbour_dig": _neighbour_dig,
    "trapped_partial": _trapped_partial,
    "street": _street,
    "assisted": _assisted,
    "in_vehicle": _in_vehicle,
    "thrown": _thrown,
}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def plan_people(cfg, ctx, rng):
    """Plan the whole population. Returns `(humans, debris, records)`.

    `ctx` keys, and who owns them:

        wrecks      [{x, y, fp, intensity, level}]  the assembly launcher
        intact      [(x, y)] standing houses         the assembly launcher
        road_pts    [(x, y, tangent_deg)]            the assembly launcher
        cars        [{x, y, toppled}]                the assembly launcher
        throw_deg   float                            `disaster.tornado`
        humans      [usd]      rigged RenderPeople   `suburb_scene.AssetPools`
        resolver, asset_pools                        `scene_generator`

    `debris` is the plank specs `trapped_partial` needs laid over its figures;
    the caller authors them with `disaster.planks`. They are returned rather
    than authored here because this module never touches a stage.
    """
    f = _Field(cfg, ctx, rng)
    want = _counts(cfg)
    # ORDER MATTERS. The scenarios that need a SPECIFIC spot run before the
    # ones that can go anywhere, so a trapped figure is never crowded out of
    # the pile edge by a bystander who could have stood two metres away.
    for name in ("trapped_partial", "neighbour_dig", "in_vehicle", "thrown",
                 "assisted", "on_the_rubble", "street"):
        _PLANNERS[name](f, want.get(name, 0))
    return f.humans, f.debris, f.records


def write_records(path, records, meta=None):
    """Ground truth to JSON, one entry per person."""
    import json

    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as fh:
        json.dump({"meta": meta or {}, "people": records}, fh, indent=1)
    return path


def summarise(records):
    """Counts per scenario and per visibility class."""
    by_s, by_v = {}, {}
    for r in records:
        by_s[r["scenario"]] = by_s.get(r["scenario"], 0) + 1
        by_v[r["visibility"]] = by_v.get(r["visibility"], 0) + 1
    return {"total": len(records), "by_scenario": by_s,
            "by_visibility": by_v,
            "alive": sum(1 for r in records if r.get("alive"))}


# ---------------------------------------------------------------------------
# WHAT IS DELIBERATELY ABSENT, and why
# ---------------------------------------------------------------------------
#
# `parking_refuge` — the LARGEST wildfire scenario at 0.30, and it has no
#   tornado counterpart. Moore, Joplin and Mayfield all had NO public tornado
#   shelters at the time of their tornadoes; Moore's emergency management says
#   so explicitly and explains why (10-15 minutes of warning is not enough to
#   leave, drive, park and get inside). Madison County AL — Huntsville, one of
#   the most tornado-conscious counties in the US — lists 14 safe rooms
#   totalling ~2,700 capacity, all in the rural fringe, and states that the
#   cities of Huntsville and Madison have none. Observed public-shelter use is
#   ~4% where one exists and 0% where none does.
#
# `gridlock` — a neighbourhood-scale traffic jam is not supported. Across six
#   metros over 2011-2018 only Oklahoma City ever produced a mass traffic
#   reversal (Hatzis et al. 2022), it was broadcast-directed, and it is a
#   metro-arterial phenomenon rather than a subdivision one. Moore 2013 had
#   ZERO vehicle deaths.
#
# `pools` / `cul_de_sac` — wildfire-specific refuge geography. A pool is
#   shelter from radiant heat; it is nothing at all in a wind event.
#
# Cellars, safe rooms and house interiors — real, and where people actually
#   shelter, but a drone benchmark cannot score a target it cannot see. This
#   is `disaster.people`'s `exposed_interior` lesson and it is not relitigated
#   here.
#
# Bystanders and onlookers at the track edge — real (40% of people go outside
#   to look during a warning, Sherman-Morris 2010) but out of scope: this
#   scene is about the people the tornado HIT.
#
# Responders, heavy equipment, search markings and triage tents — all T+12-24 h
#   artefacts. See `epoch_min`.
