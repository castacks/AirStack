"""people.py — where wildfire survivors actually are, and how to stand them there.

THE POINT OF THIS FILE IS THE PRIOR, NOT THE PLACEMENT. Scattering sixty
people uniformly over a burnt plat is easy and worthless: a search-and-rescue
model trained on it learns that a person is equally likely anywhere, which is
the one thing the literature says is false. People in a wildfire evacuation
end up in a small number of very specific places, and they are the same places
in every after-action report.

WHAT THE RESEARCH SAYS

  Camp Fire (Paradise, CA, 8 Nov 2018) — the reference event for this dataset.
  The "temporary refuge areas" (TRAs) people were actually directed to or found
  their own way to break down as **14 parking lots, 7 roadways, 6 structures,
  3 natural areas**. Parking lots dominate for a reason that generalises: a
  hectare of asphalt with nothing on it to burn, next to open ground, reachable
  by road. That ratio is why `parking_refuge` and `open_ground` carry half the
  population between them here.

  The roadway TRAs are the OTHER half of the same story. Skyway — Paradise's
  main egress — was counted at roughly **85 abandoned vehicles per mile
  (1.6 km)**: people left cars where they stopped and continued on foot, or
  sheltered in place in the lane. A gridlocked egress route with a blockage at
  its head, people in and beside the cars and more walking out past them, is
  the single most information-dense scene in the whole event, which is what
  `gridlock` builds.

  Lahaina (Maui, 8 Aug 2023) added the two variants that are hardest to see
  from the air and therefore most worth labelling: survivors **inside** cars,
  survivors **beside** cars in the road, and survivors at the seawall — i.e.
  against a large non-combustible feature. A suburban plat's version of the
  seawall is the swimming pool, and immersion survival in domestic pools is
  documented in Australian and Californian fires alike; `pools` stands people
  chest-deep IN the water — the one pool posture that still reads as a person
  from capture altitude.

  And people do not all leave. `at_home` puts people in front yards and
  driveways at the burn EDGE — the last-minute-defend / just-about-to-go case,
  which is where the front-yard fatalities and rescues both cluster — while
  `exposed_interior` puts one person on the floor plate of a house whose roof
  has fallen in, visible from above precisely because it has.

WHAT THIS MODULE IS

A PURE PLANNER. `plan_people` takes a context dict of things somebody else
already built (the street graph, the lots, the houses and their fire-derived
damage levels, the parked cars, the pools, the park, the burn-age field) and
returns four lists:

    car_placements    the evacuation queue and the refuge lot's cars, as
                      ordinary `category: "car"` placements. THESE GO IN
                      BEFORE the assembly's burnable/scorch pass so they char
                      like every other car on the plat.
    blocker_specs     what stopped the queue: a `tree_<species>_fallen`
                      archetype laid across the carriageway, or an existing
                      streetlight rotated over onto it.
    human_placements  the people, as `category: "human"` placements carrying a
                      `pose` for `scene_generator.apply_placements` to bind.
                      THESE GO IN AFTER the scorch pass — a survivor is not
                      scorched, and `"human"` deliberately matches nothing in
                      the assembly's BURNABLE list.
    records           ground truth, one dict per person, for `write_records`.

It touches no stage and imports no Isaac module, so the whole plan can be run,
asserted and drawn on the host with a stub resolver — which is how the
overlap, road and spacing invariants below are actually tested.

WHAT IT DOES NOT DECIDE. Casualties. `casualty_share` defaults to 0.0 and
every person is `alive: true`: this dataset is about finding LIVE people, and a
prone body reads as a different labelling problem. The knob exists (and reuses
`scene_generator`'s prone-casualty geometry — face-down, arms-down pose, lifted
by half the body depth) so the other dataset is one config line away.

POSTURE RULES (reviewed 2026-08-26, both on sight):

  * NOBODY WAVES, in any scene. The raised-arm pose read as a mannequin with
    a broken limb. It is gone from `scene_generator._HUMAN_POSES`, and
    `add_person` raises on it rather than letting a stale chooser through.
  * IN AN UNDAMAGED SCENE NOBODY SITS OR LIES ON THE GROUND. With nothing to
    flee, a person sitting on a lawn, crouching on a turnaround or face-down
    on a drive is a person the scene cannot explain. Seated is allowed only
    on a SEAT the caller supplies — a car seat (`in_vehicle`, occupants are
    fine) or a bench (`seat=`, parks and the urban kit have them). With
    `peacetime` set (`resolve_cfg(..., has_disaster=False)`) `add_person`
    coerces any other `sit_ground` / `sit_edge` / `crouch` / prone request to
    a stander and counts it, and `casualty_share` is ignored.
"""

import json
import math
import os
import random

_HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
# Shares are of `total` and are normalised, so editing one does not silently
# change the head count. Every default here is argued in the preset's `people:`
# block; this dict is what makes the module runnable without one.

DEFAULTS = {
    "total": 60,
    "seed": 91,
    # 0.0 = everybody alive. See the module docstring.
    "casualty_share": 0.0,
    # Nobody stands inside anybody else. 1.2 m is a shoulder-to-shoulder
    # crowd; below ~0.9 m two 0.5 m-wide figures interpenetrate.
    "min_separation_m": 1.2,
    "scenarios": {
        "parking_refuge": {"share": 0.30, "groups": [2, 3],
                           "in_car_share": 0.35,
                           # How many of the burn-ordered lots get people.
                           # One fills a single court and leaves the rest
                           # empty — see `_parking_refuge`.
                           "lots_used": 3,
                           "cars": [2, 5], "police": True,
                           "cluster_r_m": [2.0, 9.0]},
        "open_ground": {"share": 0.20, "groups": [2, 3], "group_size": [3, 8],
                        "clear_m": 15.0, "cluster_r_m": [2.0, 8.0],
                        # Radius the assembly DEACTIVATES trees inside, so the
                        # group stands in a glade rather than under a canopy
                        # that satisfies a trunk keep-out. See the launcher's
                        # step 4b-2.
                        "glade_r_m": 16.0},
        "pools": {"share": 0.10, "pools": [2, 3], "per_pool": [1, 3]},
        "gridlock": {"share": 0.25, "cars": [6, 12], "gap_m": 1.5,
                     "blocker_gap_m": [3.0, 6.0],
                     "walk_ahead_m": [20.0, 120.0], "rv_chance": 0.5,
                     # NOT EVERY BLOCKAGE IS STILL ALIGHT. A tree that came
                     # down across a road an hour ahead of the front is a
                     # blockage that never caught; one the front reached is
                     # still burning. Both are real and they look completely
                     # different from the air, so the launcher gets to choose
                     # per blocker rather than lighting all of them. 0.6 is a
                     # majority alight — the burning one is the more
                     # informative scene — with a cold blockage in most builds.
                     "fire_chance": 0.6},
        "at_home": {"share": 0.08, "houses": [3, 6]},
        # THE DEAD-END TRAP. A cul-de-sac has one way out and a 14.6 m
        # turnaround at the end of it: a household that reversed off its drive
        # and found the stem blocked has nowhere left to go, and Lahaina has
        # the case on record — a couple recovered in their car after turning
        # onto a dead-end street with the fire behind them. It is also the one
        # place a drone finds a tight cluster of cars and people that is NOT a
        # car park, so it is worth its own share rather than being folded into
        # `gridlock`.
        "cul_de_sac": {"share": 0.10, "cars": [2, 4], "bulbs": [3, 5]},
    },
}

# `exposed_interior` IS RETIRED, and the reason is worth keeping: it put one
# person on the floor plate of a roof-collapsed house, on the argument that a
# fallen roof leaves the floor open to the sky. In the built scene it does not
# — what is open is a hole between fallen roof planes, and from any altitude a
# drone flies the figure is behind a wall, a rafter or a chimney stack. A
# target that cannot be seen cannot be labelled, and an annotation for one that
# is invisible is worse than no annotation at all. Structure interiors are
# simply out of scope for this benchmark.
SCENARIOS = ("parking_refuge", "open_ground", "pools", "gridlock",
             "cul_de_sac", "at_home")

# See POSTURE RULES in the module docstring. `BANNED_POSES` are refused
# everywhere; `GROUND_POSES` are refused in peacetime unless the caller
# supplies the seat (`in_vehicle` or `seat`).
BANNED_POSES = ("wave",)
GROUND_POSES = ("sit_ground", "sit_edge", "crouch")

# House damage levels that read as "this house did not shelter anybody" — the
# ones a pool refugee is sheltering FROM, and the ones whose floor plate is
# open to the sky. `damage.level_for_age` names them.
_GUTTED = ("roof_collapsed", "burned_out", "rubble")
_OPEN_TO_SKY = ("roof_collapsed", "burned_out")

# The water plane, relative to the lot ground. `modular_house.POOL_WATER_Z_M`
# is the authority; duplicated as a fallback so a host-side plan does not need
# the kit module.
POOL_WATER_Z_M = -0.35
# Fraction of standing height at mid-chest. A person standing chest-deep has
# the surface here, so their feet go to `water_z - CHEST_FRAC * height`.
# There is no pool BASIN in this kit — the water is a plane over a hole cut in
# the lawn — so "standing in the pool" is authored by sinking the figure until
# the plane cuts it in the right place, and the submerged half is simply behind
# the (opaque) water. Nothing to collide with, nothing to fall through.
CHEST_FRAC = 0.75

# Bumper-to-bumper gap every car this module parks must leave to every car
# already on the plat. Not a tolerance — 0.0 would let two cars kiss, which
# renders as one welded lump — and not large, because a jammed queue and a
# cul-de-sac full of cars that gave up ARE tight. `build_cars` keeps the same
# order of magnitude at the kerb (it registers its kerb cars with an extra
# 0.5 m of half-length, i.e. 0.5 m nose to tail).
CAR_CLEAR_M = 0.25

# ---------------------------------------------------------------------------
# WHERE A PERSON SITS IN A CAR
# ---------------------------------------------------------------------------
# THESE ARE MEASURED VALUES, NOT PREFERENCES. Every other placement in this
# module is derived from geometry somebody can point at — a lot line, a road
# tangent, a bay centre, a pool ring. A seat pan is not. It is inside an asset
# nobody modelled for passengers, at a position no attribute records, and it
# moves per car: the same offset that seats a driver in a 1978 coupe puts the
# van's driver in the load bay. The only way to get the number is to put a
# figure in the car, look at it from three angles, and write it down — which
# is what `simulation/isaac-sim/launch_scripts/car_occupants_launch_script.py`
# exists for. Its `CARS` table is the authority and these rows are that table,
# transcribed; the offsets there were read off the stage as absolute prim
# translates and converted once.
#
# KEYED ON USD BASENAME, because that is the only stable identity a car has
# across the Nucleus path moves (`omniverse://.../Library/Stages/...` has
# already been restructured once) and across asset sets.
#
# Offsets are in the CAR'S OWN FRAME, measured from its placement anchor:
#   fwd   metres toward the nose (+) / toward the boot (-)
#   lat   metres toward the driver's side (+) / the passenger's side (-)
#   dz    metres added to the seat pan `_seat_pan_m` computes
#   dyaw  degrees relative to the car's heading. Zero on a correctly authored
#         car; a non-zero value RECORDS AN ASSET DEFECT rather than a taste —
#         the van's rear bench faces backwards, so its occupant is turned to
#         match the seat rather than the vehicle.
#   pose  an entry in `scene_generator._HUMAN_POSES`.
#
# TWO SEATS PER CAR where two were tuned, and both get used: a driver alone in
# every car in a stalled queue reads as a fleet of taxis. The second row is
# the passenger side except in the van, where it is the rear bench.
_CAR_SEATS = {
    "130.usdz": (
        # No steering wheel is modelled, so there is nothing to put a hand on
        # and both seats take the arms-down variant.
        {"fwd": -0.4446, "lat": +0.3599, "dz": -0.0439, "dyaw": 0.0,
         "pose": "seated_car_arms_down", "seat": "driver"},
        {"fwd": -0.4446, "lat": -0.4525, "dz": -0.0439, "dyaw": 0.0,
         "pose": "seated_car_arms_down", "seat": "passenger"},
    ),
    "Nissan_Fairlady_Z_S30240Z_1978.usdz": (
        # NOTE THE SIGNS: this car's driver is on the -lat side and its
        # passenger on +lat, the opposite of `130.usdz`. That is not a
        # right-hand-drive car, it is the asset's own art axis, and it is
        # exactly the kind of thing a generic formula gets wrong half the time.
        {"fwd": -0.4589, "lat": -0.4425, "dz": -0.0794, "dyaw": 0.0,
         "pose": "seated_car_arms_down", "seat": "driver"},
        {"fwd": -0.4589, "lat": +0.3813, "dz": -0.0794, "dyaw": 0.0,
         "pose": "seated_car_arms_down", "seat": "passenger"},
    ),
    "FREE_GMC_Motorhome_reimagined_low_poly.usdz": (
        {"fwd": +2.7250, "lat": +0.7700, "dz": +0.3382, "dyaw": 0.0,
         "pose": "seated_car_arms_down", "seat": "driver"},
        # -210 relative: the bench faces backwards (-120 absolute against the
        # +90 every other occupant carries). An asset defect, recorded.
        {"fwd": +2.7250, "lat": -0.6797, "dz": +0.3382, "dyaw": -210.0,
         "pose": "seated_car_arms_down", "seat": "rear passenger"},
    ),
}

# DROPPED, and worth recording so nobody tunes them again: the modular kit's
# `Car_01_0` and DownTown's `Vehicle_A`. Both are correctly sized and both are
# unusable for an occupant — their windows are painted into a single-mesh
# texture with no separable glass, so a person inside is invisible from every
# angle. Neither is tagged `glass_separable`, so neither ever reaches this
# table; they stay in the pool as parked cars, which is all they can do.


def _seat_pan_m(ht):
    """Height of the seat pan over the car's floor datum, metres.

    A real h-point is about a third of the roof height (0.5 m in a 1.5 m
    saloon, ~1.0 m in a van) and the head has to clear the roof — a seated
    figure's crown is ~0.95 m over the pan, so `ht - 1.00` is the hard
    ceiling. A flat 0.45 m cap put the motorhome's driver on the floor of its
    cab, which is the version this replaced.
    """
    return max(0.18, min(0.34 * float(ht), float(ht) - 1.00))


def _can_open(usd):
    """Whether an occupant in this car would be visible from outside.

    WAS `glass_separable`, AND THAT TAG IS THE WRONG QUESTION. It records only
    that some mesh binds a transparent material, which is how
    `Nissan_Fairlady_Z` came to lose it: its transparent materials are bound
    to the roof and body panels as well, so the generic strip turns the car
    into a convertible and the tag had to come off to stop that. The car is
    perfectly usable once the ONE offending mesh is named, which is what
    `vehicles.CABIN_RULES` does — so gating occupants on the tag left a car
    that is both tuned for passengers (`_CAR_SEATS` above) and forbidden from
    carrying any.
    """
    from detail import vehicles as _veh
    return _veh.can_open_cabin(usd)


def car_seats(usd, ln, wd, ht):
    """Seats for one car: the tuned rows if anybody measured it, else one.

    Returns a list of dicts carrying `fwd`, `lat`, `dz`, `dyaw`, `pose` and
    `source` — `"tuned"` for a row out of `_CAR_SEATS`, `"fallback"` for the
    derived one.

    THE FALLBACK EXISTS ONLY FOR CARS NOBODY HAS TUNED. It is the formula this
    module used for every car before the bench existed: a tenth of the length
    forward of the anchor and 22 % of the width to the left, which is a
    plausible driver's seat in a generic saloon and demonstrably wrong in the
    two cars that were actually measured (the Fairlady's driver is on the
    OTHER side; the van's is 2.7 m forward, not 0.8 m). It is a place to stand
    somebody so the scene is not empty, not a second source of truth — a car
    that turns up in a shot looking wrong should get a row in `_CAR_SEATS`
    rather than a change here.

    ONE SEAT in the fallback, not two: the offsets are a guess, and mirroring
    a guess produces two wrong people instead of one.
    """
    rows = _CAR_SEATS.get(os.path.basename(str(usd)))
    if rows:
        return [dict(r, source="tuned") for r in rows]
    return [{"fwd": float(ln) * 0.10, "lat": float(wd) * 0.22, "dz": 0.0,
             "dyaw": 0.0, "pose": "seated_car", "seat": "driver",
             "source": "fallback"}]


def _seat_person(plan, scenario, group, q, ln, wd, ht, seat, note=""):
    """Put one occupant on one seat of one placed car. Returns the record.

    THE PAN THE dz IS ADDED TO IS `_seat_pan_m`, this module's — not the
    bench's, which is `roof / 3.0`. The two are different functions and it is
    worth knowing by how much, because the tuned `dz` values were read against
    the bench's:

        roof 2.80 m (the van)  0.952 here vs 0.933 there  -> +0.019 m
        roof 1.45 m (the 130)  0.450 here vs 0.483 there  -> -0.033 m
        roof 1.30 m            0.300 here vs 0.433 there  -> -0.133 m

    i.e. they agree inside 2-3 cm over the whole range the tuned cars occupy,
    and diverge only on a car under ~1.5 m, where `_seat_pan_m` deliberately
    clamps to `roof - 1.00` to keep the head under the roof and `roof / 3`
    does not. That clamp is the reason to keep this pan rather than adopt the
    bench's. If a tuned car ever reads as sitting low, raise that row's `dz` —
    the number stays a measured one — rather than changing the pan, which is
    what keeps every other roof clear.
    """
    head = float(q.get("heading_deg", q["yaw_deg"]))
    hr = math.radians(head)
    fx, fy = math.cos(hr), math.sin(hr)
    lx, ly = -fy, fx                      # left of the direction of travel
    x = float(q["x_m"]) + fx * float(seat["fwd"]) + lx * float(seat["lat"])
    y = float(q["y_m"]) + fy * float(seat["fwd"]) + ly * float(seat["lat"])
    pan = _seat_pan_m(ht) + float(seat.get("dz", 0.0))
    rec = plan.add_person(
        scenario, group, x, y, pan, head + float(seat.get("dyaw", 0.0)),
        seat.get("pose", "seated_car"), in_vehicle=q["car_id"],
        note="%s seat, pan %.2f m, roof %.2f m, offsets %s%s"
             % (seat.get("seat", "driver"), pan, ht, seat["source"],
                ("; " + note) if note else ""))
    rec["seat_source"] = seat["source"]
    rec["seat"] = seat.get("seat", "driver")
    return rec


# ---------------------------------------------------------------------------
# The fallen-tree blocker
# ---------------------------------------------------------------------------
# MEASURED, host-side, off the baked archetypes in
# `scene_gen/assets/archetypes/` (usd-core, 2026-08-24). Per species: the
# world-XY bearing from the stump to the far tip of the `bole_*` meshes, and
# how far that reaches.
#
#   species            bearing   reach   tip z
#   American_Beech      132.6     3.4     0.42
#   Black_Oak           112.3     8.8     4.01
#   Common_Apple       -117.9     2.3     0.31
#   Douglas_Fir          50.0     5.0     0.73
#   Largetooth_Aspen     85.9     5.0     0.48
#   Shumard_Oak         113.6     4.4     0.76
#
# TWO FINDINGS WORTH MORE THAN THE TABLE.
#
# 1. THE `fallen` ARCHETYPE IS NOT A LONG TRUNK LYING DOWN. It is a stump, two
#    short bole segments and ~90 scattered logs, and its overall bbox is
#    16-19 m square — almost isotropic, because most of that footprint is
#    DEBRIS thrown radially (`vegetation._DEBRIS`, 7.5-10.5 m). So "take the
#    archetype's bbox long axis as the direction the bole lies" does not work:
#    the bbox is measuring the debris field, not the trunk. The bearing above
#    is taken from the BOLE MESHES ONLY, which is the only signal that means
#    anything.
# 2. `Black_Oak_fallen` IS NOT LYING DOWN AT ALL — its bole tip is 4.0 m in the
#    air, i.e. it went past balance and stopped leaning. Fine as a tree, wrong
#    as a thing lying across a road. `Largetooth_Aspen` and `Douglas_Fir` both
#    reach 5.0 m with their tips under 0.75 m, which is a bole ON THE GROUND
#    spanning half a carriageway, so those two are the blocker pool.
_BOLE_BEARING_DEG = {
    "American_Beech": 132.6,
    "Black_Oak": 112.3,
    "Common_Apple": -117.9,
    "Douglas_Fir": 50.0,
    "Largetooth_Aspen": 85.9,
    "Shumard_Oak": 113.6,
}
# Reach in metres, same measurement — how far across the road the bole gets.
_BOLE_REACH_M = {
    "American_Beech": 3.4, "Black_Oak": 8.8, "Common_Apple": 2.3,
    "Douglas_Fir": 5.0, "Largetooth_Aspen": 5.0, "Shumard_Oak": 4.4,
}
# Species whose fallen bole is genuinely horizontal AND long enough to cross a
# lane. See finding 2 above.
BLOCKER_SPECIES = ("Largetooth_Aspen", "Douglas_Fir", "Shumard_Oak")


def bole_bearing_deg(usd, species=None):
    """Bearing (deg) from the stump to the bole tip of a `*_fallen` archetype.

    Re-measured from the file when `pxr` can open it, so a re-bake with a
    different topple direction is picked up rather than silently ignored;
    falls back to `_BOLE_BEARING_DEG` (measured 2026-08-24) otherwise, which
    is the host-with-no-usd-core and the Nucleus-path cases.
    """
    if species is None and usd:
        base = os.path.splitext(os.path.basename(str(usd)))[0]
        parts = base.split("_")
        if len(parts) >= 3 and parts[0] == "tree":
            species = "_".join(parts[1:-1])
    try:
        from pxr import Gf, Usd, UsdGeom
        stage = Usd.Stage.Open(str(usd))
        if stage is None:
            raise RuntimeError("no stage")
        cache = UsdGeom.XformCache()
        best, far = None, -1.0
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh) or not prim.GetName().startswith("bole"):
                continue
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not pts:
                continue
            m = cache.GetLocalToWorldTransform(prim)
            for q in pts:
                w = m.Transform(Gf.Vec3d(q[0], q[1], q[2]))
                r = w[0] * w[0] + w[1] * w[1]
                if r > far:
                    far, best = r, (w[0], w[1])
        if best is not None and far > 1.0:
            return math.degrees(math.atan2(best[1], best[0]))
    except Exception:
        pass
    return float(_BOLE_BEARING_DEG.get(species or "", 90.0))


# ---------------------------------------------------------------------------
# small geometry
# ---------------------------------------------------------------------------

def _rot(x, y, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (x * c - y * s, x * s + y * c)


def _poly_bbox(poly):
    xs = [float(p[0]) for p in poly]
    ys = [float(p[1]) for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def _poly_area(poly):
    a = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        a += x0 * y1 - x1 * y0
    return abs(a) * 0.5


def _obb(cx, cy, ux, uy, hx, hy):
    return (float(cx), float(cy), float(ux), float(uy), float(hx), float(hy))


def _yaw_obb(cx, cy, yaw_deg, w, d):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return _obb(cx, cy, c, s, w * 0.5, d * 0.5)


class _Pts:
    """Cell hash of (point, clearance) — "is anything within r of here?".

    Same trick as `suburb_scene._Occupancy`, but each entry carries its OWN
    clearance so trees, fence panels and street furniture can share one index
    at different radii. Entries register in every cell their clearance reaches,
    so one lookup answers the query.
    """

    def __init__(self, cell=8.0):
        self.cell = float(cell)
        self.cells = {}

    def add(self, x, y, clear=0.0):
        g = max(0.0, float(clear))
        cx0 = int(math.floor((x - g) / self.cell))
        cx1 = int(math.floor((x + g) / self.cell))
        cy0 = int(math.floor((y - g) / self.cell))
        cy1 = int(math.floor((y + g) / self.cell))
        for gx in range(cx0, cx1 + 1):
            for gy in range(cy0, cy1 + 1):
                self.cells.setdefault((gx, gy), []).append((float(x), float(y), g))

    def hit(self, x, y, r=0.0):
        key = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
        for (px, py, g) in self.cells.get(key, ()):
            if (x - px) ** 2 + (y - py) ** 2 <= (g + r) ** 2:
                return True
        return False


# ---------------------------------------------------------------------------
# the ground: what a person may and may not stand on
# ---------------------------------------------------------------------------

class Ground:
    """Every keep-out a person has to clear, in one object.

    THE INDICES ARE BORROWED, NOT REBUILT. `suburb_scene` already answers
    "how far is this point from the nearest house wall" (`_ObbIndex`, exact
    point-to-oriented-box) and "is this point on a carriageway" (`_RoadIndex`,
    against the centrelines rather than the block polygon, because a block
    polygon can bulge over the road). Re-deriving either here is how two
    passes end up disagreeing about where the road is.

    WHAT IS NOT AUDITABLE HOST-SIDE. Fence panels are indexed as POINTS with a
    1.6 m clearance rather than as boxes: with `measure_usds` off a fence's
    footprint is the generic 4 x 4 m square fallback, which has no long axis,
    and the two fence passes compose their yaw differently — so any box drawn
    from a fence placement is a guess, and a guess 90 degrees out reports
    clashes that are not there. `suburb_scene.check_cars`'s scratch audit hit
    exactly this and says the same. A 1.6 m disc covers most of a 2.4 m panel
    and never lies.
    """

    # Per-category clearance for the point index, metres from the placement.
    _CLEAR = {
        "fence": 1.6, "tree": 2.0, "street_tree": 2.0, "yard_tree": 2.0,
        "plant": 0.8, "streetlight": 1.0, "sign": 0.8, "fire_hydrant": 0.9,
        "bench": 1.4, "trash_can": 0.8, "bus_stop": 2.2, "play_structure": 2.5,
        "planter": 1.0, "bike_rack": 1.2, "picnic_table": 1.6,
        "pool_edge": 0.0, "path": 0.0, "sidewalk": 0.0, "crosswalk": 0.0,
    }

    def __init__(self, ctx, sep_m=1.2):
        from suburb_scene import _ObbIndex, _RoadIndex, _rect_box
        from suburb_scene import _obb_corners
        from detail.suburb_parcel import _obb_overlap

        # BORROWED, like the indexes above and for the same reason.
        # `build_cars` decides whether a car fits with `_obb_overlap` — the
        # separating-axis test `suburb_parcel` used to place the houses — and
        # draws its box with `_obb_corners`. A second overlap test written
        # here would be a second opinion about what "two cars overlap" means,
        # and the two would drift.
        self._overlap = _obb_overlap
        self._corners = _obb_corners

        self.sep = float(sep_m)
        net = ctx.get("net")
        self.road = _RoadIndex(net) if net is not None else None

        houses = ctx.get("houses") or ()
        self.house = _ObbIndex(
            [_yaw_obb(h["x"], h["y"], h["yaw_box"], h["w"], h["d"])
             for h in houses], reach=40.0)

        # EVERY CAR ALREADY ON THE PLAT. `ctx["cars"]` is `build_cars`'s
        # whole output — driveway, row-home court and kerb — published by
        # `generate_suburb_on_stage` as `info_out["cars"]`. It is seeded here
        # twice over, because a parked car answers two different questions:
        # a person may not stand inside one (`solids`), and another car may
        # not be parked into one (`_cars_ring`, see `car_hit`).
        solids = []
        self._cars_ring = []
        for q in (ctx.get("cars") or ()):
            b = _car_obb(ctx, q)
            solids.append(b)
            self._cars_ring.append(self._car_ring(b))
        for p in (ctx.get("pools") or ()):
            b = _rect_box(p.get("water_ring"))
            if b is not None:
                solids.append(b)
        self._ObbIndex = _ObbIndex
        self._solid_boxes = solids
        self.solid = _ObbIndex(solids, reach=25.0) if solids else None

        self.props = _Pts(cell=8.0)
        for q in (ctx.get("placements") or ()):
            cat = str(q.get("category", ""))
            if cat in ("human", "car"):
                continue
            clear = None
            for key, val in self._CLEAR.items():
                if key in cat:
                    clear = val
                    break
            if clear is None:
                clear = 0.9 if cat else 0.0
            if clear > 0.0:
                self.props.add(float(q.get("x_m", 0.0)),
                               float(q.get("y_m", 0.0)), clear)
        for t in (ctx.get("trees") or ()):
            self.props.add(float(t["x"]), float(t["y"]), 2.0)

        # People placed by THIS pass, at `sep_m`. Separate from `props`
        # because it grows as the plan is built and because its radius is the
        # one invariant the audit checks by name.
        self.mine = _Pts(cell=max(2.0, self.sep * 2.0))

    # -- queries ----------------------------------------------------------
    def on_road(self, x, y, margin=0.0):
        return bool(self.road and self.road.on_road((x, y), margin))

    def house_dist(self, x, y):
        return self.house.nearest((x, y))

    def solid_dist(self, x, y):
        return self.solid.nearest((x, y)) if self.solid is not None else 99.0

    def free(self, x, y, road=False, house_m=1.0, solid_m=0.6, props=True,
             sep=True, road_margin=0.0):
        """Can a person stand at (x, y)?

        *road*, *house_m* and *solid_m* are per-scenario: a road walker is on
        the carriageway on purpose, an exposed-interior figure is inside the
        wall ring on purpose, and both would be rejected by the defaults.
        """
        if not road and self.on_road(x, y, road_margin):
            return False
        if house_m is not None and self.house_dist(x, y) < house_m:
            return False
        if solid_m is not None and self.solid_dist(x, y) < solid_m:
            return False
        if props and self.props.hit(x, y, 0.35):
            return False
        if sep and self.mine.hit(x, y, 0.0):
            return False
        return True

    def take(self, x, y):
        self.mine.add(x, y, self.sep)

    def add_solids(self, boxes):
        """Fold newly placed geometry into the solid index.

        REBUILT, NOT APPENDED. `_ObbIndex` registers each box in every cell it
        can reach at construction time, so there is no incremental add — and a
        car this pass parks is exactly as solid as one the plat parked.
        """
        self._solid_boxes.extend(boxes)
        self.solid = (self._ObbIndex(self._solid_boxes, reach=25.0)
                      if self._solid_boxes else None)

    # -- cars --------------------------------------------------------------
    # A CAR IS NOT JUST A SOLID. `solid` answers "how far is this POINT from
    # the nearest box", which is the right question for a person and the wrong
    # one for a car: two 4.6 x 1.9 m boxes can overlap with their centres 2.5 m
    # apart and every corner of each outside the other's centre distance. So
    # cars carry a second index that is tested box-against-box.
    #
    # NOT A HASH GRID, deliberately. `_CarKeepout` needs one because it holds
    # every house, garage and fence panel on the plat; this holds only cars —
    # 561 of them on a 1600 x 1200 m seed-11 plat — and each query rejects
    # almost all of them on one squared-distance compare against the
    # circumradius before any SAT test runs. Measured at well under a
    # millisecond a query host-side, against ~40 queries a build.

    def _car_ring(self, box):
        """`(corners, cx, cy, circumradius)` for the cheap-reject above."""
        cx, cy, ux, uy, hx, hy = box
        return (self._corners(cx, cy, ux, uy, hx, hy),
                float(cx), float(cy), math.hypot(hx, hy))

    def add_cars(self, boxes):
        """Register cars this pass parked: solid to people, AND to cars."""
        for b in boxes:
            self._cars_ring.append(self._car_ring(b))
        self.add_solids(boxes)

    def car_hit(self, box, pad=0.0):
        """Would a car with this box overlap one already parked anywhere?

        *pad* is passed straight to `_obb_overlap`, where it widens the
        separation the test demands — i.e. it is the CLEAR GAP required
        between the two bodies, not a floating-point tolerance.
        """
        ring, cx, cy, rad = self._car_ring(box)
        reach = rad + float(pad)
        for (other, px, py, prad) in self._cars_ring:
            if (cx - px) ** 2 + (cy - py) ** 2 > (reach + prad) ** 2:
                continue
            if self._overlap(ring, other, float(pad)):
                return True
        return False


def _car_obb(ctx, q):
    """A placed car as an oriented box, in its DIRECTION OF TRAVEL frame.

    `pools.place` composed ``yaw_deg = heading + yaw-offset``, and every car in
    the pool declares ``yaw-offset: 90`` because its art faces local -Y. So the
    measured footprint's long axis (sy) runs along the heading and sx across
    it — the same derivation `build_cars._dims` makes, done here off the
    placement rather than off the draw.
    """
    ap = ctx["asset_pools"]
    usd = q["usd"]
    fp = ctx["resolver"].get(usd, "car", scale=ap.scale_of(usd),
                             axis_up=ap.axis_of(usd))
    yo = math.radians(ap.yaw_of(usd))
    c, s = abs(math.cos(yo)), abs(math.sin(yo))
    ln = c * float(fp["sx"]) + s * float(fp["sy"])
    wd = s * float(fp["sx"]) + c * float(fp["sy"])
    head = math.radians(float(q["yaw_deg"]) - ap.yaw_of(usd))
    return _obb(float(q["x_m"]), float(q["y_m"]),
                math.cos(head), math.sin(head), ln * 0.5, wd * 0.5)


def car_dims(ctx, usd):
    """``(length_along_nose, width, height)`` for a car USD, metres."""
    ap = ctx["asset_pools"]
    fp = ctx["resolver"].get(usd, "car", scale=ap.scale_of(usd),
                             axis_up=ap.axis_of(usd))
    yo = math.radians(ap.yaw_of(usd))
    c, s = abs(math.cos(yo)), abs(math.sin(yo))
    return (c * float(fp["sx"]) + s * float(fp["sy"]),
            s * float(fp["sx"]) + c * float(fp["sy"]),
            float(fp.get("sz", 1.5)))


# ---------------------------------------------------------------------------
# houses: the table every scenario indexes into
# ---------------------------------------------------------------------------

def house_table(parcels, house_instances=None, levels=None):
    """One row per house, joining what the PLAT knows to what the FIRE knows.

    `info_out["house_instances"]` carries style / centre / yaw and nothing
    else, and `suburb_parcel` carries the box (`w`, `d`), the along-frontage
    unit `u` and — the one that cannot be recovered any other way — the INWARD
    normal `n`, which is what makes "the front yard" computable at all
    (`c - n * (d/2 + k)`; `perp(u)` is perpendicular by definition and cannot
    say which of the two normals faces the kerb).

    JOINED ON POSITION, not on index. The two lists are built by the same loop
    and are in the same order today, but only when every house draws from the
    modular kit (`modular_share: 1.0`); a mixed plat skips whole-house entries
    in one list and not the other, and an off-by-one here would put people in
    the wrong front yards silently. Centres are unique to the millimetre.

    *levels*, if given, is a per-`house_instances` damage level (what the
    assembly's step 3 computed from the burn age) and lands in `level`.
    """
    rows = []
    for p in (parcels or ()):
        for h in p.get("houses", ()):
            u = h.get("u") or (1.0, 0.0)
            n = h.get("n") or (0.0, 1.0)
            rows.append({
                "x": float(h["c"][0]), "y": float(h["c"][1]),
                "w": float(h["w"]), "d": float(h["d"]),
                "u": (float(u[0]), float(u[1])),
                "n": (float(n[0]), float(n[1])),
                # The box's own bearing: `u` runs along the frontage, which is
                # the house's WIDTH axis, so a `w x d` box at this yaw is the
                # footprint. Not the same number as the instance's `yaw`,
                # which is the art's facing (`yaw_deg + yaw_off + 90`).
                "yaw_box": math.degrees(math.atan2(u[1], u[0])),
                "lot_corners": h.get("lot_corners"),
                "style": None, "yaw": None, "index": None, "level": None,
            })
    if house_instances:
        by_pos = {}
        for row in rows:
            by_pos[(round(row["x"], 3), round(row["y"], 3))] = row
        for i, inst in enumerate(house_instances):
            row = by_pos.get((round(float(inst["x"]), 3),
                              round(float(inst["y"]), 3)))
            if row is None:
                continue
            row["style"] = inst.get("style")
            row["yaw"] = float(inst.get("yaw", 0.0))
            row["index"] = i
            if levels is not None and i < len(levels):
                row["level"] = levels[i]
    return rows


# ---------------------------------------------------------------------------
# placements
# ---------------------------------------------------------------------------

def _pose_dz(usd, pose, height_m):
    """Metres to drop a posed figure so its support meets the surface.

    Delegates to `scene_generator.pose_z_offset`, which solves a GROUND pose
    against that rig's own measured hip and falls back to the stature-scaled
    table for seat-placed poses. Scaling by stature alone cannot work here:
    hip height does not track total height across this pack, which is the
    error the note below was describing.
    """
    if not pose:
        return 0.0
    import scene_generator as sg
    return float(sg.pose_z_offset(usd, pose, height_m))


# THE MALE CHARACTERS SIT 0.15 M HIGH, and no amount of scaling fixes it.
# `_pose_dz` already scales the pose offset by the character's measured
# height, which handles the 1.73-1.86 m spread — but these are not the same
# body scaled up and down. The male RenderPeople rigs carry a longer torso
# above the hips than the female ones at the same overall stature, so a pose
# authored to put the PELVIS on a seat lands their hips proud of it and their
# heads into the roof. Measured on the occupant bench against the female
# characters in the same car; applied only to SEATED poses, because standing
# is anchored at the feet where the difference does not show.
_MALE_SEATED_DZ_M = -0.15
_MALE_RIGS = ("rp_eric", "rp_manuel", "rp_nathan", "rp_dennis")


# ONLY the poses this was measured for. The correction is about a pelvis
# placed on a SEAT the caller supplies; a ground pose is anchored the other way
# up, at the feet, against an absolute ground plane. The original predicate
# matched any pose starting with "sit"/"crouch", so it also fired on
# `sit_ground` and `crouch` and drove every male rig 0.15 m under the asphalt —
# pelvis below zero, heels buried. tools/pose_check.py is what caught it.
_SEAT_PLACED_POSES = ("seated_car", "seated_car_arms_down", "sit_edge")


def _seated_asset_dz(usd, pose):
    """Per-character correction for a SEAT-placed pose. Metres, usually 0."""
    if not pose or str(pose) not in _SEAT_PLACED_POSES:
        return 0.0
    base = os.path.basename(str(usd)).lower()
    return _MALE_SEATED_DZ_M if base.startswith(_MALE_RIGS) else 0.0


def _human_placement(ctx, usd, x, y, z_ground, yaw, pose, prone=False):
    """One `category: "human"` placement, with every asset correction applied.

    *z_ground* is the height of the surface the pose's SUPPORT lands on: the
    lawn for a stander, the coping for `sit_edge`, the seat pan for
    `seated_car`, the water plane minus chest depth for a pool stander.
    """
    ap = ctx["asset_pools"]
    sc = ap.scale_of(usd)
    au = ap.axis_of(usd)
    fp = ctx["resolver"].get(usd, "human", scale=sc, axis_up=au)
    height = float(fp.get("sz", 1.8)) or 1.8
    if prone:
        # `scene_generator._human_down`: face-down/up, i.e. +-90 of roll about
        # the body's facing axis on the PRE-YAW frame, lifted by half the body
        # DEPTH (sy in an arms-down pose, not the T-pose arm span).
        return {
            "usd": usd, "x_m": float(x), "y_m": float(y),
            "z_m": float(z_ground) + float(fp.get("sy", 0.35)) / 2.0,
            "yaw_deg": float(yaw) + ap.yaw_of(usd),
            "roll_deg": ap.roll_of(usd) + (90.0 if (x + y) >= 0 else -90.0),
            "pitch_deg": 0.0, "scale": sc, "category": "human", "axis_up": au,
            "pose": "idle",
        }
    return {
        "usd": usd, "x_m": float(x), "y_m": float(y),
        "z_m": float(fp.get("base", 0.0)) + float(z_ground)
               + _pose_dz(usd, pose, height) + _seated_asset_dz(usd, pose),
        "yaw_deg": float(yaw) + ap.yaw_of(usd),
        "roll_deg": ap.roll_of(usd), "pitch_deg": 0.0,
        "scale": sc, "category": "human", "axis_up": au,
        "pose": pose,
    }


def _car_placement(ctx, usd, x, y, heading_deg, role):
    ap = ctx["asset_pools"]
    sc = ap.scale_of(usd)
    au = ap.axis_of(usd)
    fp = ctx["resolver"].get(usd, "car", scale=sc, axis_up=au)
    return {
        "usd": usd, "x_m": float(x), "y_m": float(y),
        "z_m": float(fp.get("base", 0.0)),
        "yaw_deg": float(heading_deg) + ap.yaw_of(usd),
        "roll_deg": ap.roll_of(usd), "pitch_deg": 0.0,
        "scale": sc, "category": "car", "axis_up": au,
        # THE HEADING, KEPT. `yaw_deg` is the ART's rotation — heading plus
        # the pool's `yaw-offset`, which is 90 for every car in this set — and
        # anything that wants the direction the car is POINTING has to undo
        # that offset first. `_cul_de_sac` did not, and put its occupants
        # 90 degrees across their own seats; recording the heading is cheaper
        # than making every reader remember the correction.
        "heading_deg": float(heading_deg),
        "role": role, "glass_separable": usd in (ctx.get("glassy") or ()),
    }


# ---------------------------------------------------------------------------
# the planner
# ---------------------------------------------------------------------------

# WHAT PEOPLE ARE DOING WHEN NOTHING HAS HAPPENED.
#
# `DEFAULTS["scenarios"]` is a WILDFIRE EVACUATION mix: parking_refuge 0.30,
# gridlock 0.25, open_ground 0.20, pools 0.10 — 85% of the population posed as
# evacuees, sheltering in a refuge lot, jammed in a queue, or standing in a
# swimming pool. Correct for a fire; absurd for an untouched suburb, where it
# put an evacuation queue and a refuge lot into a scene with nothing to flee.
#
# In peacetime people are at home, in their yards, and on the street they live
# on. `at_home` and `cul_de_sac` are the two postures that mean that; the
# ambient sidewalk walkers `scene_generator` scatters are the rest.
PEACETIME_SCENARIOS = {
    "at_home":        {"share": 0.70},
    "cul_de_sac":     {"share": 0.30},
    "parking_refuge": {"share": 0.0},
    "gridlock":       {"share": 0.0},
    "open_ground":    {"share": 0.0},
    "pools":          {"share": 0.0},
}


def resolve_cfg(config, has_disaster=True):
    """The `people:` block merged over `DEFAULTS`, one nesting level deep.

    With *has_disaster* False the evacuation shares are replaced by
    `PEACETIME_SCENARIOS` BEFORE the `people:` block is applied, so a preset
    can still override either way and an explicit `people.scenarios` entry
    always wins.
    """
    out = dict(DEFAULTS)
    out["scenarios"] = {k: dict(v) for k, v in DEFAULTS["scenarios"].items()}
    if not has_disaster:
        for name, sub in PEACETIME_SCENARIOS.items():
            out["scenarios"].setdefault(name, {}).update(sub)
    block = (config or {}).get("people") or {}
    for key, val in block.items():
        if key == "scenarios":
            for name, sub in (val or {}).items():
                out["scenarios"].setdefault(name, {}).update(sub or {})
        else:
            out[key] = val
    # NOT overridable from the preset: whether the scene is damaged is a fact
    # about the scene, and the posture rules hang off it (docstring).
    out["peacetime"] = not has_disaster
    return out


def _quota(cfg):
    """How many people each scenario gets, summing exactly to `total`.

    Largest-remainder, so the shares are honoured and the total is not
    approximately right.
    """
    total = int(cfg.get("total", 0))
    shares = {k: float((cfg["scenarios"].get(k) or {}).get("share", 0.0))
              for k in SCENARIOS}
    ssum = sum(shares.values())
    if total <= 0 or ssum <= 0:
        return {k: 0 for k in SCENARIOS}
    raw = {k: total * v / ssum for k, v in shares.items()}
    out = {k: int(math.floor(v)) for k, v in raw.items()}
    rem = total - sum(out.values())
    for k in sorted(SCENARIOS, key=lambda n: raw[n] - math.floor(raw[n]),
                    reverse=True)[:max(0, rem)]:
        out[k] += 1
    return out


class _Plan:
    """Mutable state a scenario writes into. Keeps `plan_people` readable."""

    def __init__(self, ctx, cfg, rng):
        self.ctx = ctx
        self.cfg = cfg
        self.rng = rng
        self.ground = Ground(ctx, sep_m=float(cfg.get("min_separation_m", 1.2)))
        self.cars = []
        self.blockers = []
        self.humans = []
        self.records = []
        self.tally = {k: 0 for k in SCENARIOS}
        # Refused car slots, keyed `role:reason` — the same shape and the same
        # reason as `build_cars`'s `why` dict: "car 7" says nothing on its own,
        # and the three passes fail in different places for different reasons.
        self.car_why = {}
        self.notes = []
        # POSTURE RULES (module docstring). `coerced` counts every request a
        # scenario made that the rules turned into a stander, by pose.
        self.peacetime = bool(cfg.get("peacetime", False))
        self.coerced = {}

    # -- assets ----------------------------------------------------------
    def pick_human(self, pose, prone=False):
        """A character for this pose.

        The three POSED statics have no skeleton, so they can only ever stand —
        but where a stander is what is wanted they are worth reaching for,
        because six rigged characters spread over sixty people is a crowd of
        sextuplets. `_bind_human_pose` caches None for them either way, so
        handing one a pose would not break; it would just quietly do nothing,
        which is worse than not asking.
        """
        posed = self.ctx.get("humans_posed") or ()
        if prone:
            # A rigid standing static laid on its side is a shop dummy, not a
            # body: the prone geometry assumes the ARMS-DOWN pose (`sy` is the
            # body depth it lifts by), which only a rigged character can take.
            return self.rng.choice(list(self.ctx["humans"])), "idle"
        if pose in (None, "idle") and posed and self.rng.random() < 0.35:
            return self.rng.choice(list(posed)), None
        return self.rng.choice(list(self.ctx["humans"])), pose

    # -- people ----------------------------------------------------------
    def add_person(self, scenario, group, x, y, z_ground, yaw, pose,
                   in_vehicle=None, prone=False, note=None, usd=None,
                   seat=None):
        # POSTURE RULES, enforced at the one door every person comes through.
        # *seat* names the thing a `sit_edge` is on when it is not a car
        # ("bench", "wall"); a scenario that puts somebody on a seat it can
        # actually see passes it, and nothing else does.
        if pose in BANNED_POSES:
            raise ValueError("pose %r is banned in every scene (people."
                             "BANNED_POSES); a chooser still offers it" % pose)
        if (self.peacetime and in_vehicle is None and seat is None
                and (prone or pose in GROUND_POSES)):
            why = "prone" if prone else str(pose)
            self.coerced[why] = self.coerced.get(why, 0) + 1
            prone, pose = False, "idle"
            # The seat height a sitter was handed is not a stander's ground.
            z_ground = 0.0 if str(why) == "sit_edge" else z_ground
        # *usd* pins the character. Only the pool stander needs it, and it
        # needs it for a real reason: how deep to sink somebody so the water
        # cuts them at mid-chest depends on THAT character's height, so
        # measuring one and placing another is a 13 cm error over a pack that
        # runs 1.73-1.86 m.
        if usd is None:
            usd, pose = self.pick_human(pose, prone=prone)
        elif prone:
            pose = "idle"
        q = _human_placement(self.ctx, usd, x, y, z_ground, yaw, pose,
                             prone=prone)
        idx = len(self.humans)
        pid = "p%04d" % idx
        q["people_id"] = pid
        q["people_scenario"] = scenario
        near = self._nearest_house(x, y)
        rec = {
            "id": pid, "scenario": scenario, "group": int(group),
            "usd": usd, "x": round(float(x), 3), "y": round(float(y), 3),
            "z": round(float(q["z_m"]), 3),
            "yaw": round(float(q["yaw_deg"]), 2),
            "pose": pose, "alive": True, "in_vehicle": in_vehicle,
            "nearest_house": near,
            "burn_age_s": round(float(self.ctx["age"](x, y)), 1),
        }
        if note:
            rec["note"] = note
        self.humans.append(q)
        self.records.append(rec)
        self.ground.take(x, y)
        self.tally[scenario] += 1
        return rec

    def _nearest_house(self, x, y):
        best, bi = 1e9, None
        for h in (self.ctx.get("houses") or ()):
            d = (h["x"] - x) ** 2 + (h["y"] - y) ** 2
            if d < best:
                best, bi = d, h.get("index")
        return bi if best <= 120.0 ** 2 else None

    # -- cars ------------------------------------------------------------
    def add_car(self, usd, x, y, heading, role):
        """Park a car here — or refuse, and say why. `None` means refused.

        THE ONLY DOOR, and that is the whole fix. Six passes put cars on this
        plat: `build_cars`'s driveway, row-home-court and kerb ranks, and this
        module's refuge lot, evacuation queue and cul-de-sac. The first three
        already agree with each other, because they share one `_CarKeepout`
        and each folds its own car back into it. The other three did not: the
        refuge lot tested nothing, the queue registered its cars only after
        the whole queue was drawn (so a queue could not see itself), and the
        cul-de-sac marked a POINT — `ground.take` — for a 4.6 m car, which is
        why two to four of them on a 14.6 m turnaround visibly interpenetrated.

        Now every car in this module comes through here, is tested box against
        box against every car already anywhere on the plat (`Ground.car_hit`,
        seeded from `ctx["cars"]`), and is folded into that index the moment
        it is kept. A caller that gets `None` must re-site or give up; it must
        not place anyway.
        """
        q = _car_placement(self.ctx, usd, x, y, heading, role)
        box = _car_obb(self.ctx, q)
        if self.ground.car_hit(box, CAR_CLEAR_M):
            key = str(role) + ":car"
            self.car_why[key] = self.car_why.get(key, 0) + 1
            return None
        q["car_id"] = "people_car_%03d" % len(self.cars)
        self.cars.append(q)
        self.ground.add_cars([box])
        return q


def plan_people(cfg, ctx, rng=None):
    """Plan the whole population. See the module docstring for the contract.

    *cfg* is the resolved `people:` block (`resolve_cfg`), *ctx* the context
    dict described in `build_ctx`. Returns
    ``(car_placements, blocker_specs, human_placements, records)``.
    """
    cfg = dict(cfg or {})
    cfg.setdefault("scenarios", DEFAULTS["scenarios"])
    if rng is None:
        rng = random.Random(int(cfg.get("seed", 91)))
    plan = _Plan(ctx, cfg, rng)
    want = _quota(cfg)
    if int(cfg.get("total", 0)) <= 0:
        print("[people] total 0 — no survivors planned")
        return [], [], [], []

    _parking_refuge(plan, want["parking_refuge"])
    _open_ground(plan, want["open_ground"])
    _pools(plan, want["pools"])
    _gridlock(plan, want["gridlock"])
    _cul_de_sac(plan, want["cul_de_sac"])
    _at_home(plan, want["at_home"])

    _apply_casualties(plan)

    if plan.coerced:
        plan.notes.append(
            "peacetime posture rules turned %s request(s) into standers: %s"
            % (sum(plan.coerced.values()), dict(sorted(plan.coerced.items()))))

    print("[people] planned {0} of {1} requested; {2} car(s), {3} blocker(s)"
          .format(len(plan.humans), int(cfg.get("total", 0)),
                  len(plan.cars), len(plan.blockers)))
    # THE SAME LINE `build_cars` PRINTS, for the same reason: a refused car
    # slot is information, and "lot:car 3" says which pass ran out of room and
    # what it ran into. Every car this module keeps has been tested against
    # every car anywhere on the plat, so a zero here is the invariant holding
    # rather than the test not running.
    print("[people] cars: {0} parked ({1}); refused {2}".format(
        len(plan.cars),
        dict(sorted((r, sum(1 for q in plan.cars if q.get("role") == r))
                    for r in {q.get("role") for q in plan.cars})),
        dict(sorted(plan.car_why.items())) or "none"))
    _n_fire = sum(1 for b in plan.blockers if b.get("fire"))
    print("[people] blockages: {0} of {1} burning (fire_chance {2:.2f})"
          .format(_n_fire, len(plan.blockers),
                  float(((cfg.get("scenarios") or {}).get("gridlock") or {})
                        .get("fire_chance", 0.6))))
    for name in SCENARIOS:
        print("[people]   {0:<18} {1:>3} / {2:<3}".format(
            name, plan.tally[name], want[name]))
    for note in plan.notes:
        print("[people]   ! " + note)
    return plan.cars, plan.blockers, plan.humans, plan.records


def _apply_casualties(plan):
    """Turn `casualty_share` of the plan face-down. Default 0.0 — see the
    module docstring for why alive is the default rather than a setting."""
    share = float(plan.cfg.get("casualty_share", 0.0) or 0.0)
    if share <= 0.0 or not plan.humans:
        return
    if plan.peacetime:
        # A face-down body in an undamaged suburb is a body the scene cannot
        # explain — POSTURE RULES, module docstring.
        plan.notes.append("casualty_share %.2f ignored: peacetime scene, "
                          "nobody lies on the ground" % share)
        return
    pool = [i for i, r in enumerate(plan.records)
            if r["in_vehicle"] is None and r["scenario"] != "pools"]
    k = min(len(pool), int(round(share * len(plan.humans))))
    for i in plan.rng.sample(pool, k):
        rec = plan.records[i]
        q = plan.humans[i]
        ap = plan.ctx["asset_pools"]
        fp = plan.ctx["resolver"].get(q["usd"], "human",
                                      scale=q["scale"], axis_up=q["axis_up"])
        q["pose"] = "idle"
        q["roll_deg"] = ap.roll_of(q["usd"]) + plan.rng.choice([-1.0, 1.0]) * \
            plan.rng.uniform(82.0, 98.0)
        q["yaw_deg"] = plan.rng.uniform(0.0, 360.0)
        q["z_m"] = float(fp.get("sy", 0.35)) / 2.0
        rec["alive"] = False
        rec["pose"] = "idle"
        rec["z"] = round(q["z_m"], 3)
    print("[people] casualty_share {0:.2f}: {1} of {2} face-down"
          .format(share, k, len(plan.humans)))


# ---------------------------------------------------------------------------
# 1) refuge — the park's parking lot
# ---------------------------------------------------------------------------

def _parking_refuge(plan, want):
    """A group on the asphalt, clustered around a handful of parked cars.

    THE MOST COMMON TRA IN THE CAMP FIRE BY A FACTOR OF TWO (14 lots against
    7 roadways, 6 structures, 3 natural areas). The lot is where people are
    sent because it is a hectare of non-combustible ground next to open
    ground with a road to it; `suburb_park` sites one in the park for exactly
    this pass and publishes its bays in world coordinates.
    """
    if want <= 0:
        return
    # THE PARK LOT IS NOT THE ONLY LOT ANY MORE. Row-home developments carry
    # a shared parking court, published under `info_out["clusters"]` in the
    # IDENTICAL schema `suburb_park.parking_info` uses — which was the point
    # of matching that schema. A court is the same thing as the park's lot for
    # this scenario's purposes (bare asphalt, nothing on it to burn, a road to
    # it) and it is better placed: it sits inside the fabric, so the people
    # sheltering on it are the people whose houses are burning around them.
    # Preferring the one deepest in the burn is what puts the group where the
    # fire actually is rather than on whichever lot the park happened to get.
    lots, kinds = [], []
    _park_lot = ((plan.ctx.get("park") or {}).get("parking"))
    if _park_lot and _park_lot.get("bays"):
        lots.append(_park_lot)
        kinds.append("park")
    for _c in (plan.ctx.get("clusters") or ()):
        _pk = (_c or {}).get("parking")
        if _pk and _pk.get("bays"):
            lots.append(_pk)
            kinds.append("court")
    if not lots:
        plan.notes.append(
            "parking_refuge SKIPPED: no refuge lot published — neither "
            "info['park']['parking'] nor any row-home court has bays")
        return
    _agef = plan.ctx["age"]
    lots.sort(key=lambda L: -_agef(L["centre"][0], L["centre"][1]))
    # SPREAD ACROSS LOTS, DO NOT FILL ONE. Taking only the deepest-burnt lot
    # put all seventeen refugees on a single court and left the park's 72-bay
    # lot and every other court empty — which reads as one implausible crowd
    # rather than as a fabric where people gather on whatever bare asphalt is
    # near them. It also wastes the row-home courts the moment there is more
    # than one: they are the interesting refuge, because a court sits INSIDE
    # the burning fabric while the park lot sits in open ground.
    #
    # Still burn-ordered, so the lots that get people are the ones with fire
    # around them; `lots_used` just says how many of that ordering to take.
    n_lot = max(1, int((plan.cfg["scenarios"].get("parking_refuge") or {})
                       .get("lots_used", 3)))
    _order = sorted(range(len(lots)),
                    key=lambda i: -_agef(lots[i]["centre"][0],
                                         lots[i]["centre"][1]))
    picked = _order[:min(n_lot, len(_order))]
    # AT LEAST ONE ROW-HOME COURT, when the plat has any. Burn depth alone
    # kept choosing the park's 72-bay lot and whichever courts happened to sit
    # deepest, and on a plat where the park is well inside the fire that can
    # be every slot — so the developments whose whole morphology is "one
    # shared court and one way out" ended up with nobody on them. A court is
    # also the better refuge to photograph: it sits inside the burning fabric
    # rather than in open parkland.
    if "court" in kinds and not any(kinds[i] == "court" for i in picked):
        court_i = next(i for i in _order if kinds[i] == "court")
        picked[-1] = court_i
    chosen_lots = [lots[i] for i in picked]
    plan.notes.append(
        "parking_refuge: %d lot(s) available (park + row-home courts), "
        "using the %d deepest in the burn" % (len(lots), len(chosen_lots)))
    shares = _split(want, len(chosen_lots), 1, max(1, want), plan.rng)
    for _li, (lot, _wl) in enumerate(zip(chosen_lots, shares)):
        if _wl > 0:
            _one_refuge_lot(plan, lot, _wl, _li)


def _one_refuge_lot(plan, lot, want, li):
    """Cars and a crowd on ONE lot — the park's, or a row-home court."""
    scfg = plan.cfg["scenarios"].get("parking_refuge") or {}
    rng = plan.rng
    cx, cy = lot["centre"]
    age_here = plan.ctx["age"](cx, cy)
    plan.notes.append(
        "parking_refuge lot at (%.0f, %.0f), %d bays, burn age %.0f s (%s)"
        % (cx, cy, len(lot["bays"]), age_here,
           "in the burn" if age_here > 0 else "unburnt"))

    # -- cars in bays ----------------------------------------------------
    bays = list(lot["bays"])
    bays.sort(key=lambda b: (b["centre"][0] - cx) ** 2 + (b["centre"][1] - cy) ** 2)
    n_car = min(len(bays), rng.randint(*[int(v) for v in scfg.get("cars", [2, 5])]))
    # Drawn from the NEAR half so the cars cluster rather than speckling a
    # 72-bay lot, which is what makes them read as "people parked together".
    near = bays[:max(n_car, len(bays) // 2)]
    chosen = rng.sample(near, n_car)
    # A ROW-HOME COURT IS NOT AN EMPTY LOT. When the lot picked above is a
    # cluster court rather than the park's, `build_cars` has already filled
    # roughly `cars.court_occupancy` (0.6) of its bays — so more than half the
    # bays this pass would like are taken, and parking a refugee's car into
    # one is the overlap the user is seeing. `add_car` refuses those; the
    # shortfall is then made up from the REST of the near bays, in distance
    # order, so the group still gets the cars it was promised.
    # THE CARS ARE ALREADY HERE — ADOPT THEM, DO NOT PARK MORE.
    #
    # `build_cars` fills roughly `cars.court_occupancy` of every row-home
    # court and a share of the park's lot before this pass ever runs, so the
    # bays this scenario wanted were mostly taken and `add_car` kept refusing
    # them: "wanted 3 car(s), parked 1". Adding cars was the wrong instinct
    # anyway. A refuge lot inside the burn is not a lot somebody has just
    # driven to — it is the lot that was already there, whose parked cars
    # became refuge cars the moment the front arrived. Adopting them costs no
    # geometry, cannot overlap anything (they are already placed and already
    # mutually clear), and gives the scenario far MORE cars to work with than
    # it could ever have parked: a 72-bay lot comes with dozens.
    #
    # Only a lot that the fire has actually reached qualifies. An unburnt lot
    # full of ordinary parked cars is just a car park, and calling its cars
    # refuge cars would put people in vehicles nobody fled to.
    anchors = []
    if age_here > 0.0:
        corners = lot["corners"]
        for j, q in enumerate(plan.ctx.get("cars") or ()):
            qx, qy = float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0))
            if not _in_poly(corners, (qx, qy)):
                continue
            # The plat's cars carry no `car_id` — the ground truth needs a
            # stable one to point `in_vehicle` at.
            q = dict(q)
            q.setdefault("car_id", "plat_car_%04d" % j)
            anchors.append((qx, qy, q))
        anchors.sort(key=lambda a: (a[0] - cx) ** 2 + (a[1] - cy) ** 2)
        plan.notes.append("parking_refuge: adopted %d car(s) already parked "
                          "in this lot" % len(anchors))

    # Only if the lot came up nearly empty — an unburnt lot, or a court whose
    # bays `build_cars` happened to skip — is a car added at all.
    res = list(plan.ctx["car_pool"].get("residential") or ())
    if len(anchors) < 2:
        _taken = {id(b) for b in chosen}
        for b in list(chosen) + [b for b in near if id(b) not in _taken]:
            if len(anchors) >= n_car:
                break
            usd = rng.choice(res) if res else None
            if usd is None:
                break
            q = plan.add_car(usd, b["centre"][0], b["centre"][1],
                             b["yaw_deg"], "lot")
            if q is None:
                continue
            anchors.append((b["centre"][0], b["centre"][1], q))
    # ONE police car, at the mouth of the apron rather than in a bay: a squad
    # car at a refuge is parked across the way in, not tidied into a space.
    livery = list(plan.ctx["car_pool"].get("livery") or ())
    if scfg.get("police", True) and livery and lot.get("mouth"):
        mx, my = lot["mouth"]
        ex, ey = lot.get("entrance", lot["mouth"])
        head = math.degrees(math.atan2(my - ey, mx - ex))
        usd = next((u for u in livery if "Police" in os.path.basename(u)),
                   livery[0])
        # SLID IN ALONG THE APRON, not dropped on one spot. The mouth of a
        # court is exactly where `build_cars` puts its first bay, so a fixed
        # 12 % step off the mouth lands on a car about as often as not; the
        # squad car walks further in until it finds room, which is also what a
        # squad car does.
        for t in (0.12, 0.20, 0.28, 0.36, 0.46):
            if plan.add_car(usd, mx + (cx - mx) * t, my + (cy - my) * t,
                            head, "lot_police") is not None:
                break
        else:
            plan.notes.append("parking_refuge: no room for the squad car "
                              "along the apron — every step is occupied")

    # -- people ----------------------------------------------------------
    corners = lot["corners"]
    n_groups = max(1, rng.randint(*[int(v) for v in scfg.get("groups", [2, 3])]))
    if anchors:
        pick = [anchors[i % len(anchors)] for i in range(n_groups)]
        seeds = [(a[0], a[1]) for a in pick]
    else:
        seeds = [(cx, cy)] * n_groups
    r_lo, r_hi = [float(v) for v in scfg.get("cluster_r_m", [2.0, 9.0])]
    # STANDING ONLY at a refuge lot. The seated and crouching variants
    # are authored and grounded (tools/pose_check.py passes them on all six
    # rigs) — they are cut here because upright is the silhouette that survives
    # being looked at from capture altitude, and a lot full of one clean
    # posture reads as an assembly rather than as a crowd doing six things.
    bag = ["idle"]
    # SOME OF THE CROWD IS SITTING IN THE CARS. A refuge lot came out with
    # everybody standing on the tarmac — 0 of 17 inside a vehicle — which is
    # not what a lot full of people who DROVE there looks like: some are out
    # of the car and some are still in it, and the ones still in it are the
    # harder detection case worth having in the set. `in_car_share` of the
    # group is seated first, in the cars this pass just parked, and only what
    # is left over goes onto the asphalt.
    share = float(scfg.get("in_car_share", 0.35))
    n_seat = int(round(want * max(0.0, min(1.0, share))))
    seated = 0
    for (ax, ay, q) in anchors:
        if seated >= n_seat:
            break
        if not _can_open(q.get("usd")):
            continue
        ln, wd, ht = car_dims(plan.ctx, q["usd"])
        if ht < 1.18:
            continue
        for seat in car_seats(q["usd"], ln, wd, ht):
            if seated >= n_seat:
                break
            _seat_person(plan, "parking_refuge", li * 10, q, ln, wd, ht, seat,
                         note="waiting in the car at a refuge lot")
            seated += 1
    want = max(0, want - seated)
    for i in range(want):
        g = i % n_groups
        sx, sy = seeds[g]
        got = False
        for _ in range(160):
            r = rng.uniform(r_lo, r_hi)
            a = rng.uniform(0.0, 360.0)
            x = sx + r * math.cos(math.radians(a))
            y = sy + r * math.sin(math.radians(a))
            if not _in_poly(corners, (x, y)):
                continue
            if not plan.ground.free(x, y, road=True, house_m=2.0, solid_m=0.9,
                                    props=False):
                continue
            pose = rng.choice(bag)
            # Face roughly inward, toward the group's own middle.
            yaw = math.degrees(math.atan2(sy - y, sx - x)) + rng.uniform(-50, 50)
            plan.add_person("parking_refuge", li * 10 + g, x, y, 0.0,
                            yaw, pose)
            got = True
            break
        if not got:
            plan.notes.append("parking_refuge: no room for person %d" % i)


def _in_poly(poly, p):
    from layout import suburb_net as sn
    return sn.point_in_polygon([tuple(q) for q in poly], (p[0], p[1]))


# ---------------------------------------------------------------------------
# 2) refuge — open ground
# ---------------------------------------------------------------------------

def _open_ground(plan, want):
    """Groups standing on open lawn, well clear of anything that can burn.

    THREE NATURAL-AREA TRAs IN THE CAMP FIRE, against fourteen lots — the
    minority case, but the one that shows what people are actually doing:
    putting distance between themselves and fuel. The 15 m clearance is the
    whole scenario; a group of eight on a lawn 4 m from a hedge is not a
    refuge, it is a garden party.
    """
    if want <= 0:
        return
    scfg = plan.cfg["scenarios"].get("open_ground") or {}
    rng = plan.rng
    clear = float(scfg.get("clear_m", 15.0))
    regions = _open_regions(plan.ctx)
    if not regions:
        plan.notes.append("open_ground SKIPPED: no open polygons in ctx")
        return
    n_groups = max(1, rng.randint(*[int(v) for v in scfg.get("groups", [2, 3])]))
    lo, hi = [int(v) for v in scfg.get("group_size", [3, 8])]
    sizes = _split(want, n_groups, lo, hi, rng)
    r_lo, r_hi = [float(v) for v in scfg.get("cluster_r_m", [2.0, 8.0])]
    weights = [_poly_area(p) for p in regions]

    for g, size in enumerate(sizes):
        seed = _find_open_seed(plan, regions, weights, clear, rng)
        if seed is None:
            plan.notes.append("open_ground: no seed clear of %.0f m for group %d"
                              % (clear, g))
            continue
        sx, sy = seed
        placed = 0
        for _ in range(size * 60):
            if placed >= size:
                break
            r = rng.uniform(r_lo, r_hi)
            a = rng.uniform(0.0, 360.0)
            x = sx + r * math.cos(math.radians(a))
            y = sy + r * math.sin(math.radians(a))
            if plan.ground.house_dist(x, y) < clear * 0.6:
                continue
            if not plan.ground.free(x, y, house_m=None, solid_m=3.0,
                                    road_margin=2.0):
                continue
            pose = rng.choice(["idle"] * 6 + ["sit_ground"] * 4)
            yaw = math.degrees(math.atan2(sy - y, sx - x)) + rng.uniform(-60, 60)
            plan.add_person("open_ground", g, x, y, 0.0, yaw, pose)
            placed += 1


def _open_regions(ctx):
    """Every piece of ground the plat left bare, as polygons.

    The three kinds `suburb_scene.build_open_planting` documents: undeveloped
    parcels (including the ones merged into the park, which are in no `blocks`
    list at all), the park's real extent, and the interiors of developed
    blocks. The lots inside a developed block are excluded by the house/lot
    clearance rather than by clipping the polygon — a person 15 m from every
    house is out of the lots by construction.
    """
    out = []
    for poly in (ctx.get("open_polys") or ()):
        if poly and len(poly) >= 3:
            out.append([tuple(q) for q in poly])
    park = (ctx.get("park") or {}).get("poly")
    if park and len(park) >= 3:
        out.append([tuple(q) for q in park])
    for p in (ctx.get("parcels") or ()):
        poly = p.get("block")
        if poly and len(poly) >= 3 and _poly_area(poly) > 4000.0:
            out.append([tuple(q) for q in poly])
    return out


def _find_open_seed(plan, regions, weights, clear, rng):
    """A point on open ground, `clear` metres from everything, in the burn.

    PREFERS THE BURN AND SETTLES FOR ITS EDGE. A refuge inside the black is
    the informative case (that is where somebody is actually waiting to be
    found), but a plat only burns as far as the front got — so the second pass
    accepts the near-unburnt edge rather than giving up and placing nobody.
    """
    age = plan.ctx["age"]
    span = max(1.0, float(plan.ctx.get("span", 1.0)))
    total = sum(weights) or 1.0
    for mode in ("burnt", "edge", "any"):
        for _ in range(4000):
            r = rng.random() * total
            acc = 0.0
            poly = regions[-1]
            for p, w in zip(regions, weights):
                acc += w
                if r <= acc:
                    poly = p
                    break
            x0, y0, x1, y1 = _poly_bbox(poly)
            x = rng.uniform(x0, x1)
            y = rng.uniform(y0, y1)
            if not _in_poly(poly, (x, y)):
                continue
            a = age(x, y)
            if mode == "burnt" and a <= 0.0:
                continue
            if mode == "edge" and a < -0.20 * span:
                continue
            if plan.ground.house_dist(x, y) < clear:
                continue
            if plan.ground.solid_dist(x, y) < clear:
                continue
            if plan.ground.props.hit(x, y, clear):
                continue
            if plan.ground.on_road(x, y, 4.0):
                continue
            return (x, y)
    return None


def _split(total, n, lo, hi, rng):
    """*total* dealt into at most *n* group sizes, each in [lo, hi].

    THE GROUP COUNT IS AN UPPER BOUND, not a target: asking for three groups
    of at least three out of a quota of four has to give two groups, and the
    naive version gives three groups of three and quietly places nine people.
    The caller therefore iterates over the RETURNED list, never over its own
    requested *n*.
    """
    lo = max(1, int(lo))
    if n <= 0 or total <= 0:
        return []
    n = max(1, min(int(n), total // lo))
    base = [0] * n
    i = 0
    while sum(base) < total:
        j = i % n
        if base[j] < hi or all(v >= hi for v in base):
            base[j] += 1
        i += 1
    rng.shuffle(base)
    return base


# ---------------------------------------------------------------------------
# 3) pools
# ---------------------------------------------------------------------------

def _pools(plan, want):
    """People in and around a domestic pool behind a gutted house.

    THE SUBURBAN SEAWALL. Immersion in a pool is a documented last resort in
    Australian and Californian fires alike, and it is the case a burn-scar
    capture is best placed to catch: the pool survives the fire intact and
    unscorched (`damage.INCOMBUSTIBLE`), so it is one of the highest-contrast
    features left in the black. Pools behind houses that BURNED read as
    shelter; a pool behind an untouched house reads as a swim.

    ONE POSTURE: chest-deep, standing IN the water. Floating prone and sitting
    on the coping were both authored and both cut on sight. From the capture
    altitude a floater reads as debris on the surface rather than as a person,
    and a coping-sitter's legs disappear into the deck so the silhouette is a
    torso on stone — neither is the thing this scenario exists to label. A head
    and shoulders on a water plane is, and it is the posture that survives being
    looked at from 40 m up.
    """
    if want <= 0:
        return
    scfg = plan.cfg["scenarios"].get("pools") or {}
    rng = plan.rng
    age = plan.ctx["age"]
    houses = plan.ctx.get("houses") or []
    by_index = {h.get("index"): h for h in houses if h.get("index") is not None}

    cands = []
    for p in (plan.ctx.get("pools") or ()):
        cx, cy = p["centre"]
        if age(cx, cy) <= 0.0:
            continue
        h = by_index.get(p.get("house_index"))
        lvl = (h or {}).get("level")
        cands.append((0 if lvl in _GUTTED else 1, rng.random(), p))
    if not cands:
        plan.notes.append("pools SKIPPED: no pool inside the burn")
        return
    cands.sort(key=lambda t: (t[0], t[1]))
    n_pool = min(len(cands), max(1, rng.randint(
        *[int(v) for v in scfg.get("pools", [2, 3])])))
    gutted = sum(1 for c in cands[:n_pool] if c[0] == 0)
    plan.notes.append("pools: %d used, %d of them behind a gutted house"
                      % (n_pool, gutted))

    per_lo, per_hi = [int(v) for v in scfg.get("per_pool", [1, 3])]
    quota = _split(want, n_pool, per_lo, per_hi, rng)
    for g, (n, (_pri, _r, pool)) in enumerate(zip(quota, cands)):
        _pool_people(plan, g, pool, n)


def _pool_people(plan, group, pool, want):
    if want <= 0:
        return
    rng = plan.rng
    ring = [tuple(q) for q in pool["water_ring"]]
    cx, cy = pool["centre"]
    yaw = float(pool["yaw_deg"])              # the LONG axis, 8 m
    ux, uy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    vx, vy = -uy, ux
    # `modular_house.pool_at` authors 8 x 4 m, ring inset by one coping band;
    # measure it back off the ring rather than trusting the nominal size.
    hl = math.hypot(ring[1][0] - ring[0][0], ring[1][1] - ring[0][1]) / 2.0
    hw = math.hypot(ring[3][0] - ring[0][0], ring[3][1] - ring[0][1]) / 2.0
    water = POOL_WATER_Z_M

    # Everyone in the pool stands chest-deep — see `_pools` for why the other
    # two postures were cut. AN 8 x 4 m POOL IS SMALL: two independent draws
    # inside it land under a metre apart often enough that the separation
    # invariant fails, so this asks the same `mine` index every other scenario
    # asks and simply places fewer people when the water is full.
    for _i in range(want):
        got = None
        for _ in range(120):
            fx = rng.uniform(-0.55, 0.55)
            fy = rng.uniform(-0.45, 0.45)
            px = cx + ux * hl * fx + vx * hw * fy
            py = cy + uy * hl * fx + vy * hw * fy
            if not plan.ground.mine.hit(px, py, 0.0):
                got = (px, py)
                break
        if got is None:
            continue
        x, y = got
        usd, pose = plan.pick_human("idle")
        fp = plan.ctx["resolver"].get(
            usd, "human", scale=plan.ctx["asset_pools"].scale_of(usd),
            axis_up=plan.ctx["asset_pools"].axis_of(usd))
        sink = water - CHEST_FRAC * float(fp.get("sz", 1.8))
        plan.add_person("pools", group, x, y, sink,
                        rng.uniform(0.0, 360.0), pose, usd=usd,
                        note="chest-deep; water plane at %.2f m" % water)


# ---------------------------------------------------------------------------
# 4) the gridlock queue
# ---------------------------------------------------------------------------

def _gridlock(plan, want):
    """A stalled evacuation queue behind a blocked carriageway.

    SKYWAY, PARADISE: ~85 abandoned vehicles per 1.6 km of the town's main
    egress. What produces that is not traffic volume, it is a BLOCKAGE — a
    tree across the road, a downed pole, a crash — with a queue backing up
    into the burn behind it and people getting out and walking. Everything in
    this scenario follows from that one fact, which is why the blocker is
    placed first and the cars are measured back from it.

    THE CARS GO IN BEFORE THE SCORCH PASS. A queue of pristine cars sitting in
    the black is the single most obvious tell in the whole scene; handing them
    to the assembly as ordinary `category: "car"` placements means they take
    the same soot the driveway cars take, from the same field.
    """
    if want <= 0:
        return
    scfg = plan.cfg["scenarios"].get("gridlock") or {}
    rng = plan.rng
    from detail import vehicles as veh
    from layout import suburb_net as sn

    net = plan.ctx.get("net")
    if net is None:
        plan.notes.append("gridlock SKIPPED: no net in ctx")
        return
    # MORE THAN ONE JAM. The Camp Fire had 23 burnover events and "every
    # primary egress artery experienced at least one" — a plat with a single
    # blocked road reads as an isolated accident, and on 1600 x 1200 m it is
    # also a needle: one queue is roughly one part in two thousand of the
    # frame. Each queue takes its own street (the picker excludes the ones
    # already used) and its own blockage, and the head count is split between
    # them.
    q_lo, q_hi = (scfg.get("queues") or [2, 3])[:2]
    n_q = max(1, rng.randint(int(q_lo), int(q_hi)))
    shares = _split(want, n_q, 1, max(1, want), rng)
    used = set()
    for qi, share in enumerate(shares):
        if share <= 0:
            continue
        # A queue that cannot be sited is not a reason to abandon the rest:
        # try the next one, which asks for less.
        _one_queue(plan, scfg, sn, veh, share, used, qi)
    return


def _one_queue(plan, scfg, sn, veh, want, used, qi):
    """One jammed street: cars, a blockage, and the people around it."""
    rng = plan.rng
    net = plan.ctx.get("net")
    age = plan.ctx["age"]
    pick = _pick_queue_street(plan, sn, used, relax=1 if qi else 0)
    if pick is None:
        plan.notes.append("gridlock SKIPPED: no collector/arterial chain runs "
                          "out of the burn with room for a queue")
        return False
    pts, parts, dirn, half_w, sid = pick
    used.add(sid)
    length = sn.polyline_length(pts)
    age = plan.ctx["age"]

    # -- where the blocker goes -----------------------------------------
    # AT THE BURN EDGE, JUST INSIDE IT. This is the whole geometry of the
    # scenario and the first cut got it wrong: putting the blocker at a fixed
    # fraction of the chain landed it 169 s of arrival time OUTSIDE the burn,
    # so the queue behind it was in clean grass. The informative arrangement
    # is the other way round — a queue standing IN the black, its head at a
    # blockage on the last of the burnt ground, and open unburnt road ahead
    # that people are walking out along. So find where the front's arrival
    # line crosses this chain and step back inside.
    s_b = _blocker_s(pts, parts, dirn, length, age, sn, rng)
    bx, by = sn.point_at(pts, s_b)
    tx, ty = sn.tangent_at(pts, s_b)
    out_bear = math.degrees(math.atan2(ty * dirn, tx * dirn))
    plan.notes.append(
        "gridlock on street %s: %.0f m of %s (half_w %.1f m), outbound "
        "bearing %.0f deg, burn age %.0f s at the blocker"
        % (sid, length, _road_class_of(net, parts), half_w, out_bear,
           age(bx, by)))

    kind = _add_blocker(plan, pts, s_b, half_w, out_bear)

    # -- the queue -------------------------------------------------------
    n_car = rng.randint(*[int(v) for v in scfg.get("cars", [6, 12])])
    gap = float(scfg.get("gap_m", 1.5))
    head_gap = rng.uniform(*[float(v) for v in scfg.get("blocker_gap_m", [3.0, 6.0])])
    res = list(plan.ctx["car_pool"].get("residential") or ())
    rv = list(plan.ctx["car_pool"].get("rv") or ())
    if not res:
        plan.notes.append("gridlock: no residential car pool")
        return False
    rv_used = False
    s = s_b - dirn * head_gap
    queue = []
    n_bump = 0
    for i in range(n_car):
        usd = res[rng.randrange(len(res))]
        if rv and not rv_used and rng.random() < float(scfg.get("rv_chance", 0.5)):
            usd, rv_used = rv[rng.randrange(len(rv))], True
        ln, wd, ht = car_dims(plan.ctx, usd)
        # A LITTLE LATERAL SLOP AND A COUPLE OF DEGREES OF YAW is the
        # difference between a rank and a jam — drawn once, before the fitting
        # loop, so the rng sequence does not depend on how many nudges a slot
        # needed.
        j = rng.uniform(-0.45, 0.45)
        dyaw = rng.uniform(-3.5, 3.5)
        # NUDGED BACK, NOT DROPPED. The queue lies in the outbound lane of a
        # street the plat may already have parked cars on — `_pick_queue_street`
        # relaxes to `local` for the second and third jam, and `local` is one
        # of the two classes `build_cars` ranks the kerb along — and the head
        # of the queue is measured off the blocker, not off what is there. So
        # a slot that is occupied slides another metre back down the queue and
        # tries again, which is exactly what the next driver would have done.
        # Six tries is 6 m; past that the obstruction is a rank rather than one
        # car and the queue is better off ending here.
        placed = None
        for bump in range(6):
            s_c = s - dirn * (ln * 0.5 + bump * 1.0)
            if not (2.0 < s_c < length - 2.0):
                break
            edge, s_e, flipped = _edge_at(parts, pts, s_c, sn)
            side = (1 if not flipped else -1) * (1 if dirn > 0 else -1)
            x, y, head = veh.car_pose_for_lane(net, edge, s_e, side)
            nx, ny = -math.sin(math.radians(head)), math.cos(math.radians(head))
            x, y = x + nx * j, y + ny * j
            head += dyaw
            q = plan.add_car(usd, x, y, head, "queue")
            if q is not None:
                placed = (q, s_c, head)
                n_bump += bump
                break
        if placed is None:
            # Out of road, or a stretch of it that is solid with parked cars.
            break
        q, s_c, head = placed
        q["queue_index"] = i
        queue.append((q, ln, wd, ht, s_c, head))
        s = s_c - dirn * (ln * 0.5 + gap)
    if n_bump:
        plan.notes.append("gridlock: %d m of nudge-back total to keep the "
                          "queue off cars already parked on this street"
                          % n_bump)

    # -- people ----------------------------------------------------------
    # HOW MANY STAY IN THEIR CARS. Was `min(2, want // 6)` — a hard ceiling of
    # two per queue however long the jam, which is why a fourteen-person
    # gridlock came out with twelve people standing in the road and two
    # sitting down. Real abandoned queues are the other way round early on:
    # people wait in the car until the fire makes them leave it, so the
    # occupants are the majority posture, not a garnish. `in_car_share` of the
    # queue's people, capped by how many seats the cars actually have.
    n_seat = max(1, int(round(want * float(scfg.get("in_car_share", 0.40)))))
    n_walk = min(4, max(2, want // 4))
    n_kerb = 1 if want >= 6 else 0
    n_side = max(0, want - n_seat - n_walk - n_kerb)
    _queue_occupants(plan, queue, n_seat, qi)
    _queue_bystanders(plan, queue, n_side, half_w, qi)
    _queue_walkers(plan, pts, parts, s_b, dirn, length, half_w, n_walk,
                   out_bear, scfg, sn, qi)
    _queue_kerb(plan, pts, s_b, dirn, half_w, n_kerb, sn, qi)
    if plan.blockers:
        plan.blockers[-1]["queue_cars"] = len(queue)
    plan.notes.append("gridlock: %d car(s), blocker=%s, head %.1f m short"
                      % (len(queue), kind, head_gap))


def _blocker_s(pts, parts, dirn, length, age, sn, rng):
    """Arclength for the blockage: 15-60 m back inside the burn from the edge.

    The crossing is found by sampling the chain and watching the sign of
    `age`; the one used is the crossing nearest the middle whose UPSTREAM side
    (against `dirn`, i.e. deeper into the fire) is the burnt one, because that
    is the side the queue backs up along. With no crossing on this chain —
    the whole thing is inside the burn — the blockage goes two thirds of the
    way along, outbound, and the queue still sits in the black.
    """
    n = 240
    samples = [(length * i / n, age(*sn.point_at(pts, length * i / n)))
               for i in range(n + 1)]
    hits = []
    for i in range(n):
        (s0, a0), (s1, a1) = samples[i], samples[i + 1]
        if (a0 > 0.0) == (a1 > 0.0):
            continue
        # burnt on the upstream side of the crossing?
        burnt_lo = a0 > 0.0
        if (dirn > 0 and burnt_lo) or (dirn < 0 and not burnt_lo):
            hits.append(0.5 * (s0 + s1))
    back = rng.uniform(15.0, 60.0)
    if hits:
        mid = length * 0.5
        s_e = min(hits, key=lambda v: abs(v - mid))
        s_b = s_e - dirn * back
    else:
        s_b = length * (0.66 if dirn > 0 else 0.34)
    # Leave the queue room behind and the walkers road ahead.
    return max(90.0, min(length - 40.0, s_b))


def _road_class_of(net, parts):
    return parts[0][0].road_class if parts else "?"


def _edge_at(parts, pts, s, sn):
    """Map an arclength along a street CHAIN back to ``(edge, s_edge, flipped)``.

    `street_chain` hands back the edges in chain order with a `flipped` flag;
    `car_pose_for_lane` wants an arclength along the EDGE's own `pts`, so a
    flipped edge's arclength runs the other way.
    """
    acc = 0.0
    last = parts[-1]
    for (edge, flipped) in parts:
        le = sn.polyline_length(edge.pts)
        if s <= acc + le:
            se = s - acc
            return edge, (le - se if flipped else se), flipped
        acc += le
    edge, flipped = last
    le = sn.polyline_length(edge.pts)
    return edge, (0.0 if flipped else le), flipped
    return True


def _pick_queue_street(plan, sn, used=(), relax=0):
    """The best collector/arterial chain to jam, with its outbound direction.

    WHAT MAKES A STREET THE RIGHT ONE. It has to be (a) a through road — an
    evacuation does not queue on a cul-de-sac, (b) long enough for a dozen
    cars plus a hundred metres of road for people to walk out along, (c)
    burning where the queue sits, and (d) running OUT of the burn, because a
    queue heading deeper into it is not an evacuation. (d) is measured as the
    sign of the arrival-time gradient along the chain: outbound is the
    direction in which the front arrives LATER.
    """
    net = plan.ctx["net"]
    age = plan.ctx["age"]
    best = None
    # RELAX FOR THE SECOND AND THIRD JAM. The first queue should be on the
    # best egress artery there is, and after that the standard is "a through
    # road the front crossed" rather than "the best one" — a plat has only a
    # few collectors and holding every queue to that bar is what left the
    # first multi-queue build with exactly one blockage. Later passes admit
    # local streets and shorter chains; a local road is a perfectly real place
    # to be stuck, and on this fabric it is where most people actually live.
    _classes = ("collector", "arterial")
    _min_len = 180.0
    if relax >= 1:
        _classes = ("collector", "arterial", "local")
        _min_len = 120.0
    sids = sorted({e.street_id for e in net.edges.values()
                   if e.road_class in _classes})
    for sid in sids:
        if sid in used:
            continue
        pts, parts = net.street_chain(sid)
        if len(pts) < 2 or not parts:
            continue
        length = sn.polyline_length(pts)
        if length < _min_len:
            continue
        a_lo = age(*sn.point_at(pts, length * 0.20))
        a_hi = age(*sn.point_at(pts, length * 0.80))
        if max(a_lo, a_hi) <= 0.0:
            continue                      # the front never got here
        # Outbound = toward the LATER arrival = the smaller `age`.
        dirn = 1 if a_hi < a_lo else -1
        grad = abs(a_hi - a_lo)
        # The queue itself must sit in the black: check where it will go.
        s_q = length * (0.45 if dirn > 0 else 0.55)
        if age(*sn.point_at(pts, s_q)) <= 0.0:
            continue
        half_w = max(e.half_w for (e, _f) in parts)
        score = grad * min(length, 600.0)
        if best is None or score > best[0]:
            best = (score, pts, parts, dirn, half_w, sid)
    if best is None:
        return None
    return best[1], best[2], best[3], best[4], best[5]


def _blocker_debris(cx, cy, across, half_w, rng):
    """Timber and wreckage strewn over the carriageway at the blockage.

    AN ARCHETYPE ALONE DOES NOT BLOCK A ROAD. A `tree_*_fallen` reference is
    one object dropped at the kerb, and from the air it reads as a tree that
    fell NEAR the road rather than as a road nobody is driving down — its own
    ground debris is a disc around its stump, out on the verge where the tree
    stood, not spread across the lanes. What makes a blockage legible is the
    litter field ON the asphalt: trunk sections lying across the lanes at
    different bearings, and limbs between and on top of them.

    Authored as plain geometry rather than referenced, for the reason the
    occupant bench records: an archetype brings a stump rooted at its origin,
    and a stump in the tarmac is exactly the tell that a whole tree was
    dropped where a fallen one was wanted.

    Returns a list of `{"kind","p0","p1","r0","r1"}` in WORLD metres; the
    launcher authors each as a tapered tube. Z is absolute and already lifted
    clear of the carriageway.
    """
    out = []
    th = math.radians(across)
    # Along the road is perpendicular to the bole's bearing.
    ax, ay = math.cos(th), math.sin(th)
    bx, by = -ay, ax
    lift = 0.10                     # the carriageway is at ~0.10; sit ON it
    # Trunk sections, staggered along the road and crossing it at a spread of
    # bearings. Radii are real: a 0.3-0.5 m bole is what comes down on a
    # street tree, and it is what a car cannot drive over.
    for i in range(4):
        along = (i - 1.5) * rng.uniform(3.2, 5.0)
        off = rng.uniform(-0.35, 0.35) * half_w
        ln = rng.uniform(6.5, min(11.0, half_w * 2.1))
        r0 = rng.uniform(0.30, 0.48)
        r1 = r0 * rng.uniform(0.62, 0.82)
        ph = math.radians(across + rng.uniform(-32.0, 32.0))
        hx, hy = math.cos(ph) * ln / 2.0, math.sin(ph) * ln / 2.0
        px = cx + bx * along + ax * off
        py = cy + by * along + ay * off
        out.append({"kind": "log",
                    "p0": (px - hx, py - hy, lift + r0),
                    "p1": (px + hx, py + hy, lift + r1),
                    "r0": r0, "r1": r1})
    # Limbs. Most on the road, a third resting on the trunks.
    for i in range(34):
        along = rng.uniform(-9.0, 9.0)
        off = rng.uniform(-1.0, 1.0) * half_w
        ln = rng.uniform(1.1, 3.8)
        r = rng.uniform(0.05, 0.14)
        ph = rng.uniform(0.0, math.pi)
        hx, hy = math.cos(ph) * ln / 2.0, math.sin(ph) * ln / 2.0
        px = cx + bx * along + ax * off
        py = cy + by * along + ay * off
        z = lift + r + (rng.uniform(0.30, 0.75) if rng.random() < 0.32 else 0.0)
        out.append({"kind": "limb",
                    "p0": (px - hx, py - hy, z),
                    "p1": (px + hx, py + hy, z + rng.uniform(-0.2, 0.2)),
                    "r0": r, "r1": r * rng.uniform(0.6, 0.9)})
    return out


def _add_blocker(plan, pts, s_b, half_w, out_bear):
    """Put something across the carriageway. Returns its kind.

    TWO KINDS, one draw. A fallen tree is the archetypal wildfire road
    blockage and is the one that reads from altitude; a toppled streetlight is
    the second most common and is nearly free, because the plat has already
    placed thousands of them and rotating one costs a transform. The tree
    needs an archetype on disk, so the streetlight is also the fallback.
    """
    from layout import suburb_net as sn
    rng = plan.rng
    scfg = plan.cfg["scenarios"].get("gridlock") or {}
    # DRAWN FIRST AND UNCONDITIONALLY, before the kind is decided, so the rng
    # sequence does not depend on which flavour of blocker the geometry
    # happened to allow — the same rule the kerb pass keeps for its slot step.
    # THIS MODULE OWNS THE DECISION, NOT THE FLAME: no Flow code lives here.
    # The spec carries a boolean and the assembly launcher is what builds the
    # emitters over the ones that say True.
    burning = rng.random() < float(scfg.get("fire_chance", 0.6))
    cx, cy = sn.point_at(pts, s_b)
    # WHICH SIDE THE TREE FELL FROM IS NOT A COIN TOSS, and the first version
    # made it one. The queue occupies the OUTBOUND lane, which under
    # right-hand traffic is `half_w * 0.5` to the RIGHT of the outbound
    # heading; a bole reaching 5 m from the far kerb of an 11 m carriageway
    # stops at the centreline and blocks the lane nobody is in. So the stump
    # goes on the outbound lane's own side — the verge that lane runs beside —
    # and the bole lies back across it toward the centreline, which is both
    # the useful arrangement and the physical one.
    th = math.radians(out_bear)
    nx, ny = math.sin(th), -math.cos(th)      # right of the outbound heading
    across = out_bear + 90.0                  # the bole's target bearing (-n)
    arch = plan.ctx.get("arch") or {}
    species = [s for s in BLOCKER_SPECIES if ("tree_%s_fallen" % s) in arch]
    light = _nearest_streetlight(plan, cx, cy)
    # A COIN, THEN WHATEVER EXISTS. Either blocker is a real one, so the draw
    # picks a flavour rather than a fallback — but a plat with no streetlight
    # within reach of this junction, or a build with no baked archetypes,
    # must still block the road, and an earlier version left the carriageway
    # clear whenever the coin and the geometry disagreed.
    want_tree = bool(species) and (rng.random() < 0.65 or light is None)
    if not want_tree and light is None:
        want_tree = bool(species)

    if want_tree:
        sp = rng.choice(species)
        usd = arch["tree_%s_fallen" % sp]
        bearing = bole_bearing_deg(usd, sp)
        # The STUMP goes at the kerb and the bole reaches across, which is
        # what a tree that fell from the verge actually does. Referenced
        # archetypes are baked re-centred on the stump.
        sx = cx + nx * (half_w + 0.6)
        sy = cy + ny * (half_w + 0.6)
        spec = {
            "kind": "fallen_tree", "species": sp, "usd": usd,
            "debris": _blocker_debris(cx, cy, across, half_w, rng),
            "x": sx, "y": sy, "yaw_deg": across - bearing,
            "bole_bearing_deg": bearing,
            "bole_reach_m": _BOLE_REACH_M.get(sp, 4.0),
            "half_w": half_w, "road_x": cx, "road_y": cy,
            "fire": burning,
            # THE DIRECTION THE ROAD RUNS OUT OF THE BURN. The queue backs up
            # BEHIND the blockage, so an emitter centred on the blocker sits
            # among the cars; with this the launcher can push the flame and
            # the smoke along +out_bear, past the blockage and away from the
            # queue. Without it it has no way to recover which way the street
            # runs and defaults to due +X.
            "out_bear_deg": float(out_bear),
            "note": "bole bearing measured off the archetype's bole meshes; "
                    "the archetype's own bbox is the DEBRIS field and is "
                    "nearly isotropic, so it cannot give this angle",
        }
        plan.blockers.append(spec)
        return "fallen_tree " + sp

    # A streetlight already standing near the blockage point, rotated over.
    if light is None:
        plan.notes.append("gridlock: no streetlight within 60 m of the "
                          "blockage and no fallen-tree archetype on disk — "
                          "carriageway left clear")
        return "none"
    cand = light
    lx, ly = float(cand["x_m"]), float(cand["y_m"])
    # Lay the pole from where it stands, toward the centreline.
    beta = math.degrees(math.atan2(cy - ly, cx - lx))
    spec = {
        "kind": "streetlight", "usd": cand.get("usd"),
        # A DOWNED POLE ALSO NEEDS THE LITTER. Same argument as the tree: one
        # object across a lane is a thing lying in the road; the strewn field
        # is what makes the road impassable.
        "debris": _blocker_debris(cx, cy, beta + 90.0, half_w, rng),
        "prim_path": cand.get("prim_path"), "scale": float(cand.get("scale", 1.0)),
        "axis_up": cand.get("axis_up", "Z"),
        "x": lx, "y": ly, "z": 0.20,
        # ROLL FIRST, THEN YAW. USD's rotateXYZ is Rz*Ry*Rx, so a pole standing
        # along +Z takes roll +85 to (0, -0.996, 0.087) and the yaw then swings
        # that in plan: the pole ends up on bearing `yaw - 90`. Hence
        # yaw = beta + 90 to lay it toward the road.
        #
        # THE 85 IS ADDED TO THE PLACEMENT'S OWN ROLL, not written over it: a
        # Y-up asset already carries +90 to stand up (`AssetPools.roll_of`),
        # and replacing that would leave the pole lying down in the wrong
        # plane. Adding keeps the derivation above true for both.
        "yaw_deg": beta + 90.0,
        "roll_deg": float(cand.get("roll_deg", 0.0)) + 85.0,
        "road_x": cx, "road_y": cy,
        "fire": burning,
        # Same field, same reason as the fallen tree above.
        "out_bear_deg": float(out_bear),
        "note": "VERIFY ON SIGHT: z is a nominal lying-pole radius, and the "
                "pivot is the placement anchor rather than the true base",
    }
    plan.blockers.append(spec)
    return "streetlight"


def _nearest_streetlight(plan, cx, cy, reach=60.0):
    cand, best = None, reach * reach
    for q in (plan.ctx.get("placements") or ()):
        if "streetlight" not in str(q.get("category", "")):
            continue
        d = ((float(q.get("x_m", 0.0)) - cx) ** 2
             + (float(q.get("y_m", 0.0)) - cy) ** 2)
        if d < best:
            best, cand = d, q
    return cand


def _queue_occupants(plan, queue, want, qi=0):
    """People still sitting in their cars.

    ONLY IN A CAR WHOSE GLASS CAME OFF. Every window in this library renders
    OPAQUE — the renderer forces fractional opacity to 1.0 — so an occupant
    behind glass is an occupant nobody will ever see, labelled or not.
    `detail/vehicles.strip_glass` can only work on the four RetroNeighborhood
    assets tagged `glass_separable`; the Muyang cars have their windows
    PAINTED INTO THE TEXTURE and there is nothing to remove.

    AND ONLY IN A CAR TALL ENOUGH. A seated figure's head is about 0.95 m over
    the seat pan, so a car whose roof is under ~1.18 m cannot hold one without
    the head coming through it.
    """
    if want <= 0:
        return
    rng = plan.rng
    ok = [(q, ln, wd, ht, s, hd) for (q, ln, wd, ht, s, hd) in queue
          if _can_open(q.get("usd")) and ht >= 1.18]
    if not ok:
        plan.notes.append(
            "gridlock: no glass-separable car in the queue is tall enough to "
            "seat somebody (need roof >= 1.18 m) — nobody placed inside")
        return
    # BOTH SEATS, WHERE BOTH WERE MEASURED. `car_seats` returns two rows for
    # every car in `_CAR_SEATS` and one for anything nobody has tuned, so a
    # queue whose cars are the tuned ones seats a driver AND a passenger
    # before it reaches for a second car. The whole list is shuffled rather
    # than sampled down to `want`: the quota is spent on seats, not on cars.
    n = 0
    for i, (q, ln, wd, ht, _s, head) in enumerate(rng.sample(ok, len(ok))):
        if n >= want:
            break
        for seat in car_seats(q["usd"], ln, wd, ht):
            if n >= want:
                break
            _seat_person(plan, "gridlock", 900 + qi * 10 + i, q, ln, wd, ht,
                         seat)
            n += 1


def _queue_bystanders(plan, queue, want, half_w, qi=0):
    """Standing beside the cars, in the lane — the Lahaina posture."""
    if want <= 0 or not queue:
        return
    rng = plan.rng
    placed = 0
    for _ in range(want * 40):
        if placed >= want:
            break
        (q, ln, wd, _ht, _s, head) = queue[rng.randrange(len(queue))]
        hr = math.radians(head)
        fx, fy = math.cos(hr), math.sin(hr)
        lx, ly = -fy, fx
        side = rng.choice([-1.0, 1.0])
        off = wd * 0.5 + rng.uniform(0.7, 1.9)
        along = rng.uniform(-ln * 0.6, ln * 0.6)
        x = q["x_m"] + fx * along + lx * off * side
        y = q["y_m"] + fy * along + ly * off * side
        if not plan.ground.free(x, y, road=True, house_m=1.5, solid_m=0.55,
                                props=False):
            continue
        pose = rng.choice(["idle"] * 7 + ["crouch"])
        plan.add_person("gridlock", qi * 10 + 0, x, y, 0.0,
                        head + rng.choice([90.0, -90.0, 180.0])
                        + rng.uniform(-25, 25), pose)
        placed += 1


def _queue_walkers(plan, pts, parts, s_b, dirn, length, half_w, want,
                   out_bear, scfg, sn, qi=0):
    """On foot, outbound, past the blockage. The 85-cars-a-mile behaviour."""
    if want <= 0:
        return
    rng = plan.rng
    lo, hi = [float(v) for v in scfg.get("walk_ahead_m", [20.0, 120.0])]
    placed = 0
    for _ in range(want * 40):
        if placed >= want:
            break
        s = s_b + dirn * rng.uniform(lo, hi)
        if not (2.0 < s < length - 2.0):
            continue
        cx, cy = sn.point_at(pts, s)
        tx, ty = sn.tangent_at(pts, s)
        off = rng.uniform(-half_w * 0.8, half_w * 0.8)
        x = cx - ty * off
        y = cy + tx * off
        if not plan.ground.free(x, y, road=True, house_m=1.5, solid_m=0.7,
                                props=False):
            continue
        bear = math.degrees(math.atan2(ty * dirn, tx * dirn))
        plan.add_person("gridlock", qi * 10 + 1, x, y, 0.0,
                        bear + rng.uniform(-14, 14), "walk")
        placed += 1


def _queue_kerb(plan, pts, s_b, dirn, half_w, want, sn, qi=0):
    """One person sat on the kerb beside the queue."""
    if want <= 0:
        return
    rng = plan.rng
    for _ in range(60):
        s = s_b - dirn * rng.uniform(12.0, 70.0)
        cx, cy = sn.point_at(pts, s)
        tx, ty = sn.tangent_at(pts, s)
        side = rng.choice([-1.0, 1.0])
        x = cx - ty * (half_w + 0.25) * side
        y = cy + tx * (half_w + 0.25) * side
        if not plan.ground.free(x, y, road=True, house_m=1.5, solid_m=0.7,
                                props=False):
            continue
        # Facing the carriageway, legs over the kerb.
        # Facing the carriageway: the seat is at `c + (ty, -tx) * side * off`,
        # so "toward the centreline" is the negative of that normal.
        face = math.degrees(math.atan2(-tx * side, ty * side))
        # NOMINAL KERB. Nothing in this scene models a kerb face — the road is
        # a ribbon on the z ladder — so 0.05 m is a seat height that does not
        # float rather than a measured one. VERIFY ON SIGHT.
        plan.add_person("gridlock", qi * 10 + 2, x, y, 0.05, face, "sit_edge",
                        note="nominal 0.05 m kerb; no kerb geometry exists")
        return


# ---------------------------------------------------------------------------
# 5) at home, outside
# ---------------------------------------------------------------------------

def _at_home(plan, want):
    """People in their own front yards at the edge of the front.

    THE LAST-MINUTE CASE, and the reason the burn EDGE is where it belongs: a
    front yard deep in the black is a front yard whose house has already
    burned, and the person who was in it left an hour ago. At the edge — the
    front either just past or just about to arrive — is where the
    stay-or-go decision is still live, and it is where front-yard rescues and
    front-yard fatalities both cluster in the after-action reports.
    """
    if want <= 0:
        return
    scfg = plan.cfg["scenarios"].get("at_home") or {}
    rng = plan.rng
    age = plan.ctx["age"]
    span = max(1.0, float(plan.ctx.get("span", 1.0)))
    band = 0.12 * span
    houses = [h for h in (plan.ctx.get("houses") or ())
              if h.get("index") is not None]
    edge = [h for h in houses if -0.20 * span < age(h["x"], h["y"]) < band]
    if not edge:
        edge = [h for h in houses if age(h["x"], h["y"]) < band]
    if not edge:
        plan.notes.append("at_home SKIPPED: no house at the burn edge")
        return
    rng.shuffle(edge)
    # Prefer houses with a car on the drive: somebody standing beside a loaded
    # car is the posture the scenario is about.
    cars = [q for q in (plan.ctx.get("cars") or ())
            if q.get("role") == "driveway"]

    def drive_car(h):
        best, bq = 26.0 ** 2, None
        for q in cars:
            d = (float(q["x_m"]) - h["x"]) ** 2 + (float(q["y_m"]) - h["y"]) ** 2
            if d < best:
                best, bq = d, q
        return bq

    edge.sort(key=lambda h: 0 if drive_car(h) else 1)
    n_house = min(len(edge), max(1, rng.randint(
        *[int(v) for v in scfg.get("houses", [3, 6])])))
    quota = _split(want, n_house, 1, 3, rng)
    for g, (n, h) in enumerate(zip(quota, edge)):
        _front_yard(plan, g, h, drive_car(h), n)


def _front_yard(plan, group, h, car, want):
    if want <= 0:
        return
    rng = plan.rng
    nx, ny = h["n"]                 # inward, kerb -> house
    ux, uy = h["u"]                 # along the frontage
    front = h["d"] * 0.5
    # EVERY BRANCH FALLS THROUGH TO THE YARD. A house whose drive is empty, a
    # step line already taken by the walk, a candidate that lands on a fence —
    # each of those has to end with a person in the front yard rather than
    # with a person missing, which is what the quota arithmetic assumes.
    kinds = ["by_car" if car else "yard", "step", "yard"]
    if plan.peacetime:
        # No front step to sit on that the kit actually builds (the "step"
        # below is a nominal 0.18 m line), and nobody sits on the ground in an
        # undamaged scene — POSTURE RULES. Standing in the yard or by the car.
        kinds = ["by_car" if car else "yard", "yard"]
    for i in range(want):
        kind = kinds[i % len(kinds)]
        if kind == "by_car" and car is None:
            kind = "yard"
        if kind == "by_car":
            cxx, cyy = float(car["x_m"]), float(car["y_m"])
            done = False
            for _ in range(30):
                side = rng.choice([-1.0, 1.0])
                x = cxx + ux * rng.uniform(1.4, 2.2) * side \
                    + nx * rng.uniform(-1.4, 1.4)
                y = cyy + uy * rng.uniform(1.4, 2.2) * side \
                    + ny * rng.uniform(-1.4, 1.4)
                if not plan.ground.free(x, y, house_m=0.8, solid_m=0.6,
                                        props=False):
                    continue
                yaw = math.degrees(math.atan2(cyy - y, cxx - x))
                plan.add_person("at_home", group, x, y, 0.0, yaw, "idle",
                                note="beside the driveway car")
                done = True
                break
            if not done:
                kind = "yard"
        elif kind == "step":
            # THE FRONT STEP LINE, approximated. `modular_house` builds a
            # porch but publishes no step height, so this is the footprint's
            # front face plus a metre, at a nominal 0.18 m step. VERIFY.
            done = False
            for _ in range(30):
                off = rng.uniform(-h["w"] * 0.30, h["w"] * 0.30)
                x = h["x"] - nx * (front + rng.uniform(0.9, 1.4)) + ux * off
                y = h["y"] - ny * (front + rng.uniform(0.9, 1.4)) + uy * off
                if not plan.ground.free(x, y, house_m=0.6, solid_m=0.6,
                                        props=False):
                    continue
                yaw = math.degrees(math.atan2(-ny, -nx))
                plan.add_person("at_home", group, x, y, 0.18, yaw, "sit_edge",
                                note="nominal 0.18 m front step; the kit "
                                     "publishes no step height")
                done = True
                break
            if not done:
                kind = "yard"
        if kind == "yard":
            for _ in range(40):
                off = rng.uniform(-h["w"] * 0.45, h["w"] * 0.45)
                out = rng.uniform(2.5, 7.0)
                x = h["x"] - nx * (front + out) + ux * off
                y = h["y"] - ny * (front + out) + uy * off
                if plan.ground.free(x, y, house_m=1.0, solid_m=0.6):
                    yaw = math.degrees(math.atan2(-ny, -nx)) + rng.uniform(-90, 90)
                    plan.add_person("at_home", group, x, y, 0.0, yaw, "idle")
                    break


# ---------------------------------------------------------------------------
# 6) exposed interior
# ---------------------------------------------------------------------------

def _cul_de_sac(plan, want):
    """A household trapped at the head of a dead end.

    THE GEOMETRY IS THE STORY. A lollipop has one stem and a paved turnaround
    at its tip, so a car that drove up it and met a blocked stem has nowhere
    left to go but round the bulb — which is why the cars here are parked at
    ANGLES to each other rather than in a queue: they turned, and stopped.
    Lahaina recorded the case directly (a couple recovered in their car after
    turning onto a dead-end street with the fire behind them), and it is the
    one place on a plat that yields a tight cluster of cars and people that a
    detector must not learn to call a car park.

    Bulbs come from `suburb_net` as `blk["bulbs"]` — `{"c": tip, "r": lot-line
    radius, "r_pave": kerb radius}` — which is the only record of where the
    pavement actually is; the block polygon does not know about the disc (see
    `suburb_parcel`'s "GROUND THE BLOCK POLYGON DOES NOT KNOW ABOUT").
    """
    if want <= 0:
        return
    scfg = plan.cfg["scenarios"].get("cul_de_sac") or {}
    rng = plan.rng
    age = plan.ctx["age"]

    bulbs = []
    for blk in (plan.ctx.get("blocks") or ()):
        for b in (blk.get("bulbs") or ()):
            c = b.get("c") or b.get("centre")
            if not c:
                continue
            r = float(b.get("r_pave") or b.get("r") or 12.0)
            bulbs.append((float(c[0]), float(c[1]), r))
    if not bulbs:
        plan.notes.append("cul_de_sac SKIPPED: no bulbs in ctx "
                          "(are cul-de-sacs enabled in suburb_net?)")
        return
    # IN THE BURN, and preferring the deepest: a dead end the front never
    # reached is just a quiet street.
    scored = [(age(x, y), x, y, r) for (x, y, r) in bulbs]
    burnt = [t for t in scored if t[0] > 0.0]
    if not burnt:
        burnt = sorted(scored, key=lambda t: -t[0])[:2]
        plan.notes.append("cul_de_sac: no bulb inside the burn, using the "
                          "nearest to it")
    burnt.sort(key=lambda t: -t[0])
    b_lo, b_hi = (scfg.get("bulbs") or [1, 2])[:2]
    n_bulb = rng.randint(int(b_lo), int(b_hi))
    chosen = burnt[:max(1, n_bulb)]

    per = _split(want, len(chosen), 1, max(1, want), rng)
    n_done = 0
    for gi, ((_d, cx, cy, r), quota) in enumerate(zip(chosen, per)):
        if quota <= 0:
            continue
        # CARS FIRST — the people are placed against them.
        c_lo, c_hi = (scfg.get("cars") or [2, 4])[:2]
        n_car = rng.randint(int(c_lo), int(c_hi))
        cars = []
        res = list(plan.ctx["car_pool"].get("residential") or ())
        for k in range(n_car):
            if not res:
                break
            usd = res[rng.randrange(len(res))]
            # RE-SITED, NOT MARKED. This used to draw one random point on the
            # disc, park there unconditionally, and call `ground.take(x, y)` —
            # a 1.2 m person-separation POINT — for a 4.6 m car. Two to four
            # cars scattered on a 14.6 m turnaround overlapped as a matter of
            # course, and the audit found a pair on every seed tried. The draw
            # is now a candidate: `add_car` refuses one that lands on another,
            # and 24 tries is enough to fit four cars on a disc that size
            # without them turning into a rank (the whole point of the bulb is
            # that they stopped at angles to each other).
            q = None
            for _try in range(24):
                a = rng.uniform(0.0, 2.0 * math.pi)
                rr = r * rng.uniform(0.45, 0.78)
                x, y = cx + math.cos(a) * rr, cy + math.sin(a) * rr
                # Nose roughly back down the stem, splayed: they turned and
                # gave up at different moments.
                hd = (math.degrees(math.atan2(cy - y, cx - x))
                      + rng.uniform(-70.0, 70.0))
                q = plan.add_car(usd, x, y, hd, "cul_de_sac")
                if q is not None:
                    break
            if q is None:
                plan.notes.append("cul_de_sac: no room for car %d of %d on "
                                  "the bulb at (%.0f, %.0f)" % (k + 1, n_car,
                                                                cx, cy))
                continue
            cars.append(q)
        # PEOPLE: one or two still in a car where the glass allows it, the
        # rest out on the turnaround beside them.
        placed = 0
        # AT MOST HALF THE GROUP STAYS IN THE CARS. Every car in the pool is
        # openable now (`_can_open` replaced the old `glass_separable` gate),
        # so this loop will happily seat the whole quota — and it did: all six
        # cul-de-sac survivors came out inside vehicles, with nobody standing
        # on the turnaround at all. An occupant is also the HARDEST of the two
        # to see from above, because the roof is still on even once the glass
        # is off, so filling the scenario with them makes the one place on the
        # plat with a tight cluster of people read as empty tarmac.
        n_seated_cap = max(1, int(round(quota * 0.6)))
        for q in cars:
            if placed >= quota or placed >= n_seated_cap:
                break
            # SAME TWO GATES AS THE QUEUE: removable glass, and a roof a
            # seated head clears. See `_queue_occupants` for why both are
            # hard requirements rather than preferences.
            if not _can_open(q.get("usd")):
                continue
            ln, wd, ht = car_dims(plan.ctx, q["usd"])
            if ht < 1.18:
                continue
            # THE SAME MEASURED SEATS THE QUEUE USES. This pass used to put
            # the occupant on the car's own anchor with no offset at all —
            # dead centre of the vehicle, between the seats — and take its yaw
            # from `yaw_deg`, which is the ART's rotation and carries the
            # pool's +90 offset, so the figure also sat across the car rather
            # than along it. Both come from `_seat_person` now.
            for seat in car_seats(q["usd"], ln, wd, ht):
                if placed >= quota:
                    break
                _seat_person(plan, "cul_de_sac", gi, q, ln, wd, ht, seat,
                             note="still in the car at a dead end")
                placed += 1
        for k in range(quota - placed):
            for _ in range(60):
                a = rng.uniform(0.0, 2.0 * math.pi)
                rr = r * rng.uniform(0.30, 0.95)
                x, y = cx + math.cos(a) * rr, cy + math.sin(a) * rr
                if plan.ground.free(x, y, road=False):
                    # A crouch beside the car is a disaster posture; on a
                    # quiet dead end everybody is on their feet.
                    pose = ("idle" if plan.peacetime
                            else rng.choice(["idle", "idle", "crouch"]))
                    plan.add_person(
                        "cul_de_sac", gi, x, y, 0.0,
                        rng.uniform(0.0, 360.0), pose,
                        note="on the turnaround")
                    placed += 1
                    break
        n_done += placed
    plan.notes.append("cul_de_sac: %d bulb(s), %d person(s)"
                      % (len(chosen), n_done))


# ---------------------------------------------------------------------------
# ground truth
# ---------------------------------------------------------------------------

def write_records(path, records, meta=None):
    """Write `humans.json`.

    An OBJECT with a `people` array rather than a bare array, so the seed, the
    burn clock and the scenario tally travel with the labels — a ground-truth
    file that cannot say which run produced it is a file somebody will mix up
    with another run's.
    """
    tally = {}
    for r in records:
        tally[r["scenario"]] = tally.get(r["scenario"], 0) + 1
    doc = {
        "schema": "airstack.people/1",
        "count": len(records),
        "alive": sum(1 for r in records if r.get("alive")),
        "by_scenario": tally,
        "meta": dict(meta or {}),
        "people": list(records),
    }
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    return path


# ---------------------------------------------------------------------------
# context
# ---------------------------------------------------------------------------

def build_ctx(config, info, placements, resolver, asset_pools, age, elapsed,
              span, arch=None, levels=None):
    """Assemble the `ctx` dict `plan_people` reads. Keys, and who owns them:

        net, blocks, parcels, open_polys, region, pools, cars, park
                          `suburb_scene.generate_suburb_on_stage(info_out=...)`
        houses            `house_table(parcels, info["house_instances"], levels)`
        trees             `info["tree_instances"]` (assembly mode only)
        placements        everything already on the ground, for keep-outs
        resolver          `scene_generator.SizeResolver` — measured in Isaac,
                          `fallback_sizes` on the host, which is exactly the
                          difference between a plan and a build
        asset_pools       `suburb_scene.AssetPools`, loaded with `humans` and
                          `cars` so scale / axis-up / yaw-offset are applied
        humans, humans_posed, car_pool, glassy   drawn by tag
        age, elapsed, span                        the fire, from the launcher
        arch              archetype name -> usd path, for the fallen-tree
                          blocker; may be empty
    """
    from suburb_scene import AssetPools, _raw_pool

    if asset_pools is None:
        asset_pools = AssetPools(config)
    raw_h = _raw_pool(config, "humans")
    rigged = asset_pools.load_tagged(raw_h, "rigged")
    posed = asset_pools.load_tagged(raw_h, "posed_standing")
    if not rigged:
        rigged = [u for u in asset_pools.load(raw_h) if u not in posed]
    raw_c = _raw_pool(config, "cars")
    pool = {tag: asset_pools.load_tagged(raw_c, tag)
            for tag in ("residential", "livery", "rv", "vintage", "commercial")}
    glassy = frozenset(asset_pools.load_tagged(raw_c, "glass_separable"))

    return {
        "net": info.get("net"), "blocks": info.get("blocks"),
        "parcels": info.get("parcels"), "open_polys": info.get("open_polys"),
        "region": info.get("region"), "park": info.get("park"),
        # Row-home courts, in `parking_info`'s schema — see `_parking_refuge`.
        "clusters": info.get("clusters"),
        "pools": info.get("pools"), "cars": info.get("cars"),
        "trees": info.get("tree_instances"),
        "houses": house_table(info.get("parcels") or [],
                              info.get("house_instances"), levels),
        "placements": placements,
        "resolver": resolver, "asset_pools": asset_pools,
        "humans": rigged, "humans_posed": posed,
        "car_pool": pool, "glassy": glassy,
        "age": age, "elapsed": float(elapsed), "span": float(span),
        "arch": arch or {},
    }
