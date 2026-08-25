"""findability — could a searcher have reached this person with a sightline?

Stage C (`targets.py`) decides WHERE people are. This decides whether each of
them is possible to FIND, which is the one property the placement model is not
allowed to get wrong: a victim nobody could ever see is not a hard search
problem, it is a corrupt ground truth. A run scored against it is penalised for
missing someone who was never there to be missed.

    MISSIONS.md G3 — "placement validator green on 3 seeds: every human
    findable, placements seed-deterministic and seed-diverse."

THE DEFINITION, MEASURABLY
--------------------------
"Partially obstructed is fine; completely buried is not" is the spec. Made
countable:

  * A **search viewpoint** is somewhere a searcher plausibly is — a drone at
    `AERIAL_ALT_M` overhead and on an off-nadir ring, and a person on the
    ground at eye height on a ring at `GROUND_RING_M`. `viewpoints()` is the
    whole set; 17 of them per victim.
  * A **sightline** is the segment from one viewpoint to the body. Its
    **cover** is the number of metres of material it has to cross to get there.
  * Material is either **porous** — rubble, debris, a building whose geometry
    was genuinely cut open, all of which are gaps at every scale — or
    **opaque**: an intact wall, which no sightline crosses at any thickness.
  * A victim is `clear` if some sightline crosses nothing, `partial` if the
    best one crosses at most `COVER_M` of porous material, and `buried`
    otherwise. **`buried` is the failure.** `partial` is the spec's "inside a
    vehicle / building / rubble is fine".

`COVER_M` is 3 m, and that number is the mission's judgement call rather than a
measurement: it is roughly how far into a collapse pile a void space stays
workable — reachable by a camera on a pole, a search dog or a listening device
— and it is the depth `targets._s_inside_rubble` samples its rim band to. The
two are deliberately coupled: the sampler places people where they can be
found, and this module is the independent check that it did.

TWO BACKENDS, ONE DEFINITION
----------------------------
    check(victims, occluders_from_survey(survey))   pure Python, no USD
    check_on_stage(stage, victims)                  PhysX, the real geometry

`check` is the one `tests/test_findability.py` runs and the one the CLI reports
with: milliseconds, no Isaac, so the gate is checkable on every seed of a sweep
from a laptop. It models a building as a box and a debris pile as a cylinder,
which is coarse in one specific direction — it treats a wrecked building as
uniformly porous, where the stage has actual holes in actual places. So it is
CONSERVATIVE about rubble: what it passes, the stage passes.

`check_on_stage` is the ground truth, run from inside Kit at the end of
`targets.place`, and it is what a G3 sign-off cites. Same viewpoints, same
verdict rule, same row shape; only the occlusion query differs.

    python3 findability.py --config urban_quake_tiny --seeds 1,2,3
"""

from __future__ import annotations

import math

# -- the search directions --------------------------------------------------
#: Azimuths, and the elevations they are swept at. Elevation is degrees above
#: horizontal, so 90 is a drone looking straight down at the victim and 10 is a
#: person standing off at a distance. Nothing points below horizontal: a
#: searcher is never underground.
AZIMUTHS = 8
ELEVATIONS = ((10.0, "ground"), (35.0, "aerial"), (60.0, "aerial"))

#: Aim point above the victim's recorded Z. A body lying down is ~0.3 m thick;
#: measuring from its own base would start every ray in the ground.
BODY_RISE_M = 0.3

#: How far a ray is followed before giving up. Wider than a city block.
REACH_M = 60.0
#: A clear run this long is open air. Under it, the ray is still picking its
#: way through wreckage rather than out of it.
OPEN_M = 2.0

# -- the verdict ------------------------------------------------------------
#: Metres of wreckage a victim may be behind and still count as findable.
#: Roughly how far into a collapse pile a void stays workable — reachable by a
#: camera on a pole, a search dog, a listening device. It is also the band
#: `targets._s_inside_rubble` samples the trapped cohort into, deliberately:
#: the sampler places people where they can be found and this checks that it
#: did, rather than the two agreeing by luck.
COVER_M = 3.0
#: Below this the victim is standing in open air.
CLEAR_M = 0.05
VERDICTS = ("clear", "partial", "buried")

#: Fallbacks when the survey could not measure a height. Only steep rays care.
DEFAULT_BUILDING_H_M = 12.0
#: Occluders start below grade: a trapped victim is settled BELOW the local
#: surface, and a slab that stopped at z=0 would let a ray reach open air by
#: going under it — clearing exactly the case this module exists to fail.
UNDER_M = 3.0


def directions() -> list:
    """Every way out a searcher could be looking down. ``[(name, unit), …]``.

    DIRECTIONS, NOT VIEWPOINTS, and the difference is a bug this had first
    time round. Rays cast at a fixed ring radius made an unrelated intact
    building 20 m away kill an entire azimuth — a searcher would simply have
    stood five metres closer. Where the searcher stands is not the question;
    whether there is a way out along some bearing is. So each ray starts at the
    body and is followed outward until it reaches open air, and the answer does
    not depend on a radius nobody chose.
    """
    out = [("zenith", (0.0, 0.0, 1.0))]
    for i in range(AZIMUTHS):
        az = math.tau * i / AZIMUTHS
        for elev, kind in ELEVATIONS:
            e = math.radians(elev)
            out.append((f"{kind}:{i * 360 // AZIMUTHS}@{elev:.0f}",
                        (math.cos(az) * math.cos(e),
                         math.sin(az) * math.cos(e), math.sin(e))))
    return out


def body_point(v: dict) -> tuple:
    """Where the body is, in metres — where every ray starts."""
    return (float(v["x"]), float(v["y"]), float(v.get("z") or 0.0) + BODY_RISE_M)


def verdict_for(escape_m: float) -> str:
    if escape_m <= CLEAR_M:
        return "clear"
    if escape_m <= COVER_M:
        return "partial"
    return "buried"


# ---------------------------------------------------------------------------
# Backend A — analytic, from the survey. No USD.
# ---------------------------------------------------------------------------

#: What is left standing, by damage rung, as a fraction of the intact height —
#: linear from 1.0 at `pristine` to `FLAT_FRAC` at the ladder's top rung.
#: Applied only to buildings that were really `cut`: an uncut one still has the
#: geometry it was measured with, whatever its label claims.
FLAT_FRAC = 0.15


def rubble_height(b: dict, ladder) -> float:
    """How tall a wrecked building still is, in metres.

    The survey measures a building from the asset it was PLACED with, which for
    a collapsed one is the intact model it was cut out of — a pancaked
    twenty-four-metre tower still reads 24 m. Charging a ray that height is not
    conservative, it is wrong in the expensive direction: it buries every
    trapped victim under their own building and fails the gate on scenes that
    are fine. `urban_quake_tiny` seed 3 is the case; `scene_gen/tests/
    test_findability.py::test_a_collapsed_building_is_not_as_tall_as_the_model`
    is the pin.
    """
    h = float(b.get("z") or DEFAULT_BUILDING_H_M)
    rungs = max(len(ladder or ()) - 1, 1)
    t = min(max(int(b.get("level_i") or 0), 0), rungs) / rungs
    return h * (1.0 - (1.0 - FLAT_FRAC) * t)


def occluders_from_survey(survey: dict) -> list:
    """Boxes and cylinders standing in for what the scene is made of.

    A building is a box; `cut` (see `targets.mark_cut_geometry`) is what makes
    it porous, and that is the whole reason `cut` is tracked rather than the
    damage rung: a building the damage budget never reached is INTACT, and an
    intact wall is opaque no matter what its label says. The rung decides
    something else — how much of it is still standing (`rubble_height`).
    """
    out = []
    ladder = survey.get("ladder") or ()
    for b in survey.get("buildings") or ():
        cut = bool(b.get("cut"))
        out.append({
            "shape": "box", "x": float(b["x"]), "y": float(b["y"]),
            "hw": float(b["w"]) / 2.0, "hh": float(b["h"]) / 2.0,
            "yaw": float(b.get("yaw") or 0.0),
            "z0": -UNDER_M,
            "z1": rubble_height(b, ladder) if cut
                  else float(b.get("z") or DEFAULT_BUILDING_H_M),
            "porous": cut,
            "what": b.get("prim_path") or "building",
        })
    for d in survey.get("debris") or ():
        r = float(d["r"])
        out.append({
            "shape": "cyl", "x": float(d["x"]), "y": float(d["y"]), "r": r,
            "z0": -UNDER_M, "z1": float(d.get("z") or max(1.0, r)),
            "porous": True, "what": "debris",
        })
    # THE EARTH IS OPAQUE, and it has to be in the list or the model has a
    # basement. Every bearing points upward, so a ray from a victim below grade
    # is the one case where the ground is between them and daylight; without
    # this slab a body sunk past the bottom of its pile leaves the box out of
    # the underside and reports open air. `bury_frac: [3.0, 5.0]` on the full
    # `earthquake` preset is the check — 0 buried before this, which is not a
    # gate, it is a rubber stamp.
    out.append({"shape": "box", "x": 0.0, "y": 0.0, "hw": 1e6, "hh": 1e6,
                "yaw": 0.0, "z0": -1e6, "z1": 0.0,
                "porous": False, "what": "ground"})
    return out


#: Body heights for the offline settle, standing and lying. The stage measures
#: the posed prim; offline there is nothing to measure.
BODY_H_M = (1.7, 0.45)


def _contains_xy(o: dict, x: float, y: float) -> bool:
    if o["shape"] == "box":
        a = math.radians(-o["yaw"])
        dx, dy = x - o["x"], y - o["y"]
        c, sn = math.cos(a), math.sin(a)
        return (abs(dx * c - dy * sn) <= o["hw"]
                and abs(dx * sn + dy * c) <= o["hh"])
    return (x - o["x"]) ** 2 + (y - o["y"]) ** 2 <= o["r"] ** 2


def surface_z(x: float, y: float, occluders: list) -> float:
    """Top of the tallest thing covering (x, y) — the model's ground."""
    return max([0.0] + [o["z1"] for o in occluders if _contains_xy(o, x, y)])


def settle_offline(victims: list, occluders: list) -> None:
    """Give each victim the Z that `targets.settle_on_surface` would, in model.

    WITHOUT THIS THE OFFLINE GATE IS UNUSABLE, and not because it is strict —
    because it is measuring the wrong thing. `sample_targets` leaves Z at 0;
    the stage then rests everyone on whatever is under them, which for a
    trapped victim is the top of their own pile. Skip that step and every
    trapped victim sits on the ground floor with the whole pile overhead, and
    the report reads `buried` at exactly the pile height — 3.30 m and 3.12 m on
    `urban_quake_tiny` seeds 3 and 6, which is the pile, not the person.

    So the analytic backend does the analytic version of the same settle: the
    model's surface is the top of whatever covers that spot, and the trapped
    are sunk into it by the same `bury_frac` the placer uses.
    """
    for v in victims:
        s = surface_z(float(v["x"]), float(v["y"]), occluders)
        z = s
        if v.get("cohort") == "inside_rubble":
            # Grade is the floor, exactly as in `targets.settle_on_surface`.
            z = max(s - float(v.get("bury_frac", 0.5))
                    * BODY_H_M[bool(v.get("lying"))], 0.0)
        v["surface_z"] = round(s, 3)
        v["z"] = round(z, 3)


def _box_span(a, b, o):
    """``(t0, t1)`` of the segment a→b inside box *o*, or None. Slab test."""
    yaw = math.radians(o["yaw"])
    c, s = math.cos(-yaw), math.sin(-yaw)

    def local(p):
        dx, dy = p[0] - o["x"], p[1] - o["y"]
        return (dx * c - dy * s, dx * s + dy * c, p[2])

    la, lb = local(a), local(b)
    lo = (-o["hw"], -o["hh"], o["z0"])
    hi = (o["hw"], o["hh"], o["z1"])
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = lb[i] - la[i]
        if abs(d) < 1e-12:
            if la[i] < lo[i] or la[i] > hi[i]:
                return None
            continue
        u, v = (lo[i] - la[i]) / d, (hi[i] - la[i]) / d
        if u > v:
            u, v = v, u
        t0, t1 = max(t0, u), min(t1, v)
        if t0 >= t1:
            return None
    return t0, t1


def _cyl_span(a, b, o):
    """``(t0, t1)`` of the segment a→b inside the vertical cylinder *o*."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    fx, fy = a[0] - o["x"], a[1] - o["y"]
    qa = dx * dx + dy * dy
    qb = 2.0 * (fx * dx + fy * dy)
    qc = fx * fx + fy * fy - o["r"] ** 2
    if qa < 1e-12:                       # straight up the axis
        if qc > 0.0:
            return None
        t0, t1 = 0.0, 1.0
    else:
        disc = qb * qb - 4.0 * qa * qc
        if disc <= 0.0:
            return None
        sq = math.sqrt(disc)
        t0, t1 = (-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)
    dz = b[2] - a[2]
    if abs(dz) < 1e-12:
        if a[2] < o["z0"] or a[2] > o["z1"]:
            return None
    else:
        u, v = (o["z0"] - a[2]) / dz, (o["z1"] - a[2]) / dz
        if u > v:
            u, v = v, u
        t0, t1 = max(t0, u), min(t1, v)
    t0, t1 = max(t0, 0.0), min(t1, 1.0)
    return (t0, t1) if t1 > t0 else None


def escape_along(p, d, occluders, reach: float = REACH_M,
                 open_m: float = OPEN_M) -> float:
    """Metres from *p* along unit *d* before the ray reaches open air.

    0 when the body is already in the open along this bearing; ``inf`` when
    something opaque is on it, because a wall is not a thickness, it is a no.
    Runs of material separated by less than *open_m* are one run: a hand's
    width of daylight between two slabs is not a way out.
    """
    b = tuple(p[i] + d[i] * reach for i in range(3))
    spans = []
    for o in occluders:
        span = _box_span(p, b, o) if o["shape"] == "box" else _cyl_span(p, b, o)
        if span is None:
            continue
        t0, t1 = span[0] * reach, span[1] * reach
        if t1 - t0 <= 1e-6:
            continue
        if not o["porous"]:
            return float("inf")
        spans.append((t0, t1))
    if not spans:
        return 0.0
    spans.sort()
    runs, (cs, ce) = [], spans[0]
    for s, e in spans[1:]:
        if s <= ce + open_m:
            ce = max(ce, e)
        else:
            runs.append((cs, ce))
            cs, ce = s, e
    runs.append((cs, ce))
    for s, e in runs:
        if s <= open_m:                  # material begins at or beside the body
            return e
    return 0.0


def check(victims: list, occluders: list) -> list:
    """A findability row per victim, from the analytic occluder model."""
    dirs = directions()
    rows = []
    for v in victims:
        p = body_point(v)
        rows.append(_row(v, [(name, escape_along(p, d, occluders))
                             for name, d in dirs]))
    return rows


def _row(v: dict, escapes: list) -> dict:
    """One victim's verdict from its per-direction escape distances."""
    best_name, best = min(escapes, key=lambda ne: ne[1])
    return {
        "id": int(v.get("id", -1)),
        "cohort": v.get("cohort", ""),
        "visibility": v.get("visibility", ""),
        "verdict": verdict_for(best),
        "escape_m": None if best == float("inf") else round(best, 3),
        "best_dir": best_name,
        "open_dirs": sum(1 for _n, e in escapes if e <= CLEAR_M),
        "dirs": len(escapes),
        "rubble_depth_m": v.get("rubble_depth_m"),
    }


# ---------------------------------------------------------------------------
# Backend B — the real geometry, inside Kit
# ---------------------------------------------------------------------------
#: Categories whose geometry is gaps at every scale. Anything else a ray meets
#: is a wall. `debris_fragment` is what `mesh_damage` names the Voronoi cells it
#: cuts a building into, so a collapsed building reads porous here for the same
#: reason `cut` makes it porous in the analytic model.
POROUS_CATS = frozenset(("debris", "debris_pile", "debris_fragment", "rubble",
                         "victim"))

#: Step past a surface before casting again, so the next cast does not
#: re-report the one just hit.
EPS_M = 0.02
#: Give up after this many surfaces on one bearing; a ray that has crossed
#: thirty is not finding daylight.
MAX_SURFACES = 30


def _hit_is_porous(stage, path: str) -> bool:
    """Walk up from a hit prim to the nearest `assetCategory` and read it."""
    prim = stage.GetPrimAtPath(str(path))
    while prim and prim.IsValid():
        cat = prim.GetCustomDataByKey("assetCategory")
        if cat:
            return str(cat) in POROUS_CATS
        parent = prim.GetParent()
        if not parent or parent.GetPath() == prim.GetPath():
            break
        prim = parent
    return False


def check_on_stage(stage, victims: list, scene_scale_factor: float = 1.0) -> list:
    """The same verdict, measured against the colliders that are really there.

    Same definition, different query: march outward from the body one surface
    at a time and stop at the first clear run of `OPEN_M`. The distance to the
    surface that run starts behind is the escape distance — for a victim
    already in the open, the first cast finds nothing within `OPEN_M` and the
    answer is 0.

    Only `raycast_closest` is used, deliberately. Pairing entry and exit faces
    would give the material thickness rather than the boundary, and it needs
    hit normals and closed meshes — neither of which a Voronoi cell soup
    reliably provides. Where open air begins is the same boundary and needs
    neither.
    """
    try:
        import carb
        from omni.physx import get_physx_scene_query_interface
    except Exception:
        print("[findability] PhysX scene query unavailable — no stage check")
        return []
    sq = get_physx_scene_query_interface()
    ssf = float(scene_scale_factor) or 1.0
    dirs = directions()

    def escape(p, d):
        t = 0.0
        for _ in range(MAX_SURFACES):
            o = tuple(p[i] + d[i] * t for i in range(3))
            hit = sq.raycast_closest(
                carb.Float3(o[0] * ssf, o[1] * ssf, o[2] * ssf),
                carb.Float3(*d), max(REACH_M - t, 0.0) * ssf)
            if not (hit and hit.get("hit")):
                return t                       # open air from here out
            gap = float(hit["distance"]) / ssf
            if gap >= OPEN_M:
                return t                       # the run of daylight starts here
            if not _hit_is_porous(stage, hit.get("collision", "")):
                return float("inf")
            t += gap + EPS_M
            if t >= REACH_M:
                break
        return float("inf")

    rows = []
    for v in victims:
        p = body_point(v)
        rows.append(_row(v, [(name, escape(p, d)) for name, d in dirs]))
    return rows


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def summarize(rows: list) -> dict:
    by = {k: 0 for k in VERDICTS}
    for r in rows:
        by[r["verdict"]] = by.get(r["verdict"], 0) + 1
    return {"total": len(rows), "by_verdict": by,
            "buried": [r["id"] for r in rows if r["verdict"] == "buried"],
            "pass": not any(r["verdict"] == "buried" for r in rows)}


def format_report(rows: list, title: str = "") -> str:
    s = summarize(rows)
    out = [f"findability {title}".rstrip(),
           f"  {s['total']} victim(s): {s['by_verdict']}  "
           f"-> {'PASS' if s['pass'] else 'FAIL'}",
           "   id  cohort         verdict   escape_m   open  best bearing"]
    for r in sorted(rows, key=lambda r: (VERDICTS.index(r["verdict"]), r["id"]),
                    reverse=True):
        esc = "opaque" if r["escape_m"] is None else f"{r['escape_m']:6.2f}"
        out.append(f"  {r['id']:3d}  {r['cohort']:<14s} {r['verdict']:<9s} "
                   f"{esc:>8s}  {r['open_dirs']:3d}/{r['dirs']:<3d}  "
                   f"{r['best_dir']}")
    return "\n".join(out)


def check_placed(stage, victims: list, scene_scale_factor: float = 1.0,
                 title: str = "") -> list:
    """Run the stage check, print it, and stamp each victim with its verdict.

    Called at the end of `targets.place`, so the ground truth a search run is
    scored against carries "was this person findable at all" beside "where were
    they" — which is the difference between a recall number that means
    something and one that does not.
    """
    rows = check_on_stage(stage, victims, scene_scale_factor)
    if not rows:
        return []
    for v, r in zip(victims, rows):
        v["findability"] = {k: r[k] for k in
                            ("verdict", "escape_m", "open_dirs", "best_dir")}
    print(format_report(rows, title))
    return rows


# ---------------------------------------------------------------------------
# Standalone report mode — the gate, without Isaac
# ---------------------------------------------------------------------------
#: Offline footprint for a building, matching `tests/snapshot.HOUSE_FALLBACK`
#: so a CLI run and a snapshot run pack the same blocks.
HOUSE_FALLBACK = [15.0, 15.0]


def offline_scene(preset: str, seed=None, target_seed=None):
    """Build *preset* on the host and survey it. ``(config, survey, victims)``.

    All three stages, no Nucleus and no stage: footprints come from
    `fallback_sizes`, which is what makes this runnable on a laptop over a
    whole seed sweep. The cost is that a building only reads `cut` when the
    disaster swapped an ARCHETYPE into it — a live mesh fracture happens on the
    stage and cannot be seen from here — so an offline run understates how many
    buildings are really wrecked. It never overstates it, so a `pass` here is
    not a claim the stage will fail.
    """
    import compile_disaster as cd
    import generate_scene as gs
    import scene_generator as sg
    import targets as T

    cfg = cd.load_scene_config(cd.resolve_config_path(preset))
    if seed is not None:
        cfg["seed"] = int(seed)
    if target_seed is not None:
        cfg.setdefault("targets", {})["seed"] = int(target_seed)
    cfg["measure_usds"] = False
    cfg.setdefault("fallback_sizes", {}).setdefault("house", list(HOUSE_FALLBACK))
    resolver = sg._make_resolver(cfg, cache=False)
    placements, layout, _ = gs.build_scene(cfg, resolver)
    survey = T.survey_from_placements(
        placements, layout, resolver,
        str((cfg.get("disaster") or {}).get("type", "none")))
    victims = T.sample_targets(survey, cfg)
    settle_offline(victims, occluders_from_survey(survey))
    return cfg, survey, victims


def main(argv=None) -> int:
    import argparse
    import contextlib
    import io
    import json
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default="urban_quake_tiny",
                    help="preset or low-level config name")
    ap.add_argument("--seeds", default="",
                    help="comma-separated scene seeds; default is the config's")
    ap.add_argument("--target-seeds", default="",
                    help="comma-separated targets.seed values, to re-roll the "
                         "people without rebuilding the city")
    ap.add_argument("--json", default="", help="write the rows here")
    args = ap.parse_args(argv)

    seeds = [int(s) for s in args.seeds.split(",") if s.strip()] or [None]
    tseeds = [int(s) for s in args.target_seeds.split(",") if s.strip()] or [None]
    doc, ok = [], True
    for seed in seeds:
        for tseed in tseeds:
            # The generator narrates every pass to stdout; the report is the
            # output here.
            with contextlib.redirect_stdout(io.StringIO()):
                cfg, survey, victims = offline_scene(args.config, seed, tseed)
            rows = check(victims, occluders_from_survey(survey))
            tag = f"{args.config} seed={cfg.get('seed')}" + (
                f" targets.seed={tseed}" if tseed is not None else "")
            print(format_report(rows, tag))
            print()
            ok = ok and summarize(rows)["pass"]
            doc.append({"config": args.config, "seed": cfg.get("seed"),
                        "target_seed": tseed, "summary": summarize(rows),
                        "rows": rows,
                        "xy": [[v["x"], v["y"]] for v in victims]})
    if len(doc) > 1:
        same = {json.dumps(d["xy"]) for d in doc}
        print(f"diversity: {len(same)} distinct placements over {len(doc)} runs")
        ok = ok and len(same) == len(doc)
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(doc, fh, indent=2)
        print(f"-> {args.json}")
    print("GATE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
