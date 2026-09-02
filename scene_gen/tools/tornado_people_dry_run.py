"""tornado_people_dry_run.py — the tornado CASUALTY plan, without Isaac Sim.

    python3 tools/tornado_people_dry_run.py --config suburb_tornado_250 \\
        --out-json ~/raven_previews/suburb_tornado_250_people.json \\
        --overlay ~/raven_previews/suburb_tornado_250_people_overlay.png

WHY THIS EXISTS
---------------
The same trade `tornado_png.py` makes, for the people pass:
`disaster.tornado_people.plan_people(cfg, ctx, rng)` imports nothing from
`pxr` (only `math`/`os`/`random`) and every `ctx` input it reads is buildable
from `fence_png.build()`'s own return dict plus the same pure functions
`tornado_png.py` already calls — so the whole casualty plan runs on a bare
host, no Isaac Sim, no Nucleus. The ONE thing that needs a live stage —
`deck_points`, measured off the BAKED WRECK ARCHETYPES via
`Usd.TraverseInstanceProxies` + `BBoxCache` — is documented OPTIONAL in
`plan_people`'s own docstring and degrades to the flat per-level
`DEBRIS_Z_M` deck, exactly the module's own bench/host-test convention. The
real Isaac build's population can differ slightly from this preview because
of that one gap — re-validate the two picked casualties against the real
`PEOPLE_JSON` before flying (see `_plans/raven_test_scene_runbook.md`).

WHAT THIS DOES
--------------
1. Builds the layout + damage ladder + plank field + scour relief exactly the
   way `tornado_png.py` does (same seeding, same pure functions).
2. Runs `tornado_people.plan_people` on top of it and writes the casualty
   records to `--out-json`.
3. Prints every casualty as `idx, x, y, pose, occlusion, covered_frac`.
4. Picks two casualties — one fully EXPOSED (`occlusion: none`), one
   PARTIALLY COVERED (`covered_frac` in `[--partial-lo, --partial-hi]`,
   default `[0.30, 0.55]`) — either the ones given by `--pick-exposed IDX` /
   `--pick-partial IDX`, or, when not given, the one of each kind NEAREST THE
   PLATE CENTRE (a search 20 m from a casualty near the plate edge is more
   likely to run off the plate or into the boundary ring than one nearer the
   middle).
5. For each picked casualty, searches a ring `--spawn-dist-m` out for the
   point with the LOWEST damage intensity (i.e. off the debris-swept
   centreline) that is not inside any house footprint, and prints the
   resulting `SPAWN_CONFIGS` JSON, facing each drone at its casualty.
6. Optionally (`--overlay`) draws the track PNG with every casualty coloured
   by occlusion, the two picks ringed, and both spawns + facing arrows.

RUN
    uv run --with numpy --with matplotlib --with pyyaml --with shapely \\
        --with scikit-learn python3 tools/tornado_people_dry_run.py \\
        --config suburb_tornado_250 --out-json /tmp/people.json
(`--overlay` additionally needs `matplotlib`, already in that list.)
"""

import argparse
import json
import math
import os
import random
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

# `suburb_scene`/`scene_generator` import `pxr` at module scope; nothing this
# file calls ever opens a USD (`measure_usds` is forced off by `fence_png`),
# so the same stub `fence_png.py` installs for itself is enough. Applied here
# too, defensively, in case this module is ever imported before `fence_png`.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
           "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.UsdPhysics"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt",
           "UsdPhysics"):
    setattr(sys.modules["pxr"], _n, types.ModuleType(_n))

_TRACK_YAWED = ("leveled", "swept")
_TREE_TRACK_YAWED = ("leaning", "fallen", "snapped")
_CANOPY_R_M = {"pristine": 4.2, "limbed": 2.6, "leaning": 3.0}
_OCC_COLOUR = {
    "none": "#2e7d32", "feet_shins": "#9ccc65", "legs": "#c0ca33",
    "legs_hips": "#fdd835", "midriff": "#fb8c00", "torso": "#f4511e",
    "torso_head": "#e53935", "head_only": "#8e24aa", "upper_body": "#5e35b1",
    "all_but_head": "#3949ab", "all_but_feet": "#1e88e5", "banded": "#00897b",
    "flank": "#00acc1",
}


# ---------------------------------------------------------------------------
# Pure functions — no scene, no `pxr`, no I/O. Unit-tested directly.
# ---------------------------------------------------------------------------

def yaw_facing(px, py, tx, ty):
    """`(yaw_deg, quat_xyzw)` for a Z-up yaw that points FROM `(px, py)`
    TOWARD `(tx, ty)` — 0 deg = +X / East, matching `disaster.tornado`'s own
    `heading_deg` convention and `SPAWN_CONFIGS`'s `orient` field (a Z-only
    rotation: `[0, 0, sin(yaw/2), cos(yaw/2)]`)."""
    yaw = math.degrees(math.atan2(ty - py, tx - px))
    half = math.radians(yaw) / 2.0
    return yaw, [0.0, 0.0, math.sin(half), math.cos(half)]


def spawn_config_entry(x, y, yaw_deg, ndigits_xy=2, ndigits_q=4):
    """One ready-to-paste `SPAWN_CONFIGS` list entry."""
    half = math.radians(yaw_deg) / 2.0
    return {
        "x_m": round(float(x), ndigits_xy),
        "y_m": round(float(y), ndigits_xy),
        "orient": [0.0, 0.0, round(math.sin(half), ndigits_q),
                  round(math.cos(half), ndigits_q)],
    }


def pick_casualties(people, region_center=(0.0, 0.0), pick_exposed=None,
                    pick_partial=None, partial_lo=0.30, partial_hi=0.55):
    """Choose (idx_exposed, idx_partial) out of a plain `people` list of
    dicts carrying at least `x`, `y`, `occlusion`, `covered_frac`.

    Explicit `pick_exposed`/`pick_partial` indices are HONOURED as given
    (bounds-checked; a mismatch against the expected criteria is reported,
    not refused — the caller asked for a specific casualty on purpose).
    Absent a pick, the candidate nearest `region_center` is chosen — a
    spawn ring near the plate edge is more likely to clip the boundary or a
    house than one nearer the middle. Raises `ValueError` (with the reason)
    when no candidate exists and none was given explicitly.
    """
    n = len(people)

    def _dist2(i):
        p = people[i]
        return (float(p["x"]) - region_center[0]) ** 2 + \
               (float(p["y"]) - region_center[1]) ** 2

    warnings = []

    if pick_exposed is None:
        cands = [i for i, p in enumerate(people) if p.get("occlusion") == "none"]
        if not cands:
            raise ValueError("no casualty with occlusion == 'none' to auto-pick")
        idx_exposed = min(cands, key=_dist2)
    else:
        idx_exposed = int(pick_exposed)
        if not (0 <= idx_exposed < n):
            raise ValueError(
                "--pick-exposed {0} out of range (0..{1})".format(idx_exposed, n - 1))
        if people[idx_exposed].get("occlusion") != "none":
            warnings.append(
                "--pick-exposed {0} has occlusion={1!r}, not 'none' — "
                "honoured anyway".format(idx_exposed,
                                         people[idx_exposed].get("occlusion")))

    if pick_partial is None:
        cands = [i for i, p in enumerate(people)
                 if partial_lo <= float(p.get("covered_frac", -1.0)) <= partial_hi]
        if not cands:
            raise ValueError(
                "no casualty with covered_frac in [{0}, {1}] to "
                "auto-pick".format(partial_lo, partial_hi))
        idx_partial = min(cands, key=_dist2)
    else:
        idx_partial = int(pick_partial)
        if not (0 <= idx_partial < n):
            raise ValueError(
                "--pick-partial {0} out of range (0..{1})".format(idx_partial, n - 1))
        cf = float(people[idx_partial].get("covered_frac", -1.0))
        if not (partial_lo <= cf <= partial_hi):
            warnings.append(
                "--pick-partial {0} has covered_frac={1}, outside [{2}, {3}] "
                "— honoured anyway".format(idx_partial, cf, partial_lo, partial_hi))

    for w in warnings:
        print("[tornado_people_dry_run] WARNING: {0}".format(w))
    return idx_exposed, idx_partial


def pick_spawn_point(px, py, spawn_dist_m, region, in_any_house, cross_offset,
                     intensity_at, n_angles=72):
    """Best point on the circle of radius `spawn_dist_m` around `(px, py)`:
    inside `region` (2 m margin), clear of every house footprint
    (`in_any_house(x, y) -> bool`), ranked by LOWEST `intensity_at(x, y)`
    first (open ground, off the debris-swept centreline) then by the
    LARGEST `|cross_offset(x, y)|` as a tiebreak (farther off the
    meandered centreline). Returns `{"x", "y", "intensity", "cross_off",
    "angle_deg"}`, or `None` if nothing on the circle qualifies (the caller
    should widen the circle or move the casualty).
    """
    best = None
    for ai in range(n_angles):
        ang = 2.0 * math.pi * ai / n_angles
        x = px + spawn_dist_m * math.cos(ang)
        y = py + spawn_dist_m * math.sin(ang)
        if not (region[0] + 2.0 <= x <= region[2] - 2.0
                and region[1] + 2.0 <= y <= region[3] - 2.0):
            continue
        if in_any_house(x, y):
            continue
        it = float(intensity_at(x, y))
        off = abs(float(cross_offset(x, y)))
        key = (it, -off)
        if best is None or key < best[0]:
            best = (key, {"x": x, "y": y, "intensity": it, "cross_off": off,
                          "angle_deg": math.degrees(ang)})
    return best[1] if best else None


# ---------------------------------------------------------------------------
# The scene-dependent half — everything below actually builds a layout.
# ---------------------------------------------------------------------------

def build_ctx(config_name, seed_override=None):
    """Run the layout + damage + plank + relief pipeline (identical to
    `tornado_png.py`) and return `(scene, cfg_all, tcfg, region, seed, inten,
    throw_deg, houses, wrecks, intact, people_ctx, pools)` — everything
    `plan_people` needs plus everything the CLI prints/plots afterward."""
    import numpy as np
    import fence_png
    import suburb_scene as ss
    from disaster import planks, scour_relief as srl
    from disaster import tornado as tn
    from disaster import tornado_people as tpp

    scene = fence_png.build(seed=seed_override, config_name=config_name,
                            house_instances=[])
    cfg_all = scene["cfg"]
    tcfg = tn.resolve_cfg(cfg_all)
    reg = (cfg_all.get("layout") or {}).get("region_m") or [250.0, 250.0]
    rw, rh = float(reg[0]), float(reg[1])
    region = (-rw / 2.0, -rh / 2.0, rw / 2.0, rh / 2.0)
    seed = int(scene["seed"])

    inten = tn.intensity_field(tcfg, region, np.random.default_rng(seed + 23))
    throw_deg = float(tcfg["heading_deg"]) + float(tcfg["curl_deg"])

    fp_by_style = {e["style"]: max(e["w"], e["d"])
                   for e in ss.modular_catalogue(cfg_all)}
    drng = random.Random(seed + 5)
    wrecks, intact, houses = [], [], []
    for h in scene["house_instances"]:
        fp = fp_by_style.get(h.get("style"), 12.0)
        houses.append((float(h["x"]), float(h["y"]), fp))
        it = float(inten(h["x"], h["y"]))
        level = tn.house_level_for_intensity(it, drng)
        if level == "pristine":
            intact.append((h["x"], h["y"]))
        else:
            wrecks.append((h["x"], h["y"], fp, it, level))

    trng = random.Random(seed + 71)
    canopies = []
    tree_placements = [q for q in scene["placements"]
                       if "tree" in str(q.get("category", ""))]
    for q in tree_placements:
        x, y = float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0))
        sp = os.path.splitext(os.path.basename(str(q.get("usd", ""))))[0]
        it = float(inten(x, y))
        level, _yaw, _info = tn.tree_level_and_yaw(
            it, trng, sp, x, y, region, track_yaw_deg=throw_deg,
            base_yaw_deg=float(q.get("yaw_deg", 0.0)),
            track_yawed=_TREE_TRACK_YAWED)
        cr = _CANOPY_R_M.get(level)
        if cr:
            canopies.append((x, y, cr))

    road_pts = []
    for e in (getattr(scene["net"], "edges", {}) or {}).values():
        pts = list(getattr(e, "pts", ()) or ())
        for i in range(0, max(0, len(pts) - 1), 2):
            ax, ay = float(pts[i][0]), float(pts[i][1])
            if inten(ax, ay) < 0.15:
                continue
            bx, by = float(pts[i + 1][0]), float(pts[i + 1][1])
            road_pts.append((ax, ay, math.degrees(math.atan2(by - ay, bx - ax))))

    prng = random.Random(seed + 77)
    specs = []
    for (hx, hy, fp, it, _lv) in wrecks:
        specs += planks.scatter_from_wreck(hx, hy, fp, it, throw_deg,
                                           float(tcfg["throw_m"]), prng,
                                           n_pieces=140)
    specs, _n1 = planks.clip_to_region(specs, region)
    track_specs = planks.scatter_over_region(region, inten, throw_deg, prng,
                                             per_100m2=4.5, cell_m=10.0)
    track_specs, _n2 = planks.clip_to_region(track_specs, region)
    plank_specs = specs + track_specs

    kn_r = srl.knobs_from_env()
    cov = tn.scour_coverage(tcfg, region, np.random.default_rng(seed + 31),
                            intensity=inten,
                            gamma=tn.knobs_from_env(max(rw, rh))["gamma"],
                            islands=tn.knobs_from_env(max(rw, rh))["islands"])
    corr = [(list(getattr(e, "pts", ()) or ()), float(getattr(e, "half_w", 0.0)))
            for e in (getattr(scene["net"], "edges", {}) or {}).values()]
    pave_at = srl.pavement_mask(corr, region) if corr else None
    # Pool masking skipped, same reasoning `people_dry_run.py` documented:
    # `disaster.ground.skip_rects` wants polygon rings, not boxes, and the
    # only effect of skipping it is a slightly more permissive relief
    # scatter near a pool coping — cosmetic, does not touch the casualty
    # placements below.
    keep = [(x, y, 0.55 * fp + 0.8)
            for (x, y, fp, _it, lv) in wrecks if lv not in ("leveled", "swept")]

    def _skip(px, py):
        return any(abs(px - hx) <= hr and abs(py - hy) <= hr
                   for (hx, hy, hr) in keep)

    relief = srl.scatter(tcfg, region, cov, random.Random(seed + 61),
                         flow_deg=float(tcfg["heading_deg"]) + float(tcfg["curl_deg"]),
                         pavement_at=pave_at, skip=_skip, knobs=kn_r)

    pools = scene["pools"]
    humans = pools.load(ss._raw_pool(cfg_all, "humans"))
    humans = [u for u in humans if "posed" not in u.lower()] or humans

    ctx = {
        "wrecks": [{"x": w[0], "y": w[1], "fp": w[2], "intensity": w[3],
                    "level": w[4]} for w in wrecks],
        "intact": intact,
        "road_pts": road_pts,
        "throw_deg": throw_deg,
        "region": region,
        "plank_specs": plank_specs,
        "deck_points": [],
        "intensity_at": inten,
        "canopies": canopies,
        "blockers": tpp.relief_blockers(relief),
        "humans": humans,
        "resolver": scene["res"],
        "asset_pools": pools,
    }
    return {
        "scene": scene, "cfg_all": cfg_all, "tcfg": tcfg, "region": region,
        "seed": seed, "inten": inten, "throw_deg": throw_deg,
        "houses": houses, "wrecks": wrecks, "intact": intact, "ctx": ctx,
        "tn": tn, "tpp": tpp, "config_name": config_name,
    }


def run_plan_people(built):
    """`(people_records, meta)` — the actual `plan_people` call."""
    tpp = built["tpp"]
    cfg_all = built["cfg_all"]
    pcfg = tpp.resolve_cfg(cfg_all)
    rng = random.Random(built["seed"] + 91)
    _p_humans, _p_debris, p_recs = tpp.plan_people(pcfg, built["ctx"], rng)
    meta = {"seed": built["seed"], "scene_config": built.get("config_name", ""),
            "epoch_min": pcfg.get("epoch_min"),
            "track_deg": float(built["tcfg"]["heading_deg"]),
            "throw_deg": built["throw_deg"],
            "note": "HOST DRY RUN (tornado_people_dry_run.py) — no stage, no "
                    "Isaac Sim; deck_points empty (flat DEBRIS_Z_M deck), "
                    "humans list is the config's own usd paths (never "
                    "opened; measure_usds=False)."}
    return p_recs, pcfg, meta


def print_candidates(people):
    print("[tornado_people_dry_run] {0} casualt(ies):".format(len(people)))
    print("  {0:>3}  {1:>9}  {2:>9}  {3:<18}  {4:<12}  {5:>6}".format(
        "idx", "x", "y", "pose", "occlusion", "cov_f"))
    for i, p in enumerate(people):
        print("  {0:>3}  {1:>9.2f}  {2:>9.2f}  {3:<18}  {4:<12}  "
              "{5:>6.2f}".format(i, p["x"], p["y"], p.get("pose", ""),
                                 p.get("occlusion", ""),
                                 float(p.get("covered_frac", 0.0))))


def draw_overlay(built, people, idx_exposed, idx_partial, spawn_a, spawn_b,
                 out_png):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tn = built["tn"]
    tcfg, region = built["tcfg"], built["region"]
    rw, rh = region[2] - region[0], region[3] - region[1]
    inten = built["inten"]

    fig, ax = plt.subplots(figsize=(11, 11 * rh / rw))
    n = 220
    xs = np.linspace(region[0], region[2], n)
    ys = np.linspace(region[1], region[3], n)
    grid = np.array([[inten(x, y) for x in xs] for y in ys])
    im = ax.imshow(grid, origin="lower",
                   extent=(region[0], region[2], region[1], region[3]),
                   cmap="pink_r", vmin=0.0, vmax=1.0, zorder=0,
                   interpolation="bilinear")
    fig.colorbar(im, ax=ax, shrink=0.7, pad=0.02).set_label("intensity (EF proxy)")

    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    ox, oy = tcfg["origin_m"]
    half = 0.5 * float(tcfg["width_m"])
    reach = math.hypot(rw, rh)
    cl, ed_l, ed_r = [], [], []
    for k in range(-120, 121):
        a = reach * k / 120.0
        probe = (ox + ux * a, oy + uy * a)
        _a2, c2 = to_track(*probe)
        off = -c2
        cl.append((probe[0] + vx * off, probe[1] + vy * off))
        ed_l.append((probe[0] + vx * (off + half), probe[1] + vy * (off + half)))
        ed_r.append((probe[0] + vx * (off - half), probe[1] + vy * (off - half)))
    for pts, style in ((cl, dict(color="crimson", lw=1.6, ls="-")),
                       (ed_l, dict(color="crimson", lw=0.9, ls="--")),
                       (ed_r, dict(color="crimson", lw=0.9, ls="--"))):
        ax.plot([p[0] for p in pts], [p[1] for p in pts], zorder=4, **style)

    by_occ = {}
    for p in people:
        by_occ.setdefault(p.get("occlusion", "?"), []).append(p)
    for occ, pts in sorted(by_occ.items()):
        ax.scatter([p["x"] for p in pts], [p["y"] for p in pts], s=70,
                  c=_OCC_COLOUR.get(occ, "#666666"), marker="o",
                  edgecolors="k", linewidths=0.6, zorder=5,
                  label="{0} ({1})".format(occ, len(pts)))

    for idx, tag in ((idx_exposed, "A"), (idx_partial, "B")):
        p = people[idx]
        ax.scatter([p["x"]], [p["y"]], s=260, facecolors="none",
                  edgecolors="blue", linewidths=2.2, zorder=6)
        ax.annotate(tag, (p["x"], p["y"]), fontsize=11, color="blue",
                   fontweight="bold", zorder=7,
                   xytext=(6, 6), textcoords="offset points")

    for tag, spawn, idx in (("robot_1", spawn_a, idx_exposed),
                            ("robot_2", spawn_b, idx_partial)):
        p = people[idx]
        ax.scatter([spawn["x"]], [spawn["y"]], s=140, marker="^",
                  c="dodgerblue", edgecolors="k", linewidths=0.8, zorder=8)
        yaw, _q = yaw_facing(spawn["x"], spawn["y"], p["x"], p["y"])
        th = math.radians(yaw)
        alen = 0.08 * max(rw, rh)
        ax.arrow(spawn["x"], spawn["y"], alen * math.cos(th), alen * math.sin(th),
                 width=alen * 0.08, color="dodgerblue", zorder=8,
                 length_includes_head=True)
        ax.annotate(tag, (spawn["x"], spawn["y"]), fontsize=10, color="navy",
                   fontweight="bold", zorder=9, xytext=(6, -12),
                   textcoords="offset points")
        ax.plot([spawn["x"], p["x"]], [spawn["y"], p["y"]], color="dodgerblue",
                lw=0.8, ls=":", zorder=3)

    ax.add_patch(plt.Rectangle((region[0], region[1]), rw, rh, fill=False,
                               ec="0.35", lw=1.4, zorder=2))
    ax.set_xlim(region[0] - rw * 0.05, region[2] + rw * 0.05)
    ax.set_ylim(region[1] - rh * 0.05, region[3] + rh * 0.05)
    ax.set_aspect("equal")
    ax.set_title("{0} casualties by occlusion, seed {1}\n"
                "A=idx{2} (robot_1 target)   B=idx{3} (robot_2 target)".format(
                    len(people), built["seed"], idx_exposed, idx_partial))
    ax.legend(loc="upper left", fontsize=7, framealpha=0.9, ncol=2)
    out = os.path.expanduser(out_png)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("[tornado_people_dry_run] overlay -> {0}".format(out))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="suburb_tornado_250",
                    help="preset name (default: suburb_tornado_250)")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the layout seed")
    ap.add_argument("--out-json", required=True,
                    help="write the casualty records here (tornado_people.write_records format)")
    ap.add_argument("--overlay", default=None,
                    help="also draw a track+casualty PNG here (optional)")
    ap.add_argument("--pick-exposed", type=int, default=None,
                    help="casualty index for robot_1 (must be occlusion=none "
                         "to be a sensible pick, but any index is honoured)")
    ap.add_argument("--pick-partial", type=int, default=None,
                    help="casualty index for robot_2 (a partially-covered one)")
    ap.add_argument("--partial-lo", type=float, default=0.30)
    ap.add_argument("--partial-hi", type=float, default=0.55)
    ap.add_argument("--spawn-dist-m", type=float, default=20.0,
                    help="distance from each picked casualty to its drone spawn")
    args = ap.parse_args(argv)

    built = build_ctx(args.config, seed_override=args.seed)
    people, _pcfg, meta = run_plan_people(built)

    out_json = os.path.expanduser(args.out_json)
    built["tpp"].write_records(out_json, people, meta=meta)
    print("[tornado_people_dry_run] {0} -> {1}".format(len(people), out_json))

    print_candidates(people)

    try:
        idx_exposed, idx_partial = pick_casualties(
            people, region_center=(0.0, 0.0),
            pick_exposed=args.pick_exposed, pick_partial=args.pick_partial,
            partial_lo=args.partial_lo, partial_hi=args.partial_hi)
    except ValueError as exc:
        print("[tornado_people_dry_run] PICK FAILED: {0}".format(exc))
        return 1

    houses = built["houses"]

    def in_any_house(x, y, margin=1.5):
        for (hx, hy, fp) in houses:
            r = 0.5 * fp + margin
            if abs(x - hx) <= r and abs(y - hy) <= r:
                return True
        return False

    to_track, _uv, _vv = built["tn"].frame(built["tcfg"])

    def cross_offset(x, y):
        _a, c = to_track(x, y)
        return c

    spawns = {}
    for tag, idx in (("robot_1", idx_exposed), ("robot_2", idx_partial)):
        p = people[idx]
        sp = pick_spawn_point(p["x"], p["y"], args.spawn_dist_m, built["region"],
                              in_any_house, cross_offset, built["inten"])
        if sp is None:
            print("[tornado_people_dry_run] no clear spawn point {0} m from "
                  "casualty {1} — widen --spawn-dist-m".format(
                      args.spawn_dist_m, idx))
            return 1
        yaw, _q = yaw_facing(sp["x"], sp["y"], p["x"], p["y"])
        spawns[tag] = (sp, yaw, idx)
        print("[tornado_people_dry_run] {0}: casualty idx={1} ({2:.2f},{3:.2f}) "
              "occlusion={4} covered_frac={5:.2f} -> spawn ({6:.2f},{7:.2f}) "
              "intensity={8:.2f} yaw={9:.1f} deg".format(
                  tag, idx, p["x"], p["y"], p.get("occlusion"),
                  float(p.get("covered_frac", 0.0)), sp["x"], sp["y"],
                  sp["intensity"], yaw))

    spawn_configs = [spawn_config_entry(spawns["robot_1"][0]["x"],
                                       spawns["robot_1"][0]["y"],
                                       spawns["robot_1"][1]),
                    spawn_config_entry(spawns["robot_2"][0]["x"],
                                       spawns["robot_2"][0]["y"],
                                       spawns["robot_2"][1])]
    print("[tornado_people_dry_run] SPAWN_CONFIGS = " + json.dumps(spawn_configs))

    if args.overlay:
        draw_overlay(built, people, idx_exposed, idx_partial,
                    spawns["robot_1"][0], spawns["robot_2"][0], args.overlay)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
