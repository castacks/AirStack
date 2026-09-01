#!/usr/bin/env python3
"""hurricane_cameras_png.py — the hurricane review cameras, without Isaac Sim.

    python3 scene_gen/tools/hurricane_cameras_png.py \
        --gt ~/hurricane_previews/V2_L3/GT_hurricane.json \
        --out ~/hurricane_previews/offline/cameras/cameras_L3.png
    python3 scene_gen/tools/hurricane_cameras_png.py \
        --gt ~/hurricane_previews/V2_L2/GT_hurricane.json \
        --out ~/hurricane_previews/offline/cameras/cameras_L2.png

WHY THIS EXISTS
---------------
STREAM C's brief (`.agents/skills/build-hurricane-scenes/SESSION_2026-08-31.md`
S3/S4): the review lead found `deep_water_obl.png`/`dry_inland_obl.png`/
`shoreline_obl.png` on `V2_L3` framing the finite terrain slab's own cut EDGE
hanging over the background HDRI ground, and none of the six review subjects
close enough to judge debris, house damage or trees. Whether a fix actually
moves a subject off the edge, and whether a new close subject's oblique looks
inward rather than outward, is answerable from the GROUND TRUTH JSON alone —
no render needed. This is that check, plotted.

WHAT IT IS NOT: a render. It shows WHERE the cameras point, not what the
material/lighting will look like.

THE SAME FUNCTIONS THE LAUNCHER CALLS
--------------------------------------
`select_review_subjects` is the single source of truth for the 11-subject
list (6 water-gradient + 5 close, deliverable B). `suburb_hurricane_launch_
script.py`'s snapshot block imports it from here (this file is on `sys.path`
via the `tools/` dir the launcher already inserts alongside `scene_gen`), so
a framing bug fixed by editing this module is fixed in the live scene too,
not just in the offline plot. `windward_azimuth_deg`, `densest_cluster` and
`flooded_street_point` are the pieces of that selection kept as free
functions so `test_review_points_cameras.py` can exercise each one against a
synthetic case, not only against whatever `V2_L2`/`V2_L3` happen to contain.

NO KIT NEEDED FOR THE SUBJECT SELECTION. `disaster.surge`/`disaster.
washaway`/`disaster.hurricane` import no `pxr` at module scope (the
`tornado_png.py:40-56` situation, verified again here 2026-08-31 for these
three modules specifically), so `select_review_subjects` and friends import
cleanly at module scope — including inside a LIVE Isaac Sim process, which
is exactly where the launcher needs them.

THE PLOTTING HALF IS DIFFERENT, AND LAZY ON PURPOSE. `simulation/isaac-sim/
utils/snapshots_rp.py` DOES import `pxr`/`carb` at module scope — it exists
to run inside Kit — so reaching its `_clear_azimuth`/`_cap_oblique_range`
(what draws an accurate frustum) needs the SAME stub idiom `tornado_png.py`
uses for `pxr`, extended to `carb`. Doing that UNCONDITIONALLY at import
time, the way `tornado_png.py` does it, would be fine for a tool that only
ever runs standalone -- but this module is also imported from INSIDE the
live launcher (for `select_review_subjects`), where `pxr`/`carb` are the
REAL Kit modules, and `tornado_png.py`'s own `setattr` is unconditional: run
it there and it clobbers the real `pxr.Gf`/`pxr.UsdGeom` for the rest of the
process. `_ensure_plot_deps()` therefore stubs (and imports matplotlib)
ONLY when first asked to plot, and only ever ADDS an attribute that is not
already there — a live Kit process's real `pxr`/`carb` come through
untouched, and a standalone run gets the same stub `tornado_png.py` would
have built unconditionally.
"""

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_ISAAC_UTILS = os.path.normpath(
    os.path.join(_SCENE_GEN, "..", "simulation", "isaac-sim", "utils"))
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _ISAAC_UTILS)

from disaster import surge as sgw                              # noqa: E402
from disaster import hurricane as hu                           # noqa: E402

np = None            # set by `_ensure_plot_deps` -- plotting only
plt = None
Polygon = None
srp = None           # `snapshots_rp`, reached through the guarded pxr/carb stub


def _ensure_plot_deps():
    """Import matplotlib and `snapshots_rp` (behind a GUARDED pxr/carb
    stub — see the module docstring) on first use. Idempotent. Every
    plotting function below calls this before touching `np`/`plt`/`Polygon`/
    `srp`; `select_review_subjects` and the other pure subject-selection
    helpers never call it and never need to.
    """
    global np, plt, Polygon, srp
    if srp is not None:
        return
    import types
    import numpy as _np
    for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.UsdGeom", "carb"):
        sys.modules.setdefault(_m, types.ModuleType(_m))
    pxr_mod = sys.modules["pxr"]
    for _n in ("Gf", "Sdf", "UsdGeom"):
        if not hasattr(pxr_mod, _n):
            setattr(pxr_mod, _n, types.SimpleNamespace())
    carb_mod = sys.modules["carb"]
    if not hasattr(carb_mod, "log_warn"):
        carb_mod.log_warn = lambda *a, **k: None

    import matplotlib as _matplotlib
    _matplotlib.use("Agg")
    import matplotlib.pyplot as _plt
    from matplotlib.patches import Polygon as _Polygon
    import snapshots_rp as _srp

    np, plt, Polygon, srp = _np, _plt, _Polygon, _srp

_HCOLOUR = {"pristine": "#2f7d32", "shingles_lost": "#9fbf4a",
            "cover_lost": "#c9d14a", "deck_panels_lost": "#d9b23d",
            "roof_stripped": "#e8a33d", "roof_collapsed": "#d4622c",
            "partial_collapse": "#a8342a", "leveled": "#5a0f0c",
            "swept": "#3d0f0c"}
_TCOLOUR = {"pristine": "#2f7d32", "defoliated": "#8a8f4a",
            "limbed": "#8bbf4a", "leaning": "#c9b23d",
            "fallen": "#a8632a", "snapped": "#5a2f17"}

# The 5 NEW close subjects (deliverable B). Elevation/aim height are shared
# defaults; range is per-subject per the brief. `azimuth_deg` is resolved
# per-subject at selection time (windward wall, up-flow side, along the
# road, tree-fall broadside) -- these are just the fallbacks/ranges.
_CLOSE_ELEV_DEG = 32.0
_CLOSE_AIM_H_M = 1.6
_CLOSE_RANGE_M = {
    "stripped_roof_house": 28.0, "collapsed_house": 28.0,
    "raft_field": 18.0, "fallen_tree": 20.0, "flooded_street": 40.0,
}


# ---------------------------------------------------------------------------
# shared, camera-geometry-free helpers
# ---------------------------------------------------------------------------

def windward_azimuth_deg(yaw_deg, level=None, variant=None):
    """Camera azimuth (this codebase's math convention: 0 = +X/east, CCW —
    `snapshots_rp.views_around`'s `azimuth_deg`) that frames a hurricane
    house archetype's WINDWARD, DAMAGED wall face-on.

    Grounded in the BAKE convention, not in a fresh call to
    `hurricane.wind_bearing_at` — deliberately, and this is a documented
    deviation from the literal review-camera brief. `bake_hurricane_
    archetypes_launch_script.py` bakes every archetype at placement yaw 0
    with `_BAKE_BEARING = 0.0`, so `hurricane_flow.wreck_building` always
    damages the wall `modular_house.build_building` faces -Y ("south") at
    yaw 0 — `hurricane_flow._bearing_of(0) == 180`, matching
    `_BAKE_BEARING + 180`. The launcher's `_WIND_YAWED` is only
    `("partial_collapse", "leveled")`: EVERY OTHER damaged house (including
    `roof_stripped`/`cover_lost`/`deck_panels_lost`/`roof_collapsed`, i.e.
    both of this module's `stripped_roof_house`/`collapsed_house` subjects
    in the common case) keeps its ORIGINAL city-layout placement yaw, not a
    wind-derived one — a fresh `wind_bearing_at(x, y)` call would point the
    camera at whichever wall happens to face that direction, which is not
    necessarily the damaged one. The RECORDED `yaw_deg` (`h["yaw_deg"]` /
    `houses[i]["yaw_deg"]`), whatever produced it, IS the placement yaw the
    archetype was actually referenced at, so `_bearing_of`'s own algebra
    applies uniformly regardless of level: that wall's world-facing compass
    bearing is `(180 - yaw_deg) % 360`, which converts to this module's
    math-angle convention as `(yaw_deg - 90) % 360`. A camera at that
    azimuth sits on the windward wall's own outward side, looking back at
    the damaged face.
    """
    # PER-HOUSE VARIANTS (H2b, 2026-08-31): `cover_lost`/`deck_panels_lost`
    # now reference one of four cardinal variants whose dropped bays sit on
    # the LOCAL side `variant` (`hurricane_flow._SIDE_BASE_BEARING`), chosen
    # per house by `hurricane_flow.windward_variant`. For those two levels
    # the damaged face is that side, not the bake's -Y wall: its world
    # compass bearing is `(base - yaw) % 360`, converted to this module's
    # math convention as `(90 - bearing) % 360`. Every other level, and any
    # GT record without a `variant` (older files), keeps the bake algebra.
    if variant is not None and level in ("cover_lost", "deck_panels_lost"):
        base = {"n": 0.0, "e": 90.0, "s": 180.0, "w": 270.0}.get(str(variant))
        if base is not None:
            bearing = (base - float(yaw_deg)) % 360.0
            return (90.0 - bearing) % 360.0
    return (float(yaw_deg) - 90.0) % 360.0


def elevation_to_height(dist_m, elev_deg, aim_h_m):
    """`obl_h` (metres) for an oblique `dist_m` out that looks at `aim_h_m`
    with elevation angle `elev_deg` below horizontal -- the inverse of the
    `atan2` `snapshots_rp._top_ray_ground_point` uses for its centre ray."""
    return float(aim_h_m) + float(dist_m) * math.tan(math.radians(elev_deg))


def densest_cluster(points, cell_m=6.0):
    """`(x, y, n)` — centroid and point count of the densest `cell_m` grid
    cell among `points` (each an `(x, y)` pair or `{"x":.., "y":..}` dict).
    `None` if `points` is empty.

    Grid-binning rather than a proper clustering algorithm (DBSCAN, etc.) on
    purpose: `washaway.raft_specs` draws BOTH a light background scatter and
    per-house TANGLES (1-3 pieces within 0.7 m, `_RAFT_TANGLE`), so "the
    densest cluster" only needs to find which small patch of plate has the
    most pieces in it, not to segment the whole field into named clusters.
    """
    pts = []
    for p in points:
        if isinstance(p, dict):
            pts.append((float(p["x"]), float(p["y"])))
        else:
            pts.append((float(p[0]), float(p[1])))
    if not pts:
        return None
    cells = {}
    for (x, y) in pts:
        key = (math.floor(x / cell_m), math.floor(y / cell_m))
        cells.setdefault(key, []).append((x, y))
    best_key = max(cells, key=lambda k: len(cells[k]))
    cell_pts = cells[best_key]
    xs = [p[0] for p in cell_pts]
    ys = [p[1] for p in cell_pts]
    return (sum(xs) / len(xs), sum(ys) / len(ys), len(cell_pts))


def flooded_street_point(houses, depth_fn, lo=0.3, hi=0.8, yaw_tol_deg=12.0,
                         min_pair_m=4.0, max_pair_m=60.0):
    """`(x, y, street_bearing_deg)` — the midpoint between two houses that
    plausibly face the SAME street frontage (near-identical or near-
    opposite original yaw — houses on either side of a road usually face
    each other, i.e. differ by ~180 deg) with the point between them under
    `lo`-`hi` m of water. `None` if no such pair exists.

    FALLBACK, not the primary path: the launcher's `binfo["net"]` street
    graph (`layout/suburb_net.py`'s `Network`/`Edge`) is REAL road geometry
    with actual carriageway centrelines and widths, but it lives only in the
    live Isaac stage's `binfo` dict and is never serialised into
    `GT_hurricane.json` — so this offline-verified path (and a launcher run
    where `net` is for any reason not threaded through) uses this house-pair
    heuristic instead, per the review-camera brief's own fallback clause.

    `houses` is `(x, y, yaw_deg)` tuples.
    """
    best, best_d = None, float("inf")
    hs = [(float(h[0]), float(h[1]), float(h[2])) for h in houses]
    n = len(hs)
    for i in range(n):
        x0, y0, yaw0 = hs[i]
        for j in range(i + 1, n):
            x1, y1, yaw1 = hs[j]
            dyaw = abs(((yaw0 - yaw1) + 180.0) % 360.0 - 180.0)
            if dyaw > yaw_tol_deg and abs(dyaw - 180.0) > yaw_tol_deg:
                continue
            d = math.hypot(x1 - x0, y1 - y0)
            if d < min_pair_m or d > max_pair_m:
                continue
            mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
            depth = depth_fn(mx, my)
            if not (lo <= depth <= hi):
                continue
            if d < best_d:
                best_d = d
                best = (mx, my, math.degrees(math.atan2(y1 - y0, x1 - x0)))
    return best


def inset_bounds(region, margin_m):
    """`region` shrunk by `margin_m` on every side, capped at 49% of each
    half-extent — the same cap `surge.review_points`'s `edge_margin_m` uses."""
    x0, y0, x1, y1 = region
    mx = min(margin_m, 0.49 * (x1 - x0))
    my = min(margin_m, 0.49 * (y1 - y0))
    return (x0 + mx, y0 + my, x1 - mx, y1 - my)


def edge_margin(region, x, y):
    """Distance from `(x, y)` to the NEAREST side of `region`."""
    x0, y0, x1, y1 = region
    return min(x - x0, x1 - x, y - y0, y1 - y)


# ---------------------------------------------------------------------------
# the shared subject list -- SAME function the launcher's snapshot block
# imports and calls
# ---------------------------------------------------------------------------

def _by_level(records, levels):
    return [r for r in records if r.get("level") in levels]


def select_review_subjects(gt, edge_margin_m=70.0, depth_fn=None):
    """The FULL review-camera subject list computed from a `GT_hurricane.
    json` dict (or the launcher's equivalent in-memory `region`/`scfg`/
    `_h_recs`/`_t_recs`, which carry the same fields) — 6 water-gradient
    subjects (`surge.review_points` plus `worst_house`/
    `deepest_flooded_house`) and 5 close ones (deliverable B).

    Returns `{name: dict}`. Every entry has `x`, `y`. The 5 close entries
    additionally carry `obl_dist`, `obl_h`, `aim_h`, `azimuth_deg` (already
    resolved to a specific number, not a launcher default) and, when a
    level bucket this plate simply does not have forced a fallback (L2 has
    no `partial_collapse` and no `fallen`/`snapped` tree — see the SESSION
    doc's "still wrong" #2), a human-readable `note`. A subject with NO
    qualifying record at all (e.g. no flooded street pair) is dropped
    entirely rather than emitted with a placeholder position.
    """
    region = tuple(gt["region"])
    scfg = gt["surge"]
    houses = gt.get("houses") or []
    trees = gt.get("trees") or []
    depth_fn = sgw.depth_at(scfg, region, None)

    out = {}
    for name, xy in sgw.review_points(scfg, region,
                                      edge_margin_m=edge_margin_m).items():
        out[name] = {"x": xy[0], "y": xy[1]}

    if houses:
        worst = max(houses, key=lambda r: hu.HOUSE_LEVELS.index(r["level"])
                   if r["level"] in hu.HOUSE_LEVELS else 99)
        out["worst_house"] = {"x": worst["x"], "y": worst["y"]}
        wet = [r for r in houses if float(r.get("water_depth_m", 0.0)) > 0.15]
        if wet:
            w0 = max(wet, key=lambda r: r["water_depth_m"])
            out["deepest_flooded_house"] = {"x": w0["x"], "y": w0["y"]}

    # -- close subjects 1/2: stripped_roof_house / collapsed_house ---------
    def _house_subject(name, primary, fallback_ladder):
        pool = _by_level(houses, primary)
        note = None
        for lv in fallback_ladder:
            if pool:
                break
            pool = _by_level(houses, (lv,))
            if pool:
                note = ("no {0} house on this plate; used the next most "
                        "severe available ({1})"
                        .format("/".join(primary), lv))
        if not pool:
            return
        h = min(pool, key=lambda r: abs(
            sgw._signed_depth_point(scfg, r["x"], r["y"])))
        dist = _CLOSE_RANGE_M[name]
        d = {"x": h["x"], "y": h["y"], "obl_dist": dist,
            "obl_h": elevation_to_height(dist, _CLOSE_ELEV_DEG,
                                         _CLOSE_AIM_H_M),
            "aim_h": _CLOSE_AIM_H_M,
            "azimuth_deg": windward_azimuth_deg(h["yaw_deg"], h.get("level"), h.get("variant")),
            "level": h["level"]}
        if note:
            d["note"] = note
        out[name] = d

    _house_subject("stripped_roof_house",
                  ("cover_lost", "deck_panels_lost", "roof_stripped"),
                  ("shingles_lost", "roof_collapsed", "partial_collapse",
                   "leveled"))
    _house_subject("collapsed_house", ("roof_collapsed", "partial_collapse"),
                  ("leveled", "roof_stripped", "deck_panels_lost",
                   "cover_lost"))

    # -- close subject 3: raft_field -----------------------------------------
    # `GT_hurricane.json` carries no raft specs (they are never written to
    # the ground truth), so the OFFLINE path always takes the documented
    # fallback: the deepest flooded house's up-flow (seaward) side, per
    # `washaway.raft_specs`'s own "clustered against standing obstacles"
    # construction (`surge._grad_deg` at the house = the bearing debris
    # arrives FROM). The live launcher instead calls `densest_cluster` on
    # the real `_rs` list when it is reachable — see the launcher's snapshot
    # block for which path actually ran.
    wet_h = [r for r in houses if float(r.get("water_depth_m", 0.0)) > 0.15]
    if wet_h:
        w0 = max(wet_h, key=lambda r: r["water_depth_m"])
        bearing = sgw._grad_deg(depth_fn, w0["x"], w0["y"])
        standoff = 8.0
        a = math.radians(bearing)
        rx = w0["x"] + standoff * math.cos(a)
        ry = w0["y"] + standoff * math.sin(a)
        dist = _CLOSE_RANGE_M["raft_field"]
        out["raft_field"] = {
            "x": rx, "y": ry, "obl_dist": dist,
            "obl_h": elevation_to_height(dist, _CLOSE_ELEV_DEG, 0.6),
            "aim_h": 0.6,
            # Camera further seaward (same bearing) than the raft, looking
            # LANDWARD -- so the pile is in the foreground with the house
            # that trapped it behind.
            "azimuth_deg": bearing % 360.0,
            "note": ("GT has no raft specs; approximated via the deepest "
                    "flooded house's up-flow side (fallback per brief)"),
        }

    # -- close subject 4: fallen_tree ----------------------------------------
    cand = _by_level(trees, ("fallen", "snapped"))
    note = None
    # A windthrown tree standing in deep floodwater is INVISIBLE — the depth
    # boost concentrates fallen/snapped in the flooded band, and VAL_L3's
    # fallen_tree frame was open water. Prefer a dry/shallow subject when the
    # caller can tell us the local depth (the launcher passes its `depth`
    # callable; the offline plot has none and keeps the old behaviour).
    if cand and depth_fn is not None:
        try:
            dry = [t for t in cand if float(depth_fn(t["x"], t["y"])) <= 0.3]
            if dry:
                cand = dry
            else:
                note = "every fallen/snapped tree stands in deep water"
        except Exception:
            pass
    if not cand:
        cand = _by_level(trees, ("leaning",))
        note = "no fallen/snapped tree on this plate; used a leaning one"
    if not cand:
        cand = [t for t in trees if t.get("level") not in (None, "pristine")]
        note = ("no fallen/snapped/leaning tree on this plate; used the "
                "most severe canopy damage available")
    if cand:
        t = max(cand, key=lambda t: edge_margin(region, t["x"], t["y"]))
        dist = _CLOSE_RANGE_M["fallen_tree"]
        d = {"x": t["x"], "y": t["y"], "obl_dist": dist,
            "obl_h": elevation_to_height(dist, _CLOSE_ELEV_DEG, 1.2),
            "aim_h": 1.2,
            # Broadside to the trunk (perpendicular to its own fall/lean
            # yaw), which shows a fallen tree's length rather than its cut
            # cross-section.
            "azimuth_deg": (float(t.get("yaw_deg", 0.0)) + 90.0) % 360.0,
            "level": t.get("level")}
        if note:
            d["note"] = note
        out["fallen_tree"] = d

    # -- close subject 5: flooded_street --------------------------------------
    house_tuples = [(h["x"], h["y"], h["yaw_deg"]) for h in houses]
    fs = flooded_street_point(house_tuples, depth_fn)
    if fs:
        mx, my, street_bearing = fs
        dist = _CLOSE_RANGE_M["flooded_street"]
        out["flooded_street"] = {
            "x": mx, "y": my, "obl_dist": dist,
            "obl_h": elevation_to_height(dist, _CLOSE_ELEV_DEG, 0.5),
            "aim_h": 0.5, "azimuth_deg": street_bearing % 360.0,
        }

    return out


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------

def _frustum_polygon(x, y, az_deg, obl_dist, focal_mm=18.0, length_scale=1.6):
    """A matplotlib-ready `[(x, y), ...]` triangle: the oblique camera's
    horizontal field of view, drawn from the camera position out past the
    subject. Horizontal FOV uses the same formula as the vertical one
    (`snapshots_rp._half_vfov_deg` — the aperture is square)."""
    _ensure_plot_deps()
    half_fov = math.radians(srp._half_vfov_deg(focal_mm))
    a = math.radians(az_deg)
    cam_x, cam_y = x + obl_dist * math.cos(a), y + obl_dist * math.sin(a)
    view = math.radians(az_deg + 180.0)
    length = obl_dist * length_scale
    left = view + half_fov
    right = view - half_fov
    p1 = (cam_x + length * math.cos(left), cam_y + length * math.sin(left))
    p2 = (cam_x + length * math.cos(right), cam_y + length * math.sin(right))
    return [(cam_x, cam_y), p1, p2]


def _resolve_camera(subject, region, avoid):
    """`(cam_x, cam_y, az_deg, obl_dist)` for one subject dict, running the
    SAME `_clear_azimuth`/`_cap_oblique_range` the launcher's `views_around`
    call does, so the plotted frustum is the frustum a render would use."""
    _ensure_plot_deps()
    x, y = float(subject["x"]), float(subject["y"])
    obl_dist = float(subject.get("obl_dist", 45.0))
    obl_h = float(subject.get("obl_h", 22.0))
    aim_h = float(subject.get("aim_h", 1.0))
    preferred = float(subject.get("azimuth_deg", 225.0))
    az = srp._clear_azimuth(x, y, avoid, obl_dist, 1.0, preferred,
                           region=region)
    d_use, _, _ = srp._cap_oblique_range(x, y, az, obl_dist, obl_h, aim_h,
                                        region)
    a = math.radians(az)
    return (x + d_use * math.cos(a), y + d_use * math.sin(a), az, d_use)


def plot_cameras(gt, out_path, edge_margin_m=70.0, title=None):
    _ensure_plot_deps()
    region = tuple(gt["region"])
    scfg = gt["surge"]
    houses = gt.get("houses") or []
    trees = gt.get("trees") or []
    avoid = [(t["x"], t["y"]) for t in trees
            if t.get("level") not in ("fallen", "snapped")]

    subjects = select_review_subjects(gt, edge_margin_m=edge_margin_m)

    x0, y0, x1, y1 = region
    n = 200
    xs = np.linspace(x0, x1, n)
    ys = np.linspace(y0, y1, n)
    xg, yg = np.meshgrid(xs, ys)
    depth_fn = sgw.depth_at(scfg, region, None)
    depth = np.vectorize(depth_fn)(xg, yg)

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.set_facecolor("#e9e4d4")
    ax.pcolormesh(xg, yg, np.where(depth > 1e-6, depth, np.nan),
                 cmap="Blues", vmin=0.0, vmax=float(scfg["surge_m"]) or 1.0,
                 shading="auto", zorder=1)

    ins = inset_bounds(region, edge_margin_m)
    ax.add_patch(Polygon([(ins[0], ins[1]), (ins[2], ins[1]),
                          (ins[2], ins[3]), (ins[0], ins[3])],
                         fill=False, edgecolor="#444444", linestyle="--",
                         linewidth=1.2, zorder=6,
                         label="review_points edge_margin_m={0:.0f}"
                               .format(edge_margin_m)))
    ax.add_patch(Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                         fill=False, edgecolor="black", linewidth=1.5,
                         zorder=6))

    for h in houses:
        ax.scatter(h["x"], h["y"], s=14,
                  c=_HCOLOUR.get(h.get("level"), "#888888"),
                  zorder=3, linewidths=0)
    for t in trees:
        ax.scatter(t["x"], t["y"], s=4,
                  c=_TCOLOUR.get(t.get("level"), "#2f7d32"),
                  zorder=2, alpha=0.6, linewidths=0)

    for name, subj in subjects.items():
        cam_x, cam_y, az, d_use = _resolve_camera(subj, region, avoid)
        tri = _frustum_polygon(subj["x"], subj["y"], az, d_use)
        close = "obl_dist" in subj
        ax.add_patch(Polygon(tri, closed=True,
                             facecolor="#ff5500" if close else "#1f77b4",
                             edgecolor="#ff5500" if close else "#1f77b4",
                             alpha=0.18, zorder=4))
        ax.plot([subj["x"], cam_x], [subj["y"], cam_y],
               color="#ff5500" if close else "#1f77b4",
               linewidth=1.0, zorder=5)
        ax.scatter([subj["x"]], [subj["y"]], marker="*", s=140,
                  c="#ff5500" if close else "#1f77b4",
                  edgecolors="black", linewidths=0.6, zorder=7)
        margin = edge_margin(region, subj["x"], subj["y"])
        ax.annotate("{0}\n(margin {1:.0f}m)".format(name, margin),
                   (subj["x"], subj["y"]), fontsize=7, zorder=8,
                   xytext=(4, 4), textcoords="offset points")

    ax.set_xlim(x0 - 20, x1 + 20)
    ax.set_ylim(y0 - 20, y1 + 20)
    ax.set_aspect("equal")
    ax.set_title(title or gt.get("scene", "hurricane review cameras"))
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return subjects


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", required=True,
                   help="path to a GT_hurricane.json")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--edge-margin-m", type=float, default=70.0)
    args = ap.parse_args(argv)

    with open(os.path.expanduser(args.gt)) as fh:
        gt = json.load(fh)
    subjects = plot_cameras(gt, os.path.expanduser(args.out),
                            edge_margin_m=args.edge_margin_m)
    print("[hurricane_cameras_png] {0} -> {1} subject(s)"
          .format(args.out, len(subjects)))
    for name, s in subjects.items():
        note = (" -- " + s["note"]) if s.get("note") else ""
        print("  {0}: ({1:.1f}, {2:.1f}){3}".format(name, s["x"], s["y"], note))


if __name__ == "__main__":
    main()
