#!/usr/bin/env python3
"""gac_props_check.py — offline geometry audit for `detail/gac_props.py`.

    python3 scene_gen/tools/gac_props_check.py [--config downtown_gac]

The MATH pass needs no `pxr` and no Isaac Sim: it reuses `plan_png.build()` —
the same host-side layout pipeline the plan PNG runs off — to get a real
`(cfg, layout, placements, resolver)` for a config, dresses it exactly the way
`generate_scene.py` does (same config key, same RNG offset), and checks the
placement dicts `apply_placements` would actually consume against the
properties a floating, overhanging, misseated or misidentified prop would
violate:

    1. seating   — every ROOF prop's world Z lands ON the roof, not above or
                   below it: `z_m + z0` must equal the building's own
                   `z_m + H + z0` (the building's OWN pivot-to-geometry
                   offset — see `gac_props.roof_props`'s `roof_z` for why
                   dropping this term floats every prop on the library by a
                   few cm to 8+ cm).
    2. footprint — every ROOF prop's actual rotated footprint (built from its
                   OWN measured `cx`/`cy`/`W`/`D`, not from where its
                   placement thinks its pivot is) lies inside the building's
                   roof rectangle, inset by the 2.2 m parapet.
    3. no stacking — no two roof props on the same building overlap.
    4. identity  — every prop's `of` tag names a real building this run
                   actually placed.
    5. material  — every placed `roof_house`'s own material (`gac_props_
                   measure.py`'s `_classify_material`) equals the building
                   it stands on's cladding (`gac_props._cladding_material`,
                   from `_plans/gac_faces.json`'s `front` elevation) — "brick
                   doesn't go with concrete" (user, 2026-08-29, about two
                   separate placements). Skipped for a building `_cladding_
                   material` could not classify at all (`MATERIAL_UNKNOWN`)
                   — nothing to check a mismatch AGAINST — and skipped
                   entirely when `building_props.roof_house_match_material`
                   is off in *cfg*, since that is a deliberate opt-out, not a
                   defect. `--force-mismatch` proves this assertion can
                   actually fail rather than passing vacuously: it corrupts
                   one already-placed `roof_house`'s asset to a different
                   material family after generation, and this assertion is
                   what is expected to catch it.

That pass is deliberately independent of `gac_props.py`'s own internals: it
recomputes each prop's TRUE world footprint from the measured JSON rather
than trusting the placement's `x_m, y_m` to already be the footprint centre.

BUT THE JSON ITSELF IS A MEASUREMENT, and a checker that only cross-checks
against the SAME measurement it is auditing shares its blind spot — see
`.agents/skills/fix-floating-debris/SKILL.md`: "a checker built on the same
blind spot as the measurement cannot see the defect either." So a SECOND
pass, `--points` (needs `pxr` — run inside `usd_python.sh`, not standalone),
opens every prop's ACTUAL referenced USD, independently re-measures it on
TRANSFORMED MESH POINTS (`disaster.bake.world_point_bounds`, the same fix the
skill documents, reused rather than re-derived), and checks the prop's real
geometry — not its recorded `z0` — against the roof height. Z only: a Z-axis
yaw rotation cannot move the Z-extent of a box, so this is exact, not another
AABB-of-an-AABB.
"""
import argparse
import json
import math
import os
import sys
from collections import defaultdict

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_SCENE_GEN, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# CAPTURE REAL pxr BEFORE `plan_png` IS IMPORTED, if it exists (inside
# `usd_python.sh` it does; on a bare host it does not, and the math-only pass
# below never needs it). `plan_png` stubs `pxr` for its own no-Isaac-Sim
# purposes, and its stub loop is NOT `setdefault`-safe against an ALREADY
# REAL `pxr`: `setattr(sys.modules["pxr"], n, types.ModuleType(n))` for
# `n` in `Usd`/`UsdGeom`/`Gf`/... runs unconditionally, so even inside the
# container it overwrites the real `pxr.Usd`/`pxr.UsdGeom`/`pxr.Gf` with
# empty stand-ins for the rest of the PROCESS — not just inside `plan_png`,
# everywhere, including `disaster.bake.world_point_bounds`'s own lazy
# `from pxr import Gf, UsdGeom` at call time. `--points` needs the real
# modules repaired back onto `sys.modules["pxr"]` before it touches
# anything in `disaster.bake` — see `check_points`.
_REAL_PXR_SUBMODULES = {}
try:
    import pxr.Usd
    import pxr.UsdGeom
    import pxr.Gf
    import pxr.Sdf
    _REAL_PXR_SUBMODULES = {"Usd": pxr.Usd, "UsdGeom": pxr.UsdGeom,
                            "Gf": pxr.Gf, "Sdf": pxr.Sdf}
except Exception:
    pass

import plan_png                                                  # noqa: E402
from detail import gac_props                                     # noqa: E402

Z_TOL_M = 0.03       # roof seating tolerance
XY_TOL_M = 0.05      # footprint containment slack


def _rot(vx, vy, deg):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    return vx * ca - vy * sa, vx * sa + vy * ca


def _corners(cx, cy, w, d):
    """Axis-aligned local-frame corners of a w x d box centred at (cx, cy)."""
    return [(cx + sx * w / 2.0, cy + sy * d / 2.0)
            for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]


def _world_footprint(rec, placement):
    """The prop's actual rotated world-space corners, from its OWN measured
    cx/cy/W/D — independent of whether the placement's x_m/y_m happens to be
    the footprint centre (the thing being checked)."""
    ox, oy = placement["x_m"], placement["y_m"]
    yaw = placement["yaw_deg"]
    out = []
    for lx, ly in _corners(rec["cx"], rec["cy"], rec["W"], rec["D"]):
        wx, wy = _rot(lx, ly, yaw)
        out.append((ox + wx, oy + wy))
    return out


def _to_local(px, py, bx, by, byaw):
    return _rot(px - bx, py - by, -byaw)


def _sat_overlap(poly_a, poly_b):
    """True if two convex polygons (world-space point lists) overlap, via
    separating-axis test over each polygon's own two edge normals."""
    def axes(poly):
        out = []
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            ex, ey = x2 - x1, y2 - y1
            out.append((-ey, ex))
        return out

    def project(poly, ax, ay):
        vals = [px * ax + py * ay for px, py in poly]
        return min(vals), max(vals)

    for ax, ay in axes(poly_a) + axes(poly_b):
        lo_a, hi_a = project(poly_a, ax, ay)
        lo_b, hi_b = project(poly_b, ax, ay)
        if hi_a < lo_b or hi_b < lo_a:
            return False
    return True


def _hull_order(corners):
    """Reorder 4 axis-combination corners into a convex ring (they come out
    of `_corners`/`_world_footprint` as [--, -+, +-, ++]; a SAT/plot polygon
    needs them walked around the perimeter)."""
    if len(corners) != 4:
        return corners
    a, b, c, d = corners
    return [a, b, d, c]


def check_points(props, bldg):
    """Independent points-based re-verification of roof-prop seating.

    Opens each prop's ACTUAL referenced USD and re-measures its world Z from
    TRANSFORMED MESH POINTS (`disaster.bake.world_point_bounds`) rather than
    trusting `_plans/gac_props.json`'s recorded `z0` at all — the other half
    of the fix `.agents/skills/fix-floating-debris/SKILL.md` documents: "a
    checker built on the same blind spot as the measurement cannot see the
    defect either." Needs `pxr` (run inside `usd_python.sh`, not standalone);
    prints why and returns None when it isn't available, leaving the
    math-only assertions above as the whole result.

    Z ONLY, and this is exact, not another AABB-of-an-AABB: every roof prop's
    `roll_deg`/`pitch_deg` are 0 (`gac_props._place`), so the only rotation in
    its placement is `yaw_deg` about Z — which cannot move a box's Z-extent.
    Re-deriving world Z from the placement's own `(z_m, scale)` needs no
    re-rotation of the points, just the same translate + uniform-scale every
    placement gets.
    """
    if not _REAL_PXR_SUBMODULES:
        print("\n[gac_props_check --points] skipped: no pxr available. "
             "Run inside usd_python.sh.")
        return None
    # REPAIR `sys.modules["pxr"]`'s attributes back to the REAL submodules
    # captured before `plan_png` overwrote them (see the top of this file) —
    # `disaster.bake.world_point_bounds` does its own `from pxr import Gf,
    # UsdGeom` lazily, at call time, so it needs the repair to have already
    # happened on the shared `pxr` package object, not just a local name here.
    import pxr as _pxr_pkg
    for _n, _mod in _REAL_PXR_SUBMODULES.items():
        setattr(_pxr_pkg, _n, _mod)
    Usd, UsdGeom = _REAL_PXR_SUBMODULES["Usd"], _REAL_PXR_SUBMODULES["UsdGeom"]
    if _SCENE_GEN not in sys.path:
        sys.path.insert(0, _SCENE_GEN)
    from disaster import bake

    ROOF_KINDS = {"roof_prop", "roof_tank", "roof_mast", "roof_house"}
    cache = {}

    def points_bounds(usd_path):
        if usd_path in cache:
            return cache[usd_path]
        try:
            st = Usd.Stage.Open(usd_path)
            st.Load()
        except Exception as exc:
            print(f"  OPEN FAIL {usd_path}: {exc}")
            cache[usd_path] = None
            return None
        S = UsdGeom.GetStageMetersPerUnit(st)
        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        lo, hi, found = [1e30] * 3, [-1e30] * 3, False
        for prim in st.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            b = bake.world_point_bounds(prim, xcache)
            if b is None:
                continue
            found = True
            plo, phi = b
            for k in range(3):
                lo[k] = min(lo[k], plo[k])
                hi[k] = max(hi[k], phi[k])
        result = (lo, hi, S) if found else None
        cache[usd_path] = result
        return result

    n_pass = n_fail = 0
    deltas, fails = [], []
    for pl in props:
        if pl["category"] not in ROOF_KINDS:
            continue
        b = bldg.get(pl["of"])
        if b is None:
            continue
        pb = points_bounds(pl["usd"])
        if pb is None:
            n_fail += 1
            fails.append(f"points[{pl['of']} {pl['category']}]: could not "
                         f"open/measure {pl['usd']}")
            continue
        lo, hi, S = pb
        # Same translate + uniform-scale `apply_placements` gives this prim;
        # no rotation term needed for Z (see docstring).
        world_z_min = pl["z_m"] + pl["scale"] * lo[2]
        roof_z = b["z"] + b["H"] + b["z0"]
        delta = world_z_min - roof_z
        deltas.append(delta)
        ok = abs(delta) <= Z_TOL_M
        if ok:
            n_pass += 1
        else:
            n_fail += 1
            fails.append(
                f"points[{pl['of']} {pl['category']} "
                f"{os.path.basename(pl['usd'])}]: real geometry bottom at "
                f"{world_z_min:.3f}, roof at {roof_z:.3f}, delta "
                f"{delta:+.3f} m")

    deltas.sort()
    n = len(deltas)

    def pct(p):
        return deltas[min(n - 1, int(p * n))] if n else float("nan")

    print(f"\n[gac_props_check --points] {n} roof prop(s) independently "
          f"re-measured on points ({len(cache)} distinct USD file(s) opened)")
    if n:
        print(f"  seating delta (points z_min - roof_z): "
              f"p50={pct(0.5):+.4f}  p90={pct(0.9):+.4f}  p99={pct(0.99):+.4f}"
              f"  min={deltas[0]:+.4f}  max={deltas[-1]:+.4f}")
    print(f"  assertions: {n_pass} pass, {n_fail} fail "
          f"({n_pass + n_fail} total)")
    if fails:
        print(f"  FAILURES ({len(fails)}" +
              (", showing first 40" if len(fails) > 40 else "") + "):")
        for f in fails[:40]:
            print(f"    - {f}")
    else:
        print("  all assertions passed")
    return n_fail == 0, n_pass, n_fail, fails


def run(config_name="downtown_gac", verbose=False, points=False,
        force_mismatch=False):
    cfg, layout, placements, resolver = plan_png.build(config_name)
    seed = int(cfg.get("seed", 0))
    import random
    rng = random.Random(seed + 6301)
    props = gac_props.dress(cfg, placements, rng, resolver=resolver)

    by_kind, faces, dims = gac_props.load()
    props_by_name = {r["name"]: r for lst in by_kind.values() for r in lst}

    bp_cfg = (cfg.get("building_props") or {})
    flat_roof = {str(n) for n in (bp_cfg.get("flat_roof") or [])}
    match_material_cfg = bool(bp_cfg.get("roof_house_match_material", True))
    roof_house_max_h = float(bp_cfg.get("roof_house_max_h_m",
                                        gac_props.ROOF_HOUSE_MAX_H_M))

    # Rebuild the SAME (tag -> building record) map `dress()` built
    # internally, so every prop's `of` resolves to the building it was
    # actually generated against.
    bldg = {}
    for p in placements:
        if p.get("category") != "house":
            continue
        nm = gac_props._name_of(p.get("usd", ""))
        d = dims.get(nm)
        if d is None:
            if nm not in flat_roof:
                continue
            fp = resolver.get(p.get("usd", ""), "house",
                              scale=p.get("scale", 1.0),
                              axis_up=p.get("axis_up", "Z"))
            # Mirrors `gac_props.dress()`'s own resolver branch: `z0` is the
            # building's pivot-to-geometry offset, `-fp["base"]`, needed by
            # the SAME roof_z formula this checker re-derives below.
            d = {"W": fp["sx"], "D": fp["sy"], "H": fp["sz"], "z0": -fp["base"]}
        tag = "%s@%.1f,%.1f" % (nm, p["x_m"], p["y_m"])
        bldg[tag] = {"nm": nm, "x": p["x_m"], "y": p["y_m"],
                     "z": p.get("z_m", 0.0), "yaw": p.get("yaw_deg", 0.0),
                     "W": d["W"], "D": d["D"], "H": d["H"],
                     "z0": float(d.get("z0", 0.0)),
                     "face": faces.get(nm)}

    material_by_bld = gac_props.building_materials(faces)

    if force_mismatch:
        # Deliberately corrupt ONE already-placed roof_house's asset to a
        # DIFFERENT material family than the one that was actually matched,
        # proving the assertion below can fail rather than passing
        # vacuously — the acceptance test for this feature per the task
        # that added it. Only ever mutates this run's in-memory `props`
        # list, never `_plans/gac_props.json` or a preset.
        #
        # ONLY a placement on a building with a KNOWN material (`bmat !=
        # MATERIAL_UNKNOWN`) is eligible to corrupt — assertion 5 skips
        # every other one by design (see its own comment), so corrupting
        # those would pass vacuously instead of demonstrating a catch (this
        # was tried first: it picked a non-GAC `flat_roof` building, which
        # has no cladding measurement to check against at all, and the
        # material assertion correctly stayed silent while UNRELATED
        # footprint/overlap assertions failed instead, from swapping in a
        # differently-SIZED asset — not what this flag is for).
        #
        # The replacement is chosen to be the SAME measured footprint
        # (`W`/`D`, either orientation) as the original wherever one exists,
        # so ONLY the material assertion is exercised, not footprint/
        # overlap too — `SM_Superior_Construction_01`/`_02` are exactly this
        # pair (24.21 x 39.71 m each, brick vs concrete).
        house_pool = by_kind.get("roof_house", [])
        target = None
        for pl in props:
            if pl["category"] != "roof_house":
                continue
            b = bldg.get(pl["of"])
            if b is None or material_by_bld.get(
                    b["nm"], gac_props.MATERIAL_UNKNOWN) == gac_props.MATERIAL_UNKNOWN:
                continue
            target = pl
            break
        if target is None:
            print("[gac_props_check --force-mismatch] no roof_house "
                 "placement on a material-known building in this run -- "
                 "nothing to demonstrate")
        else:
            cur = props_by_name.get(gac_props._name_of(target["usd"]))
            cur_mat = cur.get("material") if cur else None
            same_size = [r for r in house_pool if r.get("material") != cur_mat
                        and ({round(r["W"], 1), round(r["D"], 1)} ==
                             {round(cur["W"], 1), round(cur["D"], 1)})] if cur else []
            alt = (same_size[0] if same_size else
                  next((r for r in house_pool if r.get("material") != cur_mat), None))
            if alt is not None:
                print(f"[gac_props_check --force-mismatch] {target['of']}: "
                     f"swapping {gac_props._name_of(target['usd'])} "
                     f"({cur_mat}) -> {alt['name']} ({alt['material']})"
                     + ("" if same_size else "  (no same-footprint "
                        "alternate found -- footprint/overlap assertions "
                        "may also fire as a side effect)"))
                target["usd"] = alt["usd"]

    n_pass, n_fail = 0, 0
    fails = []

    def check(name, ok, detail=""):
        nonlocal n_pass, n_fail
        if ok:
            n_pass += 1
        else:
            n_fail += 1
            fails.append(f"{name}: {detail}")

    ROOF_KINDS = {"roof_prop", "roof_tank", "roof_mast", "roof_house"}
    PARAPET = gac_props.PARAPET_M

    # ---- 4. identity — every prop names a building this run placed --------
    for i, pl in enumerate(props):
        of = pl.get("of")
        check(f"identity[{i}] {pl['category']} of={of!r}", of in bldg,
              "no matching building record" if of not in bldg else "")

    # ---- 5. material — a roof_house is clad like the building it stands on
    # See this run()'s own `--force-mismatch` block above for how this
    # assertion is proven to actually fail, not just pass by construction.
    # `material_by_bld` was already computed above, before that block, so
    # the corruption logic and this assertion read the identical table.
    n_material_checked = 0
    for pl in props:
        if pl["category"] != "roof_house":
            continue
        b = bldg.get(pl["of"])
        if b is None:
            continue   # already flagged by the identity check above
        bmat = material_by_bld.get(b["nm"], gac_props.MATERIAL_UNKNOWN)
        if not match_material_cfg or bmat == gac_props.MATERIAL_UNKNOWN:
            continue   # feature off, or this building has no cladding to
                       # check against at all (see docstring's assertion 5)
        rec = props_by_name.get(gac_props._name_of(pl["usd"]))
        pmat = rec.get("material") if rec else None
        n_material_checked += 1
        check(f"material[{pl['of']} {gac_props._name_of(pl['usd'])}]",
              pmat == bmat,
              f"roof_house material={pmat!r}, building {b['nm']} "
              f"(front={(b['face'] or {}).get('front')!r}) "
              f"material={bmat!r}")
    print(f"[gac_props_check] material assertion checked on "
         f"{n_material_checked} roof_house placement(s)"
         + ("" if match_material_cfg else
            " (building_props.roof_house_match_material is off in this "
            "config, so 0 is expected)"))

    # ---- eligible-but-blocked: how many buildings the gate actually costs
    # Computed DIRECTLY against `by_kind['roof_house']`, not by re-dressing
    # with the flag off: `dress()` shares ONE `rng` across every building in
    # placement order, and `roof_props`'s `rng.shuffle(house_pool)` /
    # `rng.sample(...)` calls consume a number of draws that depends on the
    # POOL'S OWN LENGTH — a shorter (material-filtered) pool desyncs every
    # random draw for every building placed AFTER the first one where the
    # two pools differ in length, not just the buildings the filter actually
    # changes. Tried first; the "after" count came out HIGHER than "before"
    # on this exact config (18 vs 16) — the signature of a desynced shared
    # RNG, not of the feature costing negative buildings. This instead
    # replicates only `roof_props`'s SIZE gate (the parapet-inset W/D fit,
    # the `ROOF_HOUSE_MIN_M2` roof area, `roof_house_max_h_m`) — no RNG
    # anywhere — against every "house" placement `bldg` already carries, so
    # the two counts are exactly comparable and the 0.55 coin flip / edge
    # fit (legitimately random, and unrelated to material) are left out of
    # both.
    house_pool_all = by_kind.get("roof_house", [])
    n_size_eligible = n_material_blocked = 0
    for b in bldg.values():
        W, D, H = b["W"], b["D"], b["H"]
        if not (W * D >= gac_props.ROOF_HOUSE_MIN_M2 and H <= roof_house_max_h):
            continue
        fitting = [r for r in house_pool_all
                  if r["W"] + 2 * gac_props.PARAPET_M < W
                  and r["D"] + 2 * gac_props.PARAPET_M < D]
        if not fitting:
            continue
        n_size_eligible += 1
        bmat = material_by_bld.get(b["nm"], gac_props.MATERIAL_UNKNOWN)
        if (match_material_cfg and bmat != gac_props.MATERIAL_UNKNOWN
                and not any(r.get("material") == bmat for r in fitting)):
            n_material_blocked += 1
    print(f"[gac_props_check] roof_house SIZE-eligible buildings: "
         f"{n_size_eligible} (big enough roof, under the height ceiling, "
         f"some roof_house asset fits under the parapet inset) -- of those, "
         f"{n_material_blocked} have NO roof_house asset matching their own "
         f"cladding, reduced to zero candidates by the material gate "
         f"regardless of how the 0.55 coin flip and edge fit would have "
         f"landed")

    # ---- 1 & 2: roof props -------------------------------------------------
    by_building_roof = defaultdict(list)
    for pl in props:
        if pl["category"] not in ROOF_KINDS:
            continue
        b = bldg.get(pl["of"])
        if b is None:
            continue  # already flagged by the identity check
        by_building_roof[pl["of"]].append(pl)

        rec = props_by_name.get(gac_props._name_of(pl["usd"]))
        label = f"{pl['category']} on {pl['of']}"
        if rec is None:
            check(f"seating[{label}]", False, "prop not in gac_props.json")
            continue

        # See `gac_props.roof_props`'s own `roof_z` comment: a PRISTINE
        # building's `z_m` already equals `-z0` (the lift that puts its
        # geometry's bottom, not its local origin, at world Z 0), so the
        # roof's true world height is `z_m + H + z0`, not `z_m + H` — the
        # bug this checker used to reproduce rather than catch, because it
        # was computing the SAME (wrong) formula `gac_props.py` was.
        roof_z = b["z"] + b["H"] + b["z0"]
        actual_z = pl["z_m"] + rec["z0"]
        check(f"seating[{label}]", abs(actual_z - roof_z) <= Z_TOL_M,
              f"z_m({pl['z_m']:.3f}) + z0({rec['z0']:.3f}) = "
              f"{actual_z:.3f}, roof is at {roof_z:.3f} "
              f"(building z_m={b['z']:.3f} + H={b['H']:.3f} + "
              f"z0={b['z0']:.3f})")

        corners = _world_footprint(rec, pl)
        local = [_to_local(x, y, b["x"], b["y"], b["yaw"])
                 for x, y in corners]
        hw, hd = b["W"] / 2.0 - PARAPET, b["D"] / 2.0 - PARAPET
        bad = [(lx, ly) for lx, ly in local
               if abs(lx) > hw + XY_TOL_M or abs(ly) > hd + XY_TOL_M]
        check(f"footprint[{label}]", not bad,
              f"corner(s) outside the {b['W']:.1f}x{b['D']:.1f} roof "
              f"(parapet-inset half-extents {hw:.2f}x{hd:.2f}): {bad}")

    # ---- 3. no roof prop-on-prop overlap -----------------------------------
    for of, plist in by_building_roof.items():
        polys = []
        for pl in plist:
            rec = props_by_name.get(gac_props._name_of(pl["usd"]))
            if rec is None:
                continue
            polys.append((pl["category"], _hull_order(_world_footprint(rec, pl))))
        for i in range(len(polys)):
            for j in range(i + 1, len(polys)):
                ci, pi = polys[i]
                cj, pj = polys[j]
                overlap = _sat_overlap(pi, pj)
                check(f"no-overlap[{of} #{i}:{ci} x #{j}:{cj}]", not overlap,
                      "footprints intersect")

    # ---- 3b. the condenser row is STRUCTURED — collinear, evenly pitched --
    # `roof_props()` draws ONE plant asset and repeats it along a single
    # axis; group roof_prop members by (building, asset) so a vent-stack
    # line (a different asset, same category) is never compared against the
    # condenser row it shares a category with, then check every such group
    # of >= 2 for collinearity and constant pitch in the BUILDING's own
    # local frame (never world — a yawed building's row is not axis-aligned
    # in world coordinates at all).
    #
    # A group can be TWO back-to-back rows (`roof_props`'s `rows == 2`), not
    # one — a single "is one axis ~constant across the whole group" test
    # fails a valid two-row layout, since the cross-axis takes two distinct
    # values, not one. `_row_clusters` splits the group by whichever axis
    # has FEWER distinct clustered values (1 for a single row, `rows` for
    # `rows` back-to-back ones) and checks pitch WITHIN each resulting row.
    ROW_TOL_M = 0.05

    def _row_clusters(points, tol):
        """[[run_value, ...], ...] — one sorted list per row. Clusters on
        whichever axis groups into FEWER distinct values (points sharing a
        cross-axis value within `tol` are one row); the untouched axis is
        the run axis, returned per row, sorted."""
        def cluster(pts, axis):
            pts = sorted(pts, key=lambda p: p[axis])
            groups, cur = [], [pts[0]]
            for p in pts[1:]:
                if p[axis] - cur[-1][axis] <= tol:
                    cur.append(p)
                else:
                    groups.append(cur)
                    cur = [p]
            groups.append(cur)
            return groups

        gx = cluster(points, 0)   # clustered by local-x => x is the CROSS axis
        gy = cluster(points, 1)   # clustered by local-y => y is the CROSS axis
        groups, run_axis = (gx, 1) if len(gx) <= len(gy) else (gy, 0)
        return [sorted(p[run_axis] for p in g) for g in groups]

    by_building_asset = defaultdict(list)
    for pl in props:
        if pl["category"] != "roof_prop":
            continue
        b = bldg.get(pl["of"])
        if b is None:
            continue
        by_building_asset[(pl["of"], gac_props._name_of(pl["usd"]))].append(pl)

    n_rows_checked = 0
    for (of, nm), plist in by_building_asset.items():
        if len(plist) < 2:
            continue
        b = bldg[of]
        # NOT named `points` — that shadows `run()`'s own `points=` CLI-pass
        # parameter for the rest of the function (a non-empty list is
        # truthy), which silently turned `--points`-less runs into
        # `--points` runs. Cost a debug round; renamed instead of nested.
        local_xy = [_to_local(pl["x_m"], pl["y_m"], b["x"], b["y"], b["yaw"])
                   for pl in plist]
        for ri, run_vals in enumerate(_row_clusters(local_xy, ROW_TOL_M)):
            n_rows_checked += 1
            if len(run_vals) < 2:
                continue   # a lone vent, or a one-unit row — nothing to pitch
            gaps = [round(b_ - a_, 3) for a_, b_ in zip(run_vals, run_vals[1:])]
            # A gap that is an INTEGER MULTIPLE of the smallest one is a
            # dropped slot, not an uneven pitch: `_put_at` skips a row
            # member outright when the bulkhead/tank/mast/vents already
            # occupy that spot (`SM_Building_19`'s `SM_Air_Tubes_Machine_
            # Part_02` row: gaps 1.7, 3.4, 1.7 — the middle slot is missing,
            # not mis-spaced). Still evidence of a laid-out grid: a genuine
            # scatter would not produce gaps that are clean multiples of one
            # base pitch.
            base = min(gaps)
            even = all(abs(g - round(g / base) * base) <= ROW_TOL_M for g in gaps)
            check(f"row[{of} {nm} row{ri}] evenly pitched ({len(run_vals)} "
                  f"member(s))", even, f"gaps along the run axis {gaps}")

    # ---- report --------------------------------------------------------
    tally = {}
    for pl in props:
        tally[pl["category"]] = tally.get(pl["category"], 0) + 1
    print(f"\n[gac_props_check] {config_name}: {len(props)} prop(s) over "
          f"{len(bldg)} eligible building(s)")
    print("  by kind: " + "  ".join(f"{k}={v}" for k, v in sorted(tally.items())))
    print(f"  roof_prop row/line groups checked for even pitch: {n_rows_checked}")
    print(f"  assertions: {n_pass} pass, {n_fail} fail "
          f"({n_pass + n_fail} total)")
    if fails:
        print(f"\n  FAILURES ({len(fails)}" +
              (f", showing first 40" if len(fails) > 40 else "") + "):")
        for f in fails[:40]:
            print(f"    - {f}")
    else:
        print("  all assertions passed")
    ok = n_fail == 0

    if points:
        pts_result = check_points(props, bldg)
        if pts_result is not None:
            pts_ok = pts_result[0]
            ok = ok and pts_ok

    return ok, tally, n_pass, n_fail, fails


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="downtown_gac")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--points", action="store_true",
                    help="also independently re-measure every roof prop's "
                         "seating on transformed mesh points (needs pxr — "
                         "run inside usd_python.sh)")
    ap.add_argument("--force-mismatch", action="store_true",
                    help="deliberately corrupt one placed roof_house's "
                         "material after generation, to prove assertion 5 "
                         "(material) can actually fail rather than passing "
                         "vacuously; exits 1 by design when it does")
    a = ap.parse_args()
    ok, *_ = run(a.config, a.verbose, points=a.points,
                force_mismatch=a.force_mismatch)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
