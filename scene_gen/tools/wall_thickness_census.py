#!/usr/bin/env python
"""
wall_thickness_census.py — how much of the building library is a ZERO-THICKNESS
SHELL rather than a solid with two faces and a gap between them.

WHY IT EXISTS
-------------
The earthquake fracture pass slices a mesh with a convex/Voronoi cell
decomposition and keeps the pieces.  That operation is only meaningful on a
CLOSED SOLID: slice a solid and you get chunks with an inside; slice a single
surface — one quad standing in for a 0.3 m brick wall — and every "chunk" is a
zero-volume plate, which is why those buildings read as sheets of paper falling
off rather than as masonry breaking.  Before anyone can fix that, somebody has
to say WHICH buildings are shells and by how much, from the files rather than
from memory.  That is all this tool does; it proposes nothing.

WHAT IT MEASURES, AND WHY THOSE THREE THINGS
--------------------------------------------
Per Mesh prim, in WORLD METRES (points pushed through
`ComputeLocalToWorldTransform` and multiplied by the stage's metersPerUnit):

  boundary_edge_fraction   Triangulate every face, weld coincident points
                           (USD/UE exports split vertices per UV island and
                           per smoothing group, so an UNWELDED edge count says
                           "open shell" about a perfectly closed box), then
                           count undirected edges used by exactly ONE triangle.
                           A closed solid has 0.  A single quad has 4/4 = 1.0.
                           A box missing its inner face has ~0.3.
                           THIS IS THE PRIMARY TEST — it is topological, so it
                           does not care how thick the author made the wall.

  thin_m                   The smallest of the three world AABB extents.  This
                           catches the second species: a mesh that IS closed
                           but was extruded 2 mm, which trimesh will slice into
                           chunks that still look like paper.

  area_m2                  Summed triangle area.  A building is rolled up
                           AREA-WEIGHTED, because "9 of 40 meshes are shells"
                           is meaningless when the 9 are the whole façade and
                           the 31 are door handles.

  VERDICT per mesh: zero-thickness  <=>  boundary_edge_fraction > 0.05
                                     OR  (thin_m < 0.02 and the other two
                                          extents both > 1.0 m)
  VERDICT per building/style: SOLID  area_zero_frac < 0.05
                              MIXED  0.05 .. 0.60
                              SHELL  > 0.60

THE TWO POPULATIONS
-------------------
1. KIT MODULES.  The 16 styles the earthquake pipeline bakes are not single
   assets: `detail/urban_building.build_building` assembles each one out of
   Nucleus kit pieces (SM_MBuilding*, SM_build_b_*, SM_SetGoverment_*).  The
   BAKED archetype (`assets/archetype/bld_<style>_DG0.usd`) is flattened
   and its prims are renamed `LOD0_37`, so the module identity is gone there —
   measuring the archetype tells you the total but never WHICH piece is the
   shell.  So the modules are measured ONE BY ONE at their source URLs and
   rolled up per style using the seed-4 build (`ARCH_SEED` default 4, the seed
   `bake_quake_archetypes_launch_script.py` and `quake.py` both use), which is
   the exact set of pieces that gets baked.  Pool pieces that seed 4 happens
   not to instance are measured too and reported separately.
   The flattened DG0 archetype is ALSO measured, as an independent check that
   the per-module roll-up agrees with the thing the fracture code actually sees.

2. THE 2026-08-26 DROP.  Every building URL in
   `config/asset_sets/urban_v2.yaml` (pools `tower+`, `midrise+`, `rowhouse+`,
   `destroyed+`), measured whole, grouped by the folder it came from.
   Cars, debris pieces and piles are NOT buildings and are skipped.

NO SIMULATIONAPP.  Same bootstrap as `tools/measure_fences.py`: three
extensions put `pxr` + the `omniverse://` resolver on a bare `python.sh`, so
this runs beside a live Isaac Sim without touching it.  Nucleus opens are
CACHED to `_plans/wall_thickness_census.cache.json`; set `CENSUS_REFRESH=1` to
re-read them.

    scene_gen/tools/wall_thickness_census.sh            # from the host
"""
import json
import os
import random
import sys
import time
import traceback

T0 = time.time()
HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
OUT = os.environ.get("CENSUS_OUT",
                     os.path.join(SCENE_GEN, "_plans", "wall_thickness_census.json"))
CACHE = os.environ.get("CENSUS_CACHE",
                       os.path.join(SCENE_GEN, "_plans", "wall_thickness_census.cache.json"))
LOG = OUT + ".log"
REFRESH = os.environ.get("CENSUS_REFRESH", "") not in ("", "0")
SEED = int(os.environ.get("ARCH_SEED") or "4")

# --- verdict thresholds, all in one place ---------------------------------
BFRAC_OPEN = 0.05      # > this share of edges used by one face -> open shell
THIN_M = 0.02          # < 2 cm on the short axis ...
BIG_M = 1.0            # ... while the other two axes are both over a metre
SOLID_MAX = 0.05       # building-level: area-weighted shell share
SHELL_MIN = 0.60
WELD_M = 1e-4          # 0.1 mm: two points closer than this are one vertex
PANEL_MAX_M = 3.0      # short axis under this -> judge it as a WALL PANEL
TWO_SIDED_MIN = 0.15   # back-sheet area under this share of the front -> no back

sys.path.insert(0, SCENE_GEN)


def log(msg):
    line = "[census %6.1fs] %s" % (time.time() - T0, msg)
    print(line, flush=True)
    with open(LOG, "a") as fh:
        fh.write(line + "\n")


os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(LOG, "w").close()

import numpy as np                                          # noqa: E402
from pxr import Usd, UsdGeom                                # noqa: E402
import yaml                                                 # noqa: E402
from detail import urban_building as ub                     # noqa: E402


# ===========================================================================
# One mesh
# ===========================================================================
def measure_mesh(prim, mpu, tc=Usd.TimeCode.Default()):
    """Return the per-mesh record, or None if the prim carries no geometry."""
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get(tc)
    if pts is None:                      # points authored only at a time sample
        pts = mesh.GetPointsAttr().Get(0)
    fvc = mesh.GetFaceVertexCountsAttr().Get(tc)
    fvi = mesh.GetFaceVertexIndicesAttr().Get(tc)
    if pts is None or fvc is None or fvi is None or len(pts) == 0 or len(fvc) == 0:
        return None
    P = np.asarray(pts, dtype=np.float64)
    counts = np.asarray(fvc, dtype=np.int64)
    idx = np.asarray(fvi, dtype=np.int64)

    # world transform, then stage units -> metres
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc)
    M = np.array([[m[i][j] for j in range(4)] for i in range(4)],
                 dtype=np.float64)                    # USD is ROW-vector: p*M
    W = (np.concatenate([P, np.ones((len(P), 1))], axis=1) @ M)[:, :3] * float(mpu)

    lo, hi = W.min(axis=0), W.max(axis=0)
    ext = (hi - lo)

    # --- fan-triangulate every face ---------------------------------------
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    tris = []
    for s, c in zip(starts, counts):
        if c < 3:
            continue
        f = idx[s:s + c]
        for k in range(1, c - 1):
            tris.append((f[0], f[k], f[k + 1]))
    if not tris:
        return None
    T = np.asarray(tris, dtype=np.int64)

    a = W[T[:, 1]] - W[T[:, 0]]
    b = W[T[:, 2]] - W[T[:, 0]]
    area = float(0.5 * np.linalg.norm(np.cross(a, b), axis=1).sum())

    # --- WELD, then count edges -------------------------------------------
    # THE WELD IS THE WHOLE TEST'S WEAK POINT, so it is done at three
    # tolerances and all three are recorded.  Without any weld a UE-exported
    # cube reads as six loose quads and every edge is a boundary edge (a false
    # SHELL); with too coarse a weld a genuine 5 mm gap closes and a shell
    # reads solid (a false SOLID).  If `boundary_edge_fraction` at 0 mm,
    # 0.1 mm and 10 mm agree, the verdict does not depend on the tolerance and
    # the claim stands on its own; where they disagree the record says so.
    E0 = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]], axis=0)

    def _bfrac(weld):
        if weld <= 0:
            E = E0
        else:
            q = np.round(W / weld).astype(np.int64)
            _u, inv = np.unique(q, axis=0, return_inverse=True)
            E = inv[E0]
        E = E[E[:, 0] != E[:, 1]]                   # drop degenerates
        if len(E) == 0:
            return None, 0, 0, 0
        E = np.sort(E, axis=1)
        _uq, c = np.unique(E, axis=0, return_counts=True)
        return (int((c == 1).sum()) / float(len(c)), int(len(c)),
                int((c == 1).sum()), int((c > 2).sum()))

    bfrac, n_edges, n_boundary, n_nonmanifold = _bfrac(WELD_M)
    if bfrac is None:
        return None
    bfrac_raw = _bfrac(0.0)[0]
    bfrac_coarse = _bfrac(1e-2)[0]

    # --- SECOND, INDEPENDENT OPINION: is there a BACK FACE? ---------------
    # The boundary-edge test is topological and has one weak spot: a mesh with
    # a few unclosed window reveals lands at bfrac ~0.05, next to a genuine
    # one-sided panel at ~0.05, and a threshold decides.  So for PANEL-SIZED
    # meshes (short axis under 3 m — a wall piece, not a whole building) the
    # thickness is measured directly instead: take the short axis, split the
    # triangles into those facing along it and those facing against it, and
    # report both the area ratio and the gap between their area-weighted mean
    # positions.  A single surface has faces pointing ONE way only (ratio ~0)
    # and no gap; a real wall has two sheets and the gap IS the wall thickness.
    # It is sign-symmetric, so mesh orientation (left/right-handed) cannot
    # change it.  Not applied to whole buildings: there the "two sides" would
    # be the opposite façades and the "thickness" the building's own depth.
    axis = int(np.argmin(ext))
    ah = np.zeros(3); ah[axis] = 1.0
    nrm = np.cross(a, b)
    tri_area = 0.5 * np.linalg.norm(nrm, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        cosv = np.where(tri_area > 1e-12,
                        (nrm @ ah) / np.maximum(2.0 * tri_area, 1e-12), 0.0)
    cen = (W[T[:, 0]] + W[T[:, 1]] + W[T[:, 2]]) / 3.0
    pos = cen @ ah
    fwd, bwd = cosv > 0.5, cosv < -0.5
    a_f, a_b = float(tri_area[fwd].sum()), float(tri_area[bwd].sum())
    ext_s0 = np.sort(ext)
    is_panel = bool(ext_s0[0] < 3.0 and ext_s0[1] > 0.3 and ext_s0[2] > 0.3)
    two_sided_ratio = (min(a_f, a_b) / max(a_f, a_b)) if max(a_f, a_b) > 0 else 0.0
    if a_f > 0 and a_b > 0:
        panel_thickness = abs(float((tri_area[fwd] * pos[fwd]).sum() / a_f
                                    - (tri_area[bwd] * pos[bwd]).sum() / a_b))
    else:
        panel_thickness = 0.0
    # one-sided when the back sheet is under 15% of the front's area
    panel_verdict = None
    if is_panel:
        panel_verdict = ("ONE_SIDED" if two_sided_ratio < 0.15
                         else ("TWO_SIDED_THIN" if panel_thickness < THIN_M
                               else "TWO_SIDED"))

    ext_sorted = np.sort(ext)
    thin = float(ext_sorted[0])
    flat_plate = bool(thin < THIN_M and ext_sorted[1] > BIG_M and ext_sorted[2] > BIG_M)
    open_shell = bool(bfrac > BFRAC_OPEN)

    # THREE CLASSES, because "zero thickness" turns out to mean two different
    # things in this library and the fix is not the same for both:
    #   SINGLE_SURFACE  the mesh is not watertight.  There is no back face, so
    #                   there is no material between two faces: the wall's
    #                   thickness is zero BY CONSTRUCTION however deep the
    #                   piece's bbox is (a citydemo tower is a 36 m-deep bbox
    #                   made of one-sided façade strips).
    #   CLOSED_THIN     watertight but flattened to under 2 cm — a sheet with
    #                   two coincident sides.
    #   CLOSED_SOLID    watertight with a real gap; `thickness_m` is its short
    #                   axis, which for a wall piece IS the wall thickness.
    # WHICH TEST DECIDES.  For a PANEL the back-face test decides, because the
    # boundary-edge test systematically UNDER-reads openness on dense art: a
    # one-sided façade with fifty window reveals shares almost every edge
    # inside its own surface, so only the silhouette is boundary and bfrac
    # comes out at 0.02-0.05 — SM_MBuilding02_Facade_B reads 0.045 and is a
    # pure single surface with not one back-facing triangle.  For anything too
    # big to be a panel (a whole building, a corner mass) there is no single
    # "short axis" to face along, so the topological test decides: an envelope
    # that is not watertight has no inner face and therefore no wall thickness.
    # BOTH TESTS MUST PASS, because each alone is fooled by a different piece
    # of this library's art, and the two failures were found by looking at the
    # actual vertices:
    #   * boundary edges alone call SM_MBuilding02_Facade_B "closed" at bfrac
    #     0.045 — it is a single surface with not one back-facing triangle.  A
    #     dense one-sided façade shares nearly every edge INSIDE its own
    #     surface, so only the silhouette is boundary and the fraction is
    #     small.  Fixed by the back-face test.
    #   * the back-face test alone calls SM_MBuilding04_Facade_B "two-sided"
    #     at ratio 1.00 and thickness 0.70 — it is 8 points and 4 triangles:
    #     a 4x3 quad at y=0 and another at y=0.7, nothing joining them.  Two
    #     sheets of paper facing each other is not a solid.  Fixed by
    #     requiring the rim to be closed.
    # So: solid  <=>  a back sheet exists AND the rim is closed.
    two_sided = bool(two_sided_ratio >= TWO_SIDED_MIN)
    closed = not open_shell
    if thin < THIN_M:
        klass, decided_by = "FLAT_SHEET", "extent"
    elif is_panel:
        decided_by = "back_face+boundary_edges"
        if not two_sided:
            klass = "NO_BACK_FACE"          # one surface; fix = give it depth
        elif not closed:
            klass = "LOOSE_SHEETS"          # front and back, no rim; fix = cap it
        else:
            klass = "SOLID_PANEL"
    else:
        klass = ("OPEN_ENVELOPE" if open_shell else "CLOSED_SOLID")
        decided_by = "boundary_edges"
    zero = klass in ("FLAT_SHEET", "NO_BACK_FACE", "LOOSE_SHEETS", "OPEN_ENVELOPE")
    # a borderline call: say so rather than let the threshold decide silently
    marginal = bool(0.5 * BFRAC_OPEN < bfrac < 2.0 * BFRAC_OPEN)

    return {"path": str(prim.GetPath()),
            "n_points": int(len(P)), "n_faces": int(len(counts)), "n_tris": int(len(T)),
            "extent_m": [round(float(v), 4) for v in ext],
            "thin_m": round(thin, 5),
            "area_m2": round(area, 3),
            "n_edges": n_edges, "n_boundary_edges": n_boundary,
            "n_nonmanifold_edges": n_nonmanifold,
            "boundary_edge_fraction": round(bfrac, 4),
            "boundary_edge_fraction_unwelded": round(bfrac_raw, 4) if bfrac_raw is not None else None,
            "boundary_edge_fraction_weld_1cm": round(bfrac_coarse, 4) if bfrac_coarse is not None else None,
            "weld_sensitive": bool(bfrac_coarse is not None
                                   and (bfrac > BFRAC_OPEN) != (bfrac_coarse > BFRAC_OPEN)),
            "flat_plate": flat_plate,
            "class": klass,
            "decided_by": decided_by,
            "has_back_sheet": two_sided,
            "rim_closed": closed,
            "is_panel": is_panel,
            "two_sided_area_ratio": round(two_sided_ratio, 4),
            "panel_thickness_m": round(panel_thickness, 5) if is_panel else None,
            "panel_verdict": panel_verdict,
            "tests_agree": (None if panel_verdict is None else
                            bool((panel_verdict == "ONE_SIDED") == open_shell)),
            "thickness_m": (round(panel_thickness, 5) if klass == "SOLID_PANEL"
                            else (round(thin, 5) if klass == "CLOSED_SOLID" else 0.0)),
            "marginal": marginal,
            "zero_thickness": zero}


def roll_up(meshes):
    """Area-weighted verdict over a list of per-mesh records."""
    n = len(meshes)
    nz = sum(1 for m in meshes if m["zero_thickness"])
    at = sum(m["area_m2"] for m in meshes)
    az = sum(m["area_m2"] for m in meshes if m["zero_thickness"])
    frac = (az / at) if at > 0 else 0.0
    verdict = "SOLID" if frac < SOLID_MAX else ("SHELL" if frac > SHELL_MIN else "MIXED")
    cls = {}
    acls = {}
    for m in meshes:
        cls[m["class"]] = cls.get(m["class"], 0) + 1
        acls[m["class"]] = acls.get(m["class"], 0.0) + m["area_m2"]
    solid = [m for m in meshes if m["class"] in ("CLOSED_SOLID", "SOLID_PANEL")]
    return {"n_meshes": n, "n_zero": nz,
            "n_flat_sheet": cls.get("FLAT_SHEET", 0),
            "n_no_back_face": cls.get("NO_BACK_FACE", 0),
            "n_loose_sheets": cls.get("LOOSE_SHEETS", 0),
            "n_open_envelope": cls.get("OPEN_ENVELOPE", 0),
            "n_solid_panel": cls.get("SOLID_PANEL", 0),
            "n_closed_solid": cls.get("CLOSED_SOLID", 0),
            "n_single_surface": (cls.get("FLAT_SHEET", 0) + cls.get("NO_BACK_FACE", 0)
                                 + cls.get("LOOSE_SHEETS", 0)
                                 + cls.get("OPEN_ENVELOPE", 0)),
            "n_closed_thin": cls.get("FLAT_SHEET", 0),
            "area_single_surface_m2": round(acls.get("FLAT_SHEET", 0.0)
                                            + acls.get("NO_BACK_FACE", 0.0)
                                            + acls.get("LOOSE_SHEETS", 0.0)
                                            + acls.get("OPEN_ENVELOPE", 0.0), 2),
            "area_flat_sheet_m2": round(acls.get("FLAT_SHEET", 0.0), 2),
            "area_no_back_face_m2": round(acls.get("NO_BACK_FACE", 0.0), 2),
            "area_loose_sheets_m2": round(acls.get("LOOSE_SHEETS", 0.0), 2),
            "area_open_envelope_m2": round(acls.get("OPEN_ENVELOPE", 0.0), 2),
            "area_closed_thin_m2": round(acls.get("FLAT_SHEET", 0.0), 2),
            "area_closed_solid_m2": round(acls.get("SOLID_PANEL", 0.0)
                                          + acls.get("CLOSED_SOLID", 0.0), 2),
            "n_marginal": sum(1 for m in meshes if m["marginal"]),
            "n_panels": sum(1 for m in meshes if m["is_panel"]),
            "n_panels_one_sided": sum(1 for m in meshes if m.get("panel_verdict") == "ONE_SIDED"),
            "n_panels_two_sided": sum(1 for m in meshes if m.get("panel_verdict") in ("TWO_SIDED", "TWO_SIDED_THIN")),
            "n_tests_disagree": sum(1 for m in meshes if m.get("tests_agree") is False),
            "panel_thickness_m": (round(float(np.median(
                [m["panel_thickness_m"] for m in meshes
                 if m.get("panel_verdict") == "TWO_SIDED"])), 4)
                if any(m.get("panel_verdict") == "TWO_SIDED" for m in meshes) else None),
            "solid_thickness_m": (round(float(np.median([m["thickness_m"] for m in solid])), 4)
                                  if solid else None),
            "n_weld_sensitive": sum(1 for m in meshes if m.get("weld_sensitive")),
            "max_bfrac": round(max([m["boundary_edge_fraction"] for m in meshes]), 4) if meshes else None,
            "median_bfrac": round(float(np.median([m["boundary_edge_fraction"] for m in meshes])), 4) if meshes else None,
            "area_m2": round(at, 2), "area_zero_m2": round(az, 2),
            "area_zero_frac": round(frac, 4),
            "min_thin_m": round(min([m["thin_m"] for m in meshes]), 5) if meshes else None,
            "verdict": verdict}


# ===========================================================================
# One stage
# ===========================================================================
def measure_stage(url, keep_meshes=False):
    """Open *url*, measure every Mesh, roll up.  Never raises: an unopenable
    asset comes back with `error` set so the report can SAY it could not be
    read rather than quietly dropping it from the denominator."""
    t = time.time()
    try:
        st = Usd.Stage.Open(url)
    except Exception as exc:                                   # noqa: BLE001
        return {"url": url, "error": "open raised: %s" % exc}
    if st is None:
        return {"url": url, "error": "Usd.Stage.Open returned None"}
    try:
        mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
        up = UsdGeom.GetStageUpAxis(st)
        recs, bad = [], []
        # TRAVERSE INSTANCE PROXIES.  A plain `stage.Traverse()` stops at an
        # instanceable prim and reports zero meshes for an asset that is one
        # prototype instanced twenty times — which would read as "no geometry"
        # rather than as an error.
        rng_ = Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies(
            Usd.PrimIsActive & Usd.PrimIsDefined & ~Usd.PrimIsAbstract))
        for prim in rng_:
            if not prim.IsA(UsdGeom.Mesh):
                continue
            try:
                r = measure_mesh(prim, mpu)
            except Exception as exc:                           # noqa: BLE001
                bad.append("%s: %s" % (prim.GetPath(), exc))
                continue
            if r is not None:
                recs.append(r)
        out = roll_up(recs)
        out.update({"url": url, "meters_per_unit": mpu, "up_axis": str(up),
                    "n_mesh_prims_skipped": len(bad), "open_s": round(time.time() - t, 2)})
        if bad:
            out["mesh_errors"] = bad[:10]
        if keep_meshes:
            out["meshes"] = recs
        return out
    except Exception as exc:                                   # noqa: BLE001
        return {"url": url, "error": "traverse raised: %s\n%s"
                                     % (exc, traceback.format_exc(limit=3))}


_cache = {}
if os.path.exists(CACHE) and not REFRESH:
    try:
        _cache = json.load(open(CACHE))
        log("cache: %d entries from %s" % (len(_cache), CACHE))
    except Exception:                                          # noqa: BLE001
        _cache = {}


def measure_cached(url):
    if url in _cache:
        return _cache[url]
    r = measure_stage(url)
    _cache[url] = r
    return r


def save_cache():
    tmp = CACHE + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(_cache, fh)
    os.replace(tmp, CACHE)


# ===========================================================================
# POPULATION 1 — the 16 kit styles
# ===========================================================================
QUAKE_STYLES = ["apartment", "office", "brownstone", "commercial", "tower",
                "office_wide", "office_plain", "apartment_tall", "apartment_long",
                "walkup", "brownstone_row", "commercial_mid", "department_store",
                "dw_terrace", "civic_offices", "highrise_04"]


def _role(category, style):
    """wall / corner / parapet / roof / other, from the placement's category
    (`bld_<style>[_wingK]_<sub>`).  The sub names come from the band `sub`
    keys in urban_building's family builders."""
    sub = category.split("_")[-1] if category else ""
    if category.endswith("_corner"):
        return "corner"
    if sub == "roof":
        return "roof"
    if sub in ("parapet", "ledge", "cornice", "rooftop", "trim"):
        return "parapet"
    if category.endswith("_extra"):
        return "balcony"
    if sub in ("portico", "pediment", "ornament"):
        return "ornament"
    return "wall"


def _spec_pieces(spec, out):
    """Every piece NAME a style's spec can draw on, whether or not seed 4
    instances it."""
    for band in spec.get("bands", []):
        for key in ("walls", "back", "front"):
            for q in band.get(key) or []:
                out.add(q)
        fe = band.get("front_extra")
        if fe:
            out.add(fe[0])
        if band.get("corner"):
            out.add(band["corner"][0])
    if spec.get("roof"):
        out.add(spec["roof"][0])
    for name, _fx, _fy, _dz in spec.get("ornaments", []) or []:
        out.add(name)
    p = spec.get("portico")
    if p:
        for q in p["column"]:
            out.add(q)
        out.add(p["pediment"][0])
    wings = list(spec.get("wings", []) or [])
    if spec.get("tower"):
        wings.append((spec["tower"], None))
    for w, _off in wings:
        _spec_pieces(w, out)
    return out


def population_kit():
    log("=== POPULATION 1: kit modules of the %d earthquake styles (seed %d) ==="
        % (len(QUAKE_STYLES), SEED))
    styles = {}
    modules = {}                       # module name -> measurement + roles seen

    for style in QUAKE_STYLES:
        spec = ub.STYLES[style]
        pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(SEED))
        used = {}
        for p in pls:
            name = os.path.splitext(os.path.basename(p["usd"]))[0]
            role = _role(p.get("category", ""), style)
            k = (name, role)
            used[k] = used.get(k, 0) + 1
        pool = _spec_pieces(spec, set())
        styles[style] = {"used": used, "pool": pool, "n_placements": len(pls)}
        for (name, _role_), _n in used.items():
            modules.setdefault(name, set())
        for name in pool:
            modules.setdefault(name, set())
    log("  %d distinct kit modules across the 16 styles" % len(modules))

    # --- measure each module ONCE -----------------------------------------
    mres = {}
    for i, name in enumerate(sorted(modules)):
        url = ub._usd(name)
        r = measure_cached(url)
        mres[name] = r
        if "error" in r:
            log("  [%3d/%3d] %-44s  COULD NOT OPEN: %s" % (i + 1, len(modules), name, r["error"]))
        else:
            log("  [%3d/%3d] %-44s  %-5s  meshes=%-3d zero=%-3d area0=%.2f thin=%.4f"
                % (i + 1, len(modules), name, r["verdict"], r["n_meshes"],
                   r["n_zero"], r["area_zero_frac"], r["min_thin_m"] or -1))
        if (i + 1) % 20 == 0:
            save_cache()
    save_cache()

    # --- roll up per style -------------------------------------------------
    out = {}
    for style in QUAKE_STYLES:
        s = styles[style]
        by_role = {}
        shells, unread = [], []
        tot = {"n": 0, "nz": 0, "a": 0.0, "az": 0.0,
               "a_ss": 0.0, "a_ct": 0.0, "a_cs": 0.0}
        for (name, role), n in sorted(s["used"].items()):
            r = mres[name]
            b = by_role.setdefault(role, {"pieces": 0, "instances": 0,
                                          "pieces_zero": 0, "instances_zero": 0,
                                          "area_m2": 0.0, "area_zero_m2": 0.0,
                                          "area_single_surface_m2": 0.0,
                                          "area_closed_thin_m2": 0.0,
                                          "area_closed_solid_m2": 0.0,
                                          "shell_modules": []})
            b["pieces"] += 1
            b["instances"] += n
            if "error" in r:
                unread.append(name)
                continue
            is_shell = r["verdict"] != "SOLID"
            if is_shell:
                b["pieces_zero"] += 1
                b["instances_zero"] += n
                b["shell_modules"].append("%s(%s,%.2f)" % (name, r["verdict"], r["area_zero_frac"]))
                if name not in shells:
                    shells.append(name)
            b["area_m2"] += r["area_m2"] * n
            b["area_zero_m2"] += r["area_zero_m2"] * n
            for kk in ("area_single_surface_m2", "area_closed_thin_m2",
                       "area_closed_solid_m2"):
                b[kk] += r.get(kk, 0.0) * n
            tot["n"] += 1
            tot["nz"] += 1 if is_shell else 0
            tot["a"] += r["area_m2"] * n
            tot["az"] += r["area_zero_m2"] * n
            tot["a_ss"] += r.get("area_single_surface_m2", 0.0) * n
            tot["a_ct"] += r.get("area_closed_thin_m2", 0.0) * n
            tot["a_cs"] += r.get("area_closed_solid_m2", 0.0) * n
        for b in by_role.values():
            for kk in ("area_single_surface_m2", "area_closed_thin_m2",
                       "area_closed_solid_m2"):
                b[kk] = round(b[kk], 1)
            b["area_m2"] = round(b["area_m2"], 1)
            b["area_zero_m2"] = round(b["area_zero_m2"], 1)
            b["area_zero_frac"] = round(b["area_zero_m2"] / b["area_m2"], 4) if b["area_m2"] else 0.0
        frac = (tot["az"] / tot["a"]) if tot["a"] else 0.0
        out[style] = {
            "n_placements": s["n_placements"],
            "modules_used": tot["n"], "modules_zero": tot["nz"],
            "area_m2": round(tot["a"], 1), "area_zero_m2": round(tot["az"], 1),
            "area_zero_frac": round(frac, 4),
            "area_single_surface_frac": round(tot["a_ss"] / tot["a"], 4) if tot["a"] else 0.0,
            "area_closed_thin_frac": round(tot["a_ct"] / tot["a"], 4) if tot["a"] else 0.0,
            "area_closed_solid_frac": round(tot["a_cs"] / tot["a"], 4) if tot["a"] else 0.0,
            "verdict": "SOLID" if frac < SOLID_MAX else ("SHELL" if frac > SHELL_MIN else "MIXED"),
            "by_role": by_role,
            "shell_modules": shells,
            "unreadable_modules": unread,
            "pool_not_instanced_at_seed": sorted(
                n for n in s["pool"] if n not in {k[0] for k in s["used"]}),
        }
    return {"styles": out,
            "modules": {n: {k: v for k, v in r.items() if k != "meshes"}
                        for n, r in mres.items()}}


# ===========================================================================
# POPULATION 1b — the flattened DG0 archetypes (what the fracture code sees)
# ===========================================================================
ARCH_DIR = os.path.join(SCENE_GEN, "assets", "archetype")


def population_archetypes():
    log("=== POPULATION 1b: baked DG0 archetypes (cross-check) ===")
    out = {}
    for style in QUAKE_STYLES:
        p = os.path.join(ARCH_DIR, "bld_%s_DG0.usd" % style)
        if not os.path.exists(p):
            out[style] = {"error": "missing: %s" % p}
            log("  %-18s MISSING %s" % (style, p))
            continue
        r = measure_stage(p)
        out[style] = r
        if "error" in r:
            log("  %-18s ERROR %s" % (style, r["error"]))
        else:
            log("  %-18s %-5s meshes=%-4d zero=%-4d area_zero=%.2f thin=%.4f"
                % (style, r["verdict"], r["n_meshes"], r["n_zero"],
                   r["area_zero_frac"], r["min_thin_m"]))
    return out


# ===========================================================================
# POPULATION 2 — last night's urban_v2 drop
# ===========================================================================
URBAN_V2 = os.path.join(SCENE_GEN, "config", "asset_sets", "urban_v2.yaml")
BUILDING_POOLS = ("tower+", "midrise+", "rowhouse+", "destroyed+")


def _folder(url):
    """The group the user thinks in: which folder of the drop it came from."""
    tail = url.split("/Projects/SEI-COA/")[-1]
    parts = tail.split("/")
    if parts[0] == "selected_citydemo":
        return "selected_citydemo/" + parts[1]
    if "downtowncity" in tail.lower():
        return "DowntownCity"
    if "standalone" in parts:
        i = parts.index("standalone")
        return "standalone/" + "/".join(parts[i + 1:i + 4])
    return "/".join(parts[:-1])


def population_drop():
    log("=== POPULATION 2: urban_v2.yaml building pools ===")
    doc = yaml.safe_load(open(URBAN_V2))
    blds = doc["usds"]["buildings"]
    entries = []
    for pool in BUILDING_POOLS:
        for e in blds.get(pool, []) or []:
            if isinstance(e, dict) and e.get("usd"):
                entries.append({"pool": pool, "url": e["usd"],
                                "material": e.get("material"),
                                "name": os.path.splitext(os.path.basename(e["usd"]))[0],
                                "folder": _folder(e["usd"])})
    log("  %d building entries in %s" % (len(entries), ", ".join(BUILDING_POOLS)))
    seen = {}
    for i, e in enumerate(entries):
        r = measure_cached(e["url"])
        e.update({k: v for k, v in r.items() if k not in ("url", "meshes")})
        if "error" in r:
            log("  [%3d/%3d] %-34s %-26s COULD NOT OPEN: %s"
                % (i + 1, len(entries), e["name"], e["folder"], r["error"]))
        else:
            log("  [%3d/%3d] %-34s %-26s %-5s meshes=%-4d zero=%-4d area_zero=%.2f thin=%.4f %.1fs"
                % (i + 1, len(entries), e["name"], e["folder"], r["verdict"],
                   r["n_meshes"], r["n_zero"], r["area_zero_frac"],
                   r["min_thin_m"] if r["min_thin_m"] is not None else -1, r["open_s"]))
        if e["url"] in seen:
            e["duplicate_of"] = seen[e["url"]]
        seen.setdefault(e["url"], e["name"])
        if (i + 1) % 10 == 0:
            save_cache()
    save_cache()
    return entries


def group_drop(entries):
    g = {}
    for e in entries:
        b = g.setdefault(e["folder"], {"n": 0, "n_zero": 0, "n_shell": 0, "n_mixed": 0,
                                       "n_solid": 0, "n_unreadable": 0,
                                       "area_m2": 0.0, "area_zero_m2": 0.0,
                                       "area_single_surface_m2": 0.0,
                                       "area_closed_thin_m2": 0.0,
                                       "area_closed_solid_m2": 0.0,
                                       "shell_buildings": [], "mixed_buildings": [],
                                       "unreadable": [], "pools": {}})
        b["n"] += 1
        b["pools"][e["pool"]] = b["pools"].get(e["pool"], 0) + 1
        if "error" in e:
            b["n_unreadable"] += 1
            b["unreadable"].append(e["name"])
            continue
        b["area_m2"] += e["area_m2"]
        b["area_zero_m2"] += e["area_zero_m2"]
        for kk in ("area_single_surface_m2", "area_closed_thin_m2",
                   "area_closed_solid_m2"):
            b[kk] += e.get(kk, 0.0)
        if e["verdict"] == "SHELL":
            b["n_shell"] += 1
            b["shell_buildings"].append(e["name"])
        elif e["verdict"] == "MIXED":
            b["n_mixed"] += 1
            b["mixed_buildings"].append(e["name"])
        else:
            b["n_solid"] += 1
        if e["verdict"] != "SOLID":
            b["n_zero"] += 1
    for b in g.values():
        b["area_zero_frac"] = round(b["area_zero_m2"] / b["area_m2"], 4) if b["area_m2"] else 0.0
        b["area_single_surface_frac"] = round(b["area_single_surface_m2"] / b["area_m2"], 4) if b["area_m2"] else 0.0
        for kk in ("area_single_surface_m2", "area_closed_thin_m2",
                   "area_closed_solid_m2"):
            b[kk] = round(b[kk], 1)
        b["area_m2"] = round(b["area_m2"], 1)
        b["area_zero_m2"] = round(b["area_zero_m2"], 1)
    return g


# ===========================================================================
# Tables
# ===========================================================================
def print_tables(res):
    W = "=" * 118
    print("\n" + W)
    print("TABLE A — the 16 kit styles the earthquake pipeline damages "
          "(modules measured at source, rolled up at seed %d)" % SEED)
    print(W)
    print("%-17s %-6s %5s %5s %8s %8s %8s  %s"
          % ("style", "verd.", "mods", "zero", "area0%", "1surf%", "solid%", "SHELL modules"))
    print("-" * 118)
    for style in QUAKE_STYLES:
        s = res["kit"]["styles"][style]
        sm = ", ".join(s["shell_modules"][:4]) + ("  (+%d)" % (len(s["shell_modules"]) - 4)
                                                  if len(s["shell_modules"]) > 4 else "")
        print("%-17s %-6s %5d %5d %7.1f%% %7.1f%% %7.1f%%  %s"
              % (style, s["verdict"], s["modules_used"], s["modules_zero"],
                 100 * s["area_zero_frac"], 100 * s["area_single_surface_frac"],
                 100 * s["area_closed_solid_frac"], sm or "-"))
        for role in ("wall", "corner", "parapet", "roof", "balcony", "ornament"):
            b = s["by_role"].get(role)
            if not b:
                continue
            print("      %-11s pieces %2d (%2d zero)   instances %4d (%4d zero)   area0 %5.1f%%"
                  % (role, b["pieces"], b["pieces_zero"], b["instances"],
                     b["instances_zero"], 100 * b["area_zero_frac"]))
        if s["unreadable_modules"]:
            print("      UNREADABLE: %s" % ", ".join(s["unreadable_modules"]))

    print("\n" + W)
    print("TABLE A2 — the baked DG0 archetypes (independent check: this is the "
          "geometry the fracture pass actually slices)")
    print(W)
    print("%-17s %-6s %6s %6s %8s %10s" % ("style", "verd.", "meshes", "zero", "area0%", "thinnest"))
    print("-" * 118)
    for style in QUAKE_STYLES:
        a = res["archetypes"][style]
        if "error" in a:
            print("%-17s ERROR %s" % (style, a["error"]))
            continue
        print("%-17s %-6s %6d %6d %7.1f%% %9.4f m"
              % (style, a["verdict"], a["n_meshes"], a["n_zero"],
                 100 * a["area_zero_frac"], a["min_thin_m"]))

    print("\n" + W)
    print("TABLE B — the 2026-08-26 urban_v2 drop, by folder")
    print(W)
    print("%-34s %5s %6s %6s %6s %6s %8s %8s" %
          ("folder", "n", "SOLID", "MIXED", "SHELL", "unread", "area0%", "1surf%"))
    print("-" * 118)
    g = res["drop"]["by_folder"]
    for f in sorted(g, key=lambda k: -g[k]["n"]):
        b = g[f]
        print("%-34s %5d %6d %6d %6d %6d %7.1f%% %7.1f%%"
              % (f, b["n"], b["n_solid"], b["n_mixed"], b["n_shell"],
                 b["n_unreadable"], 100 * b["area_zero_frac"],
                 100 * b["area_single_surface_frac"]))
    t = res["drop"]["totals"]
    print("-" * 118)
    print("%-34s %5d %6d %6d %6d %6d %7.1f%% %7.1f%%"
          % ("TOTAL", t["n"], t["n_solid"], t["n_mixed"], t["n_shell"],
             t["n_unreadable"], 100 * t["area_zero_frac"],
             100 * t["area_single_surface_frac"]))
    print("\nSHELL / MIXED buildings by folder:")
    for f in sorted(g):
        b = g[f]
        if b["shell_buildings"]:
            print("  %-32s SHELL: %s" % (f, ", ".join(b["shell_buildings"])))
        if b["mixed_buildings"]:
            print("  %-32s MIXED: %s" % (f, ", ".join(b["mixed_buildings"])))
        if b["unreadable"]:
            print("  %-32s UNREADABLE: %s" % (f, ", ".join(b["unreadable"])))

    print("\n" + W)
    print("TABLE C — is the verdict an artefact of the weld tolerance?  "
          "(boundary-edge fraction recomputed at 0 mm / 0.1 mm / 10 mm)")
    print(W)
    rb = res["robustness"]
    print("  stages measured                       %d" % rb["n_stages"])
    print("  meshes measured                       %d" % rb["n_meshes"])
    print("  meshes whose OPEN/CLOSED verdict flips between the 0.1 mm and "
          "10 mm weld   %d  (%.2f%%)" % (rb["n_weld_sensitive"],
                                         100.0 * rb["weld_sensitive_frac"]))
    print("  stages with max boundary fraction == 0.0 (fully closed)        "
          "  %d" % rb["n_stages_fully_closed"])
    print("  stages with median boundary fraction > 0.5 (mostly loose faces)"
          "  %d" % rb["n_stages_mostly_open"])
    print("  meshes within a factor 2 of the 0.05 threshold (borderline)      "
          "  %d  (%.2f%%)" % (rb["n_marginal_meshes"],
                              100.0 * rb["n_marginal_meshes"] / max(1, rb["n_meshes"])))
    print("  mesh classes (zero-thickness first three):")
    print("    FLAT_SHEET %d   NO_BACK_FACE %d   LOOSE_SHEETS %d   OPEN_ENVELOPE %d"
          "   |   SOLID_PANEL %d   CLOSED_SOLID %d"
          % (rb["n_flat_sheet"], rb["n_no_back_face"], rb["n_loose_sheets"],
             rb["n_open_envelope"], rb["n_solid_panel"], rb["n_closed_solid"]))
    print("  SECOND OPINION on panel-sized meshes (does a back face exist?):")
    print("    panels %d -> ONE_SIDED %d, TWO_SIDED %d;  disagreements with the "
          "boundary-edge test: %d (%.2f%% of panels)"
          % (rb["n_panels"], rb["n_panels_one_sided"], rb["n_panels_two_sided"],
             rb["n_tests_disagree"],
             100.0 * rb["n_tests_disagree"] / max(1, rb["n_panels"])))

    h = res["headline"]
    print("\n" + W)
    print("HEADLINE")
    print(W)
    for line in h["lines"]:
        print("  " + line)
    print(W + "\n")


# ===========================================================================
def main():
    res = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"),
           "tool": "wall_thickness_census.py (plain pxr, no SimulationApp)",
           "thresholds": {"boundary_edge_fraction_open": BFRAC_OPEN,
                          "thin_m": THIN_M, "big_m": BIG_M,
                          "solid_max_area_frac": SOLID_MAX,
                          "shell_min_area_frac": SHELL_MIN,
                          "weld_m": WELD_M},
           "arch_seed": SEED}
    res["kit"] = population_kit()
    res["archetypes"] = population_archetypes()
    entries = population_drop()
    by_folder = group_drop(entries)
    tot = {"n": len(entries),
           "n_solid": sum(1 for e in entries if e.get("verdict") == "SOLID"),
           "n_mixed": sum(1 for e in entries if e.get("verdict") == "MIXED"),
           "n_shell": sum(1 for e in entries if e.get("verdict") == "SHELL"),
           "n_unreadable": sum(1 for e in entries if "error" in e)}
    at = sum(e.get("area_m2", 0.0) for e in entries)
    az = sum(e.get("area_zero_m2", 0.0) for e in entries)
    tot["area_m2"] = round(at, 1)
    tot["area_zero_m2"] = round(az, 1)
    tot["area_zero_frac"] = round(az / at, 4) if at else 0.0
    for kk in ("area_single_surface_m2", "area_closed_thin_m2", "area_closed_solid_m2"):
        tot[kk] = round(sum(e.get(kk, 0.0) for e in entries), 1)
    tot["area_single_surface_frac"] = round(tot["area_single_surface_m2"] / at, 4) if at else 0.0
    tot["n_zero"] = tot["n_mixed"] + tot["n_shell"]
    res["drop"] = {"entries": entries, "by_folder": by_folder, "totals": tot}

    # --- the two numbers the user asked for --------------------------------
    kit_styles = res["kit"]["styles"]
    kit_zero = sum(1 for s in kit_styles.values() if s["verdict"] != "SOLID")
    lib_total = len(kit_styles) + tot["n"]
    lib_zero = kit_zero + tot["n_zero"]
    res["headline"] = {
        "kit_styles_total": len(kit_styles), "kit_styles_zero": kit_zero,
        "drop_total": tot["n"], "drop_zero": tot["n_zero"],
        "library_total": lib_total, "library_zero": lib_zero,
        "lines": [
            "kit styles (earthquake pipeline): %d of %d are not solid "
            "(%d SHELL, %d MIXED)"
            % (kit_zero, len(kit_styles),
               sum(1 for s in kit_styles.values() if s["verdict"] == "SHELL"),
               sum(1 for s in kit_styles.values() if s["verdict"] == "MIXED")),
            "last night's urban_v2 drop: %d of %d buildings are not solid "
            "(%d SHELL, %d MIXED, %d unreadable)"
            % (tot["n_zero"], tot["n"], tot["n_shell"], tot["n_mixed"], tot["n_unreadable"]),
            "whole library (16 kit styles + %d drop buildings = %d): %d are "
            "zero-wall-thickness; %d of those came in last night's drop"
            % (tot["n"], lib_total, lib_zero, tot["n_zero"]),
        ]}

    # --- robustness roll-up ------------------------------------------------
    stages = ([r for r in res["kit"]["modules"].values()]
              + [r for r in res["archetypes"].values()]
              + [e for e in entries])
    ok = [r for r in stages if "error" not in r and r.get("n_meshes")]
    res["robustness"] = {
        "n_stages": len(ok),
        "n_meshes": sum(r["n_meshes"] for r in ok),
        "n_weld_sensitive": sum(r.get("n_weld_sensitive") or 0 for r in ok),
        "weld_sensitive_frac": round(
            sum(r.get("n_weld_sensitive") or 0 for r in ok)
            / float(max(1, sum(r["n_meshes"] for r in ok))), 5),
        "n_stages_fully_closed": sum(1 for r in ok if (r.get("max_bfrac") or 0) == 0.0),
        "n_stages_mostly_open": sum(1 for r in ok if (r.get("median_bfrac") or 0) > 0.5),
        "n_marginal_meshes": sum(r.get("n_marginal") or 0 for r in ok),
        "n_single_surface": sum(r.get("n_single_surface") or 0 for r in ok),
        "n_closed_thin": sum(r.get("n_closed_thin") or 0 for r in ok),
        "n_flat_sheet": sum(r.get("n_flat_sheet") or 0 for r in ok),
        "n_no_back_face": sum(r.get("n_no_back_face") or 0 for r in ok),
        "n_loose_sheets": sum(r.get("n_loose_sheets") or 0 for r in ok),
        "n_open_envelope": sum(r.get("n_open_envelope") or 0 for r in ok),
        "n_solid_panel": sum(r.get("n_solid_panel") or 0 for r in ok),
        "n_closed_solid": sum(r.get("n_closed_solid") or 0 for r in ok),
        "n_panels": sum(r.get("n_panels") or 0 for r in ok),
        "n_panels_one_sided": sum(r.get("n_panels_one_sided") or 0 for r in ok),
        "n_panels_two_sided": sum(r.get("n_panels_two_sided") or 0 for r in ok),
        "n_tests_disagree": sum(r.get("n_tests_disagree") or 0 for r in ok),
    }

    tmp = OUT + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(res, fh, indent=1, default=lambda o: sorted(o) if isinstance(o, set) else str(o))
    os.replace(tmp, OUT)
    log("wrote %s" % OUT)
    print_tables(res)


if __name__ == "__main__":
    main()
