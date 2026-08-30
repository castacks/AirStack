#!/usr/bin/env python
"""_vtk_shell_probe — WHY `fracture.fracture_prim` DIES ON A CLIPPED SHELL.

The fire session recorded (memory `gac-fire-pipeline`) that handing a piece
produced by `detail/gac_storey_slice.slice_to_kit` — or a `frag_*` remainder,
or a referenced Xform asset — to `fracture.fracture_prim` SEGFAULTS inside
VTK, while a kit module from `detail/urban_building.py` fractures fine. The
earthquake sliced ladder (round 4, package G) needs ragged breaks on those
pieces, so the crash has to be attributed to ONE stage of the chain.

WHAT THIS DOES. It runs the real chain on real geometry, offline (bare
container python, no Kit, no GPU, safe beside a running sim), STAGE BY STAGE,
ONE SUBPROCESS PER STAGE, with `faulthandler` on. A stage that dies takes only
its own child with it and the parent records the signal plus the last `@@STEP`
the child printed, so a SIGSEGV is attributed to a named call rather than to
"the fracture".

    # everything (slice + all stages, sliced pieces AND kit modules)
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tools/_vtk_shell_probe.py \
       --asset SM_Building_09 --out /tmp/_vtk_shell"

    # re-run stages against the layer the last prep exported (no re-slice)
    ... _vtk_shell_probe.py --reuse --out /tmp/_vtk_shell --only slice_cap_fan

`--reuse` skips the slice (2-4 min) and reads `<out>/pieces.usda`, which holds
ONLY the selected pieces (copied spec by spec out of the sliced stage) plus the
kit modules as references, so `prim_to_mesh` sees exactly what it would see in
a real bake.

WHAT IT FOUND (2026-08-30). Nothing in the chain crashes offline: 254 sliced
pieces of SM_Building_09 plus two MCE kit modules, x 8 rng seeds, x the whole
ladder (`prim_to_mesh`, `solidify`, one capped/uncapped/contour slice, uniform
/ brick / prism / plank `fracture_mesh`, `fracture_prim` with `_break`'s own
kwargs, and a re-fracture of a `frag_*`) = 2,048 fractures, 12,147 fragments,
zero signals. The crash was found instead in Kit's own dump for the run that
died (`70cfa653-7231-48aa-81e159b6-f5f287c9`, gac_fire bench, SM_Building_09
F6): py-spy puts it at `_vtk_slice` line 329 — `strip.Update()` — and the
native frame under it is `vtkStripper::RequestData -> vtkPolyData::
GetPointCells -> SIGSEGV`. `--only selftest_*` reproduces exactly that, from
two lines of numpy, and shows the guard in `fracture.py` turning it into a
dropped cap:

    selftest_clean  62 cap triangles, guard on or off        (unchanged)
    selftest_oob    SIGSEGV        -> guard on: cap skipped   (THE CRASH)
    selftest_dup    0 cap triangles-> guard on: 62 restored   (a real bug too)
    selftest_degen  3 broken chains-> guard on: 1 clean loop

MEASURED PER STAGE: vertex/face counts in and out, duplicate-vertex share
(the de-indexed slicer emits three unique vertices per triangle — that is the
first suspect), zero-area triangles, boundary and NON-MANIFOLD edge counts,
`is_watertight`, wall time, peak RSS, the VTK version and
`vtkIdTypeArray().GetDataTypeSize()` (a 32-bit-id VTK fed int64 ids through
`numpy_to_vtkIdTypeArray` would explain a crash only big meshes reach).
"""
import argparse
import faulthandler
import json
import os
import random
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

GAC_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
GAC_SCALE = 0.01                 # the pack is authored in centimetres
KIT_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "ModernCityEnvironment01/Meshes/")
# `urban_building.PIECES` family 04 (brick commercial) — the modules the
# earthquake kit ladder actually fractures today.
KIT_MODULES = ("SM_MBuilding04_Facade_A", "SM_MBuilding04_FirstFloor_A")

# quake_flow.T_SOLID_M["urm"], copied rather than imported: importing
# quake_flow drags the whole earthquake stack (and its env knobs) into a probe
# whose whole point is to isolate one call.
T_URM = {"wall": 0.38, "pier": 0.38, "corner": 0.40, "parapet": 0.25,
         "parapet_corner": 0.25, "balcony": 0.18, "roof": 0.20, "core": 0.30,
         "upstand": 0.25, "deck": 0.20, None: 0.30}

# `raw_*` stages SKIP `solidify`, because the call that actually crashed does:
# `urban_fire`'s roof-lid shatter passes no `solid_m` at all, mode="plank",
# n = clamp(area/6, 8, 40), min_volume_frac=0.0008 — and `solidify` welds and
# closes the mesh on the way past, so a probe that always solidifies cannot see
# what the lid shatter sees.
STAGES = ("prim_to_mesh", "solidify", "slice_nocap", "slice_cap_fan",
          "slice_cap_contour", "frac_uniform", "frac_brick", "frac_prism",
          "fracture_prim", "refracture",
          "raw_slice_nocap", "raw_slice_cap_fan", "raw_uniform", "raw_plank",
          "raw_lid_plank", "raw_refracture",
          "selftest_clean", "selftest_oob", "selftest_dup", "selftest_degen")
RAW = tuple(q for q in STAGES if q.startswith("raw_"))
# `slice_cap_contour` is the filter the code itself documents as able to hang;
# it gets a shorter leash than the rest.
TIMEOUT = {"slice_cap_contour": 240}


# ---------------------------------------------------------------------------
# measurement
# ---------------------------------------------------------------------------
def _rss_mb():
    import resource
    return round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 1)


def mesh_stats(mesh, edges=True):
    """Everything about a mesh that could plausibly kill VTK."""
    if mesh is None:
        return {"nv": 0, "nf": 0}
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces, dtype=np.int64)
    out = {"nv": int(len(v)), "nf": int(len(f))}
    if not len(f):
        return out
    # duplicate vertices: the de-indexed slicer emits 3 unique points per
    # triangle and shares NOTHING, so this is ~0% unique-by-position sharing.
    key = np.round(v / 1e-6).astype(np.int64)
    nu = len(np.unique(key, axis=0))
    out["nv_uniq"] = int(nu)
    out["dup_pct"] = round(100.0 * (1.0 - nu / float(max(len(v), 1))), 1)
    a = np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]],
                                v[f[:, 2]] - v[f[:, 0]]), axis=1) * 0.5
    out["zero_area"] = int((a <= 1e-12).sum())
    out["area_m2"] = round(float(a.sum()), 3)
    out["deg_idx"] = int(((f[:, 0] == f[:, 1]) | (f[:, 1] == f[:, 2])
                          | (f[:, 2] == f[:, 0])).sum())
    out["nan"] = int(np.isnan(v).any()) + int(np.isinf(v).any())
    ext = v.max(0) - v.min(0)
    out["extents"] = [round(float(q), 2) for q in ext]
    if edges:
        e = np.sort(np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]),
                    axis=1)
        _u, cnt = np.unique(e, axis=0, return_counts=True)
        out["e_bound"] = int((cnt == 1).sum())
        out["e_nonmanifold"] = int((cnt > 2).sum())
        out["e_total"] = int(len(_u))
        # the same on the WELDED mesh — what VTK effectively sees once its own
        # locators merge coincident points
        try:
            from disaster import fracture as _fr
            w = _fr._weld(mesh)
            wf = np.asarray(w.faces, dtype=np.int64)
            we = np.sort(np.vstack([wf[:, [0, 1]], wf[:, [1, 2]],
                                    wf[:, [2, 0]]]), axis=1)
            _wu, wcnt = np.unique(we, axis=0, return_counts=True)
            out["w_nv"] = int(len(w.vertices))
            out["w_bound"] = int((wcnt == 1).sum())
            out["w_nonmanifold"] = int((wcnt > 2).sum())
        except Exception as exc:
            out["w_err"] = str(exc)[:80]
    try:
        out["watertight"] = bool(mesh.is_watertight)
    except Exception:
        out["watertight"] = None
    return out


def frag_stats(frags):
    if not frags:
        return {"n": 0}
    nf = [int(len(f.faces)) for f in frags]
    return {"n": len(frags), "nf_sum": int(sum(nf)), "nf_max": int(max(nf)),
            "wt": int(sum(1 for f in frags if f.is_watertight))}


def mark(step):
    sys.stdout.write("@@STEP {0} rss={1}\n".format(step, _rss_mb()))
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# CHILD — one stage, one process
# ---------------------------------------------------------------------------
def run_sweep(args):
    """EVERY item in the manifest through ONE stage in ONE process.

    A per-(item, stage) subprocess costs 1.5 s of USD import, which is fine for
    ten pieces and absurd for 254 — and 254 is the number that matters, because
    a crash that only one piece in a building triggers is invisible in a sample
    of seven. `@@STEP` before each piece attributes a segfault to the piece;
    faulthandler attributes it to the line.
    """
    faulthandler.enable()
    from pxr import Usd

    from disaster import fracture
    fracture.ensure_vtk(verbose=False)
    man = json.load(open(os.path.join(args.out, "manifest.json")))
    st = Usd.Stage.Open(man["layer"])
    rows, t_all = [], time.time()
    for k, it in enumerate(man["items"]):
        mark("{0}/{1} {2} {3}".format(k + 1, len(man["items"]), it["role"],
                                      it["prim"]))
        t0 = time.time()
        mesh = fracture.prim_to_mesh(st, it["prim"])
        if mesh is None:
            rows.append({"prim": it["prim"], "err": "no mesh"})
            continue
        solid = (0.0 if args.stage in RAW
                 else (args.solid_override if args.solid_override > 0
                       else float(it["solid"])))
        if solid:
            mesh = fracture.solidify(mesh, solid,
                                     ref=[float(q) for q in it["ref"].split(",")],
                                     verbose=False)
        mode = "plank" if args.stage.endswith("plank") else "uniform"
        kw = (dict(aspect=(1.4, 2.8), rough=0.01, consume=0.3,
                   consume_pool=1.05, min_volume_frac=0.0008)
              if mode == "plank" else
              dict(rough=0.014, consume=0.30, consume_pool=1.6,
                   min_volume_frac=0.002))
        # SEEDS ARE THE COVERAGE. One rng draws ONE set of Voronoi planes, and
        # a crash that needs a particular plane is invisible however many
        # pieces are walked. The live ladder draws a fresh seed per element.
        nfr = []
        for sd in range(max(1, args.seeds)):
            frags = fracture.fracture_mesh(mesh, args.n,
                                           np.random.default_rng(7 + sd),
                                           mode=mode, verbose=False, **kw)
            nfr.append(len(frags))
        rows.append({"prim": it["prim"], "role": it["role"],
                     "nf": int(len(mesh.faces)), "frags": nfr,
                     "s": round(time.time() - t0, 2), "rss": _rss_mb()})
    return {"sweep": args.stage, "n": len(rows), "rows": rows,
            "wall_s": round(time.time() - t_all, 1), "rss_mb": _rss_mb()}


# --- the crash itself, as a two-line synthetic ------------------------------
# `vtkStripper` builds its link table from the polydata's POINT COUNT and then
# walks it with ids taken from the CELLS, checking neither. One line cell
# naming a point that is not there is the whole bug.
def _selftest(case):
    import vtk
    from vtk.util import numpy_support as ns

    from disaster import fracture

    n = 64
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([np.cos(th), np.sin(th), np.zeros(n)], -1)
    segs = [(i, (i + 1) % n) for i in range(n)]
    if case == "oob":
        segs = segs + [(0, n + 5)]          # the crash
    elif case == "dup":
        segs = segs + segs                  # coincident decal quads
    elif case == "degen":
        segs = segs + [(3, 3), (10, 10)]
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(ring), deep=True))
    ca = vtk.vtkCellArray()
    for a, b in segs:
        ca.InsertNextCell(2)
        ca.InsertCellPoint(int(a))
        ca.InsertCellPoint(int(b))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetLines(ca)
    info = {"case": case, "in_lines": pd.GetNumberOfLines(),
            "in_pts": pd.GetNumberOfPoints(),
            "guard": os.environ.get("FRACTURE_VTK_GUARD", "1")}
    mark("_strip_input")
    src = fracture._strip_input(pd)
    info["accepted"] = src is not None
    info["kept_lines"] = 0 if src is None else src.GetNumberOfLines()
    if src is None:
        info["cap_tri"] = 0
        return info
    mark("vtkStripper.Update")
    st = vtk.vtkStripper()
    st.SetInputData(src)
    st.Update()
    lo = st.GetOutput()
    face = vtk.vtkPolyData()
    face.SetPoints(lo.GetPoints())
    face.SetPolys(lo.GetLines())
    tf = vtk.vtkTriangleFilter()
    tf.SetInputData(face)
    tf.Update()
    info["polylines"] = lo.GetNumberOfLines()
    info["cap_tri"] = tf.GetOutput().GetNumberOfPolys()
    return info


def run_stage(args):
    faulthandler.enable()
    import vtk
    from pxr import Usd

    from disaster import fracture

    if args.stage.startswith("selftest_"):
        return _selftest(args.stage.split("_", 1)[1])

    info = {"prim": args.prim, "stage": args.stage,
            "vtk": vtk.vtkVersion.GetVTKVersion(),
            "vtk_id_bytes": int(vtk.vtkIdTypeArray().GetDataTypeSize()),
            "np": np.__version__,
            "cap": os.environ.get("FRACTURE_CAP", "fan"),
            "solid_m": args.solid}
    fracture.ensure_vtk(verbose=False)
    mark("open_stage")
    st = Usd.Stage.Open(args.layer)

    mark("prim_to_mesh")
    t0 = time.time()
    mesh = fracture.prim_to_mesh(st, args.prim)
    info["t_prim_to_mesh"] = round(time.time() - t0, 2)
    info["src"] = mesh_stats(mesh)
    info["rss_mb"] = _rss_mb()
    if mesh is None:
        info["err"] = "prim_to_mesh returned None"
        return info
    if args.stage == "prim_to_mesh":
        return info

    ref = [float(q) for q in args.ref.split(",")] if args.ref else None
    if args.stage in RAW:
        info["solid_m"] = 0.0
    elif args.stage in ("solidify", "frac_uniform", "frac_brick", "frac_prism",
                        "refracture", "slice_nocap", "slice_cap_fan",
                        "slice_cap_contour"):
        mark("solidify")
        t0 = time.time()
        solid = fracture.solidify(mesh, args.solid, ref=ref, verbose=False)
        info["t_solidify"] = round(time.time() - t0, 2)
        info["solid"] = mesh_stats(solid)
        info["rss_mb"] = _rss_mb()
        if args.stage == "solidify":
            return info
        mesh = solid

    if args.stage in ("slice_nocap", "slice_cap_fan", "slice_cap_contour",
                      "raw_slice_nocap", "raw_slice_cap_fan"):
        # ONE plane, through the centroid, normal along the longest axis — the
        # cheapest thing `_cell` ever asks `slice_plane` to do.
        cap = not args.stage.endswith("slice_nocap")
        c = np.asarray(mesh.centroid, dtype=float)
        n = np.zeros(3)
        n[int(np.argmax(mesh.extents))] = 1.0
        mark("slice_plane(cap={0},CAP={1})".format(cap, info["cap"]))
        t0 = time.time()
        out = fracture.slice_plane(mesh, n, c, cap=cap)
        info["t_slice"] = round(time.time() - t0, 2)
        info["out"] = mesh_stats(out)
        info["rss_mb"] = _rss_mb()
        return info

    if args.stage in ("frac_uniform", "frac_brick", "frac_prism", "refracture",
                      "raw_uniform", "raw_plank", "raw_refracture"):
        mode = {"frac_uniform": "uniform", "frac_brick": "brick",
                "frac_prism": "prism", "refracture": "uniform",
                "raw_uniform": "uniform", "raw_plank": "plank",
                "raw_refracture": "uniform"}[args.stage]
        rng = np.random.default_rng(7)
        kw = dict(rough=0.012, consume=0.0, consume_pool=1.6,
                  min_volume_frac=0.002, blocky_m=args.solid)
        if mode == "plank":
            # `urban_fire`'s lid shatter, argument for argument
            kw = dict(mode_aspect=None, rough=0.01, consume=0.3,
                      consume_pool=1.05, min_volume_frac=0.0008)
            kw.pop("mode_aspect")
            kw["aspect"] = (1.4, 2.8)
        mark("fracture_mesh({0}, n={1})".format(mode, args.n))
        t0 = time.time()
        frags = fracture.fracture_mesh(mesh, args.n, rng, mode=mode,
                                       verbose=False, **kw)
        info["t_fracture"] = round(time.time() - t0, 2)
        info["frags"] = frag_stats(frags)
        info["rss_mb"] = _rss_mb()
        if args.stage not in ("refracture", "raw_refracture") or not frags:
            return info
        # A `frag_*` REMAINDER: the memory says re-cutting one of these is the
        # other crash. Take the biggest and put it back through the same call.
        big = max(frags, key=lambda f: float(np.prod(np.maximum(f.extents, 1e-9))))
        info["frag0"] = mesh_stats(big)
        mark("fracture_mesh(refracture)")
        t0 = time.time()
        f2 = fracture.fracture_mesh(big, max(3, args.n // 2), rng,
                                    mode="uniform", rough=0.012, consume=0.0,
                                    min_volume_frac=0.002, verbose=False)
        info["t_refracture"] = round(time.time() - t0, 2)
        info["frags2"] = frag_stats(f2)
        info["rss_mb"] = _rss_mb()
        return info

    if args.stage in ("fracture_prim", "raw_lid_plank"):
        if args.stage == "fracture_prim":
            # EXACTLY what `quake_flow._break` passes on the non-partial path.
            kw = dict(mode="uniform", rough=0.012, consume=0.0,
                      consume_pool=1.6, min_volume_frac=0.002,
                      max_piece_m=None, solid_m=args.solid, solid_ref=ref,
                      blocky_m=args.solid)
        else:
            # EXACTLY `urban_fire`'s roof-lid shatter (the call that crashed):
            # no `solid_m`, plank seeds, up to 40 cells, a very low volume
            # floor so thin plates survive.
            kw = dict(mode="plank", aspect=(1.4, 2.8), rough=0.01,
                      consume=0.3, consume_pool=1.05, min_volume_frac=0.0008)
        nrng = np.random.default_rng(7)
        mark(args.stage)
        t0 = time.time()
        made = fracture.fracture_prim(
            st, args.prim, "/W/brk", n_pieces=args.n, rng=nrng, verbose=False,
            **kw)
        info["t_fracture_prim"] = round(time.time() - t0, 2)
        info["made"] = len(made)
        info["rss_mb"] = _rss_mb()
        return info

    info["err"] = "unknown stage"
    return info


# ---------------------------------------------------------------------------
# PREP — slice one GAC asset, keep a handful of pieces + the kit modules
# ---------------------------------------------------------------------------
def place_source(stage, cell, usd, scale=GAC_SCALE):
    """`gac_fire.place_source`, copied (importing gac_fire is out of scope:
    a live session owns that file)."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(usd)
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    c = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(cell)).ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - c[0]),
                    -(0.5 * (mn[1] + mx[1]) - c[1]), -(mn[2] - c[2])))
    return holder


def prep(args):
    """Slice `--asset`, pick pieces, and export a SMALL layer with just those
    (plus the kit modules). Returns the manifest."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    from detail import gac_storey_slice as gss

    out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)
    layer_path = os.path.join(
        out_dir, "pieces.usdc" if args.max_pieces <= 0 else "pieces.usda")

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/g0"
    UsdGeom.Xform.Define(st, cell)
    t0 = time.time()
    src = place_source(st, cell, GAC_DIR + args.asset + ".usd", GAC_SCALE)
    if src is None:
        raise SystemExit("[probe] could not place " + args.asset)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
                           ).ComputeWorldBound(
                               st.GetPrimAtPath(src)).ComputeAlignedRange()
    mn, mx = bb.GetMin(), bb.GetMax()
    ref = (0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1]), 0.5 * (mn[2] + mx[2]))
    print("[probe] placed {0} in {1:.0f}s, bbox {2}".format(
        args.asset, time.time() - t0,
        [round(float(mx[i] - mn[i]), 1) for i in range(3)]))

    t0 = time.time()
    with gss.slice_lock(verbose=True):
        pls, g, measured = gss.slice_to_kit(st, src, cell, "probe",
                                            verbose=True)
    print("[probe] slice_to_kit: {0} piece(s) in {1:.0f}s (grid measured={2})"
          .format(len(pls), time.time() - t0, measured))
    if not pls:
        raise SystemExit("[probe] slice produced nothing")

    # triangle count per piece — that is what picks the interesting ones
    for p in pls:
        pr = st.GetPrimAtPath(p["prim_path"])
        cnt = UsdGeom.Mesh(pr).GetFaceVertexCountsAttr().Get()
        p["_tris"] = int(len(cnt)) if cnt else 0

    picked, seen = [], set()
    for role in ("wall", "pier", "corner", "core", "roof", "parapet",
                 "upstand", "deck"):
        cand = [p for p in pls if p["_role"] == role]
        if not cand:
            continue
        for p in (max(cand, key=lambda q: q["_tris"]),
                  min(cand, key=lambda q: q["_tris"])):
            if p["prim_path"] not in seen:
                seen.add(p["prim_path"])
                picked.append(p)
    biggest = max(pls, key=lambda q: q["_tris"])
    if biggest["prim_path"] not in seen:
        picked.append(biggest)
    # keep it to a manageable set: the largest of each role, the smallest of
    # the two commonest roles, and the single biggest piece
    if args.max_pieces <= 0:
        # EVERY piece. Seven hand-picked ones prove nothing about a crash that
        # only one geometry in a building triggers; `--sweep` walks the lot.
        picked = sorted(pls, key=lambda q: -q["_tris"])
    else:
        picked = sorted(picked, key=lambda q: -q["_tris"])[:args.max_pieces]

    # --- export ONLY those, spec by spec, into a fresh layer
    dst = Sdf.Layer.CreateNew(layer_path)
    root = st.GetRootLayer()
    Sdf.CreatePrimInLayer(dst, Sdf.Path("/W"))
    dst.GetPrimAtPath("/W").specifier = Sdf.SpecifierDef
    dst.GetPrimAtPath("/W").typeName = "Xform"
    dst.defaultPrim = "W"
    man = []
    for p in picked:
        s = Sdf.Path(p["prim_path"])
        d = Sdf.Path("/W/sliced_" + s.name)
        Sdf.CreatePrimInLayer(dst, d)
        Sdf.CopySpec(root, s, dst, d)
        man.append({"prim": str(d), "kind": "sliced", "role": p["_role"],
                    "tris": p["_tris"], "size": [round(q, 2) for q in p["_size"]],
                    "solid": T_URM.get(p["_role"], T_URM[None]),
                    "ref": "{0},{1},{2}".format(*[round(float(q), 3) for q in ref])})
    # --- and the kit modules, as REFERENCES (that is also the "referenced
    #     Xform asset" case the memory names)
    kst = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(kst, "/W")
    for nm in KIT_MODULES:
        kp = "/W/kit_" + nm
        pr = kst.DefinePrim(Sdf.Path(kp))
        pr.GetReferences().AddReference(KIT_DIR + nm + ".usd")
        kst.Load(Sdf.Path(kp))
    for nm in KIT_MODULES:
        kp = Sdf.Path("/W/kit_" + nm)
        Sdf.CreatePrimInLayer(dst, kp)
        Sdf.CopySpec(kst.GetRootLayer(), kp, dst, kp)
    dst.Save()

    # kit tri counts, measured on the reference we just copied
    chk = Usd.Stage.Open(layer_path)
    from disaster import fracture
    for nm in KIT_MODULES:
        kp = "/W/kit_" + nm
        m = fracture.prim_to_mesh(chk, kp)
        man.append({"prim": kp, "kind": "kit", "role": "wall",
                    "tris": int(len(m.faces)) if m is not None else 0,
                    "size": ([round(float(q), 2) for q in m.extents]
                             if m is not None else [0, 0, 0]),
                    "solid": T_URM["wall"],
                    "ref": "{0},{1},{2}".format(*[round(float(q), 3) for q in ref])})
    with open(os.path.join(out_dir, "manifest.json"), "w") as fh:
        json.dump({"layer": layer_path, "asset": args.asset, "items": man},
                  fh, indent=1)
    print("[probe] exported {0} ({1:.1f} MB), {2} item(s)".format(
        layer_path, os.path.getsize(layer_path) / 1e6, len(man)))
    return {"layer": layer_path, "asset": args.asset, "items": man}


# ---------------------------------------------------------------------------
# PARENT — one subprocess per (item, stage)
# ---------------------------------------------------------------------------
def spawn(item, stage, layer, n, timeout):
    env = dict(os.environ)
    env["FRACTURE_CAP"] = "contour" if stage == "slice_cap_contour" else "fan"
    env["EQ_DUMP_FRAGS"] = "0"
    # `--opt=value`, NOT `--opt value`: `ref` is a coordinate triple and its
    # first component is routinely negative, which argparse reads as an option.
    cmd = [sys.executable, os.path.abspath(__file__), "--child",
           "--layer=" + layer, "--prim=" + item["prim"], "--stage=" + stage,
           "--solid=" + str(item["solid"]), "--ref=" + item["ref"],
           "--n=" + str(n)]
    t0 = time.time()
    try:
        pr = subprocess.run(cmd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=timeout)
        raw = pr.stdout.decode("utf-8", "replace")
        rc = pr.returncode
    except subprocess.TimeoutExpired as exc:
        raw = (exc.output or b"").decode("utf-8", "replace")
        rc = "TIMEOUT"
    dt = round(time.time() - t0, 1)
    last = ""
    res = None
    for line in raw.splitlines():
        if line.startswith("@@STEP "):
            last = line[7:]
        elif line.startswith("@@RESULT "):
            try:
                res = json.loads(line[9:])
            except Exception:
                pass
    if res is not None:
        verdict = "ok"
    elif rc == "TIMEOUT":
        verdict = "TIMEOUT"
    elif isinstance(rc, int) and rc < 0:
        import signal as _sg
        try:
            verdict = _sg.Signals(-rc).name
        except Exception:
            verdict = "SIG{0}".format(-rc)
    else:
        verdict = "rc={0}".format(rc)
    return {"item": item["prim"], "kind": item["kind"], "role": item["role"],
            "tris": item["tris"], "stage": stage, "verdict": verdict,
            "wall_s": dt, "last_step": last, "result": res,
            "tail": "\n".join(raw.strip().splitlines()[-24:])}


def fmt_table(rows):
    hdr = ("kind", "role", "tris", "stage", "verdict", "s", "last_step",
           "in nv/nf", "dup%", "0area", "nmani", "bound", "wt", "out")
    lines = []
    for r in rows:
        res = r["result"] or {}
        s = res.get("solid") or res.get("src") or {}
        src = res.get("src") or {}
        out = ""
        if "frags" in res:
            out = "frags={0}".format(res["frags"].get("n"))
            if "frags2" in res:
                out += " re={0}".format(res["frags2"].get("n"))
        elif "made" in res:
            out = "prims={0}".format(res["made"])
        elif "cap_tri" in res:
            out = "cap_tri={0} lines={1} kept={2}".format(
                res.get("cap_tri"), res.get("polylines", "-"),
                res.get("kept_lines"))
        elif "out" in res:
            out = "nf={0}".format((res["out"] or {}).get("nf"))
        elif "solid" in res:
            out = "nf={0}".format((res["solid"] or {}).get("nf"))
        lines.append((
            r["kind"], r["role"], str(r["tris"]), r["stage"], r["verdict"],
            str(r["wall_s"]), (r["last_step"] or "")[:34],
            "{0}/{1}".format(src.get("nv", "-"), src.get("nf", "-")),
            str(src.get("dup_pct", "-")), str(src.get("zero_area", "-")),
            str(src.get("e_nonmanifold", "-")), str(src.get("e_bound", "-")),
            str(src.get("watertight", "-")), out))
    w = [max(len(hdr[i]), max([len(l[i]) for l in lines] or [0]))
         for i in range(len(hdr))]
    sep = "  "
    txt = [sep.join(h.ljust(w[i]) for i, h in enumerate(hdr))]
    txt.append(sep.join("-" * w[i] for i in range(len(hdr))))
    for l in lines:
        txt.append(sep.join(l[i].ljust(w[i]) for i in range(len(hdr))))
    return "\n".join(txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--child", action="store_true")
    ap.add_argument("--layer", default="")
    ap.add_argument("--prim", default="")
    ap.add_argument("--stage", default="prim_to_mesh")
    ap.add_argument("--solid", type=float, default=0.38)
    ap.add_argument("--ref", default="")
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--asset", default="SM_Building_09")
    ap.add_argument("--out", default="/tmp/_vtk_shell")
    ap.add_argument("--reuse", action="store_true")
    ap.add_argument("--only", default="", help="comma list of stages")
    ap.add_argument("--pieces", default="", help="substring filter on prim")
    ap.add_argument("--max-pieces", type=int, default=7)
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--solid-override", type=float, default=0.0,
                    help="force this thickness (0.010 = the glass case)")
    ap.add_argument("--seeds", type=int, default=1,
                    help="rng seeds per piece in --sweep")
    ap.add_argument("--sweep", action="store_true",
                    help="one process, every manifest item, `--only` stage")
    args = ap.parse_args()

    if args.child:
        try:
            info = run_sweep(args) if args.sweep else run_stage(args)
        except BaseException as exc:          # noqa: BLE001 — report anything
            import traceback
            traceback.print_exc()
            info = {"prim": args.prim, "stage": args.stage,
                    "err": "{0}: {1}".format(type(exc).__name__, exc)[:400]}
        info["rss_mb"] = _rss_mb()
        sys.stdout.write("@@RESULT " + json.dumps(info) + "\n")
        sys.stdout.flush()
        return

    man_path = os.path.join(args.out, "manifest.json")
    if args.reuse and os.path.exists(man_path):
        man = json.load(open(man_path))
        print("[probe] reusing {0} ({1} items)".format(man["layer"],
                                                       len(man["items"])))
    else:
        man = prep(args)
    stages = [s for s in (args.only.split(",") if args.only else STAGES)
              if s in STAGES]
    if args.sweep:
        env = dict(os.environ, FRACTURE_CAP="fan")
        cmd = [sys.executable, os.path.abspath(__file__), "--child", "--sweep",
               "--out=" + args.out, "--stage=" + stages[0], "--n=" + str(args.n),
               "--seeds=" + str(args.seeds),
               "--solid-override=" + str(args.solid_override)]
        pr = subprocess.run(cmd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
        raw = pr.stdout.decode("utf-8", "replace")
        print(raw[-6000:])
        print("[probe] sweep rc={0}".format(pr.returncode))
        with open(os.path.join(args.out, "sweep_{0}.log".format(stages[0])),
                  "w") as fh:
            fh.write(raw)
        return
    items = [i for i in man["items"]
             if not args.pieces or args.pieces in i["prim"]]

    rows = []
    for it in items:
        for stg in stages:
            r = spawn(it, stg, man["layer"], args.n,
                      TIMEOUT.get(stg, args.timeout))
            rows.append(r)
            print("[probe] {0:<7} {1:<8} tris={2:<7} {3:<18} -> {4:<10} "
                  "{5:>6}s  last={6}".format(
                      r["kind"], r["role"], r["tris"], stg, r["verdict"],
                      r["wall_s"], r["last_step"]))
            if r["verdict"] != "ok":
                print("---- child tail " + "-" * 50)
                print(r["tail"])
                print("-" * 65)
    table = fmt_table(rows)
    print("\n" + table + "\n")
    with open(os.path.join(args.out, "results.json"), "w") as fh:
        json.dump(rows, fh, indent=1)
    with open(os.path.join(args.out, "table.txt"), "w") as fh:
        fh.write(table + "\n")
    print("[probe] wrote {0}/results.json and table.txt".format(args.out))


if __name__ == "__main__":
    main()
