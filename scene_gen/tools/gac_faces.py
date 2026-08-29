#!/usr/bin/env python
"""Which elevations of a GreatAmericanCity building are BLANK?

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_faces.py

A kit building made for a game city is only detailed where the player sees it:
the elevations meant to abut a neighbour, or to face a back alley, are flat
untextured slabs. Placed on an open corner they read as a missing wall, so the
layout has to know which side is which.

Detected from the geometry, not by eye. Every triangle is binned by its
area-weighted outward normal into the four compass elevations, and per side we
record area, triangle DENSITY (tri per m2) and the textures covering it. A
blank elevation is a large area carrying almost no triangles — a detailed
façade with modelled reveals and cornices runs one to two orders of magnitude
denser than a flat slab.

THAT DENSITY TEST IS RELATIVE TO THE BUILDING'S OWN BEST SIDE, and it goes
blind on a building that is equally plain on every side: SM_Building_28 reads
1.04 / 0.75 / 0.86 / 1.13 tri/m2 on E/N/W/S — nothing stands out as "the
detailed one" because none of them are, they are the same flat slab
(`M_Slab_02_Inst`) repeated four times, and the relative test reports `rel
~1.0` on all four and calls it fine. A second, independent test reads what
the triangles are actually TEXTURED WITH: a window, glass, lit-interior-image
or brick material is a real façade wherever it sits; a bare `WallBack`/`Slab`
material is a blank wherever it sits, regardless of what its own siblings
look like. That is the "metadata for which faces have windows" the layout
actually needs — triangle count was only ever a proxy for it, and a proxy
that needs a detailed neighbour to have something to be relative to.

THE TEXTURE TEST HAS THE SAME BLIND SPOT IN A DIFFERENT PLACE, and both
instances found so far were caught only by re-reading the actual measured
data rather than trusting the verdict: (1) a triangle whose diffuseColor has
NO image bound at all — a flat, constant colour, `""` in `tex` — was being
counted as blank; directly probing the shading graphs of several buildings
showed this pack carries no MDL at all and the flat colour is a real,
bit-identical-across-buildings `(0.198, 0.2195, 0.2307)` grey, not a resolver
gap — but SM_Building_01's own known-good brick front is 12-17% that same
grey (window trim, mullions, AO patches — normal on a detailed façade) while
SM_Building_18/19 are 56-61% of it on every side including the front, so it
is clearly not SAFE to score either way from the colour share alone: the
share is measured (`flat_color_frac`, `flat_color`) and reported, never
scored. `window_frac` is what actually separates them — 0.65-0.77 on the
two known-good fronts against 0.00 on both disputed ones — so the
density/texture disjunction below is unchanged; (2) a first token list
included bare "wall" and "concrete", which also matches the ONE material
carried on all four elevations of towers like SM_Building_31
(`..._Wall_Inst`, 302 m) — a single-material building is the texture test's
own version of "nothing to be relative to", tracked as `cladding_materials`
/ `texture_rule` so it is visible when the test has (correctly) gone silent
rather than looking like a building with nothing wrong.

THE DENSITY TEST HAS A BLIND SPOT OF ITS OWN: `front` and its `best`
denominator used to be `max` over ALL FOUR SIDES with no area floor, so a
6.6 m2 sliver of unrelated decorative geometry (a tree/planter mesh caught by
the elevation binning on downtowncity's Amar_Tower) can carry more triangles
per m2 than a real wall ever would and win outright. That made Amar_Tower's
"front" a piece of foliage, and measured its two genuinely glazed ~18,000 m2
elevations (72% window-texture share each) as blank relative to it — the
231 m tower came out `all_blank`. `front` and `best` are now chosen only
among sides with `area_m2 >= MIN_AREA`, falling back to the unrestricted max
only if NO side clears that bar (a building that is a sliver on every side
has nothing better to report).

A THIRD signal, `glass_frac`, was added after the above two rounds: a real
curtain-wall tower (SM_Building_10/12/13/17/18/19/28/31 among them) has no
window texture AND is uniformly "flat colour" by `_tex_kind` on every side,
because glazing in this pack is a flat, untextured PBR material, not an
image — indistinguishable from a bare slab by name. `_is_glass` reads the
actual material (metallic/opacity, thresholds set from a direct A/B against
SM_Building_01's known brick front and known blank back — see below). A side
is a façade, not blank, if EITHER `window_frac` OR `glass_frac` clears its
bar; this vetoes both the density test and the material-name test, not just
one, since a curtain wall can still measure locally sparse on one elevation.

Writes `_plans/gac_faces.json`: per building, per side (N/E/S/W), the metrics
and a `blank` verdict (`blank_by` says which rule fired: `density`, `texture`,
`both`, or `""`), plus `front` — the densest REAL side, which is the one that
wants to face the street — and `all_blank` for a building with no good face on
any side at all, which the layout must never give street frontage.
"""

import json
import os
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
# Any building library, not just GreatAmericanCity: FACES_ROOT / FACES_NAMES /
# FACES_OUT override the defaults, so a new pack is classified with the same
# measurement rather than by eye.
ROOT = os.environ.get("FACES_ROOT") or (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
OUT = os.path.join(HERE, "..", "_plans",
                   os.environ.get("FACES_OUT") or "gac_faces.json")
NAMES = ([q for q in (os.environ.get("FACES_NAMES") or "").split(",") if q]
         or ["SM_Building_%02d" % i for i in range(1, 6)] +
         ["SM_Building_06_Small"] +
         ["SM_Building_%02d" % i for i in range(7, 32)])
EXT = os.environ.get("FACES_EXT", ".usd")
SIDES = ("E", "N", "W", "S")          # +X, +Y, -X, -Y
NORMALS = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
# a side is blank when it is a real wall (enough area) and carries almost no
# geometry; the split is wide in the data, not a knife-edge
# BLANK IS RELATIVE, NOT ABSOLUTE. A flat slab on a plain building and a
# flat slab on an ornate one have very different triangle densities, so a
# fixed cut-off catches 01 (0.14 against a 42.7 front) and misses 09 (2.80
# against a 5.93 front) — both of which are blank to the eye. Judging each
# side against its OWN building's best elevation catches both.
BLANK_RATIO = 0.55                    # of this building's densest side
BLANK_DENSITY = 4.0                   # ...and never call a dense side blank

# relative-to-self fails when self has no good side to be relative TO
# (SM_Building_28 — see module docstring). These tokens read the
# TEXTURE instead. Matched as a case-insensitive substring of the diffuse
# texture's basename, because this pack's naming is consistent per building
# (M_Building_NN_<kind>_Inst) but not exact-token clean across buildings
# (e.g. "Bricks", not "Brick"; "Images_Fake_03", not "fake_interior").
# Checked window-first, so a texture that happens to say both (e.g. a wall
# panel with inset windows) counts as the real façade it is.
WINDOW_TOKENS = ("window", "glass", "images", "image", "fake_interior",
                  "office", "home_light", "power_low_light", "brick")
# NOT "wall" or "concrete" as bare substrings — a first pass used those and
# it caught more than the back-of-building slabs it was meant for.
# `M_Building_31_Wall_Inst`, `M_Building_13_Wall_Inst`, `M_Building_19_
# Wall_Inst` and `M_Building_10/18_Concrete_Inst` are each the ONE material
# on ALL FOUR elevations of their building — a 302 m tower (31), a 140 m
# tower (13), a 135 m tower (10) — and "wall"/"concrete" flagged every side
# of every one of them blank, which would have dropped four of the biggest
# towers in the pack from a district that was built to use them. Restricted
# to the two tokens that are actually reliable in this pack's own naming:
# `M_Building_NN_WallBack_Inst` and `M_SlabNN_Inst` never appear on a
# modelled elevation, only on the ones meant to face a neighbour or a roof.
BLANK_TOKENS = ("wallback", "wall_back", "slab")
WINDOW_FRAC_MAX = 0.05                # a slab carries ~0 window/glass/brick area
BLANK_FRAC_MIN = 0.50                 # ...and is mostly bare wallback/slab
MIN_AREA = 40.0                       # m2 — ignore slivers

# GLASS HAS NO TEXTURE NAME TO MATCH. A curtain-wall pane's diffuseColor is a
# flat base-tint colour, not an image — by name alone it looks exactly like
# a bare concrete slab (see `_tex_kind`'s docstring: SM_Building_18/19 read
# 56-61% "flat colour, no texture" on every side). What actually separates
# them is the MATERIAL, not the texture, and it has to be read from real PBR
# inputs, thresholds set from a direct A/B against a known control — probed
# `SM_Building_01`'s own brick front and its `M_Building_01_WallBack_Inst`
# back (both metallic 0.0 constant, opacity unauthored i.e. fully opaque)
# against the small family of reused materials that shows up in small
# patches even on SM_Building_01 itself and DOMINATES SM_Building_10/12/13/
# 17/18/19/28/31: metallic 0.82-0.97 constant, opacity 0.50-0.98 constant.
# roughness/specular/ior/clearcoat were probed too and do NOT separate the
# two groups in this pack (specular in particular overlaps: 0.04-0.50 on
# both), so only metallic and opacity are used.
GLASS_METALLIC_MIN = 0.5              # glass family: 0.82-0.97; controls: 0.0
GLASS_OPACITY_MAX = 0.99              # glass family: 0.50-0.98; controls: 1.0
                                       # (unauthored, i.e. fully opaque)
GLASS_FRAC_MAX = 0.05                 # same bar as WINDOW_FRAC_MAX, for the
                                       # same reason: enough area to be real


def _tex_kind(name):
    """"window" / "blank" / "" for one texture's basename.

    UNTEXTURED IS NOT BLANK. `""` here means a triangle's diffuseColor is a
    flat constant with no image bound — confirmed by directly walking the
    shading graph, not a guess — and a flat colour on its own says nothing:
    SM_Building_01's known-good brick front is 12-17% that same colour
    (window trim, mullions), and SM_Building_18/19's disputed sides are
    56-61% of it. What actually separates a real façade from a slab is
    `window_frac` (0.65-0.77 vs 0.00 — see module docstring), which is why
    `""` is excluded here rather than folded into "blank": it is measured
    and reported (`flat_color_frac`, `flat_color` in `measure`), never
    scored.
    """
    n = (name or "").lower()
    if n == "":
        return ""
    if any(t in n for t in WINDOW_TOKENS):
        return "window"
    if any(t in n for t in BLANK_TOKENS):
        return "blank"
    return ""


# MDL diffuse/albedo inputs to try, in order, once a shader is confirmed MDL
# (`info:mdl:sourceAsset` set). Unlike UsdPreviewSurface's fixed "diffuseColor
# -> texture reader -> file" chain, an MDL material's texture input is often a
# direct asset-path VALUE on the shader itself, so each candidate is tried
# both as a direct value and (in case it is wired through a reader node
# anyway) as a connected source.
_MDL_DIFFUSE_INPUTS = ("diffuse_texture", "diffuseColor",
                        "base_color_texture", "albedo_texture")


def _asset_basename(v):
    if isinstance(v, Sdf.AssetPath) and v.path:
        return v.path.rsplit("/", 1)[-1]
    return None


def _is_glass(prim):
    """True if this subset's bound material reads as glazing by its PBR
    inputs (see GLASS_METALLIC_MIN/GLASS_OPACITY_MAX above for the A/B this
    is built from). Only CONSTANT (unconnected) metallic/opacity values are
    trusted — a TEXTURED metallic/roughness map (this pack's `..._Metal_Inst`
    materials, for one) is genuinely ambiguous from a single scalar and is
    left unclassified rather than guessed at.
    """
    try:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return False
        for c in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(c)
            if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
                continue
            m = sh.GetInput("metallic")
            if m is not None:
                try:
                    if not m.HasConnectedSource():
                        v = m.Get()
                        if v is not None and float(v) >= GLASS_METALLIC_MIN:
                            return True
                except Exception:
                    pass
            o = sh.GetInput("opacity")
            if o is not None:
                try:
                    if not o.HasConnectedSource():
                        v = o.Get()
                        if v is not None and float(v) < GLASS_OPACITY_MAX:
                            return True
                except Exception:
                    pass
            return False
    except Exception:
        return False
    return False


def _diffuse_of(prim):
    """(texture basename or "", constant (r, g, b) or None) for one subset.

    Tries UsdPreviewSurface first, MDL second (a shader carrying
    `info:mdl:sourceAsset`) — this pack turned out to have NO MDL anywhere in
    the several buildings directly probed (01/18/19/28: every material is
    UsdPreviewSurface), so the MDL branch is dead weight here, kept only in
    case a future pack (FACES_ROOT override) actually uses it. When
    UsdPreviewSurface's `diffuseColor` has no connected texture reader — a
    flat, constant colour — that value is captured and returned as the
    second element instead of "", so `measure()` can report it
    (`flat_color_frac` / `flat_color`) rather than silently discarding it as
    an unresolved "".
    """
    try:
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return "", None
        const = None
        for c in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(c)
            if not sh:
                continue
            if sh.GetIdAttr().Get() == "UsdPreviewSurface":
                d = sh.GetInput("diffuseColor")
                if d is not None:
                    if d.HasConnectedSource():
                        ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
                        f = ts.GetInput("file")
                        t = _asset_basename(f.Get() if f else None)
                        if t:
                            return t, None
                    elif const is None:
                        v = d.Get()
                        if v is not None:
                            try:
                                const = (float(v[0]), float(v[1]), float(v[2]))
                            except Exception:
                                const = None
            if c.GetAttribute("info:mdl:sourceAsset").IsValid():
                for nm in _MDL_DIFFUSE_INPUTS:
                    inp = sh.GetInput(nm)
                    if inp is None:
                        continue
                    t = _asset_basename(inp.Get())
                    if not t and inp.HasConnectedSource():
                        ts = UsdShade.Shader(inp.GetConnectedSource()[0].GetPrim())
                        f = ts.GetInput("file")
                        t = _asset_basename(f.Get() if f else None)
                    if t:
                        return t, None
        return "", const
    except Exception:
        return "", None


def measure(name):
    st = Usd.Stage.Open(ROOT + name + EXT)
    st.Load()
    S = UsdGeom.GetStageMetersPerUnit(st)
    mesh = None
    for p in st.Traverse():
        if p.IsA(UsdGeom.Mesh):
            mesh = p
            break
    if mesh is None:
        return None
    me = UsdGeom.Mesh(mesh)
    V = np.asarray(me.GetPointsAttr().Get(), dtype=np.float64) * S
    counts = np.asarray(me.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.asarray(me.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    start = np.zeros(len(counts) + 1, dtype=np.int64)
    np.cumsum(counts, out=start[1:])

    # face -> subset, for texture attribution
    sub_of = np.full(len(counts), -1, dtype=np.int64)
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
    tex, flat, glass = [], [], []
    for si, s in enumerate(subs):
        fi = np.asarray(s.GetIndicesAttr().Get() or [], dtype=np.int64)
        fi = fi[(fi >= 0) & (fi < len(counts))]
        sub_of[fi] = si
        t, c = _diffuse_of(s.GetPrim())
        tex.append(t)
        flat.append(c)
        glass.append(_is_glass(s.GetPrim()))

    lo, hi = V.min(axis=0), V.max(axis=0)
    W, D, H = hi - lo
    acc = {k: {"area": 0.0, "tris": 0, "tex": {}, "flat": {}, "glass": 0.0}
           for k in SIDES}
    for f in range(len(counts)):
        b, c = start[f], counts[f]
        for j in range(1, c - 1):
            a, p2, p3 = V[idx[b]], V[idx[b + j]], V[idx[b + j + 1]]
            n = np.cross(p2 - a, p3 - a)
            ar = 0.5 * float(np.linalg.norm(n))
            if ar <= 0.0:
                continue
            n = n / (2.0 * ar)
            if abs(n[2]) > 0.72:          # roof or floor, not an elevation
                continue
            # the elevation this triangle faces, and only if it is near the
            # OUTSIDE of the building — interior partitions face outward too
            best, bd = None, 0.0
            for k, (nx, ny) in NORMALS.items():
                d = n[0] * nx + n[1] * ny
                if d > bd:
                    best, bd = k, d
            if best is None or bd < 0.55:
                continue
            cen = (a + p2 + p3) / 3.0
            nx, ny = NORMALS[best]
            near = ((hi[0] - cen[0]) if nx > 0 else
                    (cen[0] - lo[0]) if nx < 0 else
                    (hi[1] - cen[1]) if ny > 0 else (cen[1] - lo[1]))
            if near > 2.5:                # not on the skin of that elevation
                continue
            acc[best]["area"] += ar
            acc[best]["tris"] += 1
            si = sub_of[f]
            t = tex[si] if si >= 0 else ""
            acc[best]["tex"][t] = acc[best]["tex"].get(t, 0.0) + ar
            if t == "":
                # a constant (untextured) diffuseColor — track the actual RGB
                # per side so `measure()` can report which colour (if any)
                # dominates, instead of collapsing them all under one "" key
                fc = flat[si] if si >= 0 else None
                acc[best]["flat"][fc] = acc[best]["flat"].get(fc, 0.0) + ar
            if si >= 0 and glass[si]:
                # tracked independent of texture name/kind — glazing carries
                # no window-token texture name (see `_is_glass`'s docstring),
                # so this triangle's area lands in BOTH acc["tex"] (by
                # whatever its diffuse resolves to, often "" or a plain
                # colour name) AND here
                acc[best]["glass"] += ar

    out = {"name": name, "usd": ROOT + name + EXT,
           "W": round(W, 1), "D": round(D, 1), "H": round(H, 1),
           "cx": round(0.5 * (lo[0] + hi[0]), 3),
           "cy": round(0.5 * (lo[1] + hi[1]), 3), "z0": round(lo[2], 3),
           "sides": {}}
    dens = {}
    for k in SIDES:
        a = acc[k]["area"]
        dens[k] = (acc[k]["tris"] / a) if a > 1e-6 else 0.0
    # FRONT AND best ARE CHOSEN ONLY AMONG REAL WALLS. A side under MIN_AREA
    # can be anything — a decorative planter, a roof antenna mount, a sliver
    # of geometry the elevation binning caught in passing — and a handful of
    # small triangles on a tiny area produces a huge density that has nothing
    # to do with the building's actual street face (downtowncity's Amar_Tower:
    # a 6.6 m2 patch of tree-bark texture at 85 tri/m2 beat two genuinely
    # glazed ~18,000 m2 elevations at 0.78 tri/m2, so the tower's own real
    # windows measured as "blank relative to the front" and the whole 231 m
    # asset came out `all_blank`). Falling back to the unrestricted max only
    # when NOTHING clears MIN_AREA keeps a degenerate mesh from crashing here.
    real_k = [k for k in SIDES if acc[k]["area"] >= MIN_AREA]
    best = (max((dens[k] for k in real_k), default=0.0) if real_k
            else max(dens.values())) or 1.0

    # CLADDING_MATERIALS / texture_rule — the texture test's own version of
    # "relative, not absolute". A building clad in exactly ONE material has
    # nothing for the texture test to contrast against either: on
    # SM_Building_31/13/10/18/19 that one material is a plain "Wall"/
    # "Concrete" name that no longer matches BLANK_TOKENS, so this guard is
    # currently redundant for them — but it is written down anyway, because
    # both classification failures caught in review so far were this same
    # shape: a test that could not tell "uniform" from "uniformly bad". If a
    # future token addition ever re-widens BLANK_TOKENS, this stops it from
    # reaching a single-material building UNLESS that lone material actually
    # IS recognised blank cladding — which is exactly SM_Building_28's case
    # (one material, `M_Slab_02_Inst`, and "slab" is deliberately still on
    # the list) and must keep firing.
    cladding = {t for k in SIDES for t in acc[k]["tex"] if t}
    texture_silent = (len(cladding) == 1
                       and _tex_kind(next(iter(cladding))) != "blank")

    for k in SIDES:
        a = acc[k]["area"]
        tex_sorted = sorted(acc[k]["tex"].items(), key=lambda q: -q[1])
        window_area = sum(ar for t, ar in tex_sorted if _tex_kind(t) == "window")
        blank_area = sum(ar for t, ar in tex_sorted if _tex_kind(t) == "blank")
        flat_area = acc[k]["tex"].get("", 0.0)
        window_frac = (window_area / a) if a > 1e-6 else 0.0
        blank_frac = (blank_area / a) if a > 1e-6 else 0.0
        flat_color_frac = (flat_area / a) if a > 1e-6 else 0.0
        # which constant colour, if any, is behind that share — only
        # reported when ONE colour clearly dominates the untextured area, so
        # a side with several unrelated flat-colour materials doesn't get a
        # misleading single RGB
        flat_color = None
        flat_items = acc[k]["flat"]
        if flat_area > 1e-6:
            top_c, top_a = max(flat_items.items(), key=lambda q: q[1])
            if top_c is not None and top_a >= 0.8 * flat_area:
                flat_color = [round(x, 3) for x in top_c]
        glass_frac = (acc[k]["glass"] / a) if a > 1e-6 else 0.0
        is_wall = a >= MIN_AREA
        by_density_raw = bool(is_wall and dens[k] < BLANK_DENSITY
                               and dens[k] < BLANK_RATIO * best)
        # the material-name test stands alone, unlike the density test above
        # it never asks how this side compares to its own siblings, so it is
        # what catches a building that is uniformly blank (see docstring) —
        # gated by texture_silent so it never fires on a single-material
        # building whose one material isn't itself recognised blank cladding.
        # window_frac used to be ANDed in here directly; it is now folded
        # into is_facade below instead, alongside glass, so both vetoes apply
        # uniformly to EITHER blank signal (density or material-name), not
        # just this one — a curtain-wall tower's density can read blank on a
        # side (SM_Building_10 N/S, modelled slightly sparser than E/W) even
        # though the material there is unambiguously glazing.
        by_material_raw = bool(is_wall and not texture_silent
                                and blank_frac > BLANK_FRAC_MIN)
        # A SIDE IS A FAÇADE, NOT BLANK, IF EITHER SIGNAL SAYS SO: drawn
        # windows (window_frac) or real glazing (glass_frac) are two
        # different ways the same triangle can be dressed, and this pack
        # uses both (drawn "fake interior" window images on some buildings,
        # actual glass PBR materials with no texture on others). Vetoes BOTH
        # blank tests, not just the material one — see above.
        is_facade = (window_frac >= WINDOW_FRAC_MAX
                     or glass_frac >= GLASS_FRAC_MAX)
        by_density = by_density_raw and not is_facade
        by_texture = by_material_raw and not is_facade
        blank_by = ("both" if by_density and by_texture else
                    "density" if by_density else
                    "texture" if by_texture else "")
        out["sides"][k] = {
            "area_m2": round(a, 1), "tris": acc[k]["tris"],
            "tri_per_m2": round(dens[k], 2),
            "rel": round(dens[k] / best, 3),
            "window_frac": round(window_frac, 3),
            "blank_frac": round(blank_frac, 3),
            # the share of this side's area with a flat, constant diffuse
            # colour and no image at all — measured and reported, never
            # scored (see `_tex_kind`): it does not separate a real façade
            # from a slab, `window_frac` does
            "flat_color_frac": round(flat_color_frac, 3),
            "flat_color": flat_color,
            # the share whose MATERIAL (not texture name) reads as glazing —
            # see `_is_glass`; this is what a reflective curtain wall looks
            # like to this measurement, and it is scored (unlike flat_color)
            "glass_frac": round(glass_frac, 3),
            "blank": bool(blank_by),
            "blank_by": blank_by,
            "tex": [t for t, _ in tex_sorted[:2]],
            # full per-texture breakdown, not just the top 2 — so a verdict
            # can be audited back to the material that produced it
            "tex_areas": {t: round(ar, 2) for t, ar in tex_sorted}}
    out["cladding_materials"] = len(cladding)
    out["texture_rule"] = "silent (single cladding)" if texture_silent else ""
    # same MIN_AREA gate as `best` above — a sliver of unrelated geometry
    # must never win "front" just because it is small and dense
    out["front"] = (max(real_k, key=lambda k: dens[k]) if real_k
                    else max(SIDES, key=lambda k: dens[k]))
    out["blank_sides"] = [k for k in SIDES if out["sides"][k]["blank"]]
    # THE PLACEMENT CLASS FALLS OUT OF THE COUNT, and this is the rule the
    # layout enforces (user, 2026-08-29): a building modelled on three
    # elevations belongs on a corner, two belongs at the end of a run, one
    # belongs in the middle with neighbours covering both flanks.
    n_good = 4 - len(out["blank_sides"])
    out["detailed_sides"] = n_good
    out["place"] = ("any" if n_good >= 4 else "corner" if n_good == 3
                    else "end" if n_good == 2 else "mid")
    # n_good == 0 falls into "mid" above, which tells the layout this
    # building has ONE good face to put toward the street — wrong when it
    # has NONE. all_blank is that case: every side big enough to be a real
    # wall is blank, by either rule, so there is no good face anywhere and
    # the asset must never be given street frontage. Same `real_k` gate as
    # `front`/`best` — one canonical definition of "big enough to count".
    out["all_blank"] = bool(real_k) and all(
        out["sides"][k]["blank"] for k in real_k)
    if out["all_blank"]:
        out["place"] = "none"
    return out


def main():
    res = []
    for nm in NAMES:
        try:
            r = measure(nm)
        except Exception as exc:
            print("%-22s FAILED %s" % (nm, exc), flush=True)
            continue
        if r is None:
            continue
        res.append(r)
        print("%-22s %5.1f x %5.1f x %6.1f m  front %s  place %-6s%s  blank %-12s"
              % (nm, r["W"], r["D"], r["H"], r["front"], r["place"],
                 "  ALL_BLANK" if r["all_blank"] else "",
                 ",".join(r["blank_sides"]) or "-"), flush=True)
        for k in SIDES:
            s = r["sides"][k]
            print("    %s tri/m2=%5.2f(%.2f) window=%.2f glass=%.2f blank=%.2f%s"
                  % (k, s["tri_per_m2"], s["rel"], s["window_frac"],
                     s["glass_frac"], s["blank_frac"],
                     (" *" + s["blank_by"] if s["blank_by"] else "")),
                  flush=True)
    json.dump(res, open(os.path.normpath(OUT), "w"), indent=1)
    print("\nwrote %s" % os.path.normpath(OUT))


if __name__ == "__main__":
    main()
