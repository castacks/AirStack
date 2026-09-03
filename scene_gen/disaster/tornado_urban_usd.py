"""tornado_urban_usd — apply an urban-tornado damage PLAN to a sliced
building and author its street debris.

`disaster/tornado_urban.py` (stream L, parallel round) is the PURE planner:
it walks `quake_flow.describe`'s element table, the wind model
(`tornado.wind_at`) and the T0..T4 ladder in
`scene_gen/_plans/urban_tornado_plan.md` §2.6 and produces the JSON-
serialisable plan described in that document's §2.8 — no `pxr` anywhere in
that half. This module is the OTHER half: it takes that plan and a live USD
stage and does the three things a plan can ask for on a SLICED building
(remove a piece, void its glass, rigid-displace a hanging panel or a
macroblock) plus the one thing only this module owns — turning every
removed piece into street debris.

WHY THIS REUSES `quake_sliced` RATHER THAN REDERIVING IT
----------------------------------------------------------
`disaster/quake_sliced.py` already solved "how do you damage a building that
is a bag of kit-shaped pieces cut out of one merged mesh" for the earthquake
ladder, on the SAME piece grid (`gac_slice`/`gac_storey_slice`) this ladder
damages. Its `apply_plan` is the reference for every mechanism this module
needs:

  * glass -> void: a pane is a `GeomSubset` on a piece, not a prim. The
    FIRST draft of this module called `quake_sliced._void_glass` directly
    for this, on the theory that "exactly the way `quake_sliced.apply_plan`
    does it" meant reusing its function verbatim. The lead's end-to-end
    container probe on a REAL GAC slice (SM_Building_02, level T3) found
    that theory wrong: the planner listed 13 glass pieces and `apply_plan`
    voided 0 subsets. `quake_sliced._void_glass` has TWO matching paths and
    both are dead ends on GAC — see this module's OWN `_void_glass` below,
    which replaces it, for why. `quake_sliced.py` is the earthquake
    stream's file and is not edited here; the lead is recording this blind
    spot in the tornado skill instead.
  * rigid displacement: `quake_sliced.rigid_matrix(spec)` turns a plain
    JSON dict into the 4x4 row-vector matrix `quake_flow._transform_prims`
    post-multiplies onto a prim's world transform. ONE implementation, used
    here unchanged, so a macroblock this ladder tips into the street moves
    by the exact arithmetic `quake_sliced`'s own tests already check.
  * roof furniture: `quake_sliced._sweep_roof_props_sliced` IDENTIFIES roof
    plant by two ctx-level lists, `ctx["roof_plant"]` / `ctx["roof_fixed"]`
    (paths of tanks/AC units placed on the roof by `quake_flow.dress_roof`).
    This module reuses that IDENTIFICATION — read those two keys — but not
    its band-carry / region-reseat MECHANICS, which need `plan["fit_ops"]`
    and `plan["collapse"]`, fields the tornado plan (§2.8) does not carry.
    A tornado's roof plant does not get carried down a storey that crushed
    under it (there is no crush here); it is torn off and gone, which is
    what the ladder table already says in words ("rooftop plant swept",
    "rooftop light props gone") — so `roof_props == "sweep"` here is a
    straight deactivate of whatever `ctx["roof_plant"]` / `ctx["roof_fixed"]`
    name, nothing more. Note this round's `wreck_urban` does not itself call
    `quake_flow.dress_roof` (see that function's docstring) — until a later
    round wires roof-furniture placement into the urban-tornado ctx, those
    two keys are simply absent and "sweep" is a documented no-op, exactly
    the shape the lead's container-probe ctx (`{"mats": {}, ...}`, no
    `roof_plant` key) exercises.

DEBRIS IS THE ONE THING ONLY THIS MODULE OWNS. `disaster/planks.py` is the
existing model for "a wind event's debris is boards, not fracture chunks",
and its docstring gives the reasons this round leans on again: it does not
scale, a Voronoi cut is the wrong SHAPE for shed material, and a box needs
no solver. This ladder's debris is architectural fragments (spalled panel,
brick block, coping, glass shard, roof deck sheet), not sawn timber, so it
does not reuse `planks.build` directly — but it reuses `planks.py`'s two
central ideas verbatim:

  * ONE MERGED MESH PER CLASS. `build_debris` groups by (kind, material)
    the same way `planks.build` groups by (class, skin) — a wrecked city
    block sheds thousands of fragments and a plate holds dozens of wrecked
    buildings; one prim per fragment is a six-figure prim count for
    geometry that never moves again.
  * SEAT ON THE FACE, NOT THE CORNER. This is "FLOATING DEBRIS" cause 1 in
    `.agents/skills/build-tornado-scenes/SKILL.md`: `planks._lay` used to
    seat every board so its LOWEST ROTATED CORNER touched grade, which
    means NO board ever lay on its face — the centre floated up by however
    much the piece's own tilt demanded (measured: p50 0.079 m, p90 0.373 m,
    32% of a 758-board field more than 10 cm up, with a fully DETACHED
    shadow on the low-sun render that made it unmissable). The fix bedding
    a piece `_BED_M` (2 cm) into whatever it lands on, capped so a steep
    tilt still rests on its own corner rather than sinking through, is
    reproduced here as `_seat_z` — see that function for the reduced,
    one-`tilt_deg` form this ladder's fragments need (a planner fragment
    carries a single tilt about its own long axis, not `_lay`'s independent
    pitch+roll draw, so the formula is `_lay`'s bedding rule collapsed to
    that one degree of freedom, not `_lay` called directly).

GLASS IS OPAQUE, NOT TRANSPARENT. §2.9 of the plan: a broken pane over
asphalt read from 60 m needs to be a dark HOLE, and a physically-transparent
glass shard lying flat on grey asphalt is close to invisible at that
distance — the same reasoning `quake_sliced._glass_void` already encodes
for the earthquake ladder ("NOT `mats["glass"]` — that is the pale intact
pane tint and it renders as a bright rectangle where a window should now be
a dark hole"). This module builds exactly ONE such void material
(`_ensure_void_material`) and uses it BOTH to rebind a broken pane's
`GeomSubset` (this module's own `_void_glass`, below) AND as the "glass"
debris class's own material, so a window that broke and the shard that
came from it are wearing the same dark-glass look.

THE `_void_glass` FIX (round 2, against a REAL slice). Every GAC material
prim is named `UnrealMaterial` (`gac_slice.window_centres`'s own docstring:
"Empty on GAC in the sense that matters"), so `quake_sliced._void_glass`'s
inline name check (`"glass" in cur.GetPrim().GetName().lower()`) never
fires on GAC at all. Its texture check does not either, for a subtler
reason: GAC's `UsdPreviewSurface` does not hold its diffuse map as a
`Get()`-able value on the `diffuseColor` input — that input is CONNECTED to
a separate `UsdUVTexture` shader whose `inputs:file` holds the actual
texture, so `quake_sliced._tex_of`'s `inp.Get()` returns `None` and the
texture check never matches either. Both of `quake_sliced._void_glass`'s
two matching paths are therefore dead ends on this asset, which is exactly
what the container probe measured (13 listed, 0 voided).

`detail.gac_slice.window_centres` already solved this correctly — its own
nested `_tex` closure FOLLOWS the connection — and `gac_fire._diffuse_of`
is a second, independent copy of the same fix for the same reason (neither
is importable on its own: one is a closure, the other returns a different
tuple shape for a different caller). This module's `_glass_tex_and_name` is
a THIRD copy, reproduced with attribution rather than imported, feeding
`detail.gac_slice.is_glazing` — the one glazing matcher every other
consumer in this codebase already agrees on, so this ladder's glass call
does not re-derive a fourth ad hoc test.
"""

import json as _json
import math
import os as _os

from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

from . import damage
from . import planks
from . import quake_flow as qf
from . import quake_sliced as qs

# ---------------------------------------------------------------------------
# geometry constants — the same box topology `quake_flow._box` / `planks.py`
# use (corner order, face winding, face normals). Duplicated rather than
# imported: `planks._box`'s signature takes a FULL pitch+roll spec dict and
# this ladder's fragments carry a single `tilt_deg` (see `_seat_z`), so the
# per-fragment corner maths below is its own small function, not a call into
# `planks._box` with a fabricated `pitch=0` spec. The topology itself (corner
# order, winding, face normals) is not private know-how — it already appears
# twice in this codebase (`quake_flow._box`, `planks._box`) and a third,
# identical copy here keeps this module's box authoring readable without an
# import that would otherwise be the only reason to reach into `planks`'s
# geometry internals.
# ---------------------------------------------------------------------------
_CORNERS = ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))
_FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
          (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
_FACE_N = ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0),
           (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))

_BED_M = planks._BED_M   # the 2 cm bedding sink, `planks._lay`'s own constant

# THE VOID TONE. Dark glossy near-black — a broken pane and a glass shard are
# both this, never physically transparent glass (see the module docstring).
_VOID_RGB = (0.030, 0.028, 0.030)
_VOID_ROUGHNESS = 0.15

# THE REPO SCHEME, NOT A BUILD-MACHINE PATH. `os.path.join(<this file's own
# dirname>, ...)` was the first draft here and it is exactly the trap
# `.agents/skills/freeze-portable-scenes/SKILL.md` documents: that string is
# whatever this Python file's absolute path happens to be on the machine
# that IMPORTED it, and it gets baked into the authored `Sdf.AssetPath`
# verbatim — fine on the build machine, broken the moment the layer is
# anchored on Nucleus or the checkout moves. `airstack://` is the repo's own
# portable scheme (`scene_generator.LOCAL_ASSET_ROOTS`): resolved through
# `sg._join_asset_root` AT THE POINT OF USE (`debris_material`, below), the
# same call-site pattern `quake_flow._c_look_at` uses for its own textured
# ground looks, so a frozen/portable build (`AIRSTACK_ASSET_ROOT` pointed at
# a Nucleus mirror) gets a Nucleus path and an ordinary dev build gets the
# local checkout — decided at BIND time, not import time.
_TEX_BRICK = ("airstack://scene_gen/assets/materials/megascans/"
             "Brick_Wall_Worn/T_sexkaitb_1K_B.jpg")
_TEX_CONCRETE = ("airstack://scene_gen/assets/materials/megascans/"
                 "Damaged_Concrete_Floor/T_vizbefe_2K_B.png")

# One tile per ~1.0 m — `damage._pbr`'s `scale_uv` is repeats PER METRE
# (smaller = bigger features; its own docstring: "~0.45 puts one tile across
# a bit over two metres"), so 1.0 puts one tile across ~1.0 m.
_TILE_REPEATS_PER_M = (1.0, 1.0)


def _safe_name(s):
    """A USD-prim-legal token from an arbitrary material-hint string."""
    s = str(s or "unknown")
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in s)
    return out or "unknown"


def _fix_diffuse_tint(stage, path, rgb):
    """Author `diffuse_tint` (`damage._pbr` never does) — the ONE slot that
    multiplies over a bound `diffuse_texture`.

    READ THE MDL, ROUND 4 (D3). `/isaac-sim/kit/mdl/core/Base/
    OmniPBR_ClearCoat.mdl` lines 652-654, verbatim:

        color diffuse        = tex::texture_isvalid(diffuse_texture)
                               ? desaturated_base : diffuse_color_constant;
        color tinted_diffuse = multiply_colors(diffuse, diffuse_tint, 1.0).tint;

    So with a VALID map the albedo is `texture * diffuse_tint` and
    `diffuse_color_constant` is not in the product at all — it is the
    map-failed FALLBACK colour, nothing else. Two consequences this module
    lives by (§8 D3):

      * whatever is passed HERE is the whole multiply. Round 3 passed the
        ~0.30 class rgb, which is a 70% knock-down of a cladding map that
        is itself only ~0.4 mean — an effective albedo near 0.10, i.e. the
        near-black berms of the round-3 bench (A3/B1/B3). The tint must be
        a near-neutral GRIME value (`_CLASS_LOOK`'s middle field, ~0.8-0.9,
        slightly desaturated); the TEXTURE carries the colour;
      * `diffuse_color_constant` must stay a PLAUSIBLE class albedo, never
        white. A white constant is invisible while the map resolves and
        renders a field of white litter the moment one does not — which is
        exactly what B4/B5's "white paper" reads like."""
    sh = UsdShade.Shader.Get(stage, path + "/Shader")
    if sh:
        sh.CreateInput("diffuse_tint",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))


def _ensure_void_material(stage, ctx):
    """The one dark-glass-void material, built once and cached under BOTH
    `ctx["mats"]["void"]` (where `quake_sliced._glass_void` looks for it —
    see `_glass_void`'s own key order, `("void", "scar_shadow", "crack",
    "dark_concrete")`) and `ctx["mats"]["tornado_debris:glass"]` (where
    `debris_material` looks for the glass-shard class), so a broken pane and
    the shards that came from it share the literal same material prim."""
    mats = ctx.setdefault("mats", {})
    got = mats.get("void")
    if got is not None:
        mats.setdefault("tornado_debris:glass", got)
        return got
    parent = ctx.get("parent") or "/World"
    path = "{0}/TornadoDebrisLooks/void".format(parent)
    mat = damage._pbr(stage, path, _VOID_RGB, _VOID_ROUGHNESS)
    mats["void"] = mat
    mats["tornado_debris:glass"] = mat
    return mat


def _glass_tex_and_name(mat_prim):
    """(diffuse basename, MATERIAL PRIM NAME) of `mat_prim`.

    REPRODUCED, with attribution, from `detail.gac_slice.window_centres`'s
    own nested `_tex` closure (that function, lines ~217-236) — not
    imported, because a closure cannot be, and the second existing copy of
    this exact logic, `gac_fire._diffuse_of` (line ~1161), returns a
    different tuple shape for a different caller so it is not a drop-in
    replacement either. This is deliberately the THIRD copy rather than a
    fourth ad hoc reimplementation: same walk, same fallback order, so a
    change to how GAC/downtowncity materials are laid out only needs
    finding (and fixing) in three well-known places, not guessing how many
    there are.

    THE BUG THIS EXISTS TO AVOID (found on a REAL slice, not a synthetic
    one): a GAC/downtowncity `UsdPreviewSurface`'s `diffuseColor` input is
    not a `Get()`-able value — it is CONNECTED to a separate `UsdUVTexture`
    shader whose `inputs:file` holds the actual texture. Reading
    `sh.GetInput("diffuseColor").Get()` directly (what `quake_sliced.
    _tex_of` does) returns `None` on every such material, silently. Also
    returns MATERIAL NAME (not just the texture) for the same reason
    `window_centres._tex` does: downtowncity's own window materials
    (`Glass_window`, `Window_003/4/5`, `rollershutter_window_01_001`) carry
    NO diffuse map at all, so the name is the only evidence there is —
    `gac_slice.is_glazing` takes `mat_name` as its second matching chance
    for exactly this reason.
    """
    from pxr import Sdf, Usd, UsdShade

    if not mat_prim or not mat_prim.IsValid():
        return "", ""
    mname = mat_prim.GetName()
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file") if ts else None
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and v.path:
                return v.path.rsplit("/", 1)[-1], mname
        break
    return "", mname


def _surface_tex_and_name(mat_prim):
    """(FULL diffuse-texture URL, basename, MATERIAL PRIM NAME) of
    `mat_prim` — `_glass_tex_and_name`'s connection-following walk (same
    method, same fallback order), generalised to also return a BINDABLE
    URL rather than only the basename glazing-matching needs. Reproduced as
    its own copy rather than a `_glass_tex_and_name` edit for the same
    reason that function documents its own THIRD-copy status: this file's
    glazing machinery (`_void_glass`/`_has_glazing_binding`/
    `annotate_glazing`) is a live, concurrently-edited region this round
    (`apply_plan`'s glass step), and widening its return shape is exactly
    the kind of change that region discipline says stays out.

    `Sdf.AssetPath` carries two strings: `path` (the AUTHORED value, often
    only a bare filename on this asset library — the same "no directory"
    gap `_glass_tex_and_name`'s own basename-only return already lives
    with) and `resolvedPath` (what the stage's asset resolver actually
    found it at, once the value has been composed). `resolvedPath` is
    preferred — the same order `disaster/gac_fire.py`'s own
    `_diffuse_of` (`(v.resolvedPath or v.path)`) already uses for the
    identical read — falling back to `path` when nothing resolved (an
    in-memory test stage with a made-up filename, most commonly), so a
    caller always gets *some* string it can hand `damage._pbr(texture=...)`
    rather than a bare basename with nowhere to load it from.
    """
    from pxr import Sdf, Usd, UsdShade

    if not mat_prim or not mat_prim.IsValid():
        return "", "", ""
    mname = mat_prim.GetName()
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file") if ts else None
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and (v.resolvedPath or v.path):
                url = v.resolvedPath or v.path
                name = str(v.path or url).rsplit("/", 1)[-1]
                return str(url), name, mname
        break
    return "", "", mname


#: THE OUTERMOST TOLERANCE (round 4, D2) — a candidate whose front is within
#: this much of the frontmost candidate's is still "on the outside face" of
#: the piece and competes on area. `fire_collapse.TEAR_OUTER_TOL_M`'s own
#: value, restated here rather than imported so this function keeps its
#: import-time dependency set unchanged (`fire_collapse` is imported lazily,
#: inside the call, for `is_fake_interior` only).
_SURFACE_OUTER_TOL_M = 0.25
#: A piece whose centre is closer than this to the BUILDING's centre has no
#: meaningful outward direction (a `core` piece, a slab). Ranking by
#: outermost-ness there would be noise, so those fall back to area.
_SURFACE_AXIS_MIN_M = 0.5
#: The percentile of a candidate's own face centroids, projected on the
#: outward axis, that stands for "how far forward this material sits" —
#: `fire_collapse.facade_skin`'s own rule-2 statistic, and a percentile
#: rather than a max so one stray face cannot promote a whole subset.
_SURFACE_FRONT_PCT = 90.0


def _mesh_face_proj(mesh_prim, xf_cache, axis):
    """(F,) array — every face's centroid projected on `axis`, in WORLD
    space — or None when the mesh cannot be read.

    WORLD, not local: the only thing this is used for is comparing two
    subsets OF THE SAME PIECE against an axis derived from world bboxes
    (`annotate_surface`'s own `Cp - Cb`), so both sides of the comparison
    have to be in one frame, and world is the only frame available here
    without knowing the cell path. A yawed holder therefore changes nothing
    about the ranking — the axis rotates with the geometry.
    """
    import numpy as np
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(mesh_prim)
    pts = mesh.GetPointsAttr().Get()
    cnt = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        return None
    P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
    M = xf_cache.GetLocalToWorldTransform(mesh_prim)
    R = np.asarray([[M[r][c] for c in range(3)] for r in range(3)], dtype=float)
    T = np.asarray([M[3][0], M[3][1], M[3][2]], dtype=float)
    d = (P @ R + T) @ np.asarray(axis, dtype=float)
    cnt = np.asarray(cnt, dtype=np.int64)
    idx = np.asarray(idx, dtype=np.int64)
    if idx.size != int(cnt.sum()) or idx.max(initial=-1) >= P.shape[0]:
        return None
    ends = np.cumsum(cnt)
    starts = ends - cnt
    # per-face MEAN of its own vertices' projections, with no python loop:
    # a prefix sum over the flattened corner list, differenced at the face
    # boundaries. Works for mixed-arity faces (quads and triangles).
    csum = np.concatenate(([0.0], np.cumsum(d[idx])))
    return (csum[ends] - csum[starts]) / np.maximum(cnt, 1)


def _rank_surface_candidates(cands, axis, xf_cache):
    """Pick ONE of a piece's non-glazing textured candidates — OUTERMOST
    first, area as the tie-break, exactly `fire_collapse.facade_skin`'s
    rules 1-3 (this function is that ranking applied at ANNOTATION time,
    where there is no `describe` mass frame yet, so the outward axis is
    derived from world bboxes by the caller instead of from `e["out"]`).

    ROUND 4 (D2) — WHY THIS REPLACED "BIGGEST SUBSET WINS". Measured on the
    live `SM_Building_02` slice: a GAC `wall`/opening piece on a modelled
    elevation carries BOTH its real cladding (`M_Building_01_Concrete_02`)
    and the blind back-wall filler (`M_Building_01_WallBack`), and the
    filler is the bigger subset by face count — it is one flat quad per
    storey while the façade is split into mullions, reveals and a sill
    course. So the largest-by-area rule stamped the BACK WALL as the
    piece's cladding, and every consumer of that stamp inherited it: this
    module's own tear faces (`_tear_material`) and the street debris
    (`tornado_urban._ledger_removed` -> `debris_material`). Area was never
    the property that makes something a façade; being the thing nothing
    else is in front of is.

      1. A candidate whose material name or base map matches
         `fire_collapse.FAKE_INTERIOR_HINTS` (`Office_Fake`, `Ceiling`,
         `M_Slab`, `Building_Floor`, ... — GAC hangs a printed interior
         card BEHIND its glazing, facing out) is DEMOTED, not dropped: if
         every candidate on the piece is one of those, the piece keeps it.
         A parapet whose only texture is `M_Slab` is still a parapet.
         NOTE `WallBack` is deliberately not on that list — on
         GreatAmericanCity it IS the base texture of some real outward
         subsets (`fire_collapse`'s own census, `pier_S_3_09_0102`), which
         is exactly why this is an outermost test and not a name ban.
      2. Of what is left, the OUTERMOST wins: candidates are ranked by the
         90th percentile of their face centroids on the outward axis, and
         only those within `_SURFACE_OUTER_TOL_M` of the front compete.
      3. Among those, the largest face count wins — the ORIGINAL rule, now
         applied only to what is actually on the outside face.

    `axis` None (a core piece, or a stage with one placement and therefore
    no derivable outward) falls straight through to rule 3, so the old
    behaviour is exactly preserved wherever outermost-ness cannot be
    decided. Returns `(winner, moved)`: `moved` is True when rule 2
    changed the answer, for the caller's own census.
    """
    if not cands:
        return None, False
    from . import fire_collapse as fc

    by_area = max(cands, key=lambda c: c["n"])
    pool = [c for c in cands
            if not fc.is_fake_interior(c["name"], c["mat"], c["url"])]
    if not pool:
        pool = list(cands)
    if axis is not None and len(pool) > 1:
        import numpy as np
        proj_cache = {}
        ranked = []
        for c in pool:
            key = str(c["mesh"].GetPath())
            if key not in proj_cache:
                proj_cache[key] = _mesh_face_proj(c["mesh"], xf_cache, axis)
            proj = proj_cache[key]
            if proj is None or not len(proj):
                ranked = []
                break
            faces = c["faces"]
            sel = (proj if faces is None
                   else proj[np.asarray(faces, dtype=np.int64)
                             [np.asarray(faces, dtype=np.int64) < len(proj)]])
            if not len(sel):
                ranked = []
                break
            ranked.append((c, float(np.percentile(sel, _SURFACE_FRONT_PCT))))
        if ranked:
            front = max(d for _c, d in ranked)
            pool = [c for c, d in ranked if d >= front - _SURFACE_OUTER_TOL_M]
    best = max(pool, key=lambda c: c["n"]) if pool else by_area
    return best, best is not by_area


def annotate_surface(stage, placements):
    """Stamp `_tex_url` / `_tex_name` onto each placement dict, IN PLACE,
    from a REAL read of its prim's OUTERMOST non-glazing bound texture —
    the `annotate_glazing` pattern (same walk, same "measure, don't guess"
    discipline), generalised from "does this piece carry ANY glazing" to
    "what does the REST of this piece look like". Returns the number of
    placements with a non-empty `_tex_url`.

    F3, `.agents/skills/build-tornado-scenes/SKILL.md`'s "THE DEBRIS IS NOT
    ALL SAWN TIMBER" section, applied verbatim to this ladder: "the debris
    is supposed to retain the colour of the house/roof ... BIND THE
    TEXTURE, NOT THE MATERIAL" — a piece's own kit/GAC MATERIAL is UV-space
    (or, on GAC, bound to a source-asset `Section*/UnrealMaterial` the
    debris mesh has no matching UVs for at all, being an authored box with
    no `st`); the TEXTURE is what `debris_material` re-projects triplanar
    the same way `planks.skin_material` does for the suburb ladder.

    RESOLUTION ORDER (per piece):
      1. SUBSETS FIRST. `UsdGeom.Subset.GetAllGeomSubsets` on every `Mesh`
         under the piece; for each subset, resolve its bound material via
         `_surface_tex_and_name` and skip it if `gac_slice.is_glazing`
         matches (a window pane is not cladding — the SAME exclusion
         `annotate_glazing`/`_void_glass` apply, just inverted: this
         function wants what is LEFT after glazing is removed). Among the
         REMAINING (non-glazing) subsets, `_rank_surface_candidates` picks
         the OUTERMOST one (largest face count only as the tie-break among
         everything on the outside face) — ROUND 4, D2; see that function
         for the measurement that retired the old "biggest subset wins".
      2. SUBSET-LESS MESH -> THE MESH'S OWN BINDING. A kit module (round 2
         vocabulary: `_window_named`'s sibling case) carries no subsets at
         all — one material bound directly to the mesh prim, the same
         shape `annotate_glazing`'s own subset-less branch already handles
         for the SAME reason. This is a straight either/or per mesh, not a
         combined pool: a subset-bearing mesh's own top-level binding
         (typically a `ShellFallbackLooks` placeholder on a live GAC slice)
         is never consulted, matching `_void_glass`'s own documented shape
         ("this only ever rebinds SUBSETS, never the piece prim's own
         ... binding").

    THE OUTWARD AXIS is derived here rather than taken from a caller: this
    runs BEFORE `quake_flow.describe`, so there is no mass frame and no
    `e["out"]` yet. `normalize(piece centre - building centre)`, flattened
    to horizontal, from the world bboxes of the placements themselves — the
    geometric statement of outward for a perimeter piece, right for a
    corner piece (it gives the diagonal), and degenerate exactly where it
    should be (a core piece, or a single-placement stage), which falls back
    to the old area rule.

    A piece with more than one `Mesh` prim under it (uncommon, but the walk
    does not assume exactly one) pools candidates across every mesh and
    ranks them together over the whole piece — the same "DOMINANT" reading
    `_classify`'s caller wants, not a per-mesh average.
    """
    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade
    from detail import gac_slice as gsl

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xf_cache = UsdGeom.XformCache()

    # THE BUILDING'S OWN CENTRE, from the pieces themselves — the union of
    # every placement's world bbox, so no cell path, mass frame or holder
    # yaw is needed. `None` (nothing resolvable) disables rule 2 entirely.
    lo = hi = None
    prims = {}
    for p in placements:
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            continue
        prims[path] = prim
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        a, b = r.GetMin(), r.GetMax()
        lo = [a[i] for i in range(3)] if lo is None else \
            [min(lo[i], a[i]) for i in range(3)]
        hi = [b[i] for i in range(3)] if hi is None else \
            [max(hi[i], b[i]) for i in range(3)]
    centre = None if lo is None else [(lo[i] + hi[i]) / 2.0 for i in range(3)]

    n_hit = 0
    n_moved = 0
    n_base_rebound = 0
    exterior = ("wall", "pier", "corner", "parapet", "parapet_corner")
    for p in placements:
        path = p.get("prim_path")
        prim = prims.get(path)
        cands = []
        axis = None
        if prim is not None:
            if centre is not None:
                r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
                if not r.IsEmpty():
                    c = r.GetMidpoint()
                    v = np.asarray([float(c[0]) - centre[0],
                                    float(c[1]) - centre[1], 0.0])
                    nrm = float(np.linalg.norm(v))
                    if nrm > _SURFACE_AXIS_MIN_M:
                        axis = v / nrm
            for mesh_prim in Usd.PrimRange(prim):
                if not mesh_prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(mesh_prim)
                subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
                if subs:
                    for s in subs:
                        cur = UsdShade.MaterialBindingAPI(
                            s.GetPrim()).ComputeBoundMaterial()[0]
                        if not cur or not cur.GetPrim().IsValid():
                            continue
                        url, name, mat_name = _surface_tex_and_name(cur.GetPrim())
                        if gsl.is_glazing(name, mat_name=mat_name):
                            continue
                        if not url:
                            continue
                        idx = s.GetIndicesAttr().Get()
                        cands.append({
                            "url": url, "name": name, "mat": mat_name,
                            "material_path": str(cur.GetPrim().GetPath()),
                            "n": len(idx) if idx else 0,
                            "mesh": mesh_prim,
                            "faces": list(idx) if idx else []})
                else:
                    cur = UsdShade.MaterialBindingAPI(
                        mesh_prim).ComputeBoundMaterial()[0]
                    if not cur or not cur.GetPrim().IsValid():
                        continue
                    url, name, mat_name = _surface_tex_and_name(cur.GetPrim())
                    if gsl.is_glazing(name, mat_name=mat_name) or not url:
                        continue
                    counts = mesh.GetFaceVertexCountsAttr().Get()
                    cands.append({
                        "url": url, "name": name, "mat": mat_name,
                        "material_path": str(cur.GetPrim().GetPath()),
                        "n": len(counts) if counts else 0,
                        "mesh": mesh_prim, "faces": None})
        best, moved = _rank_surface_candidates(cands, axis, xf_cache)
        p["_tex_url"] = best["url"] if best else ""
        p["_tex_name"] = best["name"] if best else ""
        p["_surface_mat_path"] = best["material_path"] if best else ""
        if best:
            n_hit += 1
            # A sliced mesh intentionally carries a role fallback at prim
            # level and its real source materials on face subsets.  That is
            # visually correct only while the subset partition remains
            # complete, and it makes Isaac's property panel misleadingly say
            # ShellFallbackMaterial for a real facade piece.  Use the
            # measured outermost source material as the weak/base binding;
            # direct bindings on material subsets still win face by face.
            if p.get("_role") in exterior:
                mat_prim = stage.GetPrimAtPath(best["material_path"])
                if mat_prim and mat_prim.IsValid():
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                        UsdShade.Material(mat_prim))
                    n_base_rebound += 1
        if moved:
            n_moved += 1

    # A clip cell can contain only source faces whose original material was
    # unbound.  Leaving the slicer's role fallback on that cell makes both
    # the surviving facade and every tear derived from it turn flat/white.
    # Fill those holes from the nearest measured exterior cell, preferring
    # the same elevation and side.  The target retains its own source UVs;
    # this only restores the material assignment lost at the source face.
    donors = [p for p in placements if p.get("_tex_url") and
              p.get("_surface_mat_path")]
    repaired = 0
    for p in placements:
        if p.get("_tex_url") or p.get("_role") not in exterior or not donors:
            continue
        def _distance(d):
            return (0 if d.get("_side") == p.get("_side") else 1000,
                    abs(int(d.get("_storey", 0)) - int(p.get("_storey", 0))),
                    abs(int(d.get("_bay", 0)) - int(p.get("_bay", 0))),
                    0 if d.get("_role") == p.get("_role") else 1)
        donor = min(donors, key=_distance)
        mat_prim = stage.GetPrimAtPath(donor["_surface_mat_path"])
        target = stage.GetPrimAtPath(p.get("prim_path") or "")
        if not mat_prim or not mat_prim.IsValid() or not target or not target.IsValid():
            continue
        UsdShade.MaterialBindingAPI.Apply(target).Bind(UsdShade.Material(mat_prim))
        p["_tex_url"] = donor["_tex_url"]
        p["_tex_name"] = donor["_tex_name"]
        p["_surface_mat_path"] = donor["_surface_mat_path"]
        repaired += 1
        n_hit += 1

    print("[tornado_urban_usd] surface: {0} of {1} pieces carry a resolved "
          "cladding texture (non-glazing, OUTERMOST subset — area only as "
          "the tie-break); the outermost rule moved {2} piece(s), nearest-"
          "facade repair filled {3} source-unbound exterior piece(s); "
          "{4} exterior base binding(s) now name their measured source "
          "material".format(
              n_hit, len(placements), n_moved, repaired, n_base_rebound))
    return n_hit


def _window_named(path):
    """Is this prim, BY NAME, a kit window/door MODULE? Kit module prim
    names carry their category (`bld_dw_terrace_windows0_4_50`), so the
    name is the honest signal on a build whose materials carry none."""
    leaf = str(path).rsplit("/", 1)[-1].lower()
    return ("window" in leaf) or ("wnd" in leaf) or ("_door" in leaf)


def _has_glazing_binding(stage, path):
    """Does ANY mesh under `path` carry a glazing binding — on a subset or
    directly on the mesh prim — that `_void_glass` could rebind? The
    partition test `apply_plan`'s glass step uses to route a listed piece
    to REBIND (sliced GAC/dtc, real glass materials) vs KNOCK-OUT (a kit
    window module with none)."""
    from pxr import Usd, UsdGeom, UsdShade
    from detail import gac_slice as gsl

    prim = stage.GetPrimAtPath(path) if path else None
    if not prim or not prim.IsValid():
        return False
    for mesh in Usd.PrimRange(prim):
        if not mesh.IsA(UsdGeom.Mesh):
            continue
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
        targets = [s.GetPrim() for s in subs] or [mesh]
        for t in targets:
            cur = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            if not cur or not cur.GetPrim().IsValid():
                continue
            tex_basename, mat_name = _glass_tex_and_name(cur.GetPrim())
            if gsl.is_glazing(tex_basename, mat_name=mat_name):
                return True
    return False


def _void_glass(stage, paths, void_mat):
    """Rebind every GLAZING `GeomSubset` under each of `paths` to
    `void_mat`. THIS IS NOT `quake_sliced._void_glass` — see the module
    docstring's "THE `_void_glass` FIX" section for the container-probe
    measurement (13 glass pieces listed, 0 subsets voided) that found why:
    that function's texture check reads `diffuseColor` without following
    its connection to the `UsdUVTexture` that actually carries the map, and
    its name check never fires either because every GAC material prim is
    named `UnrealMaterial`.

    This walks the same shape `quake_sliced._void_glass` does (a sliced
    piece keeps the source asset's `GeomSubset`s, one per material, so the
    panes are addressable even though the slicer cannot cut a window out)
    but resolves each subset's bound material with `_glass_tex_and_name`
    (which DOES follow the connection) and matches with
    `detail.gac_slice.is_glazing` — the one glazing matcher every other
    consumer in this codebase (`gac_fire`, `dtc_catalogue`, the census/probe
    tools) already agrees on, rather than a fourth inline keyword test.
    `is_glazing` also covers downtowncity's texture-less `Glass_window`-
    named materials (via `mat_name`) and excludes `awning` (`GLASS_TEX_
    NOT`), neither of which the old inline check did.

    Per the lead's note: on a live slice each PIECE prim is bound to
    `<cell>/pieces/ShellFallbackLooks/<role>` while its SUBSETS bind the
    source asset's own `Section*/UnrealMaterial` — the slicer's own design.
    On a SLICED piece this only ever rebinds SUBSETS, never the piece
    prim's own `ShellFallbackLooks` binding — which is why it walks
    `Usd.PrimRange` down to every `Mesh` rather than trusting the
    top-level binding. On a KIT module mesh (round 2), which carries NO
    subsets at all, the fallback branch rebinds the MESH itself when its
    one directly-bound material is glazing.
    """
    if void_mat is None or not paths:
        return 0
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    n = 0
    for path in paths:
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
            for s in subs:
                cur = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                tex_basename, mat_name = _glass_tex_and_name(cur.GetPrim())
                if gsl.is_glazing(tex_basename, mat_name=mat_name):
                    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(void_mat)
                    n += 1
            if not subs:
                # ROUND 2: a KIT module mesh carries no subsets (one
                # material on the mesh prim; a window is its own module).
                # Void the WHOLE MODULE when its binding is glazing — the
                # frame darkens with the pane, which is the "dark hole"
                # read a broken window wants anyway. See annotate_glazing's
                # matching branch.
                cur = UsdShade.MaterialBindingAPI(
                    mesh).ComputeBoundMaterial()[0]
                if cur and cur.GetPrim().IsValid():
                    tex_basename, mat_name = _glass_tex_and_name(
                        cur.GetPrim())
                    if gsl.is_glazing(tex_basename, mat_name=mat_name):
                        UsdShade.MaterialBindingAPI.Apply(mesh).Bind(void_mat)
                        n += 1
    return n


def annotate_glazing(stage, placements):
    """Stamp `_glass_faces` / `_glass_frac` onto each placement dict, IN
    PLACE, from a REAL read of its prim's material bindings — not a role
    guess. Returns the number of placements with `_glass_faces > 0`.

    THE LIVE PROBE FINDING this exists to fix (SM_Building_02, level T3):
    the slicer's BAY_SPLITS phase puts the actual window opening in the
    NARROW sub-panel of a bay, and on this asset that sub-panel classifies
    as role `pier` — 18 `pier` pieces and 9 `core` pieces carry `Section7`
    / `M_Building_01_Windows_Inst_BaseColor.png` (10,860 faces total,
    `is_glazing` True); every `wall` piece carries exactly ONE subset,
    `M_Building_01_WallBack`, no glazing at all. `_void_glass` above is
    correct — it rebinds every glazing subset it is HANDED — but a
    role-based "wall = pane" pick (what a planner would do without this)
    can never find glass on this asset, because the glass is not on the
    `wall` role here. This function measures the truth per piece instead of
    guessing from role, so the planner (stream L) can prefer whichever
    pieces actually carry glazing, on WHATEVER role the slicer happened to
    put it on for this particular asset.

    `_glass_faces` is the sum of `GetIndicesAttr()` lengths over every
    `GeomSubset` that `_glass_tex_and_name` + `gac_slice.is_glazing` match
    (same connection-following read `_void_glass` uses, so a piece this
    function says has glass is a piece `_void_glass` can actually void).
    `_glass_frac` is that count over the piece's TOTAL face count (summed
    `GetFaceVertexCountsAttr()` length across every `Mesh` under the
    prim), `0.0` when the prim resolves to nothing so a `0/0` never raises.
    Both are plain `int`/`float` — this is JSON-serialisable planner input,
    per §2.8, not a pxr object.
    """
    from pxr import Usd, UsdGeom, UsdShade
    from detail import gac_slice as gsl

    n_hit = 0
    n_glazing_faces_total = 0
    for p in placements:
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        n_glazing = 0
        n_faces = 0
        if prim and prim.IsValid():
            for mesh_prim in Usd.PrimRange(prim):
                if not mesh_prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(mesh_prim)
                counts = mesh.GetFaceVertexCountsAttr().Get()
                n_faces += len(counts) if counts else 0
                subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
                for s in subs:
                    cur = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                    if not cur or not cur.GetPrim().IsValid():
                        continue
                    tex_basename, mat_name = _glass_tex_and_name(cur.GetPrim())
                    if gsl.is_glazing(tex_basename, mat_name=mat_name):
                        idx = s.GetIndicesAttr().Get()
                        n_glazing += len(idx) if idx else 0
                if not subs:
                    # ROUND 2 (stream K's kit probes): a KIT module mesh has
                    # NO GeomSubsets — ONE material bound directly on the
                    # mesh prim, and a window is its own module, not a
                    # subset of a wall. Every kit probe printed "0 of N
                    # pieces carry glass" until this branch existed. When
                    # the mesh's own binding is glazing, every face of the
                    # mesh is glazing.
                    cur = UsdShade.MaterialBindingAPI(
                        mesh_prim).ComputeBoundMaterial()[0]
                    if cur and cur.GetPrim().IsValid():
                        tex_basename, mat_name = _glass_tex_and_name(
                            cur.GetPrim())
                        if gsl.is_glazing(tex_basename, mat_name=mat_name):
                            n_glazing += len(counts) if counts else 0
        p["_glass_faces"] = int(n_glazing)
        p["_glass_frac"] = float(n_glazing) / float(max(1, n_faces))
        if n_glazing > 0:
            n_hit += 1
            n_glazing_faces_total += n_glazing

    print("[tornado_urban_usd] glazing: {0} of {1} pieces carry glass "
          "(faces {2})".format(n_hit, len(placements), n_glazing_faces_total))
    return n_hit


def _classify(kind, material):
    """Which of the FIVE look buckets in §2.9 a (kind, material) pair gets
    -- `glass` / `brick` / `concrete` / `metal` / `membrane`, and nothing
    else. `None` means unrecognised — a flat neutral fallback, printed so a
    typo in a planner's material hint is visible rather than silently grey.

    R5 (round 2, 2026-09-01): the OLD sixth bucket, `deck` (matched by
    `kind == "deck"` alone, with no keyword test at all), routed to
    `planks.wood_material` in `debris_material` below -- pale sawn timber on
    a masonry/curtain-wall city, which is exactly the suburb-lumber
    signature the user's ground-evidence review called out. `deck` as a
    KIND no longer exists in the planner's own vocabulary
    (`tornado_urban._kind_of` now returns `membrane` or `metal` for a
    roof-shed piece, never `deck`), and this function's fallback-by-kind
    path is retired with it -- `membrane` is matched by MATERIAL now
    (`tornado_urban._material_hint` returns the literal string
    `"membrane"` for that kind), the same way every other kind already
    resolves its bucket, so there is no longer a kind-only escape hatch a
    stray/legacy `material` string could fall through un-checked.
    """
    m = str(material or "").lower()
    if "glass" in m or "glaz" in m or "window" in m:
        return "glass"
    if "brick" in m or "coping" in m or "masonry" in m or "stone" in m:
        return "brick"
    if "concrete" in m:
        return "concrete"
    # Round 4 (stream C's spec): joists ahead of the generic metal match --
    # "joists dark steel" per plan 8c; `tornado_collapse` stamps the
    # literal material string "steel_joist".
    if "joist" in m:
        return "steel_joist"
    if "metal" in m or "steel" in m or "alumin" in m:
        return "metal"
    if "membrane" in m or "bitumen" in m or "felt" in m or "roofing" in m:
        return "membrane"
    return None


def _resolve_texture(path):
    """`airstack://...` -> an absolute local-checkout path, or whatever
    `AIRSTACK_ASSET_ROOT` names (an `omniverse://...` Nucleus mirror for a
    portable/frozen build) — resolved HERE, at authoring time, every time a
    material is built, not baked once into a module-level constant.

    `damage._pbr` does NOT run its `texture` argument through
    `scene_generator._join_asset_root` itself — checked: it hands whatever
    string it is given straight to `Sdf.AssetPath`. (The "References an
    asset that can not be found: 'airstack://.../Soil_Mud/...'" Hydra-error
    comment this module's docstring points at belongs to
    `planks.skin_material`, a DIFFERENT function that does its own
    `sg._join_asset_root` call before binding — not to `_pbr`.) So the
    resolve happens at the CALL SITE instead, the exact pattern
    `quake_flow._c_look_at` already uses for its own `_pbr(texture=...)`
    calls: `sg._join_asset_root("airstack://..." + rel, "")` computed fresh
    right before authoring, so a later portable/frozen rebuild (a different
    `AIRSTACK_ASSET_ROOT`) resolves differently without this module needing
    to change.
    """
    import scene_generator as sg
    return sg._join_asset_root(path, "")


# THE PER-BUCKET LOOK — `(flat_rgb, grime_tint, roughness)`, ONE table for
# every branch below so the textured and the flat path can never drift.
# `glass` is deliberately absent: it never takes a texture (see
# `debris_material`'s own guard) and keeps its unconditional void-material
# path.
#
# ROUND 4 (D3) SPLIT THE OLD SINGLE `rgb` IN TWO, because the two uses are
# not the same number and sharing one is what produced the near-black berms
# on the round-3 bench:
#
#   `flat_rgb`   — `damage._pbr`'s `diffuse_color_constant`. The ACTUAL
#                  albedo on an untextured branch (`metal`, `membrane`, the
#                  unrecognised fallback), and the map-failed FALLBACK on a
#                  textured one (never white — see `_fix_diffuse_tint`).
#                  Raised across the board from round 3: 0.22-0.40 linear on
#                  a class that is then ALSO multiplied by a grime tint is
#                  black at 60-90 m over asphalt that is itself ~0.18.
#   `grime_tint`  — `_fix_diffuse_tint`'s multiply over a bound map, and the
#                  ONLY thing that survives into the albedo when a texture
#                  resolves (`texture * diffuse_tint`, MDL 652-654). Near
#                  neutral and slightly desaturated: 0.82-0.90 brightness,
#                  channel spread <= 0.06 — DUST ON THE MAP, not a colour of
#                  its own. Round 3 passed the class rgb here (0.30 for
#                  brick), i.e. a 70% knock-down of the building's own
#                  cladding map: the berm went black and the inherited
#                  texture became invisible, which is D3 in one line. The
#                  colour of a berm is now the SOURCE BUILDING's, full stop;
#                  this only dirties it.
_CLASS_LOOK = {
    #            flat / fallback rgb      grime tint over a map      rough
    "brick": ((0.44, 0.38, 0.34), (0.88, 0.85, 0.82), 0.88),
    "concrete": ((0.48, 0.475, 0.465), (0.87, 0.87, 0.86), 0.85),
    # ROUND 4 (v6 render review): 0.35 roughness on a FLAT untextured sheet
    # lying face-up specularly mirrors the whole sky dome and washes the
    # B4/B5 pile to paper-white in the render even though the constant is a
    # mid grey (bindings verified correct at USD level). Weathered deck in a
    # collapse pile is dusty, not polished: rough 0.62, albedo eased down.
    "metal": ((0.38, 0.39, 0.40), (0.90, 0.90, 0.90), 0.62),
    "steel_joist": ((0.15, 0.15, 0.16), (0.85, 0.85, 0.85), 0.45),
    "membrane": ((0.30, 0.275, 0.25), (0.83, 0.82, 0.80), 0.90),
}
_CLASS_LOOK_DEFAULT = ((0.45, 0.45, 0.45), (0.86, 0.86, 0.86), 0.70)

# ROUND 4 (v6 lit-bench review, "the B rubble is still not fixed"): THE
# MASONRY TONE, `(texture, fallback rgb, tint, roughness)` per tone token.
#
# WHAT THE OFFLINE BENCH STAGE MEASURED (bench_offline.usd, every debris
# mesh's bound shader read back): B1 `brownstone_row` and B3 `walkup` are
# WHITE STONE kit buildings and EVERY masonry mesh under them binds
# `TornadoDebrisLooks/brick` -> `T_sexkaitb_1K_B.jpg`, whose own measured
# linear mean is (0.213, 0.109, 0.071) -- a RED-BROWN brick map, R:G:B =
# 1 : 0.51 : 0.33. A white building shedding red brick is the same "some
# other building's material" defect the review has now named twice. (The v6
# RENDER is a third state again: it predates the `_tiling_safe` gate, so
# those meshes were still triplanar-projecting the `M_MBuilding03_Facades`
# ATLAS -- a packed sheet of windows and trim averaged over a 0.4 m cube,
# which is exactly the "uniform light-grey identical boxes" in B1_obl.)
#
# The class map carries the STRUCTURE, the tint carries the BUILDING'S TONE.
# Bases are the earthquake pipeline's own rubble maps (`assets/materials/
# quake/`, already in the repo and already the rubble look this codebase
# ships), picked by measured mean rather than by name:
#   rubble_rc_B_v3.jpg   linear mean (0.197, 0.177, 0.153)  R:G:B 1:0.90:0.78  sd 0.073
#   rubble_urm_B_v3.jpg  linear mean (0.252, 0.221, 0.191)  R:G:B 1:0.88:0.76  sd 0.086
# Tints are solved AGAINST those means for a target albedo, which is why
# they are above 1.0 -- `diffuse_tint` is a plain multiply (MDL 654) and
# both maps are darker than dressed stone rubble:
#   stone  0.197*1.55, 0.177*1.62, 0.153*1.75 -> (0.305, 0.287, 0.268) pale warm grey
#   tan    0.252*1.18, 0.221*1.12, 0.191*0.98 -> (0.297, 0.248, 0.187) buff
# Map max at +4sd is ~0.5, so even the 1.75 channel stays under 1.0: no
# clipping. `brick` and `concrete` are NOT tone tokens -- they are the
# untouched class branches, and the A row's approved look is exactly them.
_TEX_RUBBLE_NEUTRAL = ("airstack://scene_gen/assets/materials/quake/"
                       "rubble_rc_B_v3.jpg")
_TEX_RUBBLE_WARM = ("airstack://scene_gen/assets/materials/quake/"
                    "rubble_urm_B_v3.jpg")
_TONE_LOOK = {
    #          texture               fallback rgb          tint                 rough
    "stone": (_TEX_RUBBLE_NEUTRAL, (0.34, 0.33, 0.31), (1.55, 1.62, 1.75), 0.86),
    "tan": (_TEX_RUBBLE_WARM, (0.33, 0.29, 0.23), (1.18, 1.12, 0.98), 0.87),
}
#: Buckets a tone token may re-route. A tone says "what masonry is this
#: building made of"; it has no business touching glass, roof membrane,
#: deck metal or bar joists, which are not the building's walls.
_TONEABLE_BUCKETS = frozenset({"brick", "concrete"})

#: Per-mesh TONE JITTER (`tornado_urban._DEBRIS_SHADES` decides how many
#: shades the ledger stamps; this is the multiplier each one applies to its
#: material's tint). SHADE 0 IS EXACTLY 1.0 -- an unstamped fragment (every
#: fixture plan, every caller that passes no shade) must author byte-
#: identically to before this round. The other two straddle it, so a berm's
#: MEAN tone is still what the tone/class look says while its individual
#: meshes are not all the same value -- the "no size/tone variation,
#: identical boxes" half of the review. Only ever applied on a TEXTURED
#: branch, where `diffuse_tint` is the live multiply; the flat branches
#: (metal / membrane / steel_joist) are stream C's collapse populations and
#: are left exactly as they are.
_SHADE_MULT = (1.00, 0.94, 1.06)

# A projected map samples the whole image, while the intact asset's authored
# UVs sample its wall-course region.  SM_Building_02's brick atlas therefore
# renders orange on fragments although the intact wall is dusty rose.  This
# correction was derived from the lit A3 review, not from the filename.
_SOURCE_TINT_BY_STYLE = {"sm_building_02": (0.80, 1.25, 1.90)}


def _source_style_tint(style):
    low = str(style or "").lower()
    for suffix, tint in _SOURCE_TINT_BY_STYLE.items():
        if low.endswith(suffix):
            return tint
    return (1.0, 1.0, 1.0)


def _shade_tint(tint, shade):
    """`tint` scaled by `_SHADE_MULT[shade]`, clamped to a sane range."""
    try:
        k = _SHADE_MULT[int(shade) % len(_SHADE_MULT)]
    except (TypeError, ValueError):
        k = 1.0
    return tuple(min(2.5, max(0.0, float(c) * k)) for c in tint)

# ROUND 4 (user review of the lit bench): "this debris has the most random
# material ... If it's a concrete building then do concrete texture, brick
# for brick". The suburb's "bind the texture, not the material" lesson holds
# ONLY for TILING surface maps (siding, shingles, GAC's brick course). The
# city kit facades are ATLASES -- M_MBuilding03_Facades is a packed sheet of
# windows/doors/trim -- and a triplanar world projection of an atlas paints
# random crops across every fragment. A source texture is honoured only when
# its NAME says it is a tiling surface material; everything else falls to
# the class bucket (brick building -> brick rubble map, rc -> concrete), the
# same rule the quake rubble has always used.
_TILING_SAFE_TOKENS = ("brick", "concrete", "stone", "tile", "plaster",
                       "stucco", "shingle", "cobble", "asphalt")


def _tiling_safe(name):
    low = str(name or "").lower()
    return any(t in low for t in _TILING_SAFE_TOKENS)


# ---------------------------------------------------------------------------
# THE WINDOWLESS-ATLAS CROP (user review 2026-09-02, items 1/13) — the fix
# for "these broken parts ... their texture is just white" and "instead of
# giving it the concrete material which doesn't look right ... crop the
# window out of the png. Just take the left or right grey rectangle and use
# that for fragments, debris".
#
# A city kit / GAC facade map is an ATLAS: windows, doors and trim packed
# into one sheet. The OLD path (below) dropped such a map outright when its
# NAME was not tiling-safe and fell to a flat class bucket — which is the
# "concrete material which doesn't look right" on a brick building, and the
# path a WHITE-fallback fragment took when its class could not be resolved.
# This REPLACES that drop: the atlas is KEPT, but sampled in UV space
# (`project_uvw=False`, so the projection is POSITION-INDEPENDENT — every
# fragment of the building shows the SAME crop regardless of where it landed
# on the plate, unlike the world-triplanar path that walks a different atlas
# region under each fragment) and squeezed into a WINDOWLESS SUB-RECTANGLE of
# the atlas: an edge column, where a facade sheet's border pier / solid wall
# lives, full height. `build_debris` authors a per-face `st` on every debris
# box so the crop reads as textured wall; a caller that authors no `st` (a
# kit/AEC box) still samples the crop origin — one solid wall-colour texel of
# the RIGHT building, never a window and never white.
#
# The crop rectangle is `st * _ATLAS_CROP_UVW + _ATLAS_CROP_UV0`, i.e. u in
# [0.02, 0.14] (the left edge column) and v across most of the height. It is
# an ASSET ASSUMPTION (the left border of a facade atlas is solid wall) the
# same way `_TILING_SAFE_TOKENS` is; it cannot be verified without a running
# Kit, so it is stated here as the one thing this path bets on.
_ATLAS_CROP_UV0 = (0.02, 0.06)
_ATLAS_CROP_UVW = (0.12, 0.88)


def _atlas_crop_material(stage, ctx, tex_url, tex_name, bucket, shade=0):
    """A material that wears a WINDOWLESS crop of a facade ATLAS (`tex_url`),
    UV-projected into `_ATLAS_CROP_*` so a fragment reads as plain wall of
    the building's OWN colour rather than a random window crop, a flat class
    concrete, or a white fallback. One material per (atlas basename, shade),
    cached under `tornado_debris:crop:*`.

    Same look contract as the `:src:` (tiling) path so the black-berm guard
    (`test_textured_debris_wears_the_map_not_a_dark_class_tint`) holds: the
    class GRIME goes on `diffuse_tint` (the only slot a valid map multiplies
    by — OmniPBR.mdl 652-654, see `_fix_diffuse_tint`), the class rgb is the
    map-failed fallback constant (a plausible class albedo, never white),
    and the map itself carries the colour. UV space, not triplanar, is the
    whole point — a triplanar world projection cannot be pinned to one
    sub-rectangle of the atlas across fragments at different world positions.
    """
    mats = ctx.setdefault("mats", {})
    parent = ctx.get("parent") or "/World"
    looks = "{0}/TornadoDebrisLooks".format(parent)
    base = _safe_name(tex_name or (str(tex_url or "").rsplit("/", 1)[-1]))
    ssfx = "" if not shade else ":s%d" % shade
    key = "tornado_debris:crop:" + base + ssfx
    got = mats.get(key)
    if got is not None:
        return got
    rgb, grime, rough = _CLASS_LOOK.get(bucket, _CLASS_LOOK_DEFAULT)
    grime = _shade_tint(grime, shade)
    path = looks + "/crop_" + base + ("" if not shade else "_s%d" % shade)
    mat = damage._pbr(stage, path, rgb, rough,
                      texture=_resolve_texture(tex_url),
                      scale_uv=_ATLAS_CROP_UVW, offset_uv=_ATLAS_CROP_UV0,
                      tint=rgb)
    # UV SPACE, not the world triplanar `_pbr` defaults to: pin the sample to
    # one windowless sub-rectangle for every fragment regardless of position.
    sh = UsdShade.Shader.Get(stage, path + "/Shader")
    if sh:
        pu = sh.GetInput("project_uvw")
        if pu is None:
            pu = sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool)
        pu.Set(False)
    _fix_diffuse_tint(stage, path, grime)
    mats[key] = mat
    print("[tornado_urban_usd] {0} ({1}) -> WINDOWLESS ATLAS CROP of {2} "
          "(the building's own cladding, edge column u[{3:.2f},{4:.2f}] so "
          "no window lands on the fragment; {5} grime {6:.2f})".format(
              base, bucket or "neutral", base,
              _ATLAS_CROP_UV0[0], _ATLAS_CROP_UV0[0] + _ATLAS_CROP_UVW[0],
              bucket or "neutral", grime[0]))
    return mat


def debris_material(stage, ctx, kind, material, tex_url=None, tex_name=None,
                    tone="", shade=0):
    """One material per (kind, material) LOOK BUCKET (not per fragment,
    per §2.9), cached in `ctx["mats"]` under a `"tornado_debris:"` key — OR,
    when the fragment carries a SOURCE TEXTURE (`tex_url`, `tex_name`; from
    `tornado_urban._ledger_removed` copying `annotate_surface`'s
    `_tex_url`/`_tex_name` stamp onto a façade fragment, per the module's
    own docstring and `.agents/skills/build-tornado-scenes/SKILL.md`'s "THE
    DEBRIS IS NOT ALL SAWN TIMBER" — "the debris is supposed to retain the
    colour of the house/roof ... BIND THE TEXTURE, NOT THE MATERIAL"), ONE
    TRIPLANAR material per DISTINCT texture instead, cached under
    `"tornado_debris:src:<basename>"`.

    R5 (round 2): there is no longer a kind-based fallback bucket. The
    retired `deck` bucket (`kind == "deck"` alone, no keyword test) used to
    be exactly that fallback, and it resolved to `planks.wood_material` —
    pale sawn timber, the suburb-lumber signature a masonry/curtain-wall
    city must never wear. A fragment whose own `material` string matches
    nothing recognised now falls to the flat neutral-grey "unrecognised"
    branch below instead (or, if it DOES carry a texture, to
    `_CLASS_LOOK_DEFAULT`'s neutral tint over that texture — see below),
    same as every other kind already did.

    THE TEXTURED PATH (R3b/F3). Guarded on `tex_url` being non-empty AND
    `bucket != "glass"` — glass keeps its own void look unconditionally
    (`tornado_urban._ledger_removed` never passes a texture in for a glass
    fragment in the first place, since it is deposited from `plan["glass"]`
    by a separate call that carries no `source_tex` argument at all; this
    is a second, redundant guard here rather than a trust). One
    `damage._pbr` per distinct texture BASENAME (`tex_name`, or the tail of
    `tex_url` when no name was given), triplanar at `_TILE_REPEATS_PER_M`
    (~1.0 repeat/m) — the SAME tile rate the flat `brick`/`concrete`
    branches below already use, per the suburb skill's tile-scale lesson:
    "a fragment and a board lying beside it have to be the same material at
    the same scale or the pile reads as two buildings". The fragment's own
    CLASS bucket contributes only ROUGHNESS and a near-neutral GRIME tint
    (`_CLASS_LOOK`'s second field, ~0.85, via `_fix_diffuse_tint` — the one
    slot that multiplies over a bound map; see that function for the MDL
    lines). ROUND 4 (D3): it no longer contributes a COLOUR. A grey-stone
    brownstone's berm comes out grey stone and a red-brick building's comes
    out red brick because the MAP says so, and the class only dirties it —
    round 3 multiplied the ~0.30 class rgb over the map instead, which is
    why every berm on the round-3 bench rendered near-black whatever
    building stood over it. `build_debris` groups its meshes by (kind,
    `source_tex_name` or `material`) so a two-texture building still merges
    to two meshes, not one per fragment.
    """
    mats = ctx.setdefault("mats", {})
    bucket = _classify(kind, material)
    parent = ctx.get("parent") or "/World"
    looks = "{0}/TornadoDebrisLooks".format(parent)
    # ROUND 4 (v6): the tone/shade SUFFIX on every cache key and prim name.
    # `ctx["mats"]` is handed IN by `wreck_urban`/`wreck_kit` and a city
    # assembly shares one dict across buildings, so a key that ignored the
    # tone would hand building 2's white-stone berm the red-brick material
    # building 1 cached under the same bucket. Empty tone + shade 0 (every
    # existing fixture/test plan, and every fragment a caller stamps
    # nothing on) produces the EMPTY suffix, so those paths and keys are
    # byte-identical to before.
    tone = str(tone or "").strip().lower()
    if tone not in _TONE_LOOK or bucket not in _TONEABLE_BUCKETS:
        tone = ""
    try:
        shade = int(shade or 0)
    except (TypeError, ValueError):
        shade = 0
    sfx = ("" if not tone else ":" + tone) + ("" if not shade else ":s%d" % shade)

    tex_url = str(tex_url or "")
    if tex_url and bucket != "glass" and not (
            _tiling_safe(tex_name) or _tiling_safe(tex_url.rsplit("/", 1)[-1])):
        # ATLAS / non-tiling source (Facades, WallBack, Metal_Front,
        # M_Images, trim sheets). ROUND 5 (user review 2026-09-02, item 13):
        # do NOT drop it to a flat class bucket ("the concrete material which
        # doesn't look right", and the white fallback when a class did not
        # resolve). Sample the atlas in a WINDOWLESS EDGE COLUMN instead, so
        # the fragment wears the building's OWN cladding colour with no
        # window on it (`_atlas_crop_material`). Roof-shed sheet kinds
        # (`membrane`/`metal`) keep their flat class look — a torn membrane
        # is roofing, not facade — matching `tornado_urban._FACADE_TEX_KINDS`.
        if bucket in ("brick", "concrete") or bucket is None:
            return _atlas_crop_material(stage, ctx, tex_url, tex_name, bucket,
                                        shade=shade)
        tex_url = ""
    if tex_url and bucket != "glass":
        base = _safe_name(tex_name or tex_url.rsplit("/", 1)[-1])
        # SHADE only, never the tone: on this branch the building's own
        # tiling map already carries its colour, so two buildings with
        # different tone tokens and the same texture want the SAME
        # material, not two identical copies of it.
        ssfx = "" if not shade else ":s%d" % shade
        key = "tornado_debris:src:" + base + ssfx
        got = mats.get(key)
        if got is not None:
            return got
        rgb, grime, rough = _CLASS_LOOK.get(bucket, _CLASS_LOOK_DEFAULT)
        # The source map already contains mortar/weathering. A class-colour
        # multiply made A3's fallen bricks visibly unlike their source wall.
        style = str((ctx.get("info") or {}).get("style") or "").lower()
        grime = _shade_tint(_source_style_tint(style), shade)
        path = looks + "/src_" + base + ("" if not shade else "_s%d" % shade)
        mat = damage._pbr(stage, path, rgb, rough, texture=tex_url,
                          scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        # ROUND 4 (D3): the GRIME tint, not the class rgb. `_pbr`'s own
        # `tint=rgb` above only lands on `diffuse_color_constant`, which a
        # valid map takes out of the product entirely (MDL 652-654) — it is
        # the map-failed fallback, kept at a plausible class albedo on
        # purpose. This line is the whole multiply the berm actually wears.
        _fix_diffuse_tint(stage, path, grime)
        mats[key] = mat
        print("[tornado_urban_usd] {0}/{1} -> SOURCE TEXTURE {2} (the "
              "building's own cladding — {3} grime tint {4:.2f} on top, "
              "{5:.2f} repeats/m)".format(
                  kind, material, base, bucket or "neutral", grime[0],
                  _TILE_REPEATS_PER_M[0]))
        return mat

    key = "tornado_debris:" + (bucket or "flat_" + _safe_name(material)) + sfx
    got = mats.get(key)
    if got is not None:
        return got

    if tone:
        # THE TONE BRANCH (v6). Fires only for a masonry bucket on a
        # building whose style names a tone AND whose own facade map was
        # not tiling-safe (a tiling source map returns above -- it already
        # carries the building's colour, so B2's own `MI_Bricks_Props_B`
        # panel debris still wins over this).
        tex, rgb, tint, rough = _TONE_LOOK[tone]
        tint = _shade_tint(tint, shade)
        path = looks + "/" + tone + ("_s%d" % shade if shade else "")
        mat = damage._pbr(stage, path, rgb, rough,
                          texture=_resolve_texture(tex),
                          scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, tint)
        print("[tornado_urban_usd] {0}/{1} -> {2} MASONRY TONE (rubble map "
              "{3}, tint {4:.2f}/{5:.2f}/{6:.2f}, shade {7}) — this "
              "building's own stone, not the generic brick class".format(
                  kind, material, tone, tex.rsplit("/", 1)[-1],
                  tint[0], tint[1], tint[2], shade))
    elif bucket == "glass":
        mat = _ensure_void_material(stage, ctx)
        print("[tornado_urban_usd] {0}/{1} -> void tone (dark glossy, NOT "
              "transparent — a see-through shard over asphalt at 60 m is "
              "invisible, plan_.md section 2.9)".format(kind, material))
    elif bucket == "brick":
        path = looks + "/brick"
        # ROUND 4 (D3): the SAME double-darkening the source-textured path
        # above carried. R5 read "clean bare brick" off the round-1 tint and
        # answered it by dropping the tint to 0.30 — but that tint is the
        # only multiply a bound map gets (MDL 652-654), so Brick_Wall_Worn
        # (itself ~0.4 mean) came out at ~0.12 and the branch stopped being
        # brick-coloured at all. The map already IS worn, mortar-streaked
        # masonry; what it needed was dirtying, not a 70% knock-down. The
        # grime tint below keeps R5's intent (brightness pulled down a
        # little, channel spread compressed to 0.06 so a heap reads greyer
        # and flatter than one clean brick face) at a brightness the eye can
        # still resolve against ~0.18 asphalt.
        rgb, grime, rough = _CLASS_LOOK["brick"]
        grime = _shade_tint(grime, shade)
        path += ("_s%d" % shade if shade else "")
        mat = damage._pbr(stage, path, rgb, rough,
                          texture=_resolve_texture(_TEX_BRICK),
                          scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, grime)
        print("[tornado_urban_usd] {0}/{1} -> Brick_Wall_Worn + neutral "
              "grime tint {2:.2f} (masonry/coping texture)".format(
                  kind, material, grime[0]))
    elif bucket == "concrete":
        path = looks + "/concrete"
        rgb, grime, rough = _CLASS_LOOK["concrete"]
        grime = _shade_tint(grime, shade)
        path += ("_s%d" % shade if shade else "")
        mat = damage._pbr(stage, path, rgb, rough,
                          texture=_resolve_texture(_TEX_CONCRETE),
                          scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, grime)
        print("[tornado_urban_usd] {0}/{1} -> Damaged_Concrete_Floor + "
              "neutral grime tint {2:.2f}".format(kind, material, grime[0]))
    elif bucket == "steel_joist":
        path = looks + "/steel_joist"
        # Round 4 (stream C's spec): open-web bar joists read DARK against
        # the light roof sheets lying over them -- plan 8c "joists dark
        # steel". Flat colour like `metal` below (same no-map gap), just a
        # near-black steel value and a duller sheen.
        rgb, _grime, rough = _CLASS_LOOK["steel_joist"]
        mat = damage._pbr(stage, path, rgb, rough)
        print("[tornado_urban_usd] {0}/{1} -> flat dark steel {2:.2f}, "
              "roughness {3:.2f} (bar joists, plan 8c)".format(
                  kind, material, rgb[0], rough))
    elif bucket == "metal":
        path = looks + "/metal"
        # No metal basecolour map lives under scene_gen/assets/materials/
        # (checked: only brick/concrete/soil megascans packs and the quake
        # rubble jpgs are there) — flat grey, low roughness for a bit of
        # sheen. `damage._pbr` never exposes a metallic input (always
        # authors `metallic_constant = 0.0`), so "metallic" here is
        # approximated by roughness alone, not a true metalness flip.
        # ROUND 4 (D3): 0.30 -> 0.44. This branch binds NO map, so its rgb
        # IS the albedo and is applied exactly once — but 0.30 was picked as
        # a tint to sit ON a texture, and weathered galvanised sheet /
        # rooftop decking is a mid grey, not a dark one.
        rgb, _grime, rough = _CLASS_LOOK["metal"]
        mat = damage._pbr(stage, path, rgb, rough)
        print("[tornado_urban_usd] {0}/{1} -> flat mid grey {2:.2f}, "
              "roughness {3:.2f} (no metal texture under scene_gen/assets/"
              "materials/; damage._pbr has no metallic input either)".format(
                  kind, material, rgb[0], rough))
    elif bucket == "membrane":
        path = looks + "/membrane"
        # R5's NEW bucket, replacing the old `deck` -> `planks.wood_material`
        # route. A built-up / modified-bitumen flat roof is dark grey-brown
        # and matte — no basecolour map for it lives under
        # scene_gen/assets/materials/ (same gap `metal` above already
        # documents), so this is flat colour + high roughness, the same
        # no-texture pattern `metal` uses just above, not a texture pick.
        # ROUND 4 (D3): 0.22 -> 0.30. Bitumen roofing IS the darkest debris
        # class in the scene and stays so — but 0.22 linear against ~0.18
        # asphalt is barely a value step, and a torn sheet that cannot be
        # told from the road it is lying on is a hole in the ground truth,
        # not a dark material.
        rgb, _grime, rough = _CLASS_LOOK["membrane"]
        mat = damage._pbr(stage, path, rgb, rough)
        print("[tornado_urban_usd] {0}/{1} -> flat dark grey-brown {2:.2f}, "
              "roughness {3:.2f} (built-up/bitumen roof membrane — never "
              "planks.wood_material; no roofing-membrane texture under "
              "scene_gen/assets/materials/ either)".format(
                  kind, material, rgb[0], rough))
    else:
        path = looks + "/flat_" + _safe_name(material)
        rgb, _grime, rough = _CLASS_LOOK_DEFAULT
        mat = damage._pbr(stage, path, rgb, rough)
        print("[tornado_urban_usd] {0}/{1} -> UNRECOGNISED material hint, "
              "flat neutral grey {2:.2f} fallback".format(
                  kind, material, rgb[0]))

    mats[key] = mat
    return mat


def _seat_z(t, w, tilt_deg, ground_z=0.0):
    """The box-centre Z that seats a fragment ON ITS FACE at `ground_z`,
    reproducing `planks._lay`'s bedding rule (see the module docstring and
    the tornado skill's "FLOATING DEBRIS" section, cause 1) collapsed to the
    ONE tilt degree of freedom a planner fragment carries.

    `_lay` seats a board with independent pitch (about its width axis) and
    roll (about its length axis) draws; a plan fragment (§2.8) carries a
    single `tilt_deg`, described as pitch about the box's own LONG axis —
    mechanically `_lay`'s "roll" term with pitch fixed at zero, which is
    also why the fragment's length axis stays perfectly horizontal (a
    rotation about its own X axis does not move the X axis) while its
    width/thickness cross-section rocks.

    `half_h` is the height of the box centre above its LOWEST corner at this
    tilt — the same quantity `_lay` computes before bedding. `bed` is the
    fixed 2 cm (or half the thickness for a thin shard) sink both a flat and
    a tilted piece get, so:

      flat  (`tilt_deg == 0`): half_h == t/2, z = ground_z + t/2 - bed
      tilted:                  lowest corner = z - half_h = ground_z - bed

    — the two cases §3's test checks by name, and in both the piece's own
    lowest vertex lands within `[-bed, 0]` of `ground_z`, never above it and
    never buried past its own half-thickness.
    """
    tilt = math.radians(float(tilt_deg or 0.0))
    half_h = 0.5 * (abs(float(t) * math.cos(tilt))
                    + abs(float(w) * math.sin(tilt)))
    bed = min(_BED_M, float(t) / 2.0)
    return float(ground_z) + half_h - bed


def _frag_box(l, w, t, x, y, z, yaw_deg, tilt_deg):
    """One fragment's 8 world points and 6 face normals.

    `R = Rz(yaw) . Rx(tilt)` in the row-vector convention this codebase
    uses elsewhere (`quake_sliced._rot3`, `planks._box`) — tilt (roll about
    the box's own length/X axis) applied first, yaw (about world Z) applied
    second, matching `planks._box`'s `Rz(yaw) @ Ry(pitch) @ Rx(roll)` with
    `pitch` fixed at zero. Cross-checked numerically against that formula
    before use (not just derived on paper).
    """
    hl, hw, ht = 0.5 * float(l), 0.5 * float(w), 0.5 * float(t)
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    ct, st = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    m00, m01, m02 = cy, -sy * ct, sy * st
    m10, m11, m12 = sy, cy * ct, -cy * st
    m20, m21, m22 = 0.0, st, ct
    pts = []
    for (sx, sy2, sz) in _CORNERS:
        a, b, c = sx * hl, sy2 * hw, sz * ht
        pts.append((x + m00 * a + m01 * b + m02 * c,
                    y + m10 * a + m11 * b + m12 * c,
                    z + m20 * a + m21 * b + m22 * c))
    nrm = [(m00 * n[0] + m01 * n[1] + m02 * n[2],
            m10 * n[0] + m11 * n[1] + m12 * n[2],
            m20 * n[0] + m21 * n[1] + m22 * n[2]) for n in _FACE_N]
    return pts, nrm


def build_debris(stage, parent, fragments, ctx, ground_z=0.0):
    """Author `<parent>/tornado_debris/<kind>_<label>` — ONE merged
    `UsdGeom.Mesh` per (kind, LABEL) class, boxes seated per `_seat_z`,
    `faceVarying` normals (24 per box — a shared-vertex box averages its
    normals at the corners and renders as a pillow, the same reason
    `quake_flow._box` and `planks.build` both author them this way).
    Returns the authored mesh paths.

    ROUND 3b (§8e F3): `label` is `source_tex_name` when a fragment carries
    one, `material` otherwise — grouping by TEXTURE rather than by the
    class-hint string when one is available, so two façade fragments off
    the SAME building (same texture) still merge into one mesh/one material
    even though a texture-less fragment of the same class would have used a
    different label. A fragment with no `source_tex_name` (every roof/
    glass fragment, and any façade fragment whose piece carried no
    resolvable texture) groups exactly as before this round — `label ==
    material` in that case, so an all-untextured plan (every existing
    fixture/test) authors byte-identical paths to before.
    """
    if not fragments:
        return []

    root = "{0}/tornado_debris".format(parent)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))

    by_class = {}
    for frag in fragments:
        kind = str(frag.get("kind") or "debris")
        material = str(frag.get("material") or "unknown")
        tex_name = str(frag.get("source_tex_name") or "")
        label = tex_name or material
        # ROUND 4 (v6): `tone` (the source building's masonry colour, from
        # `tornado_urban._tone_for`) and `shade` (the per-mesh tone jitter,
        # `_DEBRIS_SHADES`) join the group key. Both default to ""/0, and a
        # group whose fragments carry neither authors the SAME prim path
        # and binds the same material as before -- every existing fixture
        # plan is untouched.
        tone = str(frag.get("tone") or "")
        try:
            shade = int(frag.get("shade") or 0)
        except (TypeError, ValueError):
            shade = 0
        entry = by_class.setdefault(
            (kind, label, tone, shade),
            {"material": material, "tex_name": tex_name,
             "tex_url": "", "frags": []})
        if not entry["tex_url"]:
            entry["tex_url"] = str(frag.get("source_tex") or "")
        entry["frags"].append(frag)

    made = []
    # Round 4 (lead): the LAST-LINE landing clamp, for EVERY population
    # that flows through here -- the planner's own reach caps only govern
    # plan_damage's ledger, and the offline bench audit measured the ROOF-
    # PEEL sheet population landing 16 m past the plate edge (onto the
    # void) through this very function. Same per-call env contract as
    # plan_damage's region fallback: TU_PLATE_REGION in the AUTHORING
    # frame, re-read here on every call; absent = no clamp (probes/tests
    # unchanged).
    import os as _os
    reg = None
    _r = _os.environ.get("TU_PLATE_REGION", "").strip()
    if _r:
        try:
            _v = tuple(float(q) for q in _r.split(","))
            if len(_v) == 4:
                reg = _v
        except ValueError:
            reg = None
    for (kind, label, tone, shade), entry in sorted(by_class.items()):
        group = entry["frags"]
        material = entry["material"]
        pts, counts, idx, nrm, uvs = [], [], [], [], []
        for frag in group:
            size = frag.get("size") or (0.5, 0.3, 0.1)
            l, w, t = (float(q) for q in size)
            x = float(frag.get("x") or 0.0)
            y = float(frag.get("y") or 0.0)
            if reg is not None:
                x = min(max(x, reg[0] + 0.6), reg[2] - 0.6)
                y = min(max(y, reg[1] + 0.6), reg[3] - 0.6)
            yaw_deg = float(frag.get("yaw_deg") or 0.0)
            tilt_deg = float(frag.get("tilt_deg") or 0.0)
            # ROUND 3 (stream DB's berm stacking, lead fix): a berm fragment
            # may carry `z_lift` — its authored height WITHIN the heap
            # (`tornado_urban._deposit_berm`'s profile, up to `berm_h`).
            # `_seat_z` alone flattens every fragment to grade, which turns
            # a rubble berm into a flat mat; the lift is added ON TOP of the
            # face-seated z, so a lifted fragment rests conceptually on the
            # heap below it (authored, never simulated — the same call
            # `quake_rubble` makes for its mound scatter). Non-berm
            # fragments carry no `z_lift` and are byte-identical.
            z = _seat_z(t, w, tilt_deg, ground_z=ground_z) \
                + max(0.0, float(frag.get("z_lift") or 0.0))
            p, n = _frag_box(l, w, t, x, y, z, yaw_deg, tilt_deg)
            base = len(pts)
            pts.extend(Gf.Vec3f(*q) for q in p)
            for fi, face in enumerate(_FACES):
                counts.append(4)
                idx.extend(base + v for v in face)
                nrm.extend([Gf.Vec3f(*n[fi])] * 4)
                # ROUND 5 (windowless-atlas crop): a per-face [0,1] quad `st`
                # so `_atlas_crop_material` (UV-space) samples the whole
                # windowless sub-rectangle across each box face. The
                # triplanar (`:src:`/class/tone) and flat (metal/membrane)
                # materials ignore `st` entirely, so this is inert for every
                # other look — it exists only for the crop path.
                uvs.extend([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0),
                            Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0)])

        path = "{0}/{1}_{2}{3}{4}".format(
            root, _safe_name(kind), _safe_name(label),
            ("_" + _safe_name(tone)) if tone else "",
            ("_s%d" % shade) if shade else "")
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        stv = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying)
        stv.Set(Vt.Vec2fArray(uvs))
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        zs = [q[2] for q in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])

        mat = debris_material(stage, ctx, kind, material,
                              tex_url=entry["tex_url"],
                              tex_name=entry["tex_name"],
                              tone=tone, shade=shade)
        if mat is not None:
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        made.append(path)
        if ctx.get("verbose", True):
            print("[tornado_urban_usd] {0:<24s} {1:3d} fragment(s) -> 1 "
                  "mesh, {2} point(s)".format(
                      "{0}/{1}{2}{3}".format(
                          kind, label, ("/" + tone) if tone else "",
                          ("/s%d" % shade) if shade else ""),
                      len(group), len(pts)))
    return made


def _seat_source_rubble(stage, ctx, fragments, ground_z=0.0):
    """Move loose tear cells to matching planned landings, preserving the
    source mesh, material graph and UVs. Returns consumed ledger indices.

    The remaining ledger entries still use the merged-box representation, so
    this is bounded and keeps authoring cost predictable.
    """
    try:
        limit = int(_os.environ.get("TU_SOURCE_RUBBLE_MAX", "96") or 96)
    except (TypeError, ValueError):
        limit = 96
    if limit <= 0:
        return set()
    by_leaf = {}
    structural = []
    for i, frag in enumerate(fragments):
        src = str(frag.get("from") or "").rsplit("/", 1)[-1]
        if src and str(frag.get("kind") or "") not in ("glass", "membrane"):
            by_leaf.setdefault(src, []).append(i)
            structural.append(i)
    used = set()
    tag_prefix = "brk_{0}_".format(ctx.get("tag") or "")
    for path in ctx.get("loose") or ():
        if len(used) >= limit:
            break
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Mesh):
            continue
        group = str(path).rsplit("/", 2)[-2]
        leaf = group[len(tag_prefix):] if group.startswith(tag_prefix) else ""
        choices = by_leaf.get(leaf) or []
        while choices and choices[0] in used:
            choices.pop(0)
        # Tear cells come from the SURVIVING border piece, whereas the debris
        # ledger is keyed by the REMOVED neighbour, so an exact source-path
        # match normally does not exist. The fallback still preserves the
        # correct BUILDING skin; it borrows only the removed neighbour's
        # already-approved landing point.
        if not choices:
            while structural and structural[0] in used:
                structural.pop(0)
            choices = structural
        if not choices:
            continue
        i = choices.pop(0)
        target = fragments[i]
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts:
            continue
        lo = [min(float(q[k]) for q in pts) for k in range(3)]
        hi = [max(float(q[k]) for q in pts) for k in range(3)]
        tx = float(target.get("x", 0.0)) - 0.5 * (lo[0] + hi[0])
        ty = float(target.get("y", 0.0)) - 0.5 * (lo[1] + hi[1])
        tz = (float(ground_z) + float(target.get("z_lift", 0.0)) + 0.01
              - lo[2])
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
        used.add(i)
    if used:
        print("[tear] source rubble ({0}): {1} loose source fragment(s) "
              "seated at planned landings with exact material/UV".format(
                  ctx.get("tag"), len(used)))
    return used


# ---------------------------------------------------------------------------
# ROUND 3b F2a (§8e, stream FX1) — RAGGED TEARS on the apply side.
#
# NOT `quake_sliced._author_tears`, called wholesale. That function's own
# SECOND half, `_author_floor_edges` -> `quake_flow._ragged_slabs` ->
# `quake_flow._a_roofify`, is unconditionally called at its end and:
#   (a) reads `ctx["fit"]["slabs"]` with a bare `ctx["fit"]` lookup — a
#       `KeyError` the moment `plan["tear_scope"]` is non-empty (any real
#       tear job) against a ctx this ladder never builds one for on its own
#       (F2b, below, builds a MUCH smaller thing under the same key, on
#       purpose — see `_author_interior`'s own docstring for why it is not
#       `quake_flow.fit_interior`'s full storey/column/content rig either);
#   (b) `_a_roofify`'s own docstring is explicit that it is destructive in a
#       way this ladder must never be: "Swap EVERY kit roof tile of `mass`
#       for an authored slab ... ALL OF THEM, not just the one a recipe is
#       about to break" — calling it here would silently overwrite F1's own
#       carefully-computed per-tile roof SURVIVAL (`tornado_urban.
#       _shed_unsupported_roof`) with a blanket re-author the moment any
#       tear exists on the building at all.
# Reused instead: the three PRIMITIVES `_author_tears`'s own first half
# calls — `fire_collapse._tear_perimeter`, `fire_collapse._own_rng`,
# `fracture.stable_seed` — in the SAME per-mass driving loop, minus the
# floor-edge call. This is not a copy of `_author_tears`'s body (its own
# per-mass loop is five lines of glue around those three calls); it is a
# small NEW function built from the same reusable pieces.
# ---------------------------------------------------------------------------
#: THE TEAR LOOK — round 4 (D2). Two multiplies over the piece's OWN map
#: (`_fix_diffuse_tint`'s slot; see that function for the MDL lines that make
#: it the only one that survives a valid texture):
#:   `_TEAR_FACE_TINT`  the surviving/broken-away WALL surface. The same
#:                      near-neutral grime `debris_material`'s brick bucket
#:                      uses, so a torn stub and the berm under it are the
#:                      same material at the same tile rate — the suburb
#:                      skill's "a fragment and a board lying beside it have
#:                      to be the same material at the same scale".
#:   `_TEAR_CUT_TINT`   the CUT faces only (the `core` GeomSubset
#:                      `fracture.face_subset` puts every invented face in).
#:                      Mortar/core darkening ONCE — a broken masonry edge is
#:                      the unweathered inside of the wall, in shadow.
_TEAR_FACE_TINT = (1.0, 1.0, 1.0)
_TEAR_CUT_TINT = (0.60, 0.575, 0.55)
_TEAR_FACE_ROUGH = 0.88
_TEAR_CUT_ROUGH = 0.93
#: The FALLBACK when a piece carries no resolvable texture: neutral grey
#: PLASTER, never a generic brick photo. `quake_flow._t_core_mat`'s own
#: fallback is brick 70 % of the time (`QuakeLooks/c_brick` — the megascans
#: `Brick_Wall_Worn` map), which is precisely the user's round-4 verdict,
#: "the damaged parts have some other building's material on it": measured
#: on the real SM_Building_02 T4 probe, **854 of 1210** tear fragments had
#: their cut faces bound to that one generic map.
_TEAR_FALLBACK_RGB = (0.52, 0.505, 0.485)
_TEAR_CUT_FALLBACK_RGB = (0.36, 0.35, 0.335)


def _tear_material(stage, ctx, tex_url, tex_name, cut, tone="", btype=""):
    """The material a torn face wears: a triplanar of THE PIECE'S OWN
    annotated cladding texture (`annotate_surface`'s `_tex_url`/`_tex_name`
    stamp), grimed for a wall face and darkened once for a cut face —
    or neutral grey plaster when the piece carries no texture at all.

    One material per (texture basename, face kind), cached in `ctx["mats"]`
    under `"tornado_tear:*"` keys — the same cache-in-ctx discipline
    `debris_material` uses for its own `"tornado_debris:*"` keys, and
    deliberately NOT `ctx["cache"]` (which `quake_flow._clad_material` owns
    and keys by raw texture string).

    WHY NOT `quake_flow._clad_material`, which is what `fire_collapse.
    _tear_perimeter` reaches for on its own: (a) it is fed
    `damage.bound_texture`, which returns the FIRST material found on the
    first mesh's first subset — on a GAC slice that is whichever subset the
    slicer wrote first, measured as `M_Building_01_WallBack` (the blind
    back-wall map) 255 times, `M_Images` (a poster/signage atlas) 24 times
    and the actual `Concrete_02` cladding 6 times on one T4 probe; and
    (b) it authors at a fixed `scale_uv=(0.45, 0.45)`, a different tile
    rate from everything else this ladder puts on the same building, which
    is the "rectangular darker mismatched-tiling patch" read. `annotate_
    surface` measures the DOMINANT NON-GLAZING texture instead, which is
    the piece's actual cladding.
    """
    mats = ctx.setdefault("mats", {})
    parent = ctx.get("parent") or "/World"
    looks = "{0}/TornadoTearLooks".format(parent)
    kind = "cut" if cut else "face"
    tex_url = str(tex_url or "")
    if tex_url and not (
            _tiling_safe(tex_name) or _tiling_safe(tex_url.rsplit("/", 1)[-1])):
        # Same atlas rule as `debris_material`: a packed facade sheet on a
        # cut face is a random crop, not mortar -- fall through to the
        # neutral plaster/mortar fallback below.
        tex_url = ""
    # Atlas pixels do not belong on invented fracture topology. Kit styles
    # carry a measured masonry tone instead; curtain-wall chunks expose the
    # concrete frame/slab rather than glass.
    tone = str(tone or "").lower()
    btype = str(btype or "").lower()
    bucket = "concrete" if btype == "rc_glass" else ""
    if bucket:
        tex_url, tone = "", ""
    if tone in _TONE_LOOK:
        tex_url = ""
        tex, flat, tint0, rough = _TONE_LOOK[tone]
        key = "tornado_tear:tone:{0}:{1}".format(tone, kind)
        got = mats.get(key)
        if got is not None:
            return got
        tint = tuple(float(c) * (0.68 if cut else 1.0) for c in tint0)
        path = "{0}/{1}_{2}".format(looks, kind, tone)
        mat = damage._pbr(stage, path, flat, _TEAR_CUT_ROUGH if cut else rough,
                          texture=_resolve_texture(tex),
                          scale_uv=_TILE_REPEATS_PER_M, tint=flat)
        _fix_diffuse_tint(stage, path, tint)
        mats[key] = mat
        return mat
    if bucket:
        key = "tornado_tear:{0}:{1}".format(bucket, kind)
        got = mats.get(key)
        if got is not None:
            return got
        flat, _tint, rough = _CLASS_LOOK[bucket]
        path = "{0}/{1}_{2}".format(looks, kind, bucket)
        mat = damage._pbr(stage, path, flat,
                          _TEAR_CUT_ROUGH if cut else rough)
        mats[key] = mat
        return mat
    if tex_url:
        base = _safe_name(tex_name or tex_url.rsplit("/", 1)[-1])
        key = "tornado_tear:src:{0}:{1}".format(base, kind)
        got = mats.get(key)
        if got is not None:
            return got
        rgb = _TEAR_CUT_FALLBACK_RGB if cut else _TEAR_FALLBACK_RGB
        style = str((ctx.get("info") or {}).get("style") or "").lower()
        st = _source_style_tint(style)
        tint = (tuple(0.72 * c for c in st) if cut and st != (1.0, 1.0, 1.0)
                else (_TEAR_CUT_TINT if cut else st))
        rough = _TEAR_CUT_ROUGH if cut else _TEAR_FACE_ROUGH
        path = "{0}/{1}_{2}".format(looks, kind, base)
        mat = damage._pbr(stage, path, rgb, rough, texture=tex_url,
                          scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, tint)
        mats[key] = mat
        return mat
    key = "tornado_tear:plaster:" + kind
    got = mats.get(key)
    if got is not None:
        return got
    rgb = _TEAR_CUT_FALLBACK_RGB if cut else _TEAR_FALLBACK_RGB
    path = "{0}/{1}_plaster".format(looks, kind)
    # NO TEXTURE, so no `_fix_diffuse_tint`: with no map the albedo IS
    # `diffuse_color_constant` (MDL 652-654, see `_fix_diffuse_tint`), and
    # multiplying a grime tint over it a second time is the double-darkening
    # that produced round 3's near-black berms.
    mat = damage._pbr(stage, path, rgb, _TEAR_CUT_ROUGH if cut
                      else _TEAR_FACE_ROUGH)
    mats[key] = mat
    return mat


def _tex_of_element(e):
    """`(url, name)` of a plan element's own annotated cladding texture."""
    p = (e or {}).get("p") or {}
    return str(p.get("_tex_url") or ""), str(p.get("_tex_name") or "")


def _tear_tex_of_element(e, elements):
    """A tiling-safe facade map for a tear, borrowing the nearest peer.

    Packed WallBack/floor/awning atlases may be valid on the original UV'd
    mesh but cannot be projected onto newly fractured faces.  Using them was
    the remaining white-patch failure after fallback bindings were repaired.
    """
    url, name = _tex_of_element(e)
    ename = str((e or {}).get("name") or "").lower()
    structural_core = ename.startswith(("core_x", "core_y"))
    if (not structural_core and url and
            (_tiling_safe(name) or _tiling_safe(url.rsplit("/", 1)[-1]))):
        return url, name
    ep = (e or {}).get("p") or {}
    side = ep.get("_side") or (e or {}).get("side")
    storey = int(ep.get("_storey", (e or {}).get("storey", 0)))
    bay = int(ep.get("_bay", 0))
    donors = []
    for d in elements or ():
        du, dn = _tex_of_element(d)
        if not du or not (_tiling_safe(dn) or
                          _tiling_safe(du.rsplit("/", 1)[-1])):
            continue
        dp = d.get("p") or {}
        words = (dn + " " + du).lower()
        facade_named = any(q in words
                           for q in ("brick", "stone", "stucco", "concrete"))
        masonry_rank = (0 if any(q in words for q in
                                 ("brick", "stone", "stucco")) else 20)
        donors.append(((0 if (dp.get("_side") or d.get("side")) == side else 1000,
                        abs(int(dp.get("_storey", d.get("storey", 0))) - storey),
                        masonry_rank,
                        0 if facade_named else 100,
                        abs(int(dp.get("_bay", 0)) - bay)), du, dn))
    if donors:
        _score, du, dn = min(donors, key=lambda q: q[0])
        return du, dn
    return "", ""


def _reface_tear_fragments(stage, ctx, by_frag, paths, stats):
    """Bind invented cut faces without replacing the exact facade skin.

    `by_frag` maps a fragment's PARENT prim name (`brk_<tag>_<piece leaf>`,
    `quake_flow._break`'s own naming) to that piece's `(tex_url, tex_name)`.

    Two rebinds per fragment, and the split matters:

      * the `core` GeomSubset — `fracture.face_subset`'s "every face that is
        NOT the façade", i.e. the CUT faces, the back and the reveals — takes
        the darkened cut material. `quake_flow._t_core_bind` has already
        bound it to `_t_core_mat`, which for a urm building is the generic
        megascans brick 70 % of the time whatever the building is made of.
        These are the LARGEST faces on a chunk (`_t_core_mat`'s own
        docstring) so they carry the read.
      * the fragment PRIM keeps `fire_collapse.skin_fragment`'s exact source
        material and source UV projection. A new triplanar material using the
        same image is not equivalent to the authored facade graph and UVs.

    The prim-level bind is WEAK (`quake_flow._bind`), the same strength
    `skin_fragment` uses and for the same reason: a `strongerThanDescendants`
    bind here would take the cut material back off the `core` subset, which
    is a child prim.
    """
    for frag in paths:
        prim = stage.GetPrimAtPath(frag) if frag else None
        if not prim or not prim.IsValid():
            stats["missing"] += 1
            continue
        parent_name = str(frag).rsplit("/", 2)[-2] if "/" in str(frag) else ""
        spec = by_frag.get(parent_name, ("", ""))
        tex_url, tex_name = spec[:2]
        tone = spec[2] if len(spec) > 2 else ""
        btype = spec[3] if len(spec) > 3 else ""
        if not tex_url:
            stats["no_tex"] += 1
        # 1) the cut faces
        sub = prim.GetChild("core")
        if sub and sub.IsValid():
            qf._bind(stage, str(sub.GetPath()),
                     _tear_material(stage, ctx, tex_url, tex_name, True,
                                    tone=tone, btype=btype))
            stats["cut"] += 1
        else:
            stats["no_core"] += 1
        # `_tear_perimeter` already reconstructed this fragment's outward
        # skin from the parent triangles. Do not replace its exact material
        # graph and UV placement with a projected approximation.
        stats["kept_skin"] += 1


def _author_tears(stage, ctx, plan):
    """Ragged tears on every surviving façade piece bordering a hole
    (`plan["tears"]`, `tornado_urban._plan_tears`/`_cap_tears`'s own JSON-
    safe job list). Re-resolves each job's `path` against `ctx["info"][
    "elements"]` (a plan round-tripped through JSON carries no live
    reference) — this ctx must therefore carry a REAL `info["elements"]`
    with authored `prim_path`s, which is true the moment `apply_plan` is
    called from `wreck_urban`/`tornado_kit.wreck_kit` (both build `info`
    from placements already on the stage). `ctx["cache"]` is set here if
    absent: `fire_collapse._tear_perimeter` -> `quake_flow._clad_material`
    reads it as a plain per-texture material cache, a key this ladder's
    own ctx never otherwise needs (its own materials all live in
    `ctx["mats"]` under `"tornado_debris:*"`/`"void"` keys instead).
    Returns the number of pieces/slabs touched.

    ROUND 4 (D2) — EVERY TORN FACE WEARS THIS BUILDING'S OWN MATERIAL.
    `_tear_perimeter`'s own material choices come from `damage.
    bound_texture` (the FIRST subset found on the piece) and, for the cut
    faces, from `quake_flow._t_core_mat` (a generic megascans brick 70 % of
    the time on a `urm` building). Measured on the real SM_Building_02 T4
    probe BEFORE this pass: of 1210 authored fragments, **854 had their cut
    faces on one generic `QuakeLooks/c_brick`**, 83 whole fragments were on
    it too, and 310 more wore a `clad_*` world triplanar of the blind
    `WallBack` map (255) / a poster atlas (24) at a tile rate nothing else
    on the building uses — the user's "the damaged parts have some other
    building's material on it" and "random textures that don't match that
    building", one number each. `_reface_tear_fragments` (above) rebinds
    what that pass got wrong from `annotate_surface`'s MEASURED per-piece
    dominant cladding texture, and leaves alone every fragment
    `fire_collapse.skin_fragment` already skinned with the parent's own
    material and UVs.
    """
    tears = [t for t in (plan.get("tears") or ()) if not t.get("dropped")]
    if not tears:
        return 0
    import random as _random

    import numpy as _np

    from . import fire_collapse as fc
    from . import fracture

    ctx.setdefault("velocity", {})
    ctx.setdefault("cache", {})
    info = ctx["info"]
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in info["elements"] if (e.get("p") or {}).get("prim_path")}
    by_mass = {}
    for t in tears:
        by_mass.setdefault(t.get("mass") or "main", []).append(t)
    n = 0
    n_failed_resolve = 0
    n_trim_skipped = 0
    tag = ctx.get("tag")
    reface = {"cut": 0, "face": 0, "kept_skin": 0, "no_core": 0,
              "no_tex": 0, "missing": 0}
    ctx.setdefault("_tear_statics", [])
    ctx.setdefault("loose", [])
    for mass in sorted(by_mass):
        m = info["masses"].get(mass) or info["masses"]["main"]
        jobs = []
        for t in by_mass[mass]:
            e = by_path.get(t.get("path"))
            if e is None or e.get("dead"):
                n_failed_resolve += 1
                continue
            # Tear fragments are an edge treatment for a torn FACADE, not
            # airborne ornament. Fracturing a narrow parapet/cornice/coping
            # scatters dozens of disconnected `brk_*/*frag_*` islands at the
            # original roof height after its neighbour is removed. They read
            # as white floating confetti and have no physical support. Shed
            # trim is already represented by the ground debris ledger.
            pmeta = e.get("p") or {}
            role = str(pmeta.get("_role") or e.get("role") or "").lower()
            words = " ".join((role, str(e.get("name") or "").lower(),
                              str(t.get("path") or "").lower()))
            if any(q in words for q in
                   ("parapet", "cornice", "coping", "roof", "ledge")):
                n_trim_skipped += 1
                continue
            jobs.append({"el": e, "side": t.get("side"),
                        "cuts": t.get("cuts") or []})
        if not jobs:
            continue
        seed = fracture.stable_seed(tag, mass, "tear")
        prng = _random.Random(seed)
        pnrng = _np.random.default_rng(seed & 0xFFFFFFFF)
        # ROUND 4 (D2): what did THIS call author? `_tear_perimeter` extends
        # `ctx["_tear_statics"]` and `ctx["loose"]` in place and returns only
        # a count, so the fragments it made are the TAIL of those two lists —
        # snapshot the lengths, diff after. (`fire_collapse` is read-only for
        # this stream; wrapping is the whole point of this local driver.)
        n_st0 = len(ctx["_tear_statics"])
        n_lo0 = len(ctx["loose"])
        ctx["_skin_loose_tears"] = True
        with fc._own_rng(ctx, prng, pnrng):
            n += fc._tear_perimeter(ctx, plan, m, prng, jobs)
        # `quake_flow._break`'s naming: fragments land under
        # `<parent>/brk_<tag>_<piece leaf name>/frag_NNN`, so the parent
        # prim's name maps a fragment back to the piece it came from — and
        # therefore to that piece's own `annotate_surface` texture stamp.
        by_frag = {}
        from . import tornado_urban as _tu
        tone = _tu._tone_for(info.get("style"))
        btype = (info.get("btype") or info.get("construction") or
                 (ctx.get("sliced") or {}).get("btype") or
                 (plan.get("btype") if isinstance(plan, dict) else ""))
        for j in jobs:
            path = ((j.get("el") or {}).get("p") or {}).get("prim_path")
            if not path:
                continue
            tex = _tear_tex_of_element(j.get("el"), info.get("elements"))
            by_frag["brk_{0}_{1}".format(tag, str(path).rsplit("/", 1)[-1])] = \
                (tex[0], tex[1], tone, btype)
        _reface_tear_fragments(
            stage, ctx, by_frag,
            list(ctx["_tear_statics"][n_st0:]) + list(ctx["loose"][n_lo0:]),
            reface)
    if tears or n_failed_resolve:
        print("[tear] tornado_urban_usd ({0}): {1} facade piece(s) touched, "
              "{2} trim job(s) skipped, {3} job(s) dropped, {4} could not "
              "be re-resolved".format(
                  tag, n,
                  n_trim_skipped,
                  sum(1 for t in (plan.get("tears") or ()) if t.get("dropped")),
                  n_failed_resolve))
        print("[tear] reface ({0}): {1} cut-face subset(s) rebound, {2} "
              "fragment surface override(s), {3} kept on "
              "fire_collapse's exact source facade skin, {4} had no `core` "
              "subset, {5} had no annotated texture (neutral plaster), {6} "
              "prim(s) missing".format(
                  tag, reface["cut"], reface["face"], reface["kept_skin"],
                  reface["no_core"], reface["no_tex"], reface["missing"]))
        ctx.setdefault("notes", []).append(
            "tears: {0} facade piece(s) torn ({1} unsupported trim job(s) "
            "suppressed); {2} cut face(s) + {3} fragment "
            "surface override(s); source facade skin retained "
            "({4} neutral-plaster fallback)".format(
                n, n_trim_skipped, reface["cut"], reface["face"],
                reface["no_tex"]))
    ctx["_tear_reface"] = reface
    return n


# ---------------------------------------------------------------------------
# ROUND 3b F2b (§8e, stream FX1) — VISIBLE INTERIORS: `quake_flow.
# fit_interior` for every OPENED storey/side, plus a dark-but-textured
# interior BACKING quad so the hole reads as a gutted, floor-lined
# interior instead of a black slab (the plan brief's own "the black slab
# reads as a texture hole, not a gutted floor").
# ---------------------------------------------------------------------------
_FIT_INTERIOR_FRAC = 0.18   # a plan's own `stats["removed_frac"]` (BY
                            # FAÇADE AREA, `tornado_urban`'s own metric) at
                            # or above this authors a fit-out on its own;
                            # below it, the OR checks below (the two "big
                            # hole" recipes, and — ROUND 4 — any removed
                            # piece or any voided pane at all) still fire it
_FIT_INTERIOR_RECIPES = frozenset({"facade_collapse", "chunk"})

# ---------------------------------------------------------------------------
# ROUND 4, DEFECT D1 (stream K) — THE BACKING REWORK
# ---------------------------------------------------------------------------
# What round 3 authored, and what the user saw. ONE quad per (side, storey)
# spanning the UNION of every removed piece's along-side extent on that
# side, at `_INTERIOR_BACKING_RGB = (0.08, 0.08, 0.08)` and 0.5 m inset.
# Two independent failures compounded:
#
#   1. THE SPAN. A union is not a hole. B1's south elevation lost pieces at
#      both ends of every storey, so the union was the WHOLE side and the
#      quad became black cladding wrapping the building — four of them
#      stacked read as a gutted black box (B1_obl, B3_obl's corner).
#      A parapet's own removal fed the same union, so the black band ran
#      up over the roof line where there is no interior at all.
#   2. THE ALBEDO. MEASURED on the exported stage by stream D: the round-3
#      backing bound `T_vizbefe_2K_B.png` with `diffuse_color_constant =
#      (0.08, 0.08, 0.08)` AND `diffuse_tint = (0.08, 0.08, 0.08)`. Per
#      OmniPBR.mdl, `diffuse_color_constant` is the MAP-FAILED FALLBACK and
#      `diffuse_tint` MULTIPLIES the resolved map — so the effective albedo
#      was 0.08 x the map's own 0.445 mean ~= 0.03. That is the black slab,
#      in numbers, and the fix is the TINT, not a bigger `rgb`.
#
# THE REWORK, in three parts:
#
#   * PER-HOLE quads. `_opened_holes` walks the REMOVED wall/pier/corner
#     pieces (never parapet, never roof — there is no room behind a
#     coping), takes each piece's OWN measured span and z-extent
#     (`fire_collapse.el_span` / `el_z_span`, the same measurement the tear
#     pass uses), and merges only pieces that actually ABUT
#     (`_HOLE_MERGE_GAP_M`). Two holes at opposite ends of one elevation
#     stay two quads with standing wall between them.
#   * A TEXTURED, DARK-WARM material. `diffuse_tint` is set so that TINT x
#     the map's own measured mean lands on `_INTERIOR_BACKING_ALBEDO` —
#     0.34 / 0.29 / 0.24, a dim warm room, inside the round-4 brief's own
#     0.25-0.35 band — and `diffuse_color_constant` is set to that SAME
#     albedo, so a build where the map does not resolve falls back to a dim
#     warm wall rather than to white litter (stream D's own standing note:
#     never leave a white constant behind a texture that can fail).
#   * THE STOREFRONT RING (B2/D4). A ground storey whose bays are GLAZED
#     is see-through: the round-3 bench's `dw_terrace` cell was a glass
#     ring you could look straight through to the far elevation's glass,
#     with the upper floors apparently standing on it. Nothing is REMOVED
#     there — ground-storey structure is never removed (`tornado_kit`'s own
#     guard) — so a hole-driven backing pass can never fix it. One
#     CONTINUOUS quad per side at storey 0, set the same inset in, is the
#     shop's back wall: correct behind intact glass, correct behind a
#     voided pane, and the four of them close a shallow room the eye
#     stops in.
# ---------------------------------------------------------------------------

#: MEASURED mean sRGB of `_TEX_CONCRETE`'s own map (PIL, 256x256 resample):
#: (0.445, 0.422, 0.403). The tint below is derived from it rather than
#: guessed so the ALBEDO is the number under review, not the multiplier.
_TEX_CONCRETE_MEAN_SRGB = (0.445, 0.422, 0.403)
#: The interior backing's effective albedo — a dim, warm room seen from
#: outside in daylight. The round-4 D1 brief's band is 0.25-0.35 and EVERY
#: channel sits inside it (0.30 mean, red > green > blue: warm, not a grey
#: card). A first cut at (0.34, 0.29, 0.24) was warmer still and put blue
#: a hair under the band — kept in the record because "mean in band" is not
#: the same claim as "in band".
#:
#: WHERE THIS DIVERGES FROM STREAM D, DELIBERATELY. D's round-4 rule for
#: DEBRIS is "diffuse_tint NEAR-NEUTRAL (~0.87, 0.85, 0.83)", and it is
#: right there: a fragment carries the SOURCE BUILDING's own texture and
#: the tint must only add grime, never re-colour it. This material has no
#: source texture — it is a generic concrete map standing in for a room
#: interior — so what is under review is the ROOM'S DARKNESS, which D1
#: pins directly (0.25-0.35) and D's 0.87 would put at 0.87 x 0.445 = 0.39,
#: outside it. The tint below (0.74 / 0.71 / 0.67, hue ratio 1 : 0.96 :
#: 0.90) is as close to D's neutral as the albedo band allows.
_INTERIOR_BACKING_ALBEDO = (0.33, 0.30, 0.27)


def _tint_for(albedo, mean=None):
    """The `diffuse_tint` that puts `albedo` on screen over a map whose own
    measured mean is `mean` — `diffuse_tint` MULTIPLIES the resolved map
    (OmniPBR.mdl; measured by stream D on the exported round-3 stage,
    where a 0.08 tint over this 0.445-mean map rendered at ~0.03). Stated
    as a function so every look in this region is authored from the ALBEDO
    it wants rather than from a multiplier someone has to reverse."""
    mean = mean or _TEX_CONCRETE_MEAN_SRGB
    return tuple(min(1.0, float(a) / float(m)) for a, m in zip(albedo, mean))


_INTERIOR_BACKING_TINT = _tint_for(_INTERIOR_BACKING_ALBEDO)
#: The FALLBACK `diffuse_color_constant` — what renders when the map does
#: not resolve. Deliberately the same albedo, never white (a white constant
#: is how a failed map turns into white litter) and never the old 0.08.
_INTERIOR_BACKING_RGB = _INTERIOR_BACKING_ALBEDO
_INTERIOR_BACKING_ROUGHNESS = 0.93
#: The backing sits this far INSIDE the wall line. 0.5 m read as a pane
#: swapped for a black one; >= 1.2 m is the round-4 requirement and reads
#: as ROOM DEPTH — you see the floor slab and the wall behind it.
_INTERIOR_INSET_M = 1.35
#: ... clamped on a small mass so two opposite backings can never cross:
#: never more than this share of the mass's own smaller plan dimension.
_INTERIOR_INSET_MAX_FRAC = 0.30
#: Two removed pieces whose spans are within this of each other are ONE
#: hole (they were adjacent bays). Anything further apart keeps standing
#: wall between it and stays a separate quad.
_HOLE_MERGE_GAP_M = 0.40
#: A backing quad is a thin slab, not a wall.
_BACKING_T_M = 0.18
#: A hole longer than this is SEGMENTED into per-bay panels rather than
#: authored as one slab: `t_out_of_plane_top` legitimately peels a whole
#: 32 m top-storey elevation on `brownstone_row`, and one 32 m plane
#: behind it is the flat black band again, however well it is textured.
#: The segments are CONTIGUOUS and COPLANAR, sized off the mass's own bay
#: module. v6 staggered alternate segments 0.55 m deeper to break the plane
#: up; on the lit bench that read as a row of free-standing slabs, so the
#: stagger is gone (lead review v7) and only the segmentation remains.
_BACKING_SEG_MAX_M = 6.0
_BACKING_SEG_MIN_M = 3.0
#: Per building, so a pathological plan cannot author hundreds of boxes.
_MAX_BACKING_QUADS = 64
#: A storefront ring is a GROUND-STOREY band, never a full-height shell:
#: on a mass whose `levels` carries no first-floor entry the storey-0 span
#: would otherwise be the whole building and the ring would hide the
#: interior it exists to reveal.
_SHOP_MAX_H_M = 7.0
#: Fit-out CONTENTS are kept at least this far inside the STOREY's own
#: measured plan rectangle (`_storey_plan_rects`) on every fitted storey —
#: `quake_flow.fit_interior` draws them on the MASS bbox inset 1.5 m, which
#: is outside the glass on a setback plan (A4).
_PROP_PLAN_INSET_M = 1.5
#: Fit-out CONTENTS (chairs, desks) are pushed at least this far in from
#: any OPENED wall line, so nothing pokes out through a torn edge. 
#: `quake_flow.fit_interior` already keeps props `WALL_INSET + 1.5` =
#: 2.05 m in from every wall; this is the extra keep-out on the sides that
#: are actually open, applied here (the caller) rather than in
#: `quake_flow.py`, which is the earthquake stream's file.
_PROP_EDGE_KEEPOUT_M = 2.6


def _ensure_fit_mats(stage, ctx):
    """`quake_flow.fit_interior` insists on plain `mats["plaster"]` (every
    partition, every btype) and `mats["timber"]` (a urm slab) — DIRECT
    dict lookups, unlike its own `concrete_look` (built through `_c_look_
    at`, which is self-seeding and needs nothing pre-populated). Tornado's
    `ctx["mats"]` carries neither key (this ladder's own materials are the
    `"void"`/`"tornado_debris:*"` keys `debris_material` builds) — built
    here, once, rather than editing `quake_flow.fit_interior` itself
    (earthquake's file, not touched this round).

    ROUND 4: both are TEXTURED now. They were flat `damage._pbr` neutrals,
    and a flat neutral partition seen through a hole is exactly the "random
    single coloured rectangles" the round-4 verdict names (rule 5: no
    untextured single-colour rectangles anywhere). Same map and the same
    single-tint arrangement as the backing (`_interior_backing_material`'s
    own docstring): white base, the look on `diffuse_tint`, world-projected
    triplanar so an authored `_box` with no UVs still shows grain at the
    right physical size. `timber` keeps a warm dark tone for a urm floor
    plate — a dusty boarded floor seen from above, never pale sawn lumber
    (rule 5 again: no lumber reads anywhere urban).
    """
    mats = ctx.setdefault("mats", {})
    parent = ctx.get("parent") or "/World"
    tex = _resolve_texture(_TEX_CONCRETE)
    if mats.get("plaster") is None:
        path = "{0}/TornadoFitLooks/plaster".format(parent)
        rgb = (0.42, 0.40, 0.37)
        mats["plaster"] = damage._pbr(
            stage, path, rgb, 0.90, texture=tex,
            scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, _tint_for(rgb))
    if mats.get("timber") is None:
        path = "{0}/TornadoFitLooks/timber".format(parent)
        rgb = (0.26, 0.21, 0.16)
        mats["timber"] = damage._pbr(
            stage, path, rgb, 0.88, texture=tex,
            scale_uv=_TILE_REPEATS_PER_M, tint=rgb)
        _fix_diffuse_tint(stage, path, _tint_for(rgb))
    return mats


def _interior_backing_material(stage, ctx):
    """ONE textured interior-wall material per building, cached on the ctx.

    THE TWO SLOTS, PER OmniPBR.mdl (measured by stream D on the exported
    round-3 stage, not inferred): `diffuse_tint` MULTIPLIES the resolved
    `diffuse_texture`; `diffuse_color_constant` is what renders when the
    map does NOT resolve. Round 3 set BOTH to `(0.08, 0.08, 0.08)`, so the
    quads rendered at 0.08 x the map's own 0.445 mean ~= 0.03 — the black
    slab. Here the tint is derived from the albedo the look wants
    (`_tint_for`) and the constant is that SAME albedo, so the quad reads
    identically whether or not the texture resolves, and neither slot is
    white (a white constant behind a failed map is white litter).
    """
    mats = ctx.setdefault("mats", {})
    key = "tornado_debris:interior_backing"
    got = mats.get(key)
    if got is not None:
        return got
    parent = ctx.get("parent") or "/World"
    path = "{0}/TornadoDebrisLooks/interior_backing".format(parent)
    mat = damage._pbr(stage, path, _INTERIOR_BACKING_RGB,
                      _INTERIOR_BACKING_ROUGHNESS,
                      texture=_resolve_texture(_TEX_CONCRETE),
                      scale_uv=_TILE_REPEATS_PER_M,
                      tint=_INTERIOR_BACKING_RGB)
    _fix_diffuse_tint(stage, path, _INTERIOR_BACKING_TINT)
    mats[key] = mat
    return mat


def _wants_fit_interior(plan):
    """Does this plan open the envelope at all?

    ROUND 4 adds the third clause. The first two (an area threshold and two
    named recipes) were tuned for the ROUND-3 ladder, where only a big
    recipe opened anything; with the kit guard in place a T3/T4 plan is a
    toothed band and a corner, well under `_FIT_INTERIOR_FRAC` by area, and
    every one of its holes still needs a room behind it. A voided PANE
    counts too — that is the `dw_terrace` storefront case (D4): nothing is
    removed there and the building is see-through anyway.
    """
    stats = plan.get("stats") or {}
    if float(stats.get("removed_frac") or 0.0) >= _FIT_INTERIOR_FRAC:
        return True
    for r in (plan.get("regions") or ()):
        if r.get("recipe") in _FIT_INTERIOR_RECIPES:
            return True
    return bool(plan.get("removed") or plan.get("glass"))


def _opened_storeys_sides(ctx, plan):
    """`{storey: {side: (t0, t1)}}` — the along-side extent (mass-local
    metres, `fire_collapse.el_span`) of every REMOVED wall/pier/corner
    piece, unioned per (storey, side).

    KEPT, and still used — but ONLY to decide WHICH STOREYS get a
    `quake_flow.fit_interior` pass and which prop keep-out sides apply.
    The BACKING no longer reads it (that is `_opened_holes`, below): a
    union spanning a whole elevation is exactly what made round 3's quads
    read as black cladding.
    """
    from . import fire_collapse as fc

    info = ctx.get("info") or {}
    masses = info.get("masses") or {}
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in (info.get("elements") or ())
              if (e.get("p") or {}).get("prim_path")}
    out = {}
    for p in plan.get("removed") or ():
        e = by_path.get(p)
        if e is None:
            continue
        pp = e.get("p") or {}
        side = pp.get("_side")
        if side not in qs.SIDES:
            continue
        storey = int(pp.get("_storey", 0))
        m = masses.get(e.get("mass") or "main") or masses.get("main")
        if m is None:
            continue
        t0, t1 = fc.el_span(m, e, side=side)
        band = out.setdefault(storey, {}).setdefault(side, [t0, t1])
        band[0] = min(band[0], t0)
        band[1] = max(band[1], t1)
    return out


_BACKING_ROLES = ("wall", "pier", "corner")
#: Roles whose measured plan footprint DEFINES a storey's own outline —
#: the exterior envelope, never a parapet (which sits above the roof and is
#: routinely proud of or inset from the wall under it) and never a roof
#: tile.
_ENVELOPE_ROLES = ("wall", "pier", "corner")


# ---------------------------------------------------------------------------
# ROUND 4 v7 (lead review) — THE PER-STOREY PLAN, AND WHY A MASS BBOX IS NOT
# ONE
# ---------------------------------------------------------------------------
# User, on the v6 lit bench: *"There's still issues with the roof, the floor
# is extending outside the side wall [A4]. What are these random slabs
# you've made inside? in the past we've done floor rectangles + pillars. We
# have rules for this. Check the fire urban setting so that it looks self
# contained in the building."*
#
# THE FLOOR SLAB. `quake_flow.fit_interior` authors every slab as ONE box of
# `m["W"] - 2*WALL_INSET` by `m["D"] - 2*WALL_INSET` centred on the mass —
# i.e. the WHOLE BUILDING's bounding box, inset 0.55 m. That is the plan
# only for a cuboid. `SM_Building_24` (A4) is not one: its curtain wall
# steps in above the podium, so a slab sized off the outer bbox stands
# proud of the glass and the damaged corner shows floor plates hanging in
# the street. This is the SAME defect the fire path already fixed once, in
# the same words — `gac_fire._storey_footprints`'s own docstring quotes the
# user from the fire review: *"the catch ... looks like it's coming outside
# the side walls ... you can't treat it like a cuboid"* — and the fire path
# fixed it two ways at once: `fit_interior(footprint=...)` clamps the
# COLUMN grid (`FIT_FOOTPRINT_M`), and `urban_fire._plate` re-authors the
# PLATE itself from the measured polygon instead of the box.
#
# `fit_interior`'s `footprint=` kwarg does NOT clamp the slab — read it:
# the slab `_box` call is unconditional and takes `W`/`D` off the mass. So
# this region mirrors the fire path on BOTH halves: it passes `footprint=`
# for the columns, and it RE-CUTS the authored slab boxes to the measured
# storey rectangle itself (`_recut_slabs`), the way `_plate` does. Editing
# `quake_flow.fit_interior` would be the tidier fix and is NOT available —
# that is the earthquake stream's file, and the fire/kit paths through it
# are frozen.
#
# WHERE THE PLAN COMES FROM. The fire path measures it off the source
# asset's merged MESH (`gac_fire._storey_footprints`), which only exists on
# the `gac_fire.prepare` path. This ladder has no such mesh on either of
# its two sources, but it does have the ELEMENT TABLE — every envelope
# piece's own measured extent and position — which is the same evidence one
# step later. `_storey_plan_rects` takes each storey's envelope pieces'
# plan footprints and returns their axis-aligned bounding rectangle in the
# MASS-LOCAL frame; the slab is that rectangle inset by `WALL_INSET`, so it
# is inside all four wall planes by construction.
#
# WHY A RECTANGLE AND NOT A HULL. "In the past we've done floor rectangles
# + pillars" — the shipped look, and an axis-aligned rectangle inset off
# the storey's own bbox is guaranteed inside every wall plane of a convex
# plan and cannot grow past one on a concave plan either (it is the bbox of
# the walls, inset). A hull would fit a setback more tightly and is not
# what the user asked for.
#
# PARTITIONS ARE OFF. The "random slabs inside" are `fit_interior`'s own
# plaster partitions — 2-3 free-standing 0.12 m walls per storey at random
# plan positions, which through a torn facade read as slabs floating in the
# room. The proven fire/quake look is FLOOR RECTANGLES + PILLARS +
# CONTENTS, so this ladder now calls `fit_interior(partitions=False)`.
# Slabs, columns and contents are unchanged.
# ---------------------------------------------------------------------------

#: A/B KNOB, the same shape `tornado_kit.TK_GUARD` uses: `TU_FIT_CLAMP=0`
#: turns the whole per-storey clamp off — `fit_interior` gets no
#: `footprint=`, the slabs are left at their mass-bbox size, the props are
#: clamped only against opened sides and the backing is placed off the mass
#: wall line. That is the v6 behaviour exactly, which is what a before/after
#: overhang measurement needs (`tools/tornado_fit_probe.py`).
TU_FIT_CLAMP = _os.environ.get("TU_FIT_CLAMP", "1").strip().lower() not in (
    "0", "false", "no")
#: The slab sits this far inside the storey's own measured wall bbox —
#: `quake_flow.WALL_INSET`'s own value, reused so a clamped slab and an
#: unclamped one read identically where the plan IS the bbox.
_SLAB_INSET_M = 0.55
#: A fit-out storey whose floor level is within this of (or above) the deck
#: line is a PARAPET / sliver band, not a real floor — it gets no fit-out
#: (no extra floor of furniture, no floating roof props). See
#: `_author_interior`'s own "NO EXTRA FLOOR ABOVE THE ROOF" comment.
_PARAPET_BAND_TOL_M = 0.5
#: What `fit_interior(footprint=)` is handed: the storey's WALL rectangle,
#: barely inset — NOT the slab rectangle. `quake_flow._inside_inset` then
#: applies its own `FIT_FOOTPRINT_M` (0.35 m) on top. Handing it the slab
#: rect instead (0.55 m in) pushes the effective keep-out to 0.90 m and
#: deletes the entire PERIMETER column ring, which is most of the pillars
#: you can actually see through a hole — measured on A4: 216 columns -> 84.
#: The fire path passes the wall hull inset 0.35 m for the same reason
#: (`gac_fire._storey_footprints(inset_m=0.35)`).
_FOOTPRINT_INSET_M = 0.10
#: A floor plate is RECESSED this much further on a side that is actually
#: OPEN at that storey. The plate genuinely runs to the facade, so where
#: the wall survives nothing changes; through a hole, a plate whose edge is
#: flush with the (now missing) wall plane reads as a floor "extending
#: outside the side wall" — the v6 review's own words on A4 — because there
#: is nothing in front of it to say where the building stopped. Stopping it
#: short puts the edge unambiguously inside.
_SLAB_OPEN_EDGE_RECESS_M = 0.90
#: A storey needs at least this many measured envelope pieces before its
#: rectangle is trusted. Below it, no clamp — the same "no measurement, no
#: clamping" rule `quake_flow._inside_inset` applies to a degenerate
#: polygon.
_PLAN_MIN_PIECES = 3
#: ... and the rectangle is never allowed to shrink the slab below this
#: share of the mass bbox: a storey whose only surviving envelope pieces
#: are one short wall run would otherwise produce a sliver plate.
_PLAN_MIN_FRAC = 0.45


def _side_run_len(m, side):
    return float(m["W"]) if side in ("S", "N") else float(m["D"])


def _inset_for(m, rect=None):
    """`_INTERIOR_INSET_M`, clamped so two opposite backings on a narrow
    plan can never cross each other. Measured on the STOREY's own rectangle
    when one was measured, else on the mass bbox."""
    if rect is not None:
        small = min(rect["lx1"] - rect["lx0"], rect["ly1"] - rect["ly0"])
    else:
        small = min(float(m.get("W") or 0.0), float(m.get("D") or 0.0))
    if small <= 0.0:
        return _INTERIOR_INSET_M
    return min(_INTERIOR_INSET_M, _INTERIOR_INSET_MAX_FRAC * small)


def _el_plan_pts(m, e):
    """A piece's plan footprint as MASS-LOCAL (lx, ly) corner points.

    TWO SOURCES, because this ladder has two. A KIT piece is measured in
    `urban_building.PIECES`, and `fire_collapse.el_footprint` already turns
    that measurement plus the piece's own yaw into exact local corners — it
    is the same call `_opened_holes` uses for spans. A SLICED piece is not
    in that table (`el_footprint` would fall through to its own
    `module x 0.4 m` placeholder box, which is not a plan), but it carries
    its measured `_size` and its centre, which is what the slicer stamped;
    the local AABB of that is exact for the axis-aligned cut pieces
    `gac_storey_slice` produces.
    """
    from . import fire_collapse as fc

    name = e.get("name")
    if name:
        try:
            from detail import urban_building as ub
            if ub.PIECES.get(name):
                return fc.el_footprint(m, e)
        except Exception:
            pass
    p = e.get("p") or {}
    sx, sy, _sz = p.get("_size") or (1.0, 1.0, 3.0)
    lx, ly = qf._to_local(m, float(e.get("x", 0.0)), float(e.get("y", 0.0)))
    hx, hy = abs(float(sx)) / 2.0, abs(float(sy)) / 2.0
    return [(lx - hx, ly - hy), (lx + hx, ly - hy),
            (lx + hx, ly + hy), (lx - hx, ly + hy)]


def _storey_plan_rects(info):  # noqa: C901
    """`{(mass, storey): {"lx0","lx1","ly0","ly1"}}` — each storey's own
    measured plan, mass-local, as the axis-aligned bbox of its ENVELOPE
    pieces' footprints (removed ones included: this is the building's plan,
    not the damage).

    A storey with fewer than `_PLAN_MIN_PIECES` measured pieces gets no
    entry and is therefore never clamped. A rectangle that comes out under
    `_PLAN_MIN_FRAC` of the mass bbox on either axis is widened back to
    that floor — a sliver is a measurement failure, not a plan.
    """
    out = {}
    if not TU_FIT_CLAMP:
        return out
    masses = info.get("masses") or {}
    per = {}
    for e in info.get("elements") or ():
        p = e.get("p") or {}
        if p.get("_role") not in _ENVELOPE_ROLES:
            continue
        mtag = e.get("mass") or "main"
        m = masses.get(mtag) or masses.get("main")
        if m is None:
            continue
        per.setdefault((mtag, int(p.get("_storey", 0))), []).append((m, e))
    for key, items in per.items():
        if len(items) < _PLAN_MIN_PIECES:
            continue
        m = items[0][0]
        xs, ys = [], []
        # Extrema include awnings, pilasters, balconies baked into facade
        # meshes. Median exact-side centres recover the structural wall
        # planes and keep synthetic slabs/columns inside the real cage.
        side_centres = {q: [] for q in ("W", "E", "S", "N")}
        for mm, e in items:
            for lx, ly in _el_plan_pts(mm, e):
                xs.append(float(lx))
                ys.append(float(ly))
            p = e.get("p") or {}
            side = str(p.get("_side") or e.get("side") or "")
            if side in side_centres:
                lx, ly = qf._to_local(mm, float(e.get("x", 0.0)),
                                      float(e.get("y", 0.0)))
                side_centres[side].append(
                    float(lx if side in ("W", "E") else ly))
        if not xs:
            continue
        lx0, lx1, ly0, ly1 = min(xs), max(xs), min(ys), max(ys)
        import statistics
        if side_centres["W"] and side_centres["E"]:
            lx0 = statistics.median(side_centres["W"])
            lx1 = statistics.median(side_centres["E"])
        if side_centres["S"] and side_centres["N"]:
            ly0 = statistics.median(side_centres["S"])
            ly1 = statistics.median(side_centres["N"])
        W, D = float(m["W"]), float(m["D"])
        if (lx1 - lx0) < _PLAN_MIN_FRAC * W:
            c = (lx0 + lx1) / 2.0
            lx0, lx1 = c - _PLAN_MIN_FRAC * W / 2.0, c + _PLAN_MIN_FRAC * W / 2.0
        if (ly1 - ly0) < _PLAN_MIN_FRAC * D:
            c = (ly0 + ly1) / 2.0
            ly0, ly1 = c - _PLAN_MIN_FRAC * D / 2.0, c + _PLAN_MIN_FRAC * D / 2.0
        # never larger than the mass bbox itself
        lx0, lx1 = max(lx0, -W / 2.0), min(lx1, W / 2.0)
        ly0, ly1 = max(ly0, -D / 2.0), min(ly1, D / 2.0)
        out[key] = {"lx0": lx0, "lx1": lx1, "ly0": ly0, "ly1": ly1}
    return out


def _rect_for(rects, m, mass, storey):
    """The storey's own rectangle, walking DOWN to the nearest measured
    storey below when this one has none of its own (a pure parapet/roof
    band index carries no envelope piece — `dw_terrace`'s trim band is the
    measured example). `None` when nothing was measured at all."""
    for st in range(int(storey), -1, -1):
        got = rects.get((mass, st))
        if got:
            return got
    return None


def _inner_rect(rect, inset):
    """`rect` shrunk by `inset` on all four sides, never inverted."""
    cx = (rect["lx0"] + rect["lx1"]) / 2.0
    cy = (rect["ly0"] + rect["ly1"]) / 2.0
    hx = max(0.5, (rect["lx1"] - rect["lx0"]) / 2.0 - inset)
    hy = max(0.5, (rect["ly1"] - rect["ly0"]) / 2.0 - inset)
    return {"lx0": cx - hx, "lx1": cx + hx, "ly0": cy - hy, "ly1": cy + hy}


def _rect_poly_world(m, rect):
    """`rect`'s four corners in world XY — `quake_flow.fit_interior`'s
    `footprint=` wants a world-XY convex polygon per storey, exactly what
    `gac_fire._storey_footprints` hands the fire path."""
    return [qf._to_world(m, rect["lx0"], rect["ly0"]),
            qf._to_world(m, rect["lx1"], rect["ly0"]),
            qf._to_world(m, rect["lx1"], rect["ly1"]),
            qf._to_world(m, rect["lx0"], rect["ly1"])]


def _recut_box(stage, path, cx, cy, cz, sx, sy, sz):
    """Re-cut an already-authored `quake_flow._box` in place: new points,
    new extent, and the EXISTING translate op re-set (never a second
    `AddTranslateOp`, which is what calling `_box` again on the same path
    would do — two translate ops compose and the prim lands at twice the
    offset)."""
    from pxr import Gf, UsdGeom, Vt

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return False
    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        return False
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    P = Gf.Vec3f
    pts = [P(-hx, -hy, -hz), P(hx, -hy, -hz), P(hx, hy, -hz), P(-hx, hy, -hz),
           P(-hx, -hy, hz), P(hx, -hy, hz), P(hx, hy, hz), P(-hx, hy, hz)]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts))
    mesh.GetExtentAttr().Set([P(-hx, -hy, -hz), P(hx, hy, hz)])
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(float(cx), float(cy), float(cz)))
            break
    return True


def _slab_overhang(m, rect, sx, sy, lcx, lcy):
    """How far (metres) a slab of local half-extents `sx/2, sy/2` centred at
    local `(lcx, lcy)` sticks out past `rect` — the worst of the four
    sides, 0.0 when it is inside. The number the audit reports."""
    over = [rect["lx0"] - (lcx - sx / 2.0), (lcx + sx / 2.0) - rect["lx1"],
            rect["ly0"] - (lcy - sy / 2.0), (lcy + sy / 2.0) - rect["ly1"]]
    return max(0.0, max(over))


def _recut_slabs(stage, info, fit, rects, opened=None):
    """Re-cut every authored fit-out slab to its own storey's measured
    rectangle, inset `_SLAB_INSET_M`. Returns
    `{"n": recut, "before_max": m, "after_max": m, "per_side": {...}}` —
    measured, so the report quotes overhang rather than intent."""
    masses = info.get("masses") or {}
    n = 0
    before_max = after_max = 0.0
    detail = []
    for (mtag, storey), path in sorted((fit.get("slabs") or {}).items()):
        m = masses.get(mtag) or masses.get("main")
        if m is None:
            continue
        rect = _rect_for(rects, m, mtag, int(storey))
        if rect is None:
            continue
        cx, cy, cz, sx, sy, sz, _yaw = qf._box_dims(stage, path)
        lcx, lcy = qf._to_local(m, cx, cy)
        before = _slab_overhang(m, rect, sx, sy, lcx, lcy)
        inner = _inner_rect(rect, _SLAB_INSET_M)
        for sd in (opened or {}).get(int(storey)) or ():
            r = _SLAB_OPEN_EDGE_RECESS_M
            if sd == "S":
                inner["ly0"] = min(inner["ly0"] + r, inner["ly1"] - 0.5)
            elif sd == "N":
                inner["ly1"] = max(inner["ly1"] - r, inner["ly0"] + 0.5)
            elif sd == "W":
                inner["lx0"] = min(inner["lx0"] + r, inner["lx1"] - 0.5)
            elif sd == "E":
                inner["lx1"] = max(inner["lx1"] - r, inner["lx0"] + 0.5)
        nsx = inner["lx1"] - inner["lx0"]
        nsy = inner["ly1"] - inner["ly0"]
        ncx, ncy = qf._to_world(m, (inner["lx0"] + inner["lx1"]) / 2.0,
                                (inner["ly0"] + inner["ly1"]) / 2.0)
        if _recut_box(stage, path, ncx, ncy, cz, nsx, nsy, sz):
            n += 1
            after = _slab_overhang(
                m, rect, nsx, nsy, (inner["lx0"] + inner["lx1"]) / 2.0,
                (inner["ly0"] + inner["ly1"]) / 2.0)
            before_max = max(before_max, before)
            after_max = max(after_max, after)
            detail.append({"storey": int(storey), "before_m": round(before, 3),
                           "after_m": round(after, 3),
                           "size": [round(nsx, 2), round(nsy, 2)]})
    return {"n": n, "before_max": round(before_max, 3),
            "after_max": round(after_max, 3), "slabs": detail}


def _drop_outside_columns(stage, info, fit, rects):
    """Remove synthetic columns whose full section crosses a real wall plane.

    ``fit_interior`` lays out its grid from the coarse mass rectangle. Sliced
    GAC buildings may model only an L/stepped structural footprint, so a grid
    point can be inside that coarse box yet visibly outside the upper-storey
    facade (A3 ``col_main_8_6_2``). The measured per-storey rectangle is the
    authority; columns are optional fit-out and are suppressed, not moved.
    """
    masses = info.get("masses") or {}
    dropped = []
    sliced_a3 = str(info.get("style") or "").lower().endswith("sm_building_02")
    world_envelope = {}
    if sliced_a3:
        from pxr import Usd, UsdGeom
        bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_,
                                    UsdGeom.Tokens.render])
        for e in info.get("elements") or ():
            p = e.get("p") or {}
            if p.get("_role") not in _ENVELOPE_ROLES:
                continue
            st = int(p.get("_storey", 0))
            prim = stage.GetPrimAtPath(p.get("prim_path") or "")
            if not prim or not prim.IsValid():
                continue
            r = bcache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            mn, mx = r.GetMin(), r.GetMax()
            b = world_envelope.setdefault(st, [1e18, -1e18, 1e18, -1e18])
            b[0], b[1] = min(b[0], float(mn[0])), max(b[1], float(mx[0]))
            b[2], b[3] = min(b[2], float(mn[1])), max(b[3], float(mx[1]))
    for key, paths in list((fit.get("columns") or {}).items()):
        mtag, storey = key
        m = masses.get(mtag) or masses.get("main")
        rect = _rect_for(rects, m, mtag, int(storey)) if m else None
        if rect is None:
            continue
        inner = _inner_rect(rect, 0.20)
        keep = []
        for path in paths:
            cx, cy, _cz, sx, sy, _sz, _yaw = qf._box_dims(stage, path)
            lx, ly = qf._to_local(m, cx, cy)
            outside = (lx - sx / 2.0 < inner["lx0"] or
                       lx + sx / 2.0 > inner["lx1"] or
                       ly - sy / 2.0 < inner["ly0"] or
                       ly + sy / 2.0 > inner["ly1"])
            # A3's source is a partial/L-shaped shell, which no rectangle can
            # represent. Its named failure is just beyond the actual source
            # geometry even though it remains inside the coarse mass box.
            wb = world_envelope.get(int(storey))
            if wb:
                outside = outside or (cx - sx / 2.0 < wb[0] + 0.10 or
                                      cx + sx / 2.0 > wb[1] - 0.10 or
                                      cy - sy / 2.0 < wb[2] + 0.10 or
                                      cy + sy / 2.0 > wb[3] - 0.10)
            if outside:
                if qf._deactivate(stage, path):
                    dropped.append(path)
                continue
            keep.append(path)
        fit["columns"][key] = keep
    if dropped:
        dead = set(dropped)
        fit["all"] = [p for p in (fit.get("all") or ()) if p not in dead]
    return dropped


def _opened_holes(ctx, plan):
    """The list of ACTUAL HOLES a backing quad goes behind.

    One record per contiguous run of removed wall/pier/corner pieces on one
    (mass, side): `{"mass", "side", "storey", "t0", "t1", "z0", "z1"}`,
    with spans and z-extents measured off each piece itself
    (`fire_collapse.el_span` / `el_z_span`) and merged only where two
    pieces are within `_HOLE_MERGE_GAP_M` of each other along the wall.

    ROLES. `parapet`/`parapet_corner` are excluded outright — there is no
    interior behind a parapet, and including them is what ran round 3's
    black band up over B1's roof line. `roof` has no side at all. A CORNER
    piece contributes to BOTH of its legs (`quake_sliced._CORNER_SIDES`),
    so a corner loss is backed on the two elevations that meet there rather
    than falling through the S/E/N/W test the way round 3's version did.
    """
    from . import fire_collapse as fc

    info = ctx.get("info") or {}
    masses = info.get("masses") or {}
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in (info.get("elements") or ())
              if (e.get("p") or {}).get("prim_path")}
    raw = {}
    for p in plan.get("removed") or ():
        e = by_path.get(p)
        if e is None:
            continue
        pp = e.get("p") or {}
        if pp.get("_role") not in _BACKING_ROLES:
            continue
        side = pp.get("_side")
        if side in qs.SIDES:
            legs = (side,)
        elif side in qs._CORNER_SIDES:
            legs = qs._CORNER_SIDES[side]
        else:
            continue
        mass = e.get("mass") or "main"
        m = masses.get(mass) or masses.get("main")
        if m is None:
            continue
        storey = int(pp.get("_storey", 0))
        z0, z1 = fc.el_z_span(m, e)
        for leg in legs:
            t0, t1 = fc.el_span(m, e, side=leg)
            raw.setdefault((mass, leg, storey), []).append(
                (min(t0, t1), max(t0, t1), float(z0), float(z1)))

    holes = []
    for (mass, side, storey), spans in sorted(raw.items()):
        m = masses.get(mass) or masses.get("main")
        run = _side_run_len(m, side)
        cur = None
        for t0, t1, z0, z1 in sorted(spans):
            t0 = max(0.0, min(run, t0))
            t1 = max(0.0, min(run, t1))
            if t1 - t0 < 0.05:
                continue
            if cur is not None and t0 <= cur["t1"] + _HOLE_MERGE_GAP_M:
                cur["t1"] = max(cur["t1"], t1)
                cur["z0"] = min(cur["z0"], z0)
                cur["z1"] = max(cur["z1"], z1)
                continue
            if cur is not None:
                holes.append(cur)
            cur = {"mass": mass, "side": side, "storey": storey,
                  "t0": t0, "t1": t1, "z0": z0, "z1": z1}
        if cur is not None:
            holes.append(cur)
    return holes


def _glazed_ground_sides(ctx, plan):
    """The sides whose GROUND storey is glazed — the storefront ring (D4).

    Evidence, in order: a storey-0 piece that `annotate_glazing` measured
    (`_glass_faces > 0`) or that the plan voided (`plan["glass"]`), then
    the grid's own opening vocabulary (`quake_sliced.is_opening`, the
    sub-index `tornado_kit.adapt` stamps from a piece's window/door NAME).
    A ground storey with no glazing anywhere is opaque, nothing can be seen
    through it, and it gets no ring."""
    info = ctx.get("info") or {}
    els = list(info.get("elements") or ())
    if not els:
        return {}
    n_sub = qs.n_sub_of(els)
    voided = set(plan.get("glass") or ())
    out = {}
    for e in els:
        pp = e.get("p") or {}
        if int(pp.get("_storey", 0)) != 0:
            continue
        side = pp.get("_side")
        if side not in qs.SIDES:
            continue
        if pp.get("_role") in ("roof", "core"):
            continue
        glazed = (float(pp.get("_glass_faces") or 0) > 0
                  or pp.get("prim_path") in voided
                  or qs.is_opening(pp, n_sub))
        if glazed:
            out.setdefault(e.get("mass") or "main", set()).add(side)
    return out


def _clamp_fit_props(stage, ctx, info, fit, opened, rects):
    """Keep every fit-out prop inside the room it is in.

    TWO CLAMPS, both on the paths `quake_flow.fit_interior` just returned
    (that function draws its contents on the MASS bbox — `lx = uniform(-W/2
    + 1.5, W/2 - 1.5)` — which is the same cuboid assumption the slab makes,
    so on a setback plan a chair lands outside the glass whether or not the
    wall beside it is damaged):

      1. THE STOREY PLAN. Every prop is pushed inside its storey's own
         measured rectangle, inset `_PROP_PLAN_INSET_M` — the fix for A4's
         setback, and the reason this now runs on EVERY fitted storey
         rather than only the opened ones.
      2. THE OPENED SIDES. A wall that is actually gone gets the bigger
         `_PROP_EDGE_KEEPOUT_M` keep-out on top, so a chair never sits on
         the lip of a torn edge (B3's own "chair floating on a ledge").

    Done here rather than in `quake_flow.fit_interior`: that is the
    earthquake stream's file, its `footprint=` hook clamps the COLUMN grid
    only, and the fire/kit paths through it are frozen. Returns the number
    of props actually moved.
    """
    from pxr import Gf, UsdGeom

    masses = info.get("masses") or {}
    moved = 0
    for (mtag, storey), paths in sorted((fit.get("props") or {}).items()):
        m = masses.get(mtag) or masses.get("main")
        if m is None:
            continue
        sides = set((opened.get(int(storey)) or {}).keys())
        rect = _rect_for(rects, m, mtag, int(storey))
        if rect is None:
            rect = {"lx0": -float(m["W"]) / 2.0, "lx1": float(m["W"]) / 2.0,
                    "ly0": -float(m["D"]) / 2.0, "ly1": float(m["D"]) / 2.0}
        room = _inner_rect(rect, _PROP_PLAN_INSET_M)
        for path in paths or ():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            top = None
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    top = op
                    break
            if top is None:
                continue
            t = top.Get()
            lx, ly = qf._to_local(m, float(t[0]), float(t[1]))
            nlx = min(max(lx, room["lx0"]), room["lx1"])
            nly = min(max(ly, room["ly0"]), room["ly1"])
            k = _PROP_EDGE_KEEPOUT_M
            if "S" in sides:
                nly = max(nly, rect["ly0"] + k)
            if "N" in sides:
                nly = min(nly, rect["ly1"] - k)
            if "W" in sides:
                nlx = max(nlx, rect["lx0"] + k)
            if "E" in sides:
                nlx = min(nlx, rect["lx1"] - k)
            # A plan narrower than two keep-outs collapses to its own
            # centreline rather than inverting.
            if (rect["ly1"] - rect["ly0"]) <= 2.0 * k:
                nly = (rect["ly0"] + rect["ly1"]) / 2.0
            if (rect["lx1"] - rect["lx0"]) <= 2.0 * k:
                nlx = (rect["lx0"] + rect["lx1"]) / 2.0
            if abs(nlx - lx) < 1e-6 and abs(nly - ly) < 1e-6:
                continue
            wx, wy = qf._to_world(m, nlx, nly)
            top.Set(Gf.Vec3d(float(wx), float(wy), float(t[2])))
            moved += 1
    return moved


def _author_interior(stage, ctx, plan):
    """`quake_flow.fit_interior` for every opened storey — FLOOR RECTANGLES
    + PILLARS + CONTENTS, all cut to the storey's own measured plan — plus
    a textured interior backing behind every actual HOLE and (when the
    ground storey is glazed) a storefront ring at street level. Gated on
    `_wants_fit_interior`. Returns a counts dict.

    See this region's own "THE PER-STOREY PLAN" comment for why the slab is
    re-cut and the partitions are off.

    `ctx["fit"] = fit_interior(...)`'s own return is stored on the ctx —
    the SAME key `quake_flow._ragged_slabs`/`_a_slab_rim` read — but
    nothing in THIS ladder's own tear path (`_author_tears`, above) reads
    it: this module's tear driver deliberately skips `quake_sliced.
    _author_tears`'s floor-edge half. Stored anyway so a probe/bench can
    inspect what was authored.
    """
    if not _wants_fit_interior(plan):
        return {"n_fit": 0, "n_backing": 0}
    opened = _opened_storeys_sides(ctx, plan)
    holes = _opened_holes(ctx, plan)
    shop = _glazed_ground_sides(ctx, plan)
    if not (opened or holes or shop):
        return {"n_fit": 0, "n_backing": 0}

    info = ctx["info"]
    _ensure_fit_mats(stage, ctx)
    parent = ctx.get("parent") or "/World"
    tag = _safe_name("tornado_{0}".format(ctx.get("tag") or "b"))
    # `ctx["rng"]` is normally the ladder's own shared generator (`wreck_
    # urban`/`wreck_kit` always construct one) -- but `fire_collapse.
    # _own_rng` (the tear pass just above, when tears exist) SAVES AND
    # RESTORES whatever was on `ctx["rng"]`/`ctx["nrng"]` at entry, and on
    # a ctx that never carried one (a minimal stub, e.g. a bare-`apply_
    # plan` test fixture) that "restore" leaves an explicit `None` behind
    # rather than the absent key it found. Falls back to a fresh,
    # per-`tag` generator rather than propagating that `None` into `fit_
    # interior`'s own `rng.randrange(...)` call — the same "tolerant of a
    # partial ctx" rule `apply_plan`'s own docstring states.
    rng = ctx.get("rng")
    if rng is None:
        import random as _random

        from . import fracture
        rng = _random.Random(fracture.stable_seed(ctx.get("tag"), "fit_interior"))
    # STOREYS. Every opened storey, plus — when a storefront ring is going
    # in — storey 0 AND storey 1: `fit_interior`'s slab `i` is the FLOOR of
    # storey `i` (its top at `levels[i]`), so storey 1's slab is what caps
    # the shop and stops the eye carrying on up into an empty shell.
    storeys = set(opened)
    storeys.update(int(h["storey"]) for h in holes)
    if shop:
        storeys.update((0, 1))

    masses = info["masses"]
    rects = _storey_plan_rects(info)
    m_main = masses.get("main") or next(iter(masses.values()))
    # ROUND 5 (user review 2026-09-02, items 3/6): NO EXTRA FLOOR ABOVE THE
    # ROOF, AND THE TOP PILLARS REACH THE TOP FLOOR. A recipe that takes a
    # PARAPET-band piece (`parapet_fall`, `top_storey_loss`) puts that band's
    # storey index into `opened`, and `fit_interior` would then author a
    # whole floor of slab + columns + FURNITURE at the parapet level — the
    # "extra floor of furniture and roof" with "roof props ... floating" the
    # review flagged (`prop_main_11_*` on A3, 17 m off the footprint because
    # the parapet band carries no measured plan to clamp to). It is also why
    # the top pillars read "3 m short": the real top floor's columns
    # correctly stop at the deck, but a phantom parapet floor ~3 m above them
    # made them look short of it. The proven fire/quake interior is FLOORS +
    # PILLARS; a parapet has neither, so clamp the fit-out to real floors
    # (a floor whose level is strictly below the deck line). Single-mass by
    # `tornado_kit._refuse_if_unsupported`, so the main mass decides.
    # TWO signals for "this fit-out storey is a PARAPET band, not a real
    # floor", unioned so a build that carries only one of them is still
    # clamped:
    #   (a) DECK-Z, when the sliced mass carries one: any floor level at or
    #       above the deck line is above the roof.
    #   (b) ENVELOPE STOREY — deck_z-INDEPENDENT, and the one that actually
    #       fires on the bench build (which does not set deck_z, so (a) fell
    #       back to the mass TOP and kept storey 11): the parapet band has NO
    #       wall/pier/corner piece (only a `parapet` role), so any fit-out
    #       storey ABOVE the highest storey that carries an envelope piece is
    #       a parapet band. This is what leaves `prop_main_11_*` floating 12 m
    #       off A3 when only (a) is used.
    deck_z = float(m_main.get("deck_z", m_main.get("top", 1e18)))
    _lv = list(m_main.get("levels") or [])
    _has_deck = m_main.get("deck_z") is not None
    _env_storeys = [int((e.get("p") or {}).get("_storey", 0))
                    for e in (info.get("elements") or ())
                    if (e.get("mass") or "main") == "main"
                    and (e.get("p") or {}).get("_role") in _ENVELOPE_ROLES]
    _top_env = max(_env_storeys) if _env_storeys else None
    real_floors = set()
    for i, z in enumerate(_lv):
        if _has_deck and float(z) >= deck_z - _PARAPET_BAND_TOL_M:
            continue
        if _top_env is not None and i > _top_env:
            continue
        real_floors.add(i)
    real_floors.add(0)  # the ground is always a real floor
    if _lv:
        dropped = sorted(s for s in storeys if s not in real_floors)
        storeys = {s for s in storeys if s in real_floors}
        if shop:
            storeys.update({0, 1} & real_floors)
        if dropped:
            print("[tornado_urban_usd] interior: dropped {0} parapet-band "
                  "storey(s) {1} from the fit-out (top envelope storey {2}, "
                  "deck_z {3}) — items 3/6".format(
                      len(dropped), dropped, _top_env,
                      "%.1f" % deck_z if _has_deck else "unset"))
    # `footprint=` is keyed by STOREY only (`quake_flow.fit_interior`'s own
    # signature, and the shape `urban_fire.burn_building` passes it), so a
    # multi-mass building gets the MAIN mass's plan — the same limitation
    # the fire path carries. Every style this ladder accepts is single-mass
    # (`tornado_kit._refuse_if_unsupported`).
    polys = {}
    for st in sorted(storeys):
        rect = _rect_for(rects, m_main, "main", st)
        if rect is not None:
            polys[st] = _rect_poly_world(m_main, _inner_rect(
                rect, _FOOTPRINT_INSET_M))
    fit = qf.fit_interior(stage, parent, info, ctx["mats"], rng,
                          storeys=sorted(storeys), tag=tag,
                          partitions=False,
                          footprint=(polys or None),
                          columns_for_urm=True)
    ctx["fit"] = fit
    ctx.setdefault("static_extra", [])
    ctx["static_extra"].extend(fit.get("all") or [])
    slab_cut = _recut_slabs(stage, info, fit, rects, opened)
    outside_columns = _drop_outside_columns(stage, info, fit, rects)
    if outside_columns:
        dead_cols = set(outside_columns)
        ctx["static_extra"] = [p for p in ctx["static_extra"]
                               if p not in dead_cols]
        print("[tornado_urban_usd] interior: suppressed {0} column(s) "
              "crossing measured storey wall planes".format(
                  len(outside_columns)))
    n_props_moved = _clamp_fit_props(stage, ctx, info, fit, opened, rects)

    # A torn facade exposes the shared fire/quake floors and columns.  The
    # former per-hole backing quads recreated walls behind the holes and read
    # as freestanding panels, so they are deliberately not authored.
    out = {"n_fit": len(fit.get("all") or []), "n_backing": 0,
           "n_backing_holes": 0, "n_backing_shop": 0,
           "n_props_clamped": n_props_moved,
           "n_columns_outside_dropped": len(outside_columns),
           "n_partitions": len(fit.get("partitions") or []),
           "n_slabs_recut": slab_cut["n"],
           "slab_overhang_before_m": slab_cut["before_max"],
           "slab_overhang_after_m": slab_cut["after_max"],
           "n_holes": len(holes), "storeys": sorted(storeys),
           "inset_m": float(_inset_for(
               m_main, _rect_for(rects, m_main, "main", 0)))}
    ctx["interior"] = out
    return out

    mat = _interior_backing_material(stage, ctx)
    root = "{0}/tornado_interior_backing".format(parent)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n_backing = 0
    seen = set()

    def _quad(name, m, rect, side, t_mid, width, cz, height):
        """One backing panel, positioned off the STOREY's own wall line —
        not the mass bbox. On a setback plan (A4) the mass bbox line is
        OUTSIDE the glass, so a quad placed against it and pushed in by
        1.35 m can still end up proud of the facade: the exact "backing
        segments protrude past the facade line" the lead flagged."""
        inset = _inset_for(m, rect)
        lx, ly = qf._p_wall_point(m, side, t_mid)
        if side == "S":
            ly = rect["ly0"] + inset
        elif side == "N":
            ly = rect["ly1"] - inset
        elif side == "W":
            lx = rect["lx0"] + inset
        else:
            lx = rect["lx1"] - inset
        wx, wy = qf._to_world(m, lx, ly)
        yaw = m["yaw"] + (0.0 if side in ("S", "N") else 90.0)
        path = "{0}/{1}".format(root, name)
        qf._box(stage, path, wx, wy, cz, float(width), _BACKING_T_M,
                float(height), yaw, mat)
        ctx["static_extra"].append(path)
        return path

    def _rect_along(m, rect, side):
        """(lo, hi) of `rect` along `side`'s own wall axis, in the same
        `t` metres `el_span`/`_p_wall_point` use."""
        if side in ("S", "N"):
            return rect["lx0"] + m["W"] / 2.0, rect["lx1"] + m["W"] / 2.0
        return rect["ly0"] + m["D"] / 2.0, rect["ly1"] + m["D"] / 2.0

    # -- 1) per-BAY quads behind each HOLE ---------------------------------
    #
    # SEGMENTED, CONTIGUOUS AND COPLANAR. v6 staggered alternate segments
    # 0.55 m deeper to break up a long plane; on the lit bench that read as
    # a row of free-standing slabs rather than as a back wall (lead review,
    # "or read as freestanding slabs"). The segmentation stays — it is what
    # keeps a hole's backing a per-bay panel run rather than one 32 m slab
    # — but every segment of one hole now sits at the SAME inset, so the
    # run reads as one surface.
    for h in holes:
        if n_backing >= _MAX_BACKING_QUADS:
            break
        m = masses.get(h["mass"]) or masses["main"]
        rect = _rect_for(rects, m, h["mass"], int(h["storey"]))
        if rect is None:
            rect = {"lx0": -float(m["W"]) / 2.0, "lx1": float(m["W"]) / 2.0,
                    "ly0": -float(m["D"]) / 2.0, "ly1": float(m["D"]) / 2.0}
        a_lo, a_hi = _rect_along(m, rect, h["side"])
        t0h = max(h["t0"], a_lo)
        t1h = min(h["t1"], a_hi)
        if t1h - t0h < 0.3:
            continue
        height = max(0.6, h["z1"] - h["z0"])
        cz = h["z0"] + height / 2.0
        span = t1h - t0h
        seg = min(_BACKING_SEG_MAX_M,
                  max(_BACKING_SEG_MIN_M, float(m.get("module") or 4.0)))
        n_seg = max(1, int(math.ceil(span / seg)))
        w_seg = span / float(n_seg)
        for k in range(n_seg):
            if n_backing >= _MAX_BACKING_QUADS:
                break
            t0 = t0h + k * w_seg
            name = "backing_{0}_{1}_{2:02d}_{3:04d}".format(
                h["side"], _safe_name(h["mass"]), int(h["storey"]),
                int(round(t0 * 10.0)))
            if name in seen:
                continue
            seen.add(name)
            _quad(name, m, rect, h["side"], t0 + w_seg / 2.0, w_seg, cz,
                  height)
            n_backing += 1

    # -- 2) the storefront ring (D4) ---------------------------------------
    n_shop = 0
    for mass, sides in sorted(shop.items()):
        m = masses.get(mass) or masses["main"]
        rect = _rect_for(rects, m, mass, 0)
        if rect is None:
            rect = {"lx0": -float(m["W"]) / 2.0, "lx1": float(m["W"]) / 2.0,
                    "ly0": -float(m["D"]) / 2.0, "ly1": float(m["D"]) / 2.0}
        lv = list(m["levels"])
        z0 = float(lv[0]) if lv else float(m.get("z0", 0.0))
        z1 = float(lv[1]) if len(lv) > 1 else float(m["top"])
        height = max(1.2, min(_SHOP_MAX_H_M, (z1 - z0) * 0.96))
        inset = _inset_for(m, rect)
        for side in sorted(sides):
            if n_backing >= _MAX_BACKING_QUADS:
                break
            a_lo, a_hi = _rect_along(m, rect, side)
            # The four quads meet at the corners of the inset rectangle:
            # each is its own run shortened by one inset at each end.
            width = max(0.6, (a_hi - a_lo) - 2.0 * inset)
            name = "backing_{0}_{1}_shop".format(side, _safe_name(mass))
            if name in seen:
                continue
            seen.add(name)
            _quad(name, m, rect, side, (a_lo + a_hi) / 2.0, width,
                  z0 + height / 2.0, height)
            n_backing += 1
            n_shop += 1

    if n_backing:
        print("[tornado_urban_usd] interior: {0} fit-out prim(s) "
              "({1} slab(s) re-cut to the measured storey plan, overhang "
              "{2:.2f} m -> {3:.2f} m; partitions off), {4} backing quad(s) "
              "({5} behind holes, {6} storefront) over {7} fitted "
              "storey(s); {8} prop(s) clamped inboard".format(
                  len(fit.get("all") or []), slab_cut["n"],
                  slab_cut["before_max"], slab_cut["after_max"], n_backing,
                  n_backing - n_shop, n_shop, len(storeys), n_props_moved))
    out = {"n_fit": len(fit.get("all") or []), "n_backing": n_backing,
           "n_backing_holes": n_backing - n_shop, "n_backing_shop": n_shop,
           "n_props_clamped": n_props_moved,
           "n_partitions": len(fit.get("partitions") or []),
           "n_slabs_recut": slab_cut["n"],
           "slab_overhang_before_m": slab_cut["before_max"],
           "slab_overhang_after_m": slab_cut["after_max"],
           "n_holes": len(holes), "storeys": sorted(storeys),
           "inset_m": float(_inset_for(m_main,
                                       _rect_for(rects, m_main, "main", 0))),
           "albedo": [float(q) for q in _INTERIOR_BACKING_ALBEDO]}
    # `apply_plan`'s own counts dict (another region of this file) only
    # forwards `n_fit`/`n_backing`; the full record goes on the ctx so a
    # probe, a test or the bench manifest can read what was actually
    # authored without re-walking the stage.
    ctx["interior"] = out
    return out



def apply_plan(stage, ctx, plan, verbose=True):
    """Do to the stage what `tornado_urban.plan_damage` decided (§2.8's
    schema). Tolerant of a partial plan — any of `"displaced"`,
    `"roof_props"`, `"debris"`, `"glass"`, `"removed"` may be absent (the
    planner may land in stages) and are treated as empty/`"keep"`; a listed
    prim path that does not resolve on the stage is COUNTED, never raised.
    Returns a small counts dict.
    """
    plan = plan or {}
    mats = ctx.setdefault("mats", {})
    ctx.setdefault("static_extra", [])
    ctx.setdefault("notes", [])

    # 1) GLASS FIRST, same reason `quake_sliced.apply_plan` orders it first:
    #    it binds materials on pieces that may be about to move, and a moved
    #    piece carries its bindings with it. Calls THIS module's OWN
    #    `_void_glass` (not `quake_sliced._void_glass` — see the module
    #    docstring's "THE `_void_glass` FIX"): the earthquake stream's
    #    version measured 0 subsets voided against a real GAC slice.
    glass_paths = list(plan.get("glass") or ())
    n_glass = 0
    n_glass_removed = 0
    if glass_paths:
        void_mat = _ensure_void_material(stage, ctx)
        rebind, knock_out = [], []
        for path in glass_paths:
            if _has_glazing_binding(stage, path):
                rebind.append(path)
            elif _window_named(path):
                knock_out.append(path)
            else:
                rebind.append(path)   # `_void_glass` no-ops on it, counted 0
        n_glass = _void_glass(stage, rebind, void_mat)
        # ROUND 2, THE KIT VOCABULARY (measured, `tools/_tk_glass_probe.py`):
        # a KIT window is its own MODULE with NO glazing material anywhere —
        # Downtown_West window modules carry subsets bound to untextured /
        # shared-atlas materials that legitimately fail `is_glazing`, and the
        # MCE families paint their windows into one facade atlas (those are
        # never listed as glass at all). So "pane out" on a kit building is
        # the module KNOCKED OUT to a dark opening — the same read the fire
        # ladder's "windows out to a black void" uses — never a rebind.
        for path in knock_out:
            if qf._deactivate(stage, path):
                n_glass_removed += 1
        if knock_out:
            ctx["notes"].append(
                "glass: {0} window MODULE(s) knocked out (kit vocabulary — "
                "no glazing material to rebind)".format(n_glass_removed))

    # 2) REMOVAL. `stage.GetPrimAtPath` on a path the plan named but the
    #    stage does not have is a MISSING count, not an exception — a
    #    refused/blacklisted building or a stale plan must never crash the
    #    apply step.
    n_removed = 0
    missing = []
    for path in sorted(plan.get("removed") or ()):
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            missing.append(path)
            continue
        if qf._deactivate(stage, path):
            n_removed += 1
    if missing:
        msg = ("[tornado_urban_usd] {0} removed-piece path(s) not found on "
               "the stage (never raised): {1}").format(
                   len(missing), ", ".join(missing[:8]) +
                   (", ..." if len(missing) > 8 else ""))
        ctx["notes"].append(msg)
        if verbose:
            print(msg)

    # 2.5) ROUND 3b F2a (§8e) — THE PERIMETER OF EVERY HOLE. After removal
    #      (the pieces a tear is "against" must already be gone) and BEFORE
    #      rigid displacement, the same ordering `quake_sliced.apply_plan`
    #      uses for its own tear step and for the same reason: a piece
    #      about to be rotated into the street (a hanging panel, a
    #      macroblock) must still be WHOLE when this runs. Safe on a
    #      fractured/sliced piece per the earthquake round's own VTK guard
    #      (`.agents/skills/build-earthquake-scenes/SKILL.md`, "GAC reseat"/
    #      "Sliced tears landed" sections) — `fire_collapse._tear_perimeter`
    #      is the SAME machinery that round measured against real slices;
    #      see `_author_tears`'s own docstring for the one piece of it
    #      (`_author_floor_edges`) this ladder deliberately does not call.
    n_tears = _author_tears(stage, ctx, plan)

    # 3) RIGID DISPLACEMENT — `quake_sliced.rigid_matrix(spec)` fed straight
    #    into a `Gf.Matrix4d` (`quake_sliced._gf`) and post-multiplied onto
    #    each prim's world transform by `quake_flow._transform_prims`, one
    #    matrix built per DISTINCT spec (grouped, the way
    #    `quake_sliced.apply_plan` groups its own `plan["displaced"]` loop).
    n_displaced = 0
    n_rect_hangers_suppressed = 0
    keep_rect_hangers = _os.environ.get(
        "TU_KEEP_RECT_HANGERS", "1").strip().lower() in ("1", "true", "yes")
    displaced = plan.get("displaced") or {}
    groups = {}
    for path in sorted(displaced):
        # A whole region-cut cell retains the slicer's rectangular outline
        # after a rigid tilt. It is semantic geometry, not acceptable visible
        # tornado fracture geometry. The ragged border fragments and source
        # rubble already carry this failure; suppress the rectangular hanger.
        if not keep_rect_hangers and "/pieces/" in str(path):
            if qf._deactivate(stage, path):
                n_rect_hangers_suppressed += 1
            continue
        key = _json.dumps(displaced[path], sort_keys=True)
        groups.setdefault(key, []).append(path)
    for key in sorted(groups):
        spec = _json.loads(key)
        M = qs._gf(spec)
        paths = groups[key]
        n_displaced += qf._transform_prims(stage, paths, M)
        ctx["static_extra"].extend(paths)

    # 3.5) ROUND 3b F2b (§8e) — VISIBLE INTERIORS. After displacement (a
    #      hanging/leaning piece is still in its ORIGINAL position for the
    #      opened-span measurement either way, since `_opened_storeys_
    #      sides` reads REMOVED pieces only, but this keeps every "what
    #      does the settled façade look like" step grouped before the
    #      purely-additive furniture/debris steps that follow).
    fit_counts = _author_interior(stage, ctx, plan)

    # 4) ROOF PROPS. See the module docstring: this reuses
    #    `_sweep_roof_props_sliced`'s IDENTIFICATION (`ctx["roof_plant"]` /
    #    `ctx["roof_fixed"]`), not its carry/reseat mechanics — a tornado
    #    sweeps roof furniture off, it does not carry it down a crushed
    #    storey (there is no crush here).
    n_roof = 0
    if plan.get("roof_props") == "sweep":
        plant = list(dict.fromkeys(
            list(ctx.get("roof_plant") or ()) + list(ctx.get("roof_fixed") or ())))
        for path in plant:
            if qf._deactivate(stage, path):
                n_roof += 1
                ctx["static_extra"].append(path)
        if not plant:
            ctx["notes"].append(
                "roof_props: sweep requested but ctx carries no "
                "roof_plant/roof_fixed paths — this round does not author "
                "roof furniture itself (see module docstring); a no-op "
                "here is expected until a later round wires that in")

    # 5) DEBRIS. Authored, never simulated — every removed piece's material
    #    is already on the ground.
    fragments = list(plan.get("debris") or ())
    n_fragments_total = len(fragments)
    n_source_rubble = 0
    made = []
    if fragments:
        ground_z = ctx.get("ground_z", 0.0)
        consumed = _seat_source_rubble(stage, ctx, fragments,
                                       ground_z=ground_z)
        if consumed:
            n_source_rubble = len(consumed)
            fragments = [f for i, f in enumerate(fragments)
                         if i not in consumed]
        made = build_debris(stage, ctx.get("parent") or "/World", fragments,
                            ctx, ground_z=ground_z)
        ctx["static_extra"].extend(made)

    for line in plan.get("notes") or ():
        if line not in ctx["notes"]:
            ctx["notes"].append(line)

    out = {"n_removed": n_removed, "n_missing": len(missing),
           "n_glass": n_glass, "n_glass_removed": n_glass_removed,
           "n_displaced": n_displaced,
           "n_rect_hangers_suppressed": n_rect_hangers_suppressed,
           "n_debris_meshes": len(made), "n_fragments": n_fragments_total,
           "n_source_rubble": n_source_rubble,
           "n_roof_props": n_roof, "n_tears": n_tears,
           "n_fit": fit_counts["n_fit"], "n_backing": fit_counts["n_backing"],
           "notes": list(ctx["notes"])}
    if verbose:
        print("[tornado_urban_usd] {0}: {1} removed ({2} missing), {3} "
              "displaced, {4} glass voided (+{5} window module(s) knocked "
              "out), {6} roof prop(s) swept, {7} debris mesh(es) from {8} "
              "fragment(s), {9} tear(s), {10} fit-out prim(s), {11} "
              "backing quad(s)".format(
                  plan.get("level") or "plan", n_removed, len(missing),
                  n_displaced, n_glass, n_glass_removed, n_roof, len(made),
                  len(fragments), n_tears, fit_counts["n_fit"],
                  fit_counts["n_backing"]))
    return out


def wreck_urban(stage, cell, placements, style, level, rng, nrng, mats, tag,
                wind, btype=None, height_class=None, intensity=None,
                usd=None, verbose=True):
    """Apply the urban-tornado ladder to one sliced building and return
    `quake_flow.wreck_building`'s ctx, unchanged in shape — the same
    contract `quake_sliced.wreck_sliced` honours, so a later bake launcher
    can call this wherever `quake_gac_bake_launch_script.py` calls that.

    `x`/`y`/`yaw` are the CELL's and are always zero here, for the same
    reason `wreck_sliced` defaults them to zero: the slicer writes every
    piece in the cell's own local frame (`gac_storey_slice.slice_to_kit`),
    so `describe` is always called at the cell origin.

    Calls `annotate_glazing(stage, placements)` BEFORE `qf.describe(...)`,
    so the stamped `_glass_faces`/`_glass_frac` land on each placement dict
    while it is still the raw dict `describe`/`classify` carry forward
    unchanged as `e["p"]` — the element table's per-piece measurement of
    glazing, not a role guess (see that function's own docstring for why a
    role guess is wrong on assets like SM_Building_02, where the glazing
    sits on `pier`/`core` pieces, never `wall`).

    Until stream L's `disaster/tornado_urban.py` lands, this guards the
    import and returns a ctx with `plan=None` rather than raising — call
    `apply_plan(stage, ctx, plan, ...)` directly against a plan built
    elsewhere (a fixture, or a real plan once L's module exists) in the
    meantime, which is exactly how this module's own tests exercise it.

    This function does NOT call `quake_flow.dress_roof` — see
    `apply_plan`'s ROOF PROPS step for why that is a documented gap, not an
    oversight: earthquake's `dress_roof` needs earthquake-only material
    keys (`mats["tank_wood"]`, `mats["plant_metal"]`) this ladder's `mats`
    is not guaranteed to carry, and roof-furniture PLACEMENT for the urban-
    tornado ladder is not yet decided (a later round's job).
    """
    annotate_glazing(stage, placements)
    annotate_surface(stage, placements)  # FX2 HOOK (§8e F3) — one line, mirrors annotate_glazing above
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    if btype is None:
        btype = qs.construction_type(usd or style, H=info.get("H"))
    info["type"] = btype

    ctx = {"stage": stage, "parent": cell, "info": info, "rng": rng,
           "nrng": nrng, "mats": mats if mats is not None else {},
           "tag": tag, "loose": [], "static_extra": [], "velocity": {},
           "authored": [], "notes": [], "verbose": verbose}
    ctx["sliced"] = {"style": style, "usd": usd, "btype": btype,
                     "level": level, "intensity": intensity,
                     "height_class": height_class, "n_pieces": len(placements)}

    try:
        from . import tornado_urban
    except ImportError as exc:
        note = ("tornado_urban planner unavailable ({0}) — no plan run; "
                "call apply_plan(stage, ctx, plan) directly with a plan "
                "from elsewhere until stream L lands its module").format(exc)
        ctx["notes"].append(note)
        ctx["plan"] = None
        if verbose:
            print("[tornado_urban_usd] " + note)
        return ctx

    plan = tornado_urban.plan_damage(info, info["elements"], level, btype,
                                     rng, wind, height_class=height_class,
                                     intensity=intensity)
    ctx["plan"] = plan
    ctx["counts"] = apply_plan(stage, ctx, plan, verbose=verbose)
    if verbose:
        print("[tornado_urban_usd] {0} ({1}, {2}, {3:.0f} m, {4} "
              "piece(s)): {5}".format(
                  style, btype, level, info.get("H") or 0.0, len(placements),
                  ctx["counts"]))

    # R7 HOOK (stream RF, `disaster/tornado_roof.py`): the roof-damage pass,
    # run on its OWN rng stream (`tornado_roof.roof_seed(tag)`) so the
    # façade plan above is byte-identical whether or not this ever runs —
    # see that module's own docstring ("THE SEPARATE RNG STREAM").
    import random as _random

    from . import tornado_roof
    roof_rng = _random.Random(tornado_roof.roof_seed(tag))
    roof_plan = tornado_roof.plan_roof(info, info["elements"], level, wind,
                                       roof_rng, height_class, intensity,
                                       facade_plan=plan)
    ctx["roof_plan"] = roof_plan
    ctx["roof_counts"] = tornado_roof.apply_roof(stage, ctx, roof_plan,
                                                 verbose=verbose)
    return ctx
