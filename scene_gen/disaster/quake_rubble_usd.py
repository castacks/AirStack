"""quake_rubble_usd — the ONLY module that imports `pxr` for rubble v2.

`disaster/quake_rubble.py` (the planner, pure numpy/python) turns a collapsed
mass into a plan dict: a heightfield mound mesh, an optional toe apron, a
handful of "large" elements (raft/stub/panel — either a referenced debris
asset, an authored box, or an EXISTING kit prim to re-lay), and one or more
named instance sets (chunks, flakes, clusters) meant for `UsdGeom.PointInstancer`.
This module is the emitter: it takes that plan and writes it onto a stage.

Nothing here decides WHERE debris goes or HOW MUCH — that is the planner's
job, entirely in `quake_rubble.plan_pile`. This module only turns numbers
into prims: mesh authoring, material caching, instancer wiring, and the
xform math for the three "large"-element cases.

Why a `UsdGeom.PointInstancer`, when `disaster/bake.py` (line ~225) explains
why the ROUND-3 heap was never instanced: round 3's lumps were `_a_lump`
boxes with independently jittered corners, so no two lumps were the same
prototype and there was nothing TO instance. Round 4's debris (this module's
"instances" sets) are references to a small SHARED catalogue of Nucleus/
standalone assets (`chunk_01..09`, `lump_01..06`, ...) — the exact case a
PointInstancer is for.

Mesh conventions (matching the plan contract):
    * the mound/apron heightfield arrives as WORLD-space points + triangle
      faces, authored as-is (no further transform);
    * a "large" element's `pos` is the asset's OWN ORIGIN in world space —
      footprint centre, base at z = 0 — so referencing it needs no bbox
      seating (contrast `quake_flow._prop`, which reads a bound because its
      props are not built to that convention).
"""
import os

import numpy as np

# ---------------------------------------------------------------------------
# asset root
# ---------------------------------------------------------------------------
ASSET_ROOT = os.environ.get(
    "RUBBLE_ASSET_ROOT",
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/",
)

# The mound/apron world-projected textures live in this repo (small, hand
# selected, checked in like `assets/materials/megascans/*`) rather than in
# the debris catalogue mirror above — they are MATERIAL SOURCE, not debris.
_MATDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "assets", "materials", "quake")

# name -> (texture stem under assets/materials/quake/, mound tint, apron tint)
# Tints are LINEAR albedo multipliers on the texture (see `damage._pbr`); the
# apron gets a paler, dustier tint than the mound crown — the design table's
# "toe apron" is thin fines blown out from the pile, not the pile itself.
#
# round-5 v6 candidates (`~/scorch_previews/rubble_r4/v6/README.md`):
# the old `_B` tiles (`make_tileable.py --composite` of a toned `Dirt_Rough`
# megascans scan + a high-passed debris-atlas detail layer) read as a smooth
# brown DIRT/MUD blanket under RTX (`0_commercial_DG5_ne.png`/`_street.png`)
# — `Dirt_Rough` is a literal photographed soil scan, and no tint/desat on
# top of it changes what the eye reads as the MATERIAL. The `_C{1,2,3}`
# candidates all drop the soil-scan base entirely and build from a real
# broken-masonry/concrete photo instead. Recommendation: RC -> C1 (the
# mound's OWN `concrete_debris_elements` atlas, self-sharpened — best value-
# match to the old look while fully fixing the frequency/"mud" problem);
# URM -> C2 (`Brick_Wall_Worn`, scrambled seamless — the richest, most
# saturated crushed-brick red of the three candidates). The `_B` files
# themselves are left on disk untouched (the v6 README's own constraint) —
# only which stem/tint/desat/uv_scale this dict points at changes.
_LOOK = {
    "urm": {"stem": "rubble_urm_C2", "mound": (0.64, 0.50, 0.43), "apron": (0.76, 0.66, 0.58), "desat": 0.10},
    "rc":  {"stem": "rubble_rc_C1",  "mound": (0.58, 0.57, 0.55), "apron": (0.72, 0.71, 0.68), "desat": 0.15},
}
_DEFAULT_LOOK = "rc"

# ~0.28-0.34 repeats/metre (damage._pbr's `scale_uv` IS repeats-per-metre,
# smaller = bigger features) — the brief's "0.25-0.35 per metre" band, per-
# candidate values from the v6 README's recommended C1 (rc) / C2 (urm) rows.
# Also used to author `primvars:st` on the mound/apron mesh itself (world x,
# y times this same scale) so a Blender/Hydra preview that cannot see the
# MDL `project_uvw` triplanar projection still gets a matching, correctly-
# scaled UV to read the real texture through (see `_add_preview_fallback`).
_UV_SCALE = {"urm": 0.34, "rc": 0.32}


def _uv_scale_for(look):
    return _UV_SCALE.get(look, _UV_SCALE[_DEFAULT_LOOK])

# Flat fallback materials for an authored (asset=None) "large" box, keyed as
# the contract specifies. LINEAR albedos as given in the round-4 brief.
_BOX_MATS = {
    "concrete_dusty": ((0.078, 0.077, 0.072), 0.92),
    "timber_dusty":   ((0.062, 0.046, 0.032), 0.88),
    "brick_dusty":    ((0.100, 0.057, 0.044), 0.90),
}

# ---------------------------------------------------------------------------
# round-4 per-set / per-large "look" overrides
# ---------------------------------------------------------------------------
# `plan["instances"][set]["look"]`, `entry["look"]` (a "large" dict) and
# `plan["apron"]["look"]` each now carry one of these six material
# identities — a DIFFERENT axis from `_LOOK` above (the mound/apron's own
# "urm"/"rc" CONSTRUCTION-type texture) and from `_debris_material_for`'s
# per-catalogue-asset rule (concrete vs. rust by the asset's own `material`/
# `textured` fields). The value sets are disjoint ("urm"/"rc" vs. these six),
# so a plan that still puts "urm"/"rc" on `apron["look"]` (pre-round-4 plans,
# this module's own older tests) falls straight through `_material_for_look`
# (returns `None`) into the OLD apron branch in `author()` unchanged.
_MATERIAL_LOOKS = {"brick", "concrete", "rust", "stone", "timber", "dust"}

# brick: TEXTURED — the same `Brick_Wall_Worn` basecolour `quake_flow._C_TEX
# ["brick"]` names, at 0.7 repeats/m (matching `_C_TEX["brick"]`'s own
# scale), dust-tinted via `_apply_diffuse_tint` the SAME WAY `_debris_look`'s
# "concrete" override tints Damaged_Asphalt below: `_C_TEX["brick"]`'s own,
# already-proven tint (used as-is for `quake_flow._c_look`'s ground brick
# look) x a modest "dust" darkening factor — NOT
# `quake_flow.A_DEBRIS["brick_dusty"]`'s standalone flat-material numbers,
# which are calibrated as a finished on-screen colour for an UNTEXTURED box,
# not as a multiplier on top of an already-coloured photo texture. Using
# those directly here first-pass darkened the sampled brick photo (mean sRGB
# ~0.49/0.35/0.28) to near-black (found rendering `tools/rubble_preview.py`
# — a URM dome's chunks came out coal-dark, not brick-red; see this agent's
# report) — `diffuse_tint` MULTIPLIES the sample, it does not replace it.
# stone: flat, no map. timber/dust: flat, no map, and literally
# `quake_flow.A_DEBRIS["timber_dusty"]` / `A_DEBRIS["dust"]`'s own numbers —
# those two are genuinely untextured flat looks, so no multiplier problem.
_LOOK_MATERIALS = {
    "brick": {"rel": "megascans/Brick_Wall_Worn/T_sexkaitb_1K_B.jpg",
              "base_rgb": (0.48, 0.40, 0.36), "dust": 0.85,
              "rough": 0.90, "scale": 0.7, "desat": 0.15},
    "stone": {"rgb": (0.30, 0.27, 0.23), "rough": 0.95},
    "timber": {"rgb": (0.062, 0.046, 0.032), "rough": 0.98},
    "dust": {"rgb": (0.135, 0.127, 0.115), "rough": 1.0},
}


# ---------------------------------------------------------------------------
# round-5: the BEAM look for authored bar-shaped concrete/masonry debris
# ---------------------------------------------------------------------------
# User, on the first 500 m OSMO scene: the authored lintels/quoins/column
# stubs/sills read as flat pale bars. `_BOX_MATS`/`_LOOK_MATERIALS["stone"]`
# give them a single LINEAR albedo with no map at all, which is the same
# "smooth pale marshmallow" problem `_debris_look` was written to fix for the
# catalogue pieces. These get the freshly-pulled "Damaged Concrete Floor"
# megascans surface instead, bound the same way everything else textured in
# this module is: `damage._pbr(texture=...)` (MDL `project_uvw`, world
# triplanar — an authored box's own `st` is only ever read by the Blender
# preview shader) plus a dust `diffuse_tint`.
#
# RESILIENT BY DESIGN. The pack is being imported by another agent while this
# lands, so `_beam_tex_path` GLOBS for it and returns None when the folder is
# not there yet; `_beam_look` then returns None and `_author_large` falls
# straight back to the pre-round-5 rule. Never a crash, never a skipped chip.
_BEAM_DIR = "megascans/Damaged_Concrete_Floor"
_BEAM_SPEC = {
    "rgb": (0.520, 0.512, 0.495),   # `_DEBRIS_CONCRETE`'s neighbourhood
    "dust": 0.86,
    "rough": 0.92,
    "scale": 0.9,                   # repeats/m — a 0.3 m bar wants a tighter
    "desat": 0.25,                  #   tile than a 4 m raft
}
# The kinds this applies to: the AUTHORED (asset=None, prim_path=None) bar
# pieces of a masonry/concrete pile. `joist` is timber and `rebar` is steel —
# both keep the look the planner already gives them.
_BEAM_KINDS = ("lintel", "quoin", "column", "sill")
_BEAM_MISSING_WARNED = [False]


def _beam_tex_path():
    """The Damaged_Concrete_Floor basecolour, or None if it is not on disk.

    Globbed (`T_*_B.*`) rather than named: the megascans export's stem is a
    per-asset hash (`T_vizcebf_2K_B.png` for Damaged_Asphalt) that this agent
    cannot know before the pack is unzipped.
    """
    import glob
    root = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "assets",
        "materials", _BEAM_DIR))
    hits = sorted(glob.glob(os.path.join(root, "T_*_B.*")))
    if not hits:
        if not _BEAM_MISSING_WARNED[0]:
            _BEAM_MISSING_WARNED[0] = True
            print("[quake_rubble_usd] {0} not on disk yet — authored beam "
                  "pieces keep their flat look (chips are unaffected)"
                  .format(root))
        return None
    return _megascans_tex_path(_BEAM_DIR + "/" + os.path.basename(hits[0]))


def _beam_look(stage, parent, mats):
    """The shared Damaged_Concrete_Floor material for authored bar pieces,
    cached at `<parent>/QuakeLooks/rubble_beam` like every other look here.
    Returns None when the texture is not available, so the caller can fall
    back to its own rule."""
    from pxr import Sdf, UsdShade

    path = "{0}/QuakeLooks/rubble_beam".format(parent)
    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats.setdefault("rubble_beam", existing)
        return existing

    tex = _beam_tex_path()
    if tex is None:
        return None

    from . import damage
    spec = _BEAM_SPEC
    tint = tuple(float(c) * spec["dust"] for c in spec["rgb"])
    got = damage._pbr(stage, path, tint, spec["rough"], texture=tex,
                      scale_uv=(spec["scale"], spec["scale"]), tint=tint)
    _apply_diffuse_tint(stage, path, tint, spec["desat"])
    _add_preview_fallback(stage, path, tint, spec["rough"], texture=tex)
    if mats is not None:
        mats["rubble_beam"] = got
    return got


def _material_for_look(stage, parent, look, mats):
    """Resolve one of the round-4 per-set/per-large `"look"` values
    (`_MATERIAL_LOOKS`) to a material cached at `<parent>/QuakeLooks/
    rubble_<look>` — the SAME cache path/scheme `_rubble_look`'s mound/apron
    cache and `_debris_look`'s debris-override cache use, so "concrete"/
    "rust" here are literally the same cached material those two already
    produce (delegated straight to `_debris_look`, not re-implemented).

    Returns `None` when `look` is not one of the six values (absent, or one
    of `_LOOK`'s "urm"/"rc" construction-type keys) — every call site then
    falls back to its own OLD rule: the apron's construction-type texture,
    or the catalogue-material override for instances/large elements.
    """
    if look not in _MATERIAL_LOOKS:
        return None
    if look in ("concrete", "rust"):
        return _debris_look(stage, parent, look, mats)

    from pxr import Sdf, UsdShade

    path = "{0}/QuakeLooks/rubble_{1}".format(parent, look)
    cache_key = "rubble_" + look

    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats.setdefault(cache_key, existing)
        return existing

    from . import damage
    spec = _LOOK_MATERIALS[look]
    if "rel" in spec:
        tex = _megascans_tex_path(spec["rel"])
        tint = tuple(float(c) * spec["dust"] for c in spec["base_rgb"])
        got = damage._pbr(stage, path, tint, spec["rough"], texture=tex,
                           scale_uv=(spec["scale"], spec["scale"]), tint=tint)
        _apply_diffuse_tint(stage, path, tint, spec.get("desat", 0.0))
        _add_preview_fallback(stage, path, tint, spec["rough"], texture=tex)
    else:
        rgb, rough = spec["rgb"], spec["rough"]
        got = damage._pbr(stage, path, rgb, rough)
        _add_preview_fallback(stage, path, rgb, rough)

    if mats is not None:
        mats[cache_key] = got
    return got


# per-parent fallback uid counters, used only when the caller does not pass
# its own `uid` callable. Keyed by the STRING parent path so two `author()`
# calls under the same parent (even across separate plans/tags) never repeat
# a "large_NN" suffix.
_PARENT_COUNTERS = {}


def _make_uid(parent, uid):
    if uid is not None:
        return uid
    key = str(parent)

    def _next():
        n = _PARENT_COUNTERS.get(key, 0)
        _PARENT_COUNTERS[key] = n + 1
        return n
    return _next


def _join(root, rel):
    root = str(root or "")
    rel = str(rel or "").lstrip("/")
    if not root:
        return rel
    return root.rstrip("/") + "/" + rel


def _resolve_catalogue():
    """`disaster.quake_rubble.CATALOGUE` MERGED with `disaster.quake_rubble.
    HD_CATALOGUE` (round-5: the 840-piece textured HD debris library —
    `_select_proto_sets` can put an HD name into a plan's instance/large
    entries, and this is the ONLY place that resolves a name to a URL/size
    for the emitter, so it has to see both catalogues), imported lazily so
    this module never hard-fails if the planner is not there yet (round-4
    parallel build) or a caller supplies its own `plan["catalogue"]`. No
    name collisions exist between the two (verified: CATALOGUE's ~46 flat-
    colour entries and HD_CATALOGUE's 840 piece names are disjoint), so a
    plain dict merge is safe either way."""
    try:
        from . import quake_rubble
        merged = dict(quake_rubble.CATALOGUE)
        merged.update(quake_rubble.HD_CATALOGUE)
        return merged
    except Exception as exc:
        print("[quake_rubble_usd] disaster.quake_rubble.CATALOGUE unavailable "
              "({0}); asset references will not resolve unless plan['catalogue'] "
              "or an explicit prim_path is used.".format(exc))
        return {}


def _resolve_asset_url(name, catalogue, asset_root):
    if not name:
        return None
    entry = catalogue.get(name)
    if not entry:
        print("[quake_rubble_usd] unknown catalogue asset:", name)
        return None
    rel = entry.get("url") or entry.get("path")
    if not rel:
        return None
    return _join(asset_root, rel)


def _mat_tex_path(stem):
    """Resolve `<stem>.<ext>` under `assets/materials/quake/` to a path a
    `diffuse_texture` input can use.

    Prefers the `airstack://` local-asset-root scheme `quake_flow._c_look`
    resolves its own ground textures through (`scene_generator._join_asset_root`)
    so a scene running off an uploaded `AIRSTACK_ASSET_ROOT` still finds it;
    falls back to the plain absolute local path when `scene_generator` is not
    importable (host-side tests, the offline preview harness) or the scheme
    fails to resolve for any reason.
    """
    for ext in (".jpg", ".jpeg", ".png"):
        local = os.path.join(_MATDIR, stem + ext)
        if os.path.exists(local):
            try:
                import scene_generator as sg
                return sg._join_asset_root(
                    "airstack://scene_gen/assets/materials/quake/" + stem + ext, "")
            except Exception:
                return local
    return None


def _megascans_tex_path(rel):
    """Resolve `assets/materials/<rel>` (a megascans map OUTSIDE the
    `assets/materials/quake/` mound/apron directory `_mat_tex_path` serves)
    the same way `quake_flow._c_look` resolves its own ground textures:
    prefer the `airstack://` local-asset-root scheme
    (`scene_generator._join_asset_root`) so a scene running off an uploaded
    `AIRSTACK_ASSET_ROOT` still finds it, falling back to the plain
    absolute local path when `scene_generator` is not importable (host-side
    tests, the offline preview harness)."""
    local = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "assets", "materials", rel))
    if not os.path.exists(local):
        print("[quake_rubble_usd] texture not found:", local)
        return None
    try:
        import scene_generator as sg
        return sg._join_asset_root("airstack://scene_gen/assets/materials/" + rel, "")
    except Exception:
        return local


def _bind(stage, path, mat):
    from pxr import Sdf, UsdShade
    pr = stage.GetPrimAtPath(Sdf.Path(path))
    if pr and pr.IsValid() and mat is not None:
        UsdShade.MaterialBindingAPI.Apply(pr).Bind(mat)


def _bind_override(stage, path, mat):
    """Like `_bind`, but `bindingStrength=strongerThanDescendants` — for
    overriding a material a REFERENCED asset already bound on its own mesh
    (the 34 flat-grey `standalone/debris/pieces` catalogue entries; see
    `_debris_material_for`). Bound on the WRAPPER Xform `author()` creates
    to hold the reference (`_author_large_asset`'s `path`, an instancer
    prototype's Xform, or a flat instance's Xform) — an ancestor of
    whatever mesh the reference composes in — which is exactly what
    `strongerThanDescendants` is for: without it, USD's default
    (`weakerThanDescendants`) lets the asset's OWN binding on its mesh win
    over anything bound here."""
    from pxr import Sdf, UsdShade
    pr = stage.GetPrimAtPath(Sdf.Path(path))
    if pr and pr.IsValid() and mat is not None:
        UsdShade.MaterialBindingAPI.Apply(pr).Bind(
            mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)


def _apply_diffuse_tint(stage, mat_path, rgb, desaturation=0.0):
    """OmniPBR's `diffuse_color_constant` is the albedo a bound
    `diffuse_texture` REPLACES, not a multiplier over it — `damage._pbr`'s
    `tint=` argument writes to that same input and so has NO visible effect
    once a texture is bound (confirmed the hard way in `quake_flow._c_look`
    and `scour_relief`'s ground looks, both of which do this exact fix-up
    after calling `_pbr`; see either file's docstring for the "three
    benches rendered identically" story). `diffuse_tint` IS multiplied over
    the final sampled albedo, and `albedo_desaturation` is the only control
    that can pull a texture's own colour cast toward `rgb` at all. Called
    after `damage._pbr(..., texture=...)` for every textured look this
    module authors (mound/apron via `_rubble_look`, the debris override via
    `_debris_look`) so the tint the brief specifies actually lands on
    screen instead of silently no-opping."""
    from pxr import Gf, Sdf, UsdShade
    sh = UsdShade.Shader.Get(stage, mat_path + "/Shader")
    if not sh:
        return
    sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    if desaturation:
        sh.CreateInput("albedo_desaturation", Sdf.ValueTypeNames.Float).Set(float(desaturation))


def _add_preview_fallback(stage, mat_path, rgb, roughness, texture=None, st_name="st"):
    """A plain `UsdPreviewSurface`, bound to the UNIVERSAL render context, on
    top of the `damage._pbr` OmniPBR material already on `mat_path`.

    `damage._pbr` only ever authors `outputs:mdl:surface` (an MDL-render-
    context output) — correct for Kit/RTX, which asks for that context by
    name, but INVISIBLE to any consumer that resolves the universal (un-
    suffixed) context instead, which is what `bpy.ops.wm.usd_import` does.
    Without this, every mound/apron/ground/box material this module creates
    imports into Blender with no usable shader at all and renders solid
    black (found rendering this module's own `tools/rubble_preview.py` —
    see that file's docstring and this agent's report).

    When `texture` is given, wires `UsdUVTexture` (reading the `st_name`
    primvar via `UsdPrimvarReader_float2`) into `diffuseColor`, `scale` set
    to `(*rgb, 1.0)` so the SAME dust tint that shades the MDL texture also
    tints this one. This is ONLY meaningful for a mesh that actually carries
    that primvar — `_author_heightfield` authors a `st` = world (x, y) x the
    same repeats-per-metre scale the MDL `project_uvw` uses (see
    `_rubble_look`), so the mound/apron look genuinely textured in a Blender
    preview and not just tinted. An authored "large" box has no UVs at all
    (see `_box`), so its callers never pass `texture` here and it stays a
    flat colour — matching what the MDL side does for it too (no map).

    Without `texture`, this is a flat-tinted stand-in, not a substitute for
    verifying the real triplanar texture on curved/vertical surfaces — that
    still needs a Kit/Isaac render. Kit is unaffected either way: it
    resolves the "mdl" context and never sees any of this.
    """
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Get(stage, Sdf.Path(mat_path))
    if not mat or not mat.GetPrim().IsValid():
        return
    sh = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("PreviewSurface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    if texture:
        reader = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("StReader"))
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set(st_name)
        reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("DiffuseTex"))
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        # the dust tint, applied the same way the MDL side's `diffuse_tint`
        # multiplies its texture — `scale`/`bias` are UsdUVTexture's own
        # post-sample colour transform (output = sample * scale + bias).
        tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0))
        tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0.0, 0.0, 0.0, 0.0))
        tex_out = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))

    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")


def _rubble_look(stage, parent, look, mats, apron=False):
    """The mound/apron OmniPBR, triplanar-projected exactly like
    `quake_flow._c_look`, cached under `<parent>/QuakeLooks/rubble_<look>`
    (`_apron` suffix for the apron variant) so many piles under the same
    parent share ONE material. Cached at the STAGE level (a `Get` on the
    expected path) as well as in `mats` if given, so the cache survives
    separate `author()` calls that do not thread a `mats` dict through.
    """
    from pxr import Sdf, UsdShade

    look = look if look in _LOOK else _DEFAULT_LOOK
    suffix = "_apron" if apron else ""
    key = "rubble_" + look + suffix
    path = "{0}/QuakeLooks/rubble_{1}{2}".format(parent, look, suffix)

    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats.setdefault(key, existing)
        return existing

    spec = _LOOK[look]
    tint = spec["apron"] if apron else spec["mound"]
    tex = _mat_tex_path(spec["stem"])
    scale = _uv_scale_for(look)

    from . import damage
    got = damage._pbr(stage, path, tint, 0.95, texture=tex,
                       scale_uv=(scale, scale), tint=tint)
    _apply_diffuse_tint(stage, path, tint, spec.get("desat", 0.0))
    _add_preview_fallback(stage, path, tint, 0.95, texture=tex)
    if mats is not None:
        mats[key] = got
    return got


def _box_key(entry):
    """Which flat fallback ("concrete_dusty" | "timber_dusty" | "brick_dusty")
    an authored (asset=None) "large" box gets.

    The plan contract's "large" dict (round4 plan, API contract section) does
    not carry an explicit material field, only `kind` — so this infers from
    `kind` (or an optional forward-compatible `material` key, honoured first
    if present) rather than the contract specifying it. See the module's
    closing note / the report this agent filed for the mismatch.
    """
    mat = str(entry.get("material") or "").strip().lower()
    if mat in _BOX_MATS:
        return mat
    for k in _BOX_MATS:
        if mat and mat.split("_")[0] in k:
            return k
    kind = str(entry.get("kind") or "").lower()
    if any(s in kind for s in ("timber", "joist", "wood", "rafter")):
        return "timber_dusty"
    if any(s in kind for s in ("brick", "urm", "lintel", "quoin", "masonry")):
        return "brick_dusty"
    return "concrete_dusty"


def _box_material_for(stage, parent, key, mats):
    """The dusty flat material for an authored box: `mats[key]` if the caller
    already provided one (e.g. reusing the building's own kit material),
    else a plain `damage._pbr` flat colour cached under
    `<parent>/QuakeLooks/a_<key>` and (if `mats` was given) stored back into
    it for reuse."""
    if mats is not None and mats.get(key) is not None:
        return mats[key]

    from pxr import Sdf, UsdShade
    path = "{0}/QuakeLooks/a_{1}".format(parent, key)
    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats[key] = existing
        return existing

    rgb, rough = _BOX_MATS.get(key, _BOX_MATS["concrete_dusty"])
    from . import damage
    got = damage._pbr(stage, path, rgb, rough)
    _add_preview_fallback(stage, path, rgb, rough)
    if mats is not None:
        mats[key] = got
    return got


# ---------------------------------------------------------------------------
# debris override looks — the 34 `standalone/debris/pieces` catalogue
# entries (`quake_rubble.CATALOGUE`'s slab/chunk/lump/rebar/sheet_NN) are
# `textured: False`: a flat 0.55-grey mesh with real `st` UVs and no map of
# its own, which reads as a smooth pale "marshmallow" against the textured
# mound (round-4 v2 review: rc_dome_s3_close.png, urm_dome_s1_contact.png).
# The FAB spreads/piles (`concrete_debris_elements`, `brick_debris_pile`,
# `huge_concrete_rubble_pile`, ...) are `textured: True` — they already
# carry a real baked-in material and must NOT be touched.
# ---------------------------------------------------------------------------
# `rgb x dust` lands the final `diffuse_tint` average at ~0.42 (the round-4
# debris-material brief's screen target) — texture is the SAME
# Damaged_Asphalt map `quake_flow._C_TEX["pave"]` uses (a plain grey cracked
# surface, not Worn_Pavement's mossy joints — see that file's comment),
# `rgb` is literally `_C_TEX["pave"]`'s own tint; `dust` darkens it a notch
# further for "cement-dust-coated debris" rather than "clean pavement".
_DEBRIS_CONCRETE = {
    "rel": "megascans/Damaged_Asphalt/T_vizcebf_2K_B.png",
    "rgb": (0.500, 0.500, 0.490),
    "dust": 0.85,
    "rough": 0.90,
    "scale": 0.6,
    "desat": 0.30,
}
# Rebar tangles and the corrugated `sheet_NN` panels are `material: "steel"`
# in the catalogue — a bent/torn rust-streaked metal reads nothing like
# aggregate concrete, so these get a flat (no map) rust tint instead of
# `_DEBRIS_CONCRETE`'s texture.
_DEBRIS_RUST_RGB = (0.30, 0.19, 0.13)
_DEBRIS_RUST_ROUGH = 0.55


def _debris_look(stage, parent, key, mats):
    """The shared override material for untextured debris, cached under
    `<parent>/QuakeLooks/rubble_concrete` or `.../rubble_rust` — ONE
    material per parent per key, exactly like `_rubble_look`'s mound/apron
    cache, so a pile with hundreds of instanced chunks pays for it once.
    """
    from pxr import Sdf, UsdShade

    key = key if key in ("concrete", "rust") else "concrete"
    cache_key = "rubble_" + key
    path = "{0}/QuakeLooks/rubble_{1}".format(parent, key)

    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats.setdefault(cache_key, existing)
        return existing

    from . import damage
    if key == "concrete":
        spec = _DEBRIS_CONCRETE
        tint = tuple(float(c) * spec["dust"] for c in spec["rgb"])
        tex = _megascans_tex_path(spec["rel"])
        got = damage._pbr(stage, path, tint, spec["rough"], texture=tex,
                           scale_uv=(spec["scale"], spec["scale"]), tint=tint)
        _apply_diffuse_tint(stage, path, tint, spec["desat"])
        # these assets carry their own `st` (unlike the mound/apron, which
        # has none and needs a synthetic world-projected one authored onto
        # it) — the default `st_name="st"` reads it straight, so Blender
        # shows the real map on any of the 34 catalogue pieces this binds
        # to. An authored box (e.g. a "column" stub, see `_author_large`)
        # has no `st` at all, so it falls back to the UV-(0,0) texel there
        # — a flat tint, same as it would have gotten from `_BOX_MATS`.
        _add_preview_fallback(stage, path, tint, spec["rough"], texture=tex)
    else:
        got = damage._pbr(stage, path, _DEBRIS_RUST_RGB, _DEBRIS_RUST_ROUGH)
        _add_preview_fallback(stage, path, _DEBRIS_RUST_RGB, _DEBRIS_RUST_ROUGH)

    if mats is not None:
        mats[cache_key] = got
    return got


# ---------------------------------------------------------------------------
# textured (FAB spread/pile) debris override — round-4 v5
# ---------------------------------------------------------------------------
# `_debris_material_for` used to return `None` outright for every
# `textured: True` catalogue entry (the FAB spreads/piles already have a
# real baked material, so no generic `rubble_concrete`/`rubble_rust`
# override applies) and leave the asset's own material completely
# untouched. True as far as it went, but round-4 v5 review
# (`rc_dome_s3_contact.png`, `urm_dome_s1_contact.png`) found these reading
# as bright WHITE BLOBS from the air next to everything else in the pile —
# every OTHER look this module authors is dust-darkened one way or another
# (`_LOOK_MATERIALS`'s brick x0.85, `_DEBRIS_CONCRETE`'s x0.85, the
# mound/apron tints), these clusters were the one look with no dust pass at
# all. Fix: a COPY of the asset's own baked map, found by opening the
# asset's own referenced stage (`_reference_diffuse_texture`) and re-
# authored here with a neutral x0.85 `diffuse_tint` — never a flat colour
# substitute, so the asset's own photo still reads as itself, just dustier.
_TEXTURED_DUST_TINT = (0.85, 0.85, 0.85)
# Per-material multiplier for the scanned FAB spreads (round-4 Isaac pass,
# 2026-08-30, r4_commercial bench): under RTX the x0.85 neutral tint left
# `brick_debris_pile` / `concrete_debris_*` rendering near-WHITE — they read
# as snow piles beside a brick pile (the scans' albedo is a pale, dusty
# mortar-grey and a multiplicative 0.85 cannot move it). A brick building's
# fines are brick-and-mortar coloured, a concrete one's a mid grey, so the
# tint now depends on the catalogue entry's `material` and lands the cluster
# between the mound tile and the kit's brick chunks. Keyed by material; the
# neutral constant above is the fallback for anything unlisted.
_TEXTURED_DUST_TINT_BY_MATERIAL = {
    "brick": (0.55, 0.42, 0.36),
    "concrete": (0.52, 0.50, 0.47),
}
_TEXTURED_UV_SCALE = 0.5
_TEXTURED_ROUGH = 0.92


def _reference_diffuse_texture(url):
    """Open `url`'s own referenced stage and return the resolved absolute
    path of whatever image its `UsdPreviewSurface.diffuseColor` reads
    through a `UsdUVTexture` — the shape every measured FAB catalogue asset
    (`concrete_rubble_debris/split/*`) actually has: `Principled_BSDF`
    (`UsdPreviewSurface`) <- `Image_Texture` (`UsdUVTexture`, `file` a
    `./textures/<name>_baseColor.jpg` RELATIVE asset path) <- `UV_Map`
    (`UsdPrimvarReader_float2`, `varname="st"`, matching the mesh's own
    `primvars:st`, itself measured off the same assets).

    `Usd.Attribute.Get()` on an Asset-typed input returns an `Sdf.AssetPath`
    whose `.resolvedPath` is already anchored against the LAYER that
    authored it (proven against `huge_concrete_rubble_pile.usdc` and
    `brick_debris_pile.usdc` in the local mirror) — no path-joining of our
    own needed, and this works the same for a local mirror or an
    `omniverse://` asset root, since both go through the same Usd/Ar
    resolution `Get()` already performs.

    Returns `None` (never raises) when `url` will not open, has no
    `UsdPreviewSurface`, its `diffuseColor` is an unconnected flat colour
    (no map to copy — e.g. this module's own older test fixture,
    `test_textured_fab_spread_not_overridden`), or the resolved file does
    not exist on disk — every caller treats `None` as "leave the asset's
    own material untouched", exactly the pre-round-4-v5 behaviour.
    """
    from pxr import Usd, UsdShade

    try:
        ref_stage = Usd.Stage.Open(str(url))
    except Exception:
        return None
    if not ref_stage:
        return None

    def _resolved(file_input):
        if not file_input:
            return None
        val = file_input.Get()
        if val is None:
            return None
        found = getattr(val, "resolvedPath", "") or ""
        return found if found and os.path.exists(found) else None

    fallback = None
    for prim in ref_stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(prim)
        sid = sh.GetIdAttr().Get()
        if sid == "UsdUVTexture" and fallback is None:
            fallback = _resolved(sh.GetInput("file"))
        if sid != "UsdPreviewSurface":
            continue
        diffuse = sh.GetInput("diffuseColor")
        if not diffuse:
            continue
        sources, _invalid = diffuse.GetConnectedSources()
        if not sources:
            continue
        resolved = _resolved(UsdShade.Shader(sources[0].source).GetInput("file"))
        if resolved:
            return resolved
    return fallback


def _textured_debris_look(stage, parent, name, catalogue, asset_root, mats):
    """The round-4 v5 x0.85 dust tint for one TEXTURED FAB catalogue asset
    (`textured: True`), cached at `<parent>/QuakeLooks/rubble_tex_<name>`
    exactly like every other look this module caches (once per asset per
    parent), bound `strongerThanDescendants` on the wrapper Xform over
    whatever material the reference itself already bound — see
    `_bind_override`.

    Built by re-authoring `damage._pbr` (MDL/OmniPBR, world-projected
    triplanar — the same convention every other textured look here uses,
    so a Kit/RTX render is consistent regardless of any one asset's own UV
    unwrap) with the asset's OWN texture file (`_reference_diffuse_texture`)
    in place of a generic map, then `_apply_diffuse_tint` (so the x0.85
    actually multiplies the sample — a bare `damage._pbr(tint=)` alone is a
    silent no-op once a texture is bound, see that function's docstring)
    and `_add_preview_fallback` with the SAME texture read through the
    default `st_name="st"` — the mesh's own real primvar, not triplanar —
    so a Blender/Hydra preview shows the actual photo, correctly UV mapped,
    just x0.85 darker.

    Returns `None` (no override; caller leaves the asset's own material
    bound, the pre-round-4-v5 behaviour) when no texture can be found — a
    missing map or an unusual material graph is not a reason to break the
    render.
    """
    from pxr import Sdf, UsdShade

    cache_key = "rubble_tex_" + name
    path = "{0}/QuakeLooks/rubble_tex_{1}".format(parent, name)

    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing and existing.GetPrim().IsValid():
        if mats is not None:
            mats.setdefault(cache_key, existing)
        return existing
    if mats is not None and mats.get(cache_key) is not None:
        return mats[cache_key]

    url = _resolve_asset_url(name, catalogue, asset_root)
    if not url:
        return None
    tex = _reference_diffuse_texture(url)
    if not tex:
        return None

    from . import damage
    entry = catalogue.get(name) or {}
    tint = _TEXTURED_DUST_TINT_BY_MATERIAL.get(
        str(entry.get("material") or "").lower(), _TEXTURED_DUST_TINT)
    got = damage._pbr(stage, path, tint, _TEXTURED_ROUGH, texture=tex,
                       scale_uv=(_TEXTURED_UV_SCALE, _TEXTURED_UV_SCALE), tint=tint)
    _apply_diffuse_tint(stage, path, tint, 0.0)
    _add_preview_fallback(stage, path, tint, _TEXTURED_ROUGH, texture=tex)

    if mats is not None:
        mats[cache_key] = got
    return got


def _debris_material_for(stage, parent, name, catalogue, asset_root, mats):
    """Which override material (if any) a CATALOGUE entry's referenced
    piece gets, by asset `name`. `None` when the entry is missing (defensive
    — do not stomp on an unknown asset's real look). `textured: False` (the
    34 flat-grey pieces this round's catalogue lists): `rubble_rust` for
    `material: "steel"` (rebar, sheet), else `rubble_concrete` — unchanged
    from before round-4 v5. `textured: True` (or the field absent, same
    default as before): the round-4 v5 x0.85 dust tint on a COPY of the
    asset's own map (`_textured_debris_look`) — these used to get NO
    override at all; see that function's docstring for why.
    """
    entry = catalogue.get(name) if name else None
    if not entry:
        return None
    if entry.get("textured", True):
        return _textured_debris_look(stage, parent, name, catalogue, asset_root, mats)
    key = "rust" if str(entry.get("material") or "").lower() == "steel" else "concrete"
    return _debris_look(stage, parent, key, mats)


# ---------------------------------------------------------------------------
# mesh helpers
# ---------------------------------------------------------------------------
def _smooth_normals(points, faces):
    """Per-vertex smooth normals for a shared-vertex triangle mesh — the
    heightfield mound needs Hydra to shade it as one continuous surface, not
    as a faceted heap of flat triangles."""
    p0 = points[faces[:, 0]]
    p1 = points[faces[:, 1]]
    p2 = points[faces[:, 2]]
    fn = np.cross(p1 - p0, p2 - p0)
    n = np.zeros_like(points, dtype=np.float64)
    np.add.at(n, faces[:, 0], fn)
    np.add.at(n, faces[:, 1], fn)
    np.add.at(n, faces[:, 2], fn)
    lens = np.linalg.norm(n, axis=1)
    lens[lens == 0.0] = 1.0
    return n / lens[:, None]


def _author_heightfield(stage, path, hf, uv_scale=None):
    """Write `hf["points"]`/`hf["faces"]` as a `UsdGeom.Mesh` with authored
    smooth vertex normals, `subdivisionScheme none`, `doubleSided True`. No
    material binding here (the caller picks mound vs. apron look).

    `uv_scale`, if given, authors `primvars:st` = world (x, y) times that
    same repeats-per-metre scale, `vertex` interpolation. The MDL material
    never reads this (it stays `project_uvw` triplanar, world-space, exactly
    as before) — this primvar exists ONLY so a Hydra consumer that cannot
    see the "mdl" render context (Blender's `usd_import`; see
    `_add_preview_fallback`) still has a correctly-scaled UV to sample the
    real rubble texture through, instead of a flat tint.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    points = np.asarray(hf["points"], dtype=np.float64).reshape(-1, 3)
    faces = np.asarray(hf["faces"], dtype=np.int64).reshape(-1, 3)

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in points]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [int(i) for tri in faces for i in tri]))

    if len(points) and len(faces):
        nrm = _smooth_normals(points, faces)
        m.CreateNormalsAttr(Vt.Vec3fArray(
            [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in nrm]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    if uv_scale and len(points):
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray(
            [Gf.Vec2f(float(x) * uv_scale, float(y) * uv_scale) for x, y, _ in points]))

    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    if len(points):
        lo = points.min(axis=0)
        hi = points.max(axis=0)
    else:
        lo = hi = np.zeros(3)
    m.CreateExtentAttr([Gf.Vec3f(*[float(v) for v in lo]),
                         Gf.Vec3f(*[float(v) for v in hi])])
    return path


def recentre_for_loose(stage, paths):
    """Give a `_author_heightfield` mound/apron a LOCAL frame before a
    caller adds it to `ctx["loose"]`.

    `_author_heightfield` bakes `points` directly in WORLD space with NO
    xform ops at all (see the module docstring) — correct, and cheap, for
    the ordinary case where the mound stays a STATIC collider forever. But
    `r_soft_storey`/`quake_collapse._author_heaps` both have a MID-storey
    case where the collar is authored floating beside the crushed band and
    is deliberately handed to `settle.run` as a loose RigidBody so physics
    carries it down to the base. `settle.run` puts that body's origin at
    its parent's identity frame while every point of a world-baked mesh
    sits tens of metres away — the exact failure `settle.py`'s own
    `no_local_frame` check is built to catch — so the random angular kick
    every loose body gets swings that arm and throws the mound (measured: a
    95.50 m "worst mover", then a hard `SettleNotConverged` — round 7,
    s4g2/office_DG4's `rubble_office_DG4_collar_1_mound`).

    Recentres `points` (and `extent`) on their own centroid and authors a
    `translate` op putting that centroid back at the mesh's original world
    position — same shape of fix `settle.bake` already assumes every OTHER
    loose prim arrives with (a wrapper Xform around a referenced asset
    always carries its own translate/rotate/scale). The world position of
    every point is unchanged; `primvars:st` (baked from the ORIGINAL world
    x, y) still lines up with it exactly.

    A no-op for anything that already has an xform op (so calling this
    twice, or on a "large"/instancer path that was never world-baked to
    begin with, does nothing) or that is not a `UsdGeom.Mesh`. Returns the
    number of paths actually recentred.
    """
    from pxr import Gf, UsdGeom, Vt

    n = 0
    for path in paths or ():
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Mesh):
            continue
        xformable = UsdGeom.Xformable(prim)
        if xformable.GetOrderedXformOps():
            continue
        mesh = UsdGeom.Mesh(prim)
        pts_attr = mesh.GetPointsAttr()
        pts = pts_attr.Get()
        if not pts:
            continue
        cnt = len(pts)
        cx = sum(float(p[0]) for p in pts) / cnt
        cy = sum(float(p[1]) for p in pts) / cnt
        cz = sum(float(p[2]) for p in pts) / cnt
        pts_attr.Set(Vt.Vec3fArray(
            [Gf.Vec3f(float(p[0]) - cx, float(p[1]) - cy, float(p[2]) - cz)
             for p in pts]))
        ext_attr = mesh.GetExtentAttr()
        ext = ext_attr.Get()
        if ext:
            ext_attr.Set([Gf.Vec3f(float(v[0]) - cx, float(v[1]) - cy,
                                   float(v[2]) - cz) for v in ext])
        xformable.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
        n += 1
    return n


# ---------------------------------------------------------------------------
# round-5: chipped authored boxes  ("they shouldn't look perfect")
# ---------------------------------------------------------------------------
# Every AUTHORED (asset=None, prim_path=None) "large" element is a literal
# cuboid: `LINTEL_LONG_SIZE`/`LINTEL_QUOIN_SIZE`/`JOIST_SIZE`/`COLUMN_SIZE_*`
# straight into `_box`. On screen they are the gift boxes the user called out
# on the first 500 m OSMO scene. `fracture.chip_box` bites corners out of them
# with oblique VTK plane clips, roughens the cast faces and (timber only)
# bends them.
#
# PER KIND, and the differences are the point:
#   * `ends` is the share of chips aimed at the BREAK faces of a bar — the
#     user's second note, "the actual breaking should not be clean". A snapped
#     lintel / joist / column stub is broken ACROSS, so its two small end
#     faces are fracture surfaces and get the heaviest, most overlapping
#     treatment; a quoin is a cut stone that mostly SPALLED, so it gets fewer.
#   * concrete and masonry get chips + roughening and NO bend — concrete does
#     not bow. Timber joists do, so they alone carry `warp_frac`/`twist_deg`.
#   * `depth_frac` is deliberately wide and drawn log-uniformly inside
#     `chip_box` (see `_chip_depth`): most pieces are nicked, a few lose a
#     real corner chunk. Measured over 24 seeds x 5 kinds: 2-30 % volume
#     loss, median ~19 %.
# `warp_frac` is a fraction of the piece's own LONGEST dimension, converted to
# metres here (a 3.2 m joist bows 6 cm), because `warp_mesh` works in metres.
#
# ROUND 6 — WHY EVERY ENTRY GREW `bites` AND `gouges`. The round-5 tables
# above tuned `chips`/`depth_frac`/`ends`, and the user still called the
# pillars perfect cuboids, for a reason no tuning of those three could reach:
# a plane clip always removes material at the piece's EXTREME CORNER along the
# cut normal, and every corner of a 0.4 x 0.4 x 3.5 m column is at one of its
# two ends. `tools/pillar_break_bench.py` measured the cross-section fill at 44
# stations on exactly that column through exactly this table: 22 %, 42 %, 61 %
# at the bottom, 72-92 % at the top, and 85-99 % AT EVERY ONE OF THE FORTY
# STATIONS IN BETWEEN. The renders agree — a straight prism with a bevelled
# top. So each kind now also asks for
#   `bites`   very large corner bites sized by the SECTION (`bite_frac`),
#             which is what "random from small to very large chips" needs and
#             what `depth_frac` — measured against the span along the cut
#             normal, i.e. mostly against the piece's LENGTH — never gave;
#   `gouges`  scallops bitten out at a station chosen ALONG the piece, the
#             only mechanism in `fracture` that damages a shaft or the middle
#             of a slab edge rather than a corner.
# `max_loss` rises with them (a piece that loses a third of its section HAS
# lost a third of its volume, and the old 0.27 ceiling would have scaled the
# piece back up to hide it), and `rough_lam_frac` fixes a wavelength that was
# being derived from the piece's LONGEST extent — 2.9 m on this column, i.e.
# one gentle bow rather than surface relief.
_GOUGE_COMMON = {"gouge_depth": (0.07, 0.26), "gouge_big_p": 0.26,
                 "gouge_big_frac": (0.30, 0.45), "gouge_vol_frac": 0.20,
                 "gouge_budget": 760, "rough_lam_frac": 1.7,
                 "bite_frac": (0.32, 0.78),
                 "min_loss": 0.07, "max_loss": 0.36}
_CHIP_KIND = {
    "lintel": dict(_GOUGE_COMMON, chips=(2, 6), depth_frac=(0.025, 0.15),
                   ends=0.55, rough_frac=0.055, bites=(1, 2), gouges=(2, 4)),
    "quoin":  dict(_GOUGE_COMMON, chips=(2, 6), depth_frac=(0.03, 0.18),
                   ends=0.30, rough_frac=0.06, bites=(1, 2), gouges=(2, 3),
                   gouge_budget=420),
    "column": dict(_GOUGE_COMMON, chips=(2, 6), depth_frac=(0.025, 0.15),
                   ends=0.55, rough_frac=0.055, bites=(1, 2), gouges=(2, 4)),
    "sill":   dict(_GOUGE_COMMON, chips=(2, 6), depth_frac=(0.025, 0.15),
                   ends=0.50, rough_frac=0.055, bites=(1, 2), gouges=(2, 4)),
    "joist":  dict(_GOUGE_COMMON, chips=(2, 6), depth_frac=(0.025, 0.16),
                   ends=0.60, rough_frac=0.07, bites=(1, 2), gouges=(2, 4),
                   warp_frac=0.012, twist_deg=5.0),
}
_CHIP_DEFAULT = dict(_GOUGE_COMMON, chips=(2, 5), depth_frac=(0.03, 0.15),
                     ends=0.35, rough_frac=0.055, bites=(1, 2), gouges=(2, 4))


def _chip_seed(*parts):
    """A stable 32-bit seed from the piece's own identity.

    NOT a draw off a shared rng: `_author_large` is an emitter with no rng of
    its own, and threading one in would make every piece's shape depend on how
    many pieces were authored before it — which is exactly the shared-draw
    coupling `fire_collapse`'s private-rng rule exists to avoid. Hashing the
    piece's own (tag, index, kind, size, position) gives the same shape for the
    same plan every time, and nothing else moves when one piece changes.
    """
    from . import fracture
    return fracture.stable_seed(*parts)


def _chip_spec(kind, seed):
    """The `chip_box` kwargs for one authored box, or None when chipping is
    off (`QC_CHIP=0`). `kind` is the plan's own `entry["kind"]`."""
    from . import fracture
    if not fracture.chips_enabled():
        return None
    kw = dict(_CHIP_KIND.get(str(kind or "").lower(), _CHIP_DEFAULT))
    kw["seed"] = int(seed)
    return kw


def _planar_st(points, faces, scale):
    """faceVarying `st` by dominant-axis planar projection, in metres x
    `scale` (repeats/m) — the same convention `_author_heightfield` uses for
    the mound.

    ONLY the Blender/Hydra preview reads this. The MDL material stays
    `project_uvw` world-triplanar exactly as it is for every other textured
    look here, so Kit's render is identical with or without the primvar; the
    primvar is what stops the offline proof render from showing a flat tint on
    the one population this round is about (see `_add_preview_fallback`'s note,
    which recorded the authored box's lack of UVs as a known preview gap —
    this closes it).
    """
    p = np.asarray(points, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)
    n = np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]])
    ax = np.argmax(np.abs(n), axis=1)                 # dominant normal axis
    u_ax = np.where(ax == 0, 1, 0)                    # x->y, else ->x
    v_ax = np.where(ax == 2, 1, 2)                    # z->y, else ->z
    tri = p[f]                                        # (m, 3, 3)
    idx = np.arange(len(f))
    out = np.empty((len(f), 3, 2), dtype=np.float64)
    for c in range(3):
        out[:, c, 0] = tri[idx, c, u_ax] * float(scale)
        out[:, c, 1] = tri[idx, c, v_ax] * float(scale)
    return out.reshape(-1, 2)


def _chipped_box(stage, path, sx, sy, sz, mat, spec, uv_scale=0.9):
    """`_box`, but irregular. Same bottom-centre origin, same crisp
    faceVarying normals, plus a preview `st`. Returns the mesh, or None if
    the chip pass produced nothing (the caller then authors the plain box)."""
    import random as _random

    from pxr import Gf, Sdf, UsdGeom, Vt

    from . import fracture

    kw = dict(spec)
    seed = kw.pop("seed", 0)
    warp_frac = float(kw.pop("warp_frac", 0.0))
    long_m = float(max(abs(sx), abs(sy), abs(sz)))
    pts, faces = fracture.chip_box(
        sizes=(float(sx), float(sy), float(sz)),
        rng=_random.Random(int(seed)), bottom=True,
        warp_m=warp_frac * long_m, **kw)
    if pts is None or faces is None or not len(faces):
        return None
    # THE BOTTOM-CENTRE ORIGIN IS A CONTRACT, not a detail of how `_box`
    # happens to build its points: the planner has already sunk the piece's
    # LOWEST POINT `bury` below the mound surface and hands the emitter the
    # final world position of that origin (see `_author_large`'s docstring).
    # A chip that bites the bottom corner, or a roughening pass that pushes a
    # bottom vertex down, moves that lowest point — so put it back on z = 0.
    # x/y are deliberately NOT re-centred: an asymmetric footprint is the
    # whole point, and the planner places by origin, not by centroid.
    pts[:, 2] -= float(pts[:, 2].min())

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(
        [int(i) for tri in faces for i in tri]))
    nrm = np.cross(pts[faces[:, 1]] - pts[faces[:, 0]],
                   pts[faces[:, 2]] - pts[faces[:, 0]])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.repeat(nrm / np.maximum(ln, 1e-12), 3, axis=0)
    m.CreateNormalsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in nrm]))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set(Vt.Vec2fArray([Gf.Vec2f(float(a), float(b))
                          for a, b in _planar_st(pts, faces, uv_scale)]))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    m.CreateExtentAttr([Gf.Vec3f(*[float(v) for v in lo]),
                        Gf.Vec3f(*[float(v) for v in hi])])
    if mat is not None:
        _bind(stage, path, mat)
    return m


def _box(stage, path, sx, sy, sz, mat=None, chip=None):
    """A closed box mesh with its LOCAL origin at the bottom-centre — points
    span [-sx/2, sx/2] x [-sy/2, sy/2] x [0, sz] — NOT centred like
    `quake_flow._box`, whose points sit about the body centre because its
    boxes are rigid-body-simulated. These boxes are never simulated (design
    table layer 2: "physics: none"), so there is no reason to give up the
    bottom-centre convention.

    This MATCHES the referenced "large" assets' own convention (footprint
    centre, base at z = 0): `pos` is the FINAL world position of that
    bottom-centre origin for every "large" kind alike (asset ref, authored
    box, or an existing prim's pivot) — the planner has already placed the
    piece's lowest point `bury` (a FRACTION of its own rotated thickness,
    not metres) below the mound surface, so the emitter translates straight
    to `pos` and never re-applies `bury` itself. Faces get faceVarying
    constant normals — a rubble slab is a hard edge, not a pillow
    (`planks.py`'s reason for the same choice).

    `chip` (round 5), when given, replaces the 8-point cuboid with the
    irregular chipped/warped solid `_chipped_box` builds from the same three
    dimensions and the same bottom-centre origin. `chip=None` — and
    `QC_CHIP=0`, which makes `_chip_spec` return None at every call site —
    takes the original body below unchanged, byte for byte."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    if chip is not None:
        got = _chipped_box(stage, path, sx, sy, sz, mat, chip)
        if got is not None:
            return got
        # a chip pass that produced nothing falls through to the plain box

    hx, hy = sx / 2.0, sy / 2.0
    P = Gf.Vec3f
    pts = [P(-hx, -hy, 0.0), P(hx, -hy, 0.0), P(hx, hy, 0.0), P(-hx, hy, 0.0),
           P(-hx, -hy, sz), P(hx, -hy, sz), P(hx, hy, sz), P(-hx, hy, sz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    nrm = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)]

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    m.CreateNormalsAttr(Vt.Vec3fArray([P(*n) for n in nrm for _ in range(4)]))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateExtentAttr([P(-hx, -hy, 0.0), P(hx, hy, sz)])
    if mat is not None:
        _bind(stage, path, mat)
    return m


def _quath(q):
    from pxr import Gf
    w, x, y, z = q
    return Gf.Quath(float(w), Gf.Vec3h(float(x), float(y), float(z)))


def _quatf(q):
    from pxr import Gf
    w, x, y, z = q
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


# ---------------------------------------------------------------------------
# "large" elements
# ---------------------------------------------------------------------------
def _author_large_asset(stage, path, url, pos, rot_deg, scale):
    """`pos` is already the FINAL world position of the asset's own
    bottom-centre origin — the planner (`quake_rubble.plan_pile`) has
    already sunk the piece `bury` (a fraction of its own rotated thickness)
    below the mound surface, so this translates straight to `pos` and never
    re-applies `bury` itself (round-4 contract clarification)."""
    from pxr import Gf, Sdf, UsdGeom
    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    if not prim.GetReferences().AddReference(url):
        return None
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(float(rot_deg[0]), float(rot_deg[1]), float(rot_deg[2])))
    xf.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))
    return path


def _author_large_box(stage, parent, path, size, pos, rot_deg, mat, chip=None):
    """Same `pos`-is-final convention as `_author_large_asset` — `_box`
    already puts the box's own local origin at its bottom-centre, so no
    `bury` offset is applied here either.

    `chip` is `_chip_spec`'s dict (round 5) or None."""
    from pxr import Gf, Usd, UsdGeom
    sx, sy, sz = (size or (1.0, 1.0, 1.0))
    m = _box(stage, path, float(sx), float(sy), float(sz), mat=mat, chip=chip)
    xf = UsdGeom.Xformable(m)
    tr_op = xf.AddTranslateOp()
    tr_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(float(rot_deg[0]), float(rot_deg[1]), float(rot_deg[2])))
    # ROUND-6 SEAT FIX: `_chipped_box` re-seats the chipped mesh's lowest
    # point to local z=0, but the PLANNER seats by the analytic BOX's
    # rotated minimum — and a chip can remove exactly the corner that would
    # have been lowest after rotation, leaving the real mesh up to ~15 cm
    # above the planned bottom (caught by the rot_deg/orientation convention
    # test: a raft floated 12.5 cm). Compensate with USD's own math: after
    # authoring the ops, transform the actual points AND the analytic box
    # corners by the prim's local-to-world and translate down by the
    # deficit, so the chipped mesh's true rotated minimum lands exactly
    # where the planner put the box's.
    pts_attr = m.GetPointsAttr().Get() if hasattr(m, "GetPointsAttr") else None
    if chip and pts_attr:
        w = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        actual_min = min(w.Transform(Gf.Vec3d(q))[2] for q in pts_attr)
        hx, hy = float(sx) / 2.0, float(sy) / 2.0
        corners = [Gf.Vec3d(ix, iy, iz) for ix in (-hx, hx)
                   for iy in (-hy, hy) for iz in (0.0, float(sz))]
        box_min = min(w.Transform(q)[2] for q in corners)
        dz = float(actual_min - box_min)
        if dz > 1e-4:
            tr_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]),
                               float(pos[2]) - dz))
    return path


def _lay_existing(stage, prim_path, pos, rot_deg, parent=None, mats=None,
                  extra_static=None):
    """Re-lay an EXISTING kit/sliced prim on the pile: translate its own
    pivot (the centre of its CURRENT world bound) to `pos`, rotating
    `rot_deg` about that pivot, without re-referencing it.

    Done in world space and then converted back to the prim's own PARENT
    frame, so this is correct wherever the prim happens to sit in the
    stage's existing hierarchy (a building's own Xform nest, typically) —
    not just for a prim that is already a direct child of `parent`.

    Round-5 follow-up: once the baseline lay above is set, `_dress_laid_panel`
    (gated on `LAY_PANEL_DRESS`) sinks/tilts it further and scatters edge
    chunks — see that function's docstring for why. `parent`/`mats` are only
    used for that: the dressing pass is a no-op without a `parent` to author
    edge chunks under. `extra_static`, when given, is a list this call
    APPENDS every edge-chunk path onto — `_author_large`'s own return is one
    path (the re-laid prim itself, unchanged contract), so this is the only
    way `author()` learns about the extra prims and can fold them into
    `result["all"]`/`result["static"]`.
    """
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print("[quake_rubble_usd] prim_path not found:", prim_path)
        return None
    xf = UsdGeom.Xformable(prim)
    if not xf:
        print("[quake_rubble_usd] prim_path not Xformable:", prim_path)
        return None

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    pivot = rng.GetMidpoint() if not rng.IsEmpty() else Gf.Vec3d(0.0, 0.0, 0.0)

    m_orig_world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    parent_prim = prim.GetParent()
    parent_xf = UsdGeom.Xformable(parent_prim) if parent_prim and parent_prim.IsValid() else None
    m_parent_to_world = (parent_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                         if parent_xf else Gf.Matrix4d(1.0))
    world_to_parent = m_parent_to_world.GetInverse()

    rx, ry, rz = float(rot_deg[0]), float(rot_deg[1]), float(rot_deg[2])
    rot = (Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), rx))
           * Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), ry))
           * Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), rz)))
    m_to_origin = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(-pivot[0], -pivot[1], -pivot[2]))
    m_to_pos = Gf.Matrix4d(1.0).SetTranslate(
        Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

    m_new_world = m_orig_world * m_to_origin * rot * m_to_pos
    m_new_local = m_new_world * world_to_parent

    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m_new_local)

    if _panel_dress_enabled():
        dress_parent = parent or str(prim_path).rsplit("/", 1)[0]
        chunks = _dress_laid_panel(stage, prim_path, dress_parent, mats)
        if extra_static is not None and chunks:
            extra_static.extend(chunks)
    return str(prim_path)


# ---------------------------------------------------------------------------
# round-5 follow-up: dressing a LAID panel ("still looks like a huge clean
# rectangle on the mound")
# ---------------------------------------------------------------------------
# `_lay_existing` re-lays an EXISTING kit/sliced prim — a wide opening panel
# `quake_sliced._total_collapse` chose, or a fit-out column racked into the
# pile by `_apply_fit_ops`'s "columns_to_pile" — and it is NEVER cut (the
# `vtkStripper` SIGSEGV rule: the referenced/sliced piece may be a shell), so
# nothing below touches its geometry. What breaks the clean-rectangle read
# instead: sink it 25-45% deeper into the mound than the planner's own
# `bury` already put it, add a little extra tilt and a yaw jitter so two
# panels from the same plan spot do not share a silhouette, and scatter a
# handful of small CHIPPED boxes (`_box`'s own `chip=` path, the same round
# trip `_chipped_box`/`_author_large` use) along its exposed edges so the
# OUTLINE itself is broken, not just the surface.
_LAY_PANEL_DRESS_ENV = "LAY_PANEL_DRESS"
PANEL_SINK_FRAC = (0.25, 0.45)      # extra fraction of the panel's OWN
                                     # current world Z-extent sunk below its
                                     # planned rest — "half-swallowed"
PANEL_EXTRA_TILT_DEG = (5.0, 15.0)
PANEL_YAW_JITTER_DEG = (5.0, 20.0)
PANEL_EDGE_CHIPS_N = (3, 8)
PANEL_EDGE_CHIP_SIZE_M = (0.15, 0.55)


def _panel_dress_enabled():
    """`LAY_PANEL_DRESS=0` turns the whole dressing pass off. Read live, like
    `fracture.chips_enabled()`, so a test can flip it with no reload."""
    return os.environ.get(_LAY_PANEL_DRESS_ENV, "1").strip().lower() not in (
        "0", "false", "no", "off")


def _dress_laid_panel(stage, prim_path, parent, mats):
    """Sink + tilt + yaw-jitter a just-laid panel in place, then scatter
    edge chunks along its (pre-dress) footprint perimeter.

    Reads the panel's own POST-LAY world bound — `_lay_existing` has already
    set the baseline transform above — rather than trusting the planner's
    `size`, so this is correct for a `prim_path` "large" entry from EITHER
    source `_lay_existing` is handed (a wall panel, or a fit-out column
    racked by `columns_to_pile`): whatever it re-lays gets the same
    treatment. Returns the list of edge-chunk paths authored (empty on any
    prim that could not be measured — never raises; a dress pass is
    cosmetic).
    """
    import math
    import random as _random

    from pxr import Gf, Usd, UsdGeom

    from . import fracture

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return []
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return []
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    wb = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if wb.IsEmpty():
        return []
    lo, hi = wb.GetMin(), wb.GetMax()
    pivot = wb.GetMidpoint()
    thickness = max(0.05, float(min(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])))

    seed = fracture.stable_seed(str(prim_path), "panel_dress")
    rr = _random.Random(seed)
    sink = rr.uniform(*PANEL_SINK_FRAC) * thickness
    tilt_deg = rr.uniform(*PANEL_EXTRA_TILT_DEG)
    az = rr.uniform(0.0, 360.0)
    yaw_jit = rr.uniform(*PANEL_YAW_JITTER_DEG) * (1.0 if rr.random() < 0.5 else -1.0)

    ax, ay = math.cos(math.radians(az)), math.sin(math.radians(az))
    m_world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    to_origin = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(-pivot[0], -pivot[1], -pivot[2]))
    tilt_r = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(ax, ay, 0.0), tilt_deg))
    yaw_r = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw_jit))
    to_sunk = Gf.Matrix4d(1.0).SetTranslate(
        Gf.Vec3d(pivot[0], pivot[1], pivot[2] - sink))
    m_new_world = m_world * to_origin * tilt_r * yaw_r * to_sunk

    parent_prim = prim.GetParent()
    parent_xf = UsdGeom.Xformable(parent_prim) if parent_prim and parent_prim.IsValid() else None
    m_p2w = (parent_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
             if parent_xf else Gf.Matrix4d(1.0))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m_new_world * m_p2w.GetInverse())

    if not fracture.chips_enabled():
        return []
    n = rr.randint(*PANEL_EDGE_CHIPS_N)
    return _author_panel_edge_chunks(stage, parent, prim_path, lo, hi, sink,
                                     mats, rr, n)


def _author_panel_edge_chunks(stage, parent, panel_path, lo, hi, sink, mats,
                              rr, n):
    """`n` small chipped boxes scattered along `panel_path`'s exposed
    perimeter — its PRE-sink world bbox (`lo`/`hi`), close enough to where
    the panel now sits since the extra sink/tilt are a fraction of its own
    thickness. The panel itself is never touched; this is what breaks its
    clean-rectangle silhouette from the air. `sink` seats the chunks near
    the panel's new, lower bottom edge rather than floating at the old one.
    Returns the list of authored paths.
    """
    from pxr import Gf, UsdGeom

    key = _box_key({"kind": "panel"})
    mat = _box_material_for(stage, parent, key, mats)
    tag = _safe_name(str(panel_path).rsplit("/", 1)[-1])
    w, d = float(hi[0] - lo[0]), float(hi[1] - lo[1])
    edges = (("S", w, (0.0, -1.0)), ("N", w, (0.0, 1.0)),
             ("W", d, (-1.0, 0.0)), ("E", d, (1.0, 0.0)))
    ground_z = float(lo[2]) - sink
    made = []
    for i in range(n):
        side, length, (nx, ny) = edges[rr.randrange(4)]
        if length <= 1e-6:
            continue
        t = rr.uniform(0.08, 0.92)
        if side == "S":
            x, y = lo[0] + t * w, lo[1]
        elif side == "N":
            x, y = lo[0] + t * w, hi[1]
        elif side == "W":
            x, y = lo[0], lo[1] + t * d
        else:
            x, y = hi[0], lo[1] + t * d
        off = rr.uniform(0.05, 0.45)
        x += nx * off + rr.uniform(-0.15, 0.15)
        y += ny * off + rr.uniform(-0.15, 0.15)
        z = ground_z + rr.uniform(-0.05, 0.12)
        s = rr.uniform(*PANEL_EDGE_CHIP_SIZE_M)
        sx, sy, sz = s, s * rr.uniform(0.6, 1.3), s * rr.uniform(0.4, 0.9)
        path = "{0}/rubble_panel_edge_{1}_{2:02d}".format(parent, tag, i)
        seed = _chip_seed("panel_edge", str(panel_path), i, sx, sy, sz)
        spec = _chip_spec("panel_edge", seed)
        _box(stage, path, sx, sy, sz, mat=mat, chip=spec)
        pxf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
        pxf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        pxf.AddRotateZOp().Set(rr.uniform(0.0, 360.0))
        made.append(path)
    return made


def _author_large(stage, parent, tag, entry, mats, catalogue, asset_root, n,
                  extra_static=None):
    """`entry["pos"]` is the FINAL world position of the piece's own
    bottom-centre origin for every kind alike (asset ref, authored box, or
    an existing prim's pivot) — `entry["bury"]` is a FRACTION of the piece's
    own rotated thickness that the PLANNER has already folded into `pos`
    (how much of its lowest point sinks below the mound surface), not a
    metres offset for the emitter to re-apply. So this never reads `bury`
    at all; every branch below translates straight to `pos`.

    `entry["look"]` (round-4 B contract), when one of `_MATERIAL_LOOKS`,
    OVERRIDES whichever material this element would otherwise get — the
    catalogue-material rule for a referenced asset, or the `_box_key`/
    "column" rule for an authored box — so B can put a construction-type-
    correct material (e.g. "brick" on a URM lintel) on a piece the
    catalogue's own `material` field does not distinguish. Absent (or not
    one of the six), falls straight back to the pre-round-4 rule. Never
    consulted for the `prim_path` (re-lay an existing prim) branch — that
    piece keeps whatever material it already has.
    """
    path = "{0}/rubble_{1}_large_{2:02d}".format(parent, tag, n)
    pos = entry.get("pos") or (0.0, 0.0, 0.0)
    rot = entry.get("rot_deg") or (0.0, 0.0, 0.0)
    scale = float(entry.get("scale") if entry.get("scale") is not None else 1.0)

    prim_path = entry.get("prim_path")
    if prim_path:
        return _lay_existing(stage, prim_path, pos, rot, parent=parent,
                             mats=mats, extra_static=extra_static)

    kind = str(entry.get("kind") or "").lower()
    size = entry.get("size")
    # LAZY (round 5): `_material_for_look` DEFINES the material prim as a side
    # effect, so calling it for a piece that is about to take the beam look
    # instead would leave an orphan `QuakeLooks/rubble_stone` on every URM
    # pile. Resolve it only where it is actually consulted.
    beam_kind = (kind in _BEAM_KINDS and not entry.get("asset"))

    asset = entry.get("asset")
    if asset:
        look_override = _material_for_look(stage, parent, entry.get("look"), mats)
        url = _resolve_asset_url(asset, catalogue, asset_root)
        if url is None:
            return None
        out = _author_large_asset(stage, path, url, pos, rot, scale)
        if out:
            override = (look_override if look_override is not None else
                        _debris_material_for(stage, parent, asset, catalogue, asset_root, mats))
            if override is not None:
                _bind_override(stage, out, override)
        return out

    # ROUND 5, the beam surface. Every AUTHORED bar-shaped concrete/masonry
    # piece takes the Damaged_Concrete_Floor scan ahead of `look_override` and
    # ahead of the `_box_key` rule — the flat "stone"/"concrete_dusty" albedo
    # those give is exactly the pale-bar look the user objected to. `joist`
    # (timber) and `rebar` (steel) are NOT in `_BEAM_KINDS` and keep theirs.
    # Falls through to the old rule when the pack is not on disk.
    mat = _beam_look(stage, parent, mats) if beam_kind else None
    if mat is None:
        look_override = _material_for_look(stage, parent, entry.get("look"), mats)
        if look_override is not None:
            mat = look_override
        else:
            key = _box_key(entry)
            # "column" stubs are the one authored (asset=None) kind the
            # round-4 v2 review called out by name alongside the catalogue's
            # raft/chunk/flake pieces — a snapped RC column reads the same
            # "flat marshmallow" way `_BOX_MATS["concrete_dusty"]` renders, so
            # it gets the SAME textured override (world-projected, so the
            # box's lack of UVs does not matter for the real MDL render — the
            # offline Blender preview now gets a planar `st` from
            # `_chipped_box`; see `_debris_look`'s comment).
            if key == "concrete_dusty" and kind == "column":
                mat = _debris_look(stage, parent, "concrete", mats)
            else:
                mat = _box_material_for(stage, parent, key, mats)

    chip = _chip_spec(kind, _chip_seed(tag, n, kind,
                                       *[float(q) for q in (size or (0, 0, 0))],
                                       *[float(q) for q in pos]))
    return _author_large_box(stage, parent, path, size, pos, rot, mat,
                             chip=chip)


# ---------------------------------------------------------------------------
# instance sets
# ---------------------------------------------------------------------------
def _safe_name(name):
    out = []
    for c in str(name):
        out.append(c if (c.isalnum() or c == "_") else "_")
    if not out or not (out[0].isalpha() or out[0] == "_"):
        out = ["p"] + out
    return "".join(out)


def _instancer_extent(positions, proto_index, protos, catalogue):
    from pxr import Gf
    if not len(positions):
        z = Gf.Vec3f(0, 0, 0)
        return [z, z]
    pos = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    sizes = []
    for name in protos:
        e = catalogue.get(name) or {}
        s = e.get("size") or (1.0, 1.0, 1.0)
        sizes.append(max(s) if s else 1.0)
    pad = 1.0
    if sizes and len(proto_index):
        idx = np.asarray(proto_index, dtype=np.int64)
        sizes_a = np.asarray(sizes, dtype=np.float64)
        idx = np.clip(idx, 0, len(sizes_a) - 1)
        pad = float(sizes_a[idx].max()) * 0.6
    lo = pos.min(axis=0) - pad
    hi = pos.max(axis=0) + pad
    return [Gf.Vec3f(*[float(v) for v in lo]), Gf.Vec3f(*[float(v) for v in hi])]


def _author_instancer(stage, parent, tag, set_name, inst, catalogue, asset_root, mats):
    from pxr import Sdf, UsdGeom, Vt

    path = "{0}/rubble_{1}_{2}".format(parent, tag, set_name)
    protos = list(inst.get("protos") or [])
    proto_index = list(inst.get("proto_index") or [])
    positions = list(inst.get("positions") or [])
    orientations = list(inst.get("orientations") or [])
    scales = list(inst.get("scales") or [])

    pi = UsdGeom.PointInstancer.Define(stage, Sdf.Path(path))
    proto_scope_path = Sdf.Path(path).AppendChild("Prototypes")
    scope = stage.DefinePrim(proto_scope_path, "Scope")
    UsdGeom.Imageable(scope).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    # `inst["look"]` (round-4 B contract): when one of `_MATERIAL_LOOKS`,
    # overrides EVERY prototype in this set with the SAME material,
    # regardless of what each catalogue asset's own `material`/`textured`
    # fields would otherwise pick — e.g. a URM chunk set tagged "brick" no
    # longer gets the generic dusty-concrete override every untextured piece
    # got before (round-4 v3 review: "URM chunks are concrete-dark").
    set_override = _material_for_look(stage, parent, inst.get("look"), mats)

    proto_targets = []
    for name in protos:
        ppath = proto_scope_path.AppendChild(_safe_name(name))
        prim = stage.DefinePrim(ppath, "Xform")
        url = _resolve_asset_url(name, catalogue, asset_root)
        if url:
            prim.GetReferences().AddReference(url)
        # bound ONCE on the shared prototype prim — every instance pointing
        # at this proto index inherits it, exactly like the reference does.
        override = (set_override if set_override is not None else
                    _debris_material_for(stage, parent, name, catalogue, asset_root, mats))
        if override is not None:
            _bind_override(stage, str(ppath), override)
        proto_targets.append(ppath)

    pi.CreatePrototypesRel().SetTargets(proto_targets)
    pi.CreateProtoIndicesAttr(Vt.IntArray([int(i) for i in proto_index]))
    pi.CreatePositionsAttr(Vt.Vec3fArray(
        [_gf_vec3f(p) for p in positions]))
    pi.CreateOrientationsAttr(Vt.QuathArray([_quath(q) for q in orientations]))
    pi.CreateScalesAttr(Vt.Vec3fArray(
        [_gf_vec3f((s, s, s)) for s in scales]))
    pi.CreateExtentAttr(_instancer_extent(positions, proto_index, protos, catalogue))
    return str(path)


def _gf_vec3f(v):
    from pxr import Gf
    x, y, z = v
    return Gf.Vec3f(float(x), float(y), float(z))


def _author_flat_instances(stage, parent, tag, set_name, inst, catalogue, asset_root, next_id, mats):
    from pxr import Gf, Sdf, UsdGeom

    protos = list(inst.get("protos") or [])
    proto_index = list(inst.get("proto_index") or [])
    positions = list(inst.get("positions") or [])
    orientations = list(inst.get("orientations") or [])
    scales = list(inst.get("scales") or [])

    # same per-set `look` override as `_author_instancer` — see its comment.
    set_override = _material_for_look(stage, parent, inst.get("look"), mats)

    out = []
    for i, pos in enumerate(positions):
        pidx = proto_index[i] if i < len(proto_index) else 0
        name = protos[pidx] if 0 <= pidx < len(protos) else None
        url = _resolve_asset_url(name, catalogue, asset_root) if name else None
        path = "{0}/rubble_{1}_{2}_{3:03d}".format(parent, tag, set_name, next_id())
        prim = stage.DefinePrim(Sdf.Path(path), "Xform")
        if url:
            prim.GetReferences().AddReference(url)
        override = (set_override if set_override is not None else
                    _debris_material_for(stage, parent, name, catalogue, asset_root, mats))
        if override is not None:
            _bind_override(stage, path, override)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        if i < len(orientations):
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(_quatf(orientations[i]))
        s = float(scales[i]) if i < len(scales) else 1.0
        xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))
        out.append(path)
    return out


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------
def author(stage, parent, plan, mats=None, tag="rubble", uid=None, asset_root=None,
           flatten_instances=False):
    """Write one `quake_rubble.plan_pile()` plan onto `stage` under `parent`.

    `flatten_instances=True` writes every instance as its own referencing
    Xform instead of a `UsdGeom.PointInstancer` — for renderers that do not
    import instancers (this module's own `tools/rubble_preview.py`, which
    renders through Blender's `bpy.ops.wm.usd_import`).

    Returns
    -------
    dict with the contract's keys: "mound", "apron" (path or None), "static"
    (mound + apron + every "large" path — what the plan's design table calls
    out as things a settling shell fragment should be able to land on),
    "instancers" (PointInstancer paths, or per-instance Xform paths when
    flattened), "large", "all" (every path this call authored/touched).
    """
    if mats is None:
        mats = {}
    asset_root = asset_root or ASSET_ROOT
    next_id = _make_uid(parent, uid)
    # Resolved lazily (only if some entry actually needs a catalogue lookup)
    # so a plan that only uses `prim_path` / authored-box "large" entries
    # never prints the "CATALOGUE unavailable" notice for no reason.
    catalogue = plan.get("catalogue")
    if catalogue is None:
        needs_catalogue = bool(plan.get("instances")) or any(
            e.get("asset") for e in (plan.get("large") or []))
        catalogue = _resolve_catalogue() if needs_catalogue else {}

    result = {"mound": None, "apron": None, "static": [], "instancers": [],
              "large": [], "all": []}

    mound = plan.get("mound")
    if mound:
        look = mound.get("look") or _DEFAULT_LOOK
        mpath = "{0}/rubble_{1}_mound".format(parent, tag)
        _author_heightfield(stage, mpath, mound, uv_scale=_uv_scale_for(look))
        _bind(stage, mpath, _rubble_look(stage, parent, look, mats, apron=False))
        result["mound"] = mpath
        result["all"].append(mpath)
    else:
        look = _DEFAULT_LOOK

    apron = plan.get("apron")
    if apron:
        apath = "{0}/rubble_{1}_apron".format(parent, tag)
        alook = apron.get("look")
        # round-4 B contract: `apron["look"]` is now one of `_MATERIAL_LOOKS`
        # ("dust" — a thin dust skirt, FLAT, no rubble map at all; round-4
        # v3 review: the textured apron read as a paved plaza rectangle).
        # `_material_for_look` returns `None` for the OLD "urm"/"rc"
        # construction-type value (or an absent key), so a pre-round-4 plan
        # (this module's own older tests) still gets the textured apron.
        mat = _material_for_look(stage, parent, alook, mats)
        if mat is not None:
            _author_heightfield(stage, apath, apron, uv_scale=_uv_scale_for(look))
            _bind(stage, apath, mat)
        else:
            ctype = alook or look
            _author_heightfield(stage, apath, apron, uv_scale=_uv_scale_for(ctype))
            _bind(stage, apath, _rubble_look(stage, parent, ctype, mats, apron=True))
        result["apron"] = apath
        result["all"].append(apath)

    for set_name, inst in (plan.get("instances") or {}).items():
        if not inst or not inst.get("positions"):
            continue
        if flatten_instances:
            paths = _author_flat_instances(stage, parent, tag, set_name, inst,
                                           catalogue, asset_root, next_id, mats)
            result["instancers"].extend(paths)
            result["all"].extend(paths)
        else:
            ipath = _author_instancer(stage, parent, tag, set_name, inst,
                                      catalogue, asset_root, mats)
            result["instancers"].append(ipath)
            result["all"].append(ipath)

    n_box, n_panel = 0, 0
    extra_static = []          # edge chunks `_dress_laid_panel` authors as a
                                # SIDE EFFECT of a "panel" entry — `_author_large`
                                # itself still returns one path per entry (its
                                # own re-laid prim), so this is the only route
                                # they reach `result["all"]`/`["static"]`
    for entry in (plan.get("large") or []):
        if entry.get("prim_path"):
            n_panel += 1
        elif not entry.get("asset"):
            n_box += 1
        lpath = _author_large(stage, parent, tag, entry, mats, catalogue,
                              asset_root, next_id(), extra_static=extra_static)
        if lpath:
            result["large"].append(lpath)
            result["all"].append(lpath)
    if extra_static:
        result["all"].extend(extra_static)

    # ROUND-5 FOLLOW-UP PROOF LINES. There was "no positive log evidence
    # that chips fire during a real bake" — one line per population per
    # building, not per piece, so a bake log can be grepped for `[chip]` and
    # show the chip/dress passes actually ran (or, just as usefully, that
    # they were off).
    if n_box:
        from . import fracture
        vtk_on = fracture.chips_enabled()
        print("[chip] rubble large boxes ({0}): {1} chipped, {2} "
              "passed-through (vtk={3})".format(
                  tag, n_box if vtk_on else 0, 0 if vtk_on else n_box, vtk_on))
    if n_panel:
        dress_on = _panel_dress_enabled()
        print("[chip] laid panels ({0}): {1} dressed (sink+tilt+edge-chunks), "
              "{2} passed-through (LAY_PANEL_DRESS={3}), {4} edge chunk(s)"
              .format(tag, n_panel if dress_on else 0,
                      0 if dress_on else n_panel, dress_on, len(extra_static)))

    result["static"] = ([p for p in (result["mound"], result["apron"]) if p]
                        + list(result["large"]) + extra_static)
    return result
