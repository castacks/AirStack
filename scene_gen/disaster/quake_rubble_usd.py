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
_LOOK = {
    "urm": {"stem": "rubble_urm_B", "mound": (0.60, 0.53, 0.46), "apron": (0.74, 0.70, 0.63), "desat": 0.20},
    "rc":  {"stem": "rubble_rc_B",  "mound": (0.55, 0.54, 0.52), "apron": (0.70, 0.69, 0.66), "desat": 0.20},
}
_DEFAULT_LOOK = "rc"

# ~0.28-0.32 repeats/metre (damage._pbr's `scale_uv` IS repeats-per-metre,
# smaller = bigger features) — the brief's "0.25-0.35 per metre" band. Also
# used to author `primvars:st` on the mound/apron mesh itself (world x, y
# times this same scale) so a Blender/Hydra preview that cannot see the MDL
# `project_uvw` triplanar projection still gets a matching, correctly-scaled
# UV to read the real texture through (see `_add_preview_fallback`).
_UV_SCALE = {"urm": 0.28, "rc": 0.32}


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
    """`disaster.quake_rubble.CATALOGUE`, imported lazily so this module
    never hard-fails if the planner is not there yet (round-4 parallel
    build) or a caller supplies its own `plan["catalogue"]`."""
    try:
        from . import quake_rubble
        return quake_rubble.CATALOGUE
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
    tint = _TEXTURED_DUST_TINT
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


def _box(stage, path, sx, sy, sz, mat=None):
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
    (`planks.py`'s reason for the same choice)."""
    from pxr import Gf, Sdf, UsdGeom, Vt

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


def _author_large_box(stage, parent, path, size, pos, rot_deg, mat):
    """Same `pos`-is-final convention as `_author_large_asset` — `_box`
    already puts the box's own local origin at its bottom-centre, so no
    `bury` offset is applied here either."""
    from pxr import Gf, UsdGeom
    sx, sy, sz = (size or (1.0, 1.0, 1.0))
    m = _box(stage, path, float(sx), float(sy), float(sz), mat=mat)
    xf = UsdGeom.Xformable(m)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(float(rot_deg[0]), float(rot_deg[1]), float(rot_deg[2])))
    return path


def _lay_existing(stage, prim_path, pos, rot_deg):
    """Re-lay an EXISTING kit/sliced prim on the pile: translate its own
    pivot (the centre of its CURRENT world bound) to `pos`, rotating
    `rot_deg` about that pivot, without re-referencing it.

    Done in world space and then converted back to the prim's own PARENT
    frame, so this is correct wherever the prim happens to sit in the
    stage's existing hierarchy (a building's own Xform nest, typically) —
    not just for a prim that is already a direct child of `parent`.
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
    return str(prim_path)


def _author_large(stage, parent, tag, entry, mats, catalogue, asset_root, n):
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
        return _lay_existing(stage, prim_path, pos, rot)

    look_override = _material_for_look(stage, parent, entry.get("look"), mats)

    asset = entry.get("asset")
    if asset:
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

    if look_override is not None:
        mat = look_override
    else:
        key = _box_key(entry)
        # "column" stubs are the one authored (asset=None) kind the round-4
        # v2 review called out by name alongside the catalogue's raft/chunk/
        # flake pieces — a snapped RC column reads the same "flat marshmallow"
        # way `_BOX_MATS["concrete_dusty"]` renders, so it gets the SAME
        # textured override (world-projected, so the box's lack of UVs does
        # not matter for the real MDL render — only the offline Blender
        # preview loses the map on this one shape; see `_debris_look`'s
        # comment).
        if key == "concrete_dusty" and str(entry.get("kind") or "").lower() == "column":
            mat = _debris_look(stage, parent, "concrete", mats)
        else:
            mat = _box_material_for(stage, parent, key, mats)
    return _author_large_box(stage, parent, path, entry.get("size"), pos, rot, mat)


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

    for entry in (plan.get("large") or []):
        lpath = _author_large(stage, parent, tag, entry, mats, catalogue,
                              asset_root, next_id())
        if lpath:
            result["large"].append(lpath)
            result["all"].append(lpath)

    result["static"] = [p for p in (result["mound"], result["apron"]) if p] + list(result["large"])
    return result
