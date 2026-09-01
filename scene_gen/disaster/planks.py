"""planks stage — the sawn-timber debris field a wind event leaves behind.

WHY THIS IS AUTHORED GEOMETRY AND NOT FRACTURE
----------------------------------------------
`disaster.fracture` cuts real modules into real fragments and `disaster.settle`
arranges them with PhysX. That is the right tool for a house coming apart, and
it is the wrong one for the debris FIELD, for three reasons that all point the
same way:

  * IT DOES NOT SCALE. Fracture is CPU trimesh and settle is GPU PhysX, and
    together they are the entire ~27-minute cost of the 250 m mini block. A
    tornado track across a plate is tens of thousands of loose boards. The
    whole point of the bake-and-assemble path is that the expensive work is
    done once per ARCHETYPE, and a debris field is per-PLACE by definition:
    it is where the wind went, not what the house was made of.
  * WHAT IT PRODUCES IS THE WRONG SHAPE ANYWAY. A Voronoi cell is a chunk. A
    building blown apart sheds BOARDS — studs, joists, sheathing, siding,
    decking — because that is the shape the mill cut them and the shape the
    fasteners let go at. `tornado.jpeg` is a field of pale rectangles.
  * A BOX NEEDS NO SOLVER. A board lying flat on open ground is fully
    described by a position, a yaw and a couple of degrees of tilt. There is
    nothing for physics to discover, and authoring it directly means the
    field can be RETUNED WITHOUT REBAKING ANYTHING — which is most of what
    iterating on this scene consists of.

So: `scatter_*` decides where every piece goes, `build` authors them, and the
two are pure geometry with no stage-side cost beyond the meshes themselves.

ONE MESH PER PATCH, NOT ONE PER BOARD
-------------------------------------
Same argument `ground.build_overlay` makes for its bands. A wrecked house
sheds a few hundred pieces and a plate holds hundreds of wrecked houses; at
one prim each that is a six-figure prim count for geometry that never moves,
never animates and shares one material. `build` merges every piece in a patch
into a single `UsdGeom.Mesh` — 8 points and 6 faces a board — so the whole
field costs a few hundred prims. It is also why the pieces are BOXES rather
than referenced assets: an instanced reference cannot be merged, and merging
is worth more here than instancing is.

NO CHAR, ANYWHERE
-----------------
This is bare structural timber that was inside a wall an hour ago. It takes
one plain wood material (`wood_material`) and nothing composites soot onto it.
`Ash_Planks` is the pack, chosen for its VALUE: fresh framing lumber and the
back of siding are pale, which is why a tornado debris field photographs
light against green grass while a burn scar photographs dark. Getting that
backwards is the single most visible way to make this read as the wrong
disaster.
"""

import math

# The lumber. Ash rather than oak because a debris field is mostly the INSIDE
# of the building — framing, sheathing, the unpainted back of everything — and
# that is pale. `damage._pbr` only carries a diffuse map, so this module
# authors its own OmniPBR to keep the normal and ORM that make a board read as
# a board rather than as a coloured rectangle.
WOOD_BASE = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
             "Wood/Ash_Planks/Ash_Planks_BaseColor.png")
WOOD_NORMAL = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
               "Wood/Ash_Planks/Ash_Planks_Normal.png")
WOOD_ORM = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
            "Wood/Ash_Planks/Ash_Planks_ORM.png")


# THE STOCK LIST, and it is a real cutting list rather than a size range.
#
# A uniform draw over "length 0.5-4 m, width 0.05-0.6 m" produces a cloud of
# arbitrary rectangles, and arbitrary is the one thing building material is
# not: a house is made of a handful of standard sections, and the debris from
# one is those sections in broken lengths. Naming them also makes the mix
# tunable by what it IS — more sheathing, fewer studs — instead of by moving
# an abstract number.
#
# (weight, length_range_m, width_range_m, thickness_range_m)
STOCK = {
    # 2x4 / 2x6 framing. The most numerous thing in a stick-built house and
    # the most numerous thing in its debris.
    "stud":      (0.26, (0.7, 3.4), (0.09, 0.15), (0.038, 0.048)),
    # Rafters, joists, headers — longer and deeper, and what makes a pile
    # read as structure rather than as kindling.
    "joist":     (0.11, (1.8, 5.0), (0.18, 0.30), (0.038, 0.058)),
    # OSB / plywood sheathing, in broken sheets. THE BIG PALE RECTANGLES.
    # These are what you actually see from altitude — a stud is a stick at
    # 60 m, a half-sheet of sheathing is an object.
    "sheathing": (0.19, (0.9, 2.4), (0.55, 1.22), (0.011, 0.019)),
    # Fence boards, trim, interior decking. Long, thin, and they scatter
    # furthest because they have the most area per unit mass. BARE TIMBER —
    # the painted exterior cladding is `siding`.
    "board":     (0.11, (1.0, 4.2), (0.14, 0.24), (0.016, 0.026)),
    # THE OUTSIDE OF THE WALL, in runs and in broken sheets, WITH ITS FINISH
    # STILL ON IT. Added 2026-08-27 on review: the field had no exterior
    # cladding class at all, so every piece of a levelled house came out the
    # same sawn-timber colour and a destroyed block read as a lumber yard.
    # A debris photograph is not one colour — it is a mat of house-coloured
    # cladding and grey roof slab with bare framing showing between them, and
    # the cladding is the largest single area of any wrecked house.
    # `scatter_from_wreck(skins=...)` gives this class the WRECKED HOUSE'S OWN
    # wall material, so the debris off a white timber cottage is white timber
    # and the debris off a red brick house is brick.
    "siding":    (0.18, (0.9, 3.0), (0.20, 0.95), (0.018, 0.030)),
    # Roof decking with its covering still on, in slabs. The one class that is
    # genuinely a slab rather than a board, and — like `siding` — it takes the
    # wrecked house's own ROOF material. Raised from 0.06: a pitched roof is
    # the single largest continuous surface on a house and it is the first
    # thing to leave, so a track with almost no roof slab in it is wrong, and
    # grey shingle against pale timber is most of what breaks the field up.
    "deck":      (0.15, (0.8, 2.2), (0.6, 1.5), (0.05, 0.09)),
}

# Per-class colour tint on the shared map, so a field is not one uniform
# lumber-yard colour. Values, not hues: fresh framing is pale, sheathing is
# a shade browner, decking with shingles on it is dark.
_TINT = {
    "stud":      (1.00, 0.98, 0.94),
    "joist":     (0.93, 0.90, 0.86),
    "sheathing": (0.86, 0.79, 0.68),
    "board":     (1.00, 1.00, 0.98),
    # THE FALLBACK ONLY. `siding` and `deck` normally carry the wrecked
    # house's own wall and roof materials (`scatter_from_wreck(skins=...)`,
    # `build(skin_mats=...)`); these tints are what they get when the caller
    # supplies none — the corridor scatter, which came from a house nobody can
    # name, and any bench that has no palette to hand.
    #
    # RE-TUNED, DEBRIS D6 review (hurricane raft-field green-cast audit,
    # 2026-09-01): both were near-neutral pale (6.25% and 9.1% channel
    # spread respectively) — UNDER `test_e_tint_targets_are_saturated_not_
    # neutral`'s own 10% floor for the sibling `washaway._RAFT_TINT` table,
    # the exact "almost no colour of its own, so ambient tint reads through
    # it" shape that table's docstring diagnoses at length for `wall`/
    # `panel`/`fence`/`siding_strip`/`timber`. These two are the ONLY
    # fallback this module reaches for when a caller (a bare corridor
    # scatter, a tornado bench with no palette, or a hurricane piece beyond
    # `washaway._HOUSE_SKIN_MATCH_R_M` with no `skin_pool` supplied) has no
    # house to match — saturating them the same way closes the same gap for
    # every OTHER caller of this table, not just the hurricane raft field.
    "siding":    (0.74, 0.68, 0.58),
    "deck":      (0.38, 0.32, 0.26),
}

# THE TWO CLASSES THAT CARRY A HOUSE'S COLOUR. Everything else in the field is
# the inside of a wall and is bare timber wherever it came from.
SKINNED = ("siding", "deck")


def _weighted(rng, classes=None):
    """Draw a stock class by weight.

    `classes`, ADDED 2026-08-31 for the hurricane land-debris pass
    (`disaster.washaway.land_debris_specs`): an optional iterable restricting
    the draw to a NAMED SUBSET of `STOCK` (weights renormalised within it),
    so a house that only lost its shingles can shed `deck`/`siding` without
    a `stud` or `joist` ever coming up. `None` (the default) is the ORIGINAL,
    UNRESTRICTED draw over every class — every existing caller (the tornado
    plank field) gets byte-identical behaviour, because this is purely
    additive over the plain `_weighted(rng)` contract.
    """
    pool = {k: v for k, v in STOCK.items() if classes is None or k in classes}
    if not pool:
        pool = dict(STOCK)
    r = rng.random() * sum(v[0] for v in pool.values())
    for name, spec in pool.items():
        r -= spec[0]
        if r <= 0.0:
            return name
    return next(iter(pool))


# ---------------------------------------------------------------------------
# material
# ---------------------------------------------------------------------------

def wood_material(stage, path, tile_m=1.1, tint=(1.0, 1.0, 1.0),
                  roughness=0.78):
    """A plain sawn-timber OmniPBR, projected from WORLD coordinates.

    TRIPLANAR IS NOT OPTIONAL HERE, for the reason `damage._pbr` records: the
    boards are authored meshes with no UVs at all, so a UV-space material
    falls back to a per-face default and the map repeats inside every single
    triangle. World projection makes `texture_scale` metric — repeats per
    metre, so SMALLER means BIGGER features — and `tile_m` is its reciprocal.

    ~1.1 m a tile against a map that is itself a run of ~15 cm boards puts
    about seven board-widths across a tile, so a 2.4 m sheet of sheathing
    carries a plausible grain and a 0.10 m stud gets one board's worth of it
    rather than a compressed atlas.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg

    existing = UsdShade.Material.Get(stage, path)
    if existing:
        return existing

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_BASE, "")))
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_NORMAL, "")))
    sh.CreateInput("enable_ORM_texture", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("ORM_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_ORM, "")))
    # `diffuse_color_constant` DOES NOT MULTIPLY THE MAP. That belief is what
    # this comment used to assert, and it is wrong: in OmniPBR a bound
    # `diffuse_texture` REPLACES `diffuse_color_constant` outright, and the
    # surviving multiply slot is `diffuse_tint` (the same trap `damage._pbr`
    # and `surge._make` both document).
    #
    # So every `tint=` this function has ever been passed was inert. The
    # evidence was recorded twice without the cause being found:
    # `quake_flow` tried a cedar tank at 0.50 and then at 0.085 and got
    # PIXEL-IDENTICAL renders, concluded "the tint does not reach OmniPBR's
    # diffuse when a texture is bound", and gave up on a textured tank
    # entirely; `washaway.build_rafts` hit the same wall and worked around it
    # by reaching past this function to set `diffuse_tint` on the shader
    # itself. Two independent workarounds for one unfixed primitive.
    #
    # A large parameter change producing an identical render means the knob
    # is not connected — that is the whole diagnosis, and it applies here.
    #
    # `diffuse_color_constant` is still set: it is what OmniPBR uses if the
    # texture ever fails to resolve, and an untinted fallback would be worse.
    # `_TINT`, `quake_flow`'s 0.50 and the tornado skins are all authored as
    # MULTIPLIERS (near 1.0 = unchanged), which is exactly what `diffuse_tint`
    # wants, so they need no mean-normalisation — unlike `surge._make`, whose
    # `rgb` is an absolute albedo and must be divided by the texture's mean.
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("diffuse_tint",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / max(1e-6, float(tile_m))
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def skin_material(stage, path, texture, tint=(1.0, 1.0, 1.0), tile_m=1.05,
                  roughness=0.86):
    """A house's own cladding or roof map, projected from WORLD coordinates.

    THE DEBRIS WEARS WHAT THE HOUSE WORE, and it has to be projected rather
    than UV-mapped for the same reason `wood_material` is: these boards are
    authored boxes with no `st` at all, so a UV-space material — which is what
    every kit material is — renders black on them. `detail.modular_house.
    palette_texture` pulls the base-colour map and the palette's albedo
    multiplier out of the kit material; this puts them on a triplanar OmniPBR.

    NO NORMAL OR ORM MAP, deliberately. The kit's maps are authored in the
    same UV space and would be as wrong as the albedo; a shingle slab lying in
    a street at 40 m is read by its COLOUR and its rectangle, and a flat
    roughness costs nothing at that range.

    `tile_m` is a metre per texture repeat, so SMALLER means BIGGER features.
    1.05 m puts about a course and a half of shingle, or two brick courses,
    across a tile — near enough to the real spacing that a 1.2 m slab of roof
    deck reads as roof deck rather than as a photograph of one.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg

    existing = UsdShade.Material.Get(stage, path)
    if existing:
        return existing
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    if texture:
        # `sg._join_asset_root` — NOT the raw string. This was the site of
        # the "References an asset that can not be found:
        # 'airstack://.../Soil_Mud/T_pjuph20_2K_B.png'" Hydra error that
        # persisted through the hurricane water/silt work: `washaway.
        # apply_washaway`'s "wet"/"scoured" levels call this function with
        # `surge.SILT_TEXTURE` — an unexpanded `airstack://` string — and
        # every OTHER texture-binding site in this codebase (`surge.py`,
        # `ground.py`, `damage._pbr`) already runs its texture argument
        # through `_join_asset_root` before handing it to `Sdf.AssetPath`.
        # This one bound the scheme straight through, so the reference
        # reached Hydra unexpanded and the resolver rejected it outright.
        # `_join_asset_root` is a no-op passthrough for an already-absolute
        # path or a real URL (omniverse://, file://, ...), so this is safe
        # for every existing caller regardless of what kind of path it
        # passes.
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(sg._join_asset_root(str(texture), "")))
        # `diffuse_tint`, ADDED (DEBRIS D6 review, 2026-09-01) -- THE SAME BUG
        # `wood_material` (above) documents at length and was fixed for: a
        # bound `diffuse_texture` REPLACES `diffuse_color_constant` outright
        # in OmniPBR, so setting only `diffuse_color_constant` here meant
        # every `tint` this function was ever passed was INERT whenever
        # `texture` was truthy -- which is every real caller
        # (`land_debris_specs`/`raft_specs`'s own "# LAND DEBRIS"/"# DEBRIS
        # RAFTS" blocks always pass one). Concretely: `detail.modular_house.
        # palette_texture`'s whole reason for returning a tint alongside a
        # texture path is that several styles' roofs (`shingle_slate`/
        # `shingle_grey`/`shingle_brown`) share ONE base texture (`_SHINGLE`)
        # differentiated ONLY by that tint (`MATERIAL_SOURCE`'s own blue-
        # grey/weathered-grey/brown multipliers) -- with `diffuse_tint` never
        # authored, every one of those styles' "deck"-skinned debris rendered
        # as the SAME untinted charcoal shingle map regardless of which house
        # it matched, which is exactly the "still looks the same debris
        # everywhere" defect the nearest-house-matching feature this fix
        # ships alongside is supposed to cure. `tint` is already a near-1.0
        # MULTIPLIER (`palette_texture`'s own docstring: "the palette's own
        # albedo multiplier folded into an RGB tint"), the same convention
        # `wood_material`'s `tint` argument uses, so it goes on `diffuse_tint`
        # directly -- no texture-mean normalisation needed (that is only for
        # an ABSOLUTE target colour, e.g. `washaway.build_rafts`'s own
        # `_RAFT_TINT` patch).
        #
        # DELIBERATELY GUARDED INSIDE `if texture:`, NOT AUTHORED
        # UNCONDITIONALLY THE WAY `wood_material` DOES IT: that function
        # always binds a texture, so `diffuse_tint`/`diffuse_color_constant`
        # being identical is harmless there (the constant is dead the moment
        # a texture exists). This function's `texture` is OPTIONAL --
        # `_slab_material`/`apply_washaway`'s mud-ring/yard-patch callers
        # sometimes pass `None` -- and it is NOT verified that OmniPBR leaves
        # `diffuse_color_constant` un-multiplied by a `diffuse_tint` that
        # still has SOME authored value when no texture is bound (the
        # untextured branch is exactly the one this codebase's own offline
        # predictor, `hurricane_layout_png.omnipbr_predict`, does NOT model
        # a tint multiply for). Authoring `diffuse_tint` only when a texture
        # is actually present keeps every existing no-texture caller
        # (`test_skin_material_with_no_texture_authors_none`) byte-identical.
        sh.CreateInput("diffuse_tint",
                      Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / max(1e-6, float(tile_m))
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def materials(stage, parent_path, tile_m=1.1):
    """One material per stock class, under `<parent_path>/PlankLooks`."""
    return {name: wood_material(stage,
                                "{0}/PlankLooks/{1}".format(parent_path, name),
                                tile_m=tile_m, tint=_TINT.get(name,
                                                              (1.0, 1.0, 1.0)))
            for name in STOCK}


# ---------------------------------------------------------------------------
# where the pieces go
# ---------------------------------------------------------------------------

def _piece(rng, klass=None, scale=1.0, classes=None):
    """One board's dimensions and its class. `classes` -- see `_weighted`."""
    k = klass or _weighted(rng, classes=classes)
    _w, ln, wd, th = STOCK[k]
    return (k,
            rng.uniform(*ln) * scale,
            rng.uniform(*wd) * scale,
            rng.uniform(*th))


# HOW FAR A BOARD MAY BED INTO WHAT IT LANDS ON. Debris does not rest on a
# geometric plane: it lands on turf, on mud, on the fringe of the pile, and it
# sinks a little. Two centimetres is a blade of grass and it is what lets a
# board that is a degree or two off level sit DOWN rather than be levered up
# onto one corner — see `_lay`.
_BED_M = 0.02


def _lay(x, y, k, ln, wd, th, heading, rng, ground_z=0.0, tilt_p=0.05):
    """Pose one board on the ground. Returns the spec dict `build` consumes.

    FLAT MEANS FLAT, AND IT DID NOT (fixed 2026-08-27). This function seated
    every board so that its LOWEST ROTATED CORNER touched `ground_z`, which
    means no board in the field ever lay on its face: each one was levered up
    onto a corner by however much its own tilt demanded. Measured over 200,000
    draws against the real `STOCK` table and weights, the board's CENTRE
    finished at

        p50 0.079 m   p75 0.122   p90 0.373   p95 0.534   p99 0.819   max 1.42

    above nominal ground, and its high corner at p50 0.150 m / p90 0.750 m.
    On this preset's 758 boards that is 245 (32%) more than 10 cm up and 113
    (15%) more than 25 cm — every one of them with clear air underneath.

    AND THE SUN MAKES IT UNMISSABLE. The scene borrows its sky from
    `RetroNeighborhood.stage.usd`, whose rig is at 16.36 h with the sun at
    **24.4 degrees of elevation** — 2.21 m of shadow displacement per metre of
    height. Four centimetres of float throws a 9 cm shadow gap; a p90 board
    throws 0.82 m and a tilted joist 1.17 m. Measured on a lawn patch of the
    100 m render, twelve boards had twelve FULLY DETACHED shadows at 0.57 to
    2.42 m of offset. That is the "floating debris" report, and it was not a
    physics failure — it was this arithmetic.

    So a board is now seated on its FACE and allowed to bed `_BED_M` into what
    it landed on: the sink is capped so a steeply tilted piece still rests on
    its corner, and the flat majority stops hovering.

    MOSTLY FLAT, AND THAT IS THE OBSERVATION. Debris photographs as a mat, not
    as a bramble: a board that lands on open ground lies down, and only the
    ones that came to rest ON something else are at an angle. `tilt_p` is the
    share that did — and it was 0.22 everywhere, which put ~167 boards a plate
    at 34 degrees of pitch with NOTHING AUTHORED UNDER THEM. A tilted board
    over open lawn is not "came to rest on something else", it is a floater.
    The default is now 0.05 and the callers raise it only where the mat is
    genuinely deep enough to lean on (`scatter_from_wreck` does this by
    distance from the slab).

    ALIGNED, WEAKLY, TO THE FLOW. A long thin object in a moving fluid ends up
    lying across the flow more often than along it, and survey photographs of
    debris mats do show a grain. `heading + 90` with a wide spread reproduces
    it without asserting more than the picture supports.
    """
    if rng.random() < tilt_p:
        # AND A TILTED BOARD IS STILL LEANING ON SOMETHING, so how far its top
        # may rise is bounded by how deep the pile it is leaning on actually
        # is. A flat 34 degrees applied to a 5 m joist stands it 2.8 m in the
        # air — a flagpole, not debris. Cap the RISE at ~1.2 m, which is about
        # the depth of a levelled house's pile, and let the angle fall out of
        # the piece's own length.
        _cap = math.degrees(math.asin(min(1.0, 1.2 / max(1.2, ln))))
        pitch = rng.uniform(-1.0, 1.0) * min(34.0, _cap)
        roll = rng.uniform(-1.0, 1.0) * min(30.0, _cap)
    else:
        # TIGHTER THAN IT WAS (+-4 / +-6). On a 4.2 m board four degrees of
        # pitch lifts the far end 0.29 m, so even the "flat" band was a
        # visible prop. Scaled by length, so a 0.7 m stud keeps a couple of
        # degrees of character and a 5 m joist lies down.
        _s = 2.0 / max(2.0, ln)
        pitch = rng.uniform(-4.0, 4.0) * _s
        roll = rng.uniform(-6.0, 6.0) * _s
    yaw = heading + 90.0 + rng.gauss(0.0, 46.0)
    # THE TRUE VERTICAL HALF-EXTENT of the rotated box, matching `_box`'s own
    # R = Rz . Ry . Rx. The width term was `|w sin r|` and should carry the
    # pitch cosine with it; the error was small (p99 1.8 cm, max 5.7 cm) and
    # always in the float direction, so it is fixed here rather than kept.
    p, r = math.radians(pitch), math.radians(roll)
    half_h = 0.5 * (abs(th * math.cos(p) * math.cos(r))
                    + abs(wd * math.cos(p) * math.sin(r))
                    + abs(ln * math.sin(p)))
    # ...AND THEN LET IT BED IN. `half_h` is the height of the centre above
    # the lowest corner, so seating at `ground_z + half_h` is exactly the
    # lever-it-up behaviour. Sink by up to `_BED_M`, but never past the point
    # where the board's own centre-plane thickness is resting on the ground —
    # otherwise a thin sheet would vanish into the turf.
    flat_h = 0.5 * abs(th * math.cos(p) * math.cos(r))
    z = ground_z + half_h - min(_BED_M, max(0.0, half_h - flat_h))
    return {"x": x, "y": y, "z": z,
            "l": ln, "w": wd, "t": th,
            "yaw": yaw, "pitch": pitch, "roll": roll, "class": k}


def scatter_from_wreck(cx, cy, footprint_m, intensity, heading_deg, reach_m,
                       rng, n_pieces=140, ground_z=0.0, scale=1.0, skins=None,
                       classes=None, base_frac=0.30, base_s_scale=0.34,
                       base_t_scale=0.40, tail_pow=1.9, tail_lateral_base=0.30,
                       tail_lateral_growth=0.26):
    """The debris trail off ONE wrecked building. Returns a list of specs.

    `classes`, ADDED 2026-08-31 for `disaster.washaway.land_debris_specs`:
    an optional iterable of `STOCK` names restricting every piece this call
    draws (see `_weighted`'s docstring) -- a hurricane house that only lost
    its shingles sheds `deck`/`siding`, not framing. `None` keeps the
    original unrestricted draw, unchanged for the tornado field.

    `base_frac`/`base_s_scale`/`base_t_scale`/`tail_pow`/`tail_lateral_base`/
    `tail_lateral_growth`, ADDED for the hurricane DEBRIS-stream "ring, not a
    comet" fix: every one of these defaults to the ORIGINAL hardcoded
    constant this function always used (0.30, 0.34, 0.40, 1.9, 0.30, 0.26),
    so an unmodified caller (the tornado plank field, `scatter_over_region`)
    draws a BYTE-IDENTICAL sequence -- same coin, same formula, just named
    instead of inlined. `disaster.washaway.land_debris_specs` is the one
    caller that overrides them: a hurricane house sheds roof/wall material
    into open yard and the review found the result reading as a symmetric
    halo around the house rather than a directional trail agreeing with the
    wind, because the median tail draw (`reach * u**1.9` is concentrated
    near `u=0`, i.e. near the building) landed inside the base cloud's own
    spread. Lowering `tail_pow` biases `u**p` toward 1 (toward `reach`
    rather than toward the building) and tightening the two lateral
    coefficients narrows the cone the tail fans into as it travels — see
    that function's own call for the tuned values and the offline plot that
    verifies the resulting cone fraction.

    `skins` IS WHAT THIS HOUSE WAS MADE OF: `{"siding": <material name>,
    "deck": <material name>}`, from `detail.modular_house.palette_skins` on the
    wrecked house's own palette. Every `siding` and `deck` piece in the trail
    is stamped with it and `build` binds it, so the cladding lying in the road
    is the cladding that was on THAT house and the roof slab is its roof. The
    rest of the classes are framing and sheathing — the inside of a wall — and
    are bare timber whichever house they came from.

    Omit it and the pieces fall back to `_TINT`, which is what the corridor
    scatter does: material that came from somewhere nobody can name.

    THE SHAPE IS A COMET, and it is the single most recognisable feature of a
    tornado track from the air: a dense mat on and around the slab, thinning
    into a tail that runs downtrack and FANS as it goes. Three draws make it:

      * `s` — distance downtrack, `reach * u ** 1.9`, so most pieces stay
        near the building and a minority carry a long way. A linear draw
        gives an even smear, which reads as a painted stripe rather than as
        material that was thrown.
      * `t` — lateral offset, gaussian with a width that GROWS with `s`. A
        constant width is a corridor, and a corridor is what a bulldozer
        leaves. The spreading is what says the debris was airborne.
      * a base cloud inside the footprint itself, because the slab is never
        actually clean — even at EF5 there is a rim of sill plate, plumbing
        and the bottom course of everything.

    `intensity` scales the count and the reach together: a house at the edge
    of the path loses its shingles into the next garden, one on the centreline
    is found in the next field.
    """
    i = max(0.0, min(1.0, float(intensity)))
    th = math.radians(float(heading_deg))
    ux, uy = math.cos(th), math.sin(th)
    vx, vy = -math.sin(th), math.cos(th)
    fp = max(2.0, float(footprint_m))
    reach = float(reach_m) * (0.35 + 0.65 * i)
    n = max(0, int(round(n_pieces * (0.25 + 0.75 * i))))

    out = []
    for _ in range(n):
        if rng.random() < base_frac:
            # The mat on and immediately around the slab.
            s = rng.gauss(0.0, fp * base_s_scale)
            t = rng.gauss(0.0, fp * base_t_scale)
        else:
            s = reach * (rng.random() ** tail_pow)
            t = rng.gauss(0.0, fp * tail_lateral_base + s * tail_lateral_growth)
        k, ln, wd, tk = _piece(rng, scale=scale, classes=classes)
        # TILT ONLY WHERE THERE IS SOMETHING TO LEAN ON. A board at 34 degrees
        # is resting on other debris, so the share has to follow the DEPTH of
        # the mat and not be a constant over the whole comet. Inside about a
        # footprint of the slab the pile is metres deep and a third of the
        # pieces are propped on each other; out in the tail a board lands on
        # grass and lies down, and a tilted one there is the floater the
        # 2026-08-27 review was looking at.
        _near = math.hypot(ux * s + vx * t, uy * s + vy * t) / fp
        _tp = 0.34 if _near < 0.75 else (0.12 if _near < 1.4 else 0.02)
        sp = _lay(cx + ux * s + vx * t, cy + uy * s + vy * t,
                  k, ln, wd, tk, float(heading_deg), rng, ground_z=ground_z,
                  tilt_p=_tp)
        if skins and k in SKINNED and skins.get(k):
            sp["skin"] = skins[k]
        out.append(sp)
    return out


def scatter_over_region(region, intensity_at, heading_deg, rng,
                        per_100m2=0.55, cell_m=12.0, ground_z=0.0,
                        min_intensity=0.12, scale=1.0, classes=None):
    """Loose timber strewn across the CORRIDOR, not just on the lots.

    THE TRACK ITSELF IS COVERED, and this is what a per-building scatter
    cannot produce. In `tornado.jpeg` the debris does not stop at the property
    lines: it runs continuously across roads, verges, the field beyond and
    everything in between, because what is lying there came from somewhere
    else. A field assembled only from per-house trails leaves clean green
    gaps between the lots and the corridor stops reading as one event.

    Sampled on a `cell_m` lattice with the density scaled by the local
    intensity, so the corridor's own gradient carries into the debris: dense
    on the centreline, thinning to nothing at the path edge, with no separate
    edge to tune.
    """
    x0, y0, x1, y1 = region
    nx = max(1, int(round((x1 - x0) / float(cell_m))))
    ny = max(1, int(round((y1 - y0) / float(cell_m))))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area = dx * dy / 100.0
    out = []
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            i = float(intensity_at(ax + dx * 0.5, ay + dy * 0.5))
            if i < min_intensity:
                continue
            # Expected count, then a Poisson-ish draw so the field is not a
            # visible lattice of equal clumps.
            lam = per_100m2 * area * (i ** 1.4)
            n = int(lam) + (1 if rng.random() < (lam - int(lam)) else 0)
            for _ in range(n):
                k, ln, wd, tk = _piece(rng, scale=scale, classes=classes)
                out.append(_lay(ax + rng.random() * dx, ay + rng.random() * dy,
                                k, ln, wd, tk, float(heading_deg), rng,
                                ground_z=ground_z))
    return out


def clip_to_region(specs, region, verbose=True):
    """Drop boards whose GEOMETRY leaves the plate. Returns `(kept, n_dropped)`.

    `scatter_from_wreck` throws a comet tail `reach_m` downtrack of a wreck and
    knows nothing about the region — which is correct for the model (a tornado
    does not stop at a property line, and it does not stop at ours either) and
    wrong for the picture. `suburb_scene.apply_ground` lays its base sheet over
    exactly `region` and nothing beyond it, so a board past the boundary hangs
    over the void with no ground under it and reads as FLOATING, which is what
    the 100 m plate's first render showed at its downtrack corner.

    Measured on that plate (46 m track, `throw_m` 16, six wrecks): 11 boards of
    696, overshooting by up to 4.7 m. A small share, and every one of them is
    conspicuous — a pale rectangle against sky is the most visible thing an
    aerial frame can contain.

    THE WHOLE BOARD, not its centre. A 4 m joist centred 1 m inside the
    boundary still puts a metre of itself over the edge, and `_box` already
    computes the eight rotated corners this needs.
    """
    x0, y0, x1, y1 = (float(q) for q in region)
    kept, dropped = [], 0
    for s in specs:
        pts, _n = _box(s)
        if (min(q[0] for q in pts) < x0 or max(q[0] for q in pts) > x1
                or min(q[1] for q in pts) < y0 or max(q[1] for q in pts) > y1):
            dropped += 1
            continue
        kept.append(s)
    if verbose and dropped:
        print("[planks] {0} board(s) overhung the plate edge and were dropped"
              .format(dropped))
    return kept, dropped


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

# Corner order is (sx, sy, sz) over (-1,-1,-1), (1,-1,-1), (1,1,-1),
# (-1,1,-1), (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1) — see `_box`.
#
# EVERY FACE IS WOUND COUNTER-CLOCKWISE SEEN FROM OUTSIDE, so the right-hand
# rule on the winding agrees with the normal beside it. A box whose windings
# and normals disagree is not a cosmetic problem: USD's default `orientation`
# is `rightHanded`, so the renderer culls on the winding and shades on the
# normal, and the result is boards that are lit from inside and disappear from
# half the angles you look at them from. Verified by cross product, face by
# face, rather than by eye — the six are easy to get individually right and
# collectively inconsistent.
_FACES = ((0, 3, 2, 1),        # -Z, the underside
          (4, 5, 6, 7),        # +Z
          (0, 1, 5, 4),        # -Y
          (1, 2, 6, 5),        # +X
          (2, 3, 7, 6),        # +Y
          (3, 0, 4, 7))        # -X
_FACE_N = ((0, 0, -1), (0, 0, 1), (0, -1, 0),
           (1, 0, 0), (0, 1, 0), (-1, 0, 0))


def _box(spec):
    """One board's 8 world points and its 6 face normals, from its spec."""
    hl, hw, ht = 0.5 * spec["l"], 0.5 * spec["w"], 0.5 * spec["t"]
    cy_, sy_ = (math.cos(math.radians(spec["yaw"])),
                math.sin(math.radians(spec["yaw"])))
    cp, sp = (math.cos(math.radians(spec["pitch"])),
              math.sin(math.radians(spec["pitch"])))
    cr, sr = (math.cos(math.radians(spec["roll"])),
              math.sin(math.radians(spec["roll"])))

    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll), written out rather than assembled
    # with numpy: this runs once per board over tens of thousands of boards
    # and the matrix product dominates if it allocates.
    m00 = cy_ * cp
    m01 = cy_ * sp * sr - sy_ * cr
    m02 = cy_ * sp * cr + sy_ * sr
    m10 = sy_ * cp
    m11 = sy_ * sp * sr + cy_ * cr
    m12 = sy_ * sp * cr - cy_ * sr
    m20 = -sp
    m21 = cp * sr
    m22 = cp * cr
    x, y, z = spec["x"], spec["y"], spec["z"]

    pts = []
    for (sx, sy2, sz) in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                          (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)):
        a, b, c = sx * hl, sy2 * hw, sz * ht
        pts.append((x + m00 * a + m01 * b + m02 * c,
                    y + m10 * a + m11 * b + m12 * c,
                    z + m20 * a + m21 * b + m22 * c))
    nrm = [(m00 * n[0] + m01 * n[1] + m02 * n[2],
            m10 * n[0] + m11 * n[1] + m12 * n[2],
            m20 * n[0] + m21 * n[1] + m22 * n[2]) for n in _FACE_N]
    return pts, nrm


def build(stage, root, specs, mats, ssf, verbose=True, skin_mats=None):
    """Author the field. ONE MESH PER (CLASS, SKIN). Returns the prim paths.

    Grouping by CLASS rather than by patch, because the material is per-class
    and a mesh can carry exactly one binding without GeomSubsets — and a
    GeomSubset per class inside one giant mesh buys nothing over a handful of
    meshes.

    ...AND BY SKIN, since 2026-08-27. A `siding` or `deck` piece carries the
    name of the material the house it came off was wearing (`spec["skin"]`, set
    by `scatter_from_wreck(skins=...)`), and `skin_mats` maps that name to a
    `UsdShade.Material` — in the assembly, `modular_house.palette_material`, so
    the debris and the standing houses are wearing literally the same material
    prim. A plate with four palettes on it therefore costs four extra meshes,
    not four hundred: the grouping is by MATERIAL, not by building. Unknown or
    absent skins fall through to the per-class sawn-timber material, which is
    what the corridor scatter and every bench get.

    Normals are authored `faceVarying`. A board is a hard-edged box and the
    renderer's fallback is to average normals at shared vertices, which
    rounds every corner and makes a stack of lumber look like a heap of
    pillows — visible immediately at close range and, oddly, still visible
    from altitude as a loss of the crisp rectangular read that is the whole
    point of authoring boxes.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if not specs:
        return []
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    by_class = {}
    for s in specs:
        sk = s.get("skin") if (skin_mats and s.get("skin") in skin_mats) else None
        by_class.setdefault((s["class"], sk), []).append(s)

    made = []
    for (k, sk), group in sorted(by_class.items(),
                                 key=lambda kv: (kv[0][0], kv[0][1] or "")):
        pts, counts, idx, nrm = [], [], [], []
        for s in group:
            p, n = _box(s)
            base = len(pts)
            pts.extend(Gf.Vec3f(float(q[0]) * ssf, float(q[1]) * ssf,
                                float(q[2]) * ssf) for q in p)
            for fi, face in enumerate(_FACES):
                counts.append(4)
                idx.extend(base + v for v in face)
                nrm.extend([Gf.Vec3f(*(float(c) for c in n[fi]))] * 4)
        path = "{0}/{1}".format(root, k if not sk else "%s__%s" % (k, sk))
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr([Gf.Vec3f(0.66, 0.58, 0.48)])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])
        mat = (skin_mats or {}).get(sk) if sk else (mats or {}).get(k)
        if mat is not None:
            # APPLY the schema before binding. `MaterialBindingAPI(prim).Bind()`
            # authors the relationship either way and Kit honours it, but core
            # USD then warns "Found material bindings on prim ... but
            # MaterialBindingAPI is not applied" on every read — which is noise
            # in exactly the place (an offline validation pass) where you are
            # reading the file to find real problems.
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        made.append(path)
        if verbose:
            print("[planks] {0:<10s} {1:6d} piece(s) -> 1 mesh, {2} point(s)"
                  .format(k if not sk else "%s/%s" % (k, sk), len(group),
                          len(pts)))
    return made
