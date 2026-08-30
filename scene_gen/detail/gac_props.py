"""gac_props — dress a GreatAmericanCity building with its own roof kit.

The pack ships 122 props beside its 31 buildings and the generator places none
of them, so every roof is a bare slab from the air — which is the view this
dataset actually flies.

WHAT GOES WHERE IS A PROPERTY OF THE ASSET, and it is measured
(`tools/gac_props_measure.py` -> `_plans/gac_props.json`), not guessed:

    roof_tank    SM_Water_Tank        3.4 x 3.1 x  5.7   the rooftop tank
    roof_mast    SM_Tower             2.5 x 2.5 x 21.7   comms mast
    roof_plant   SM_Generator_Eletric 2.8 x 6.8 x  2.9   plant / AC units
    roof_house   SM_Superior_Const_01  24 x  40 x 11.9   stair/lift overrun —
                                                         BIG, needs a big roof

FIRE ESCAPES WERE TRIED AND REJECTED. An earlier version stacked
`SM_Building_Stair` (1.6 x 3.3 x 4.2 m, one storey) up a blank elevation — the
pack has no purpose-built escape asset, and the measurement is what said this
one was a plausible stand-in. Built, reviewed against the scene in Isaac:
"I see some fire escapes but they don't look right so just remove them" (user,
2026-08-29). Removed from placement outright, not gated behind a knob —
`wall_props` no longer has a stair path at all. `SM_Building_Stair` stays
measured in `_plans/gac_props.json` (costs nothing, records what the asset
is) so nobody re-derives the same stacking idea and re-discovers the same
rejection.

MEASURED ON POINTS, NOT `UsdGeom.BBoxCache` — see
`.agents/skills/fix-floating-debris/SKILL.md`. `tools/gac_props_measure.py`
re-measures every prop from its transformed mesh POINTS
(`disaster.bake.world_point_bounds`), the fix the skill documents for exactly
this failure mode elsewhere in the repo. For this pack's clean, single-mesh
props the bbox and points numbers turned out to agree almost everywhere
(these are artist-modelled objects, not Voronoi debris — no diagonal sliver
for `BBoxCache`'s AABB-of-an-AABB to over-report) — `SM_Steel_Pipe_Plastic`
is the one exception, and it isn't a measurement artifact either: its pivot
really is 6.16 m from its own geometry, confirmed on points too, which is why
it is blacklisted rather than merely re-measured (see `gac_props_measure.py`'s
`KINDS`). The real floating source, once the props themselves cleared, turned
out to be the ROOF-HEIGHT formula, not the prop measurements — see
`roof_props`'s `roof_z`.

FRAME. A placement's `yaw_deg` already includes the per-asset `yaw-offset` that
normalises the pack's mixed fronts, so a measured side maps to world by that one
rotation and nothing else has to be unwound. Props are seated by their measured
`z0`, because several have a pivot that is not on their base.

BEYOND GREATAMERICANCITY. The 31 GAC buildings are the authority — every one of
them is flat-roofed by construction, so every one is eligible. Nothing else in
the pack has been looked at this way: a plant box floating on a pitched or
domed roof reads worse than a bare roof, so any OTHER building is eligible only
when its basename is named explicitly in `building_props.flat_roof`, and its
W/D/H then come from the generator's own `SizeResolver` rather than a measured
JSON — see `dress()`.

A ROOF HOUSE MUST BE CLAD IN THE SAME MATERIAL AS THE BUILDING IT STANDS ON.
Reviewed against the built 800 m scene, pointed at two placements directly:
"/World/stage/generated/roof_house_125_4768 — don't have this roof house on
the one it's on, they don't match. One is brick the other is concrete" and
"/World/stage/generated/roof_house_129_5053 — don't do roof houses that don't
match with the base building (brick doesn't go with concrete)" (user,
2026-08-29). Resolved from `_scene_assets.tsv`: both flagged prims are
`SM_Superior_Construction_04` and `SM_Superior_Construction_01` — and a direct
material probe of all five `roof_house` assets (walking each bound material's
own `info:unreal:sourceAsset`, area-weighted per subset — see
`gac_props_measure.py`'s `_classify_material`) found EXACTLY the fault line
the user described: `_01` and `_04` are 35-36% `M_Bricks_Superior_
Construction_Inst` by triangle area (BRICK), `_02` and `_03` are the same
bulkhead mesh with the same secondary materials but 35-38%
`M_Wall_Superior_Construction_Inst` instead (this pack's non-brick precast/
stucco skin — CONCRETE), and `SM_Glass_Roof` is 100% named glass materials.
Both complaints were a brick variant landing on a building whose own front
elevation reads concrete. `_cladding_material` derives the SAME four-way
classification for a building from its already-measured `_plans/gac_faces.json`
(`front` side's `tex_areas` plus `glass_frac` — never re-measured here, see
that function's own docstring), and `roof_props` filters its `roof_house`
candidates to the building's own material before choosing one — see
`building_props.roof_house_match_material` in `dress()`.
"""

import json
import math
import os

_SG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROPS = os.path.join(_SG_DIR, "_plans", "gac_props.json")
_FACES = os.path.join(_SG_DIR, "_plans", "gac_faces.json")
_BLDGS = os.path.join(_SG_DIR, "_plans", "gac_buildings.json")

_SIDE_DIR = {"E": (1.0, 0.0), "N": (0.0, 1.0),
             "W": (-1.0, 0.0), "S": (0.0, -1.0)}

PARAPET_M = 2.2        # keep roof plant back from the edge
ROOF_HOUSE_MIN_M2 = 1400.0
ROOF_HOUSE_MAX_H_M = 90.0
# A stair/lift bulkhead is a mid-rise thing: it is what makes a roof read as
# a real building UNDER it, but on a 130-310 m tower it is a shed on the
# roof of a mountain — "Don't add roof houses to high rises, it looks weird"
# (user, 2026-08-29). Alongside `ROOF_HOUSE_MIN_M2` (too small a roof) this
# is the other half of the gate: too TALL a building, tested against its own
# measured height, also gets none.

# `disaster/urban_fire.py`'s own constants for its structured (non-scattered)
# rooftop plant, borrowed rather than reinvented — see `roof_props`'s
# docstring for how each is used here. Its own comments there: "AC_PITCH =
# 1.7  # bolt spacing along a row" and "PAD_MARGIN = 0.45  # housekeeping pad
# margin past the units".
AC_PITCH = 1.7
PAD_MARGIN = 0.45


def load(props=None, faces=None, bldgs=None):
    """(props_by_kind, faces_by_name, dims_by_name)."""
    P = json.load(open(os.path.normpath(props or _PROPS)))
    by_kind = {}
    for r in P:
        by_kind.setdefault(r["kind"], []).append(r)
    F = {r["name"]: r for r in
         json.load(open(os.path.normpath(faces or _FACES)))}
    B = {r["name"]: r for r in
         json.load(open(os.path.normpath(bldgs or _BLDGS)))}
    return by_kind, F, B


# ---------------------------------------------------------------------------
# MATERIAL CLASSIFICATION — a roof house must be clad like the building it
# stands on. See this module's own docstring for the review that forced this
# and the measurement behind it.
# ---------------------------------------------------------------------------

# `roof_props` compares a `roof_house` record's `material` (set by
# `gac_props_measure.py`'s `_classify_material`, from the asset's own bound
# material names) against a building's material from `_cladding_material`
# below (from the building's already-measured `_plans/gac_faces.json`) — both
# land in this same four-way vocabulary, so a plain `==` is the whole test.
MATERIAL_UNKNOWN = "unknown"

# (substring, family) — checked in this order, first match wins. Applied to
# a texture BASENAME from `_plans/gac_faces.json`'s `tex_areas` (a building
# elevation, measured by `gac_faces.py`'s image-name probe), not to a bound
# material's own asset name (a roof house, measured differently — see
# `gac_props_measure.py`'s own `_MATERIAL_FAMILY`, which independently
# derived the identical "wall" -> concrete mapping from the SAME pack's roof
# kit — two different probes on two different asset classes agreeing on what
# "wall" means here is what makes it safe to use unguessed). Excludes
# "wallback"/"wall_back" explicitly: that is `gac_faces.py`'s own
# `BLANK_TOKENS` — the pack's blank REAR-panel material — never a street
# elevation's cladding, so it must never fall through to "wall" -> concrete.
_BUILDING_MATERIAL_FAMILY = (("brick", "brick"), ("concrete", "concrete"),
                             ("metal", "metal"), ("glass", "glass"))


def _cladding_material(front_side):
    """(material, evidence_m2) for one building's FRONT elevation record —
    the `sides["<front>"]` dict `_plans/gac_faces.json` already carries, with
    its own `tex_areas` (per-texture triangle area) and `glass_frac` (the
    PBR-material glazing test `gac_faces.py`'s `_is_glass` already ran, for
    the curtain-wall towers whose glazing carries no texture name at all —
    see that module's own docstring). NEVER RE-MEASURES A BUILDING: every
    number this reads was already produced by `gac_faces.py`, independently,
    for a different purpose (blank-elevation detection) — this is a second
    reading of the same data, not a fourth pass over the geometry.

    `material` is `"brick"` / `"concrete"` / `"glass"` / `"metal"`, or
    `None` when nothing on the front elevation clears any family — a
    building that big a mystery gets no roof house at all rather than a
    guessed one (see `roof_props`). `evidence_m2` is the per-family area
    tally, kept for `dress()`'s and `gac_props_check.py`'s own printing so a
    verdict can be read back to the numbers that produced it, the same
    discipline `gac_faces.py` itself uses for `blank_by`.

    TOKEN MATCH FIRST, PBR GLASS SECOND. Every texture basename in
    `tex_areas` is matched against `_BUILDING_MATERIAL_FAMILY` — "wall" (not
    "wallback") counts as concrete, per this pack's own naming (see that
    tuple's own comment). A real curtain wall's glazing has NO texture name
    at all (`gac_faces.py`: "GLASS HAS NO TEXTURE NAME TO MATCH" — a flat
    PBR colour, indistinguishable from a bare slab by name), so its area
    never appears under any texture key; `glass_frac * area_m2` is added to
    the glass family directly, on top of whatever a literal "glass" token
    already contributed, so a glazed tower still reads as glass even though
    none of its area is reachable by name.
    """
    cats = {"brick": 0.0, "concrete": 0.0, "glass": 0.0, "metal": 0.0}
    for tex, area in (front_side.get("tex_areas") or {}).items():
        n = (tex or "").lower()
        if not n:
            continue
        if "wallback" in n or "wall_back" in n:
            continue
        matched = False
        for token, family in _BUILDING_MATERIAL_FAMILY:
            if token in n:
                cats[family] += area
                matched = True
                break
        if not matched and "wall" in n:
            cats["concrete"] += area
    cats["glass"] += float(front_side.get("glass_frac", 0.0)) * \
        float(front_side.get("area_m2", 0.0))
    total = sum(cats.values())
    material = max(cats, key=cats.get) if total > 0.0 else None
    return material, {k: round(v, 1) for k, v in cats.items()}


def building_materials(faces):
    """{building_name: material_or_None} for every building in *faces* —
    `_plans/gac_faces.json`'s own dict, keyed by name (`load()`'s second
    return value). A name ABSENT from this dict (never mapped to
    `MATERIAL_UNKNOWN` here — that is the caller's job, see `dress()`) is a
    non-GAC building `_cladding_material` was never asked about; a name
    PRESENT with value `None` is a GAC building that WAS measured and came
    up with no material signal at all on its front elevation (`SM_
    Building_04`'s front is a 49.9 m2 sliver of pure trim — see
    `_cladding_material`'s own evidence tally for it). The two cases read
    identically to `roof_props`'s filter once `dress()` has substituted
    `MATERIAL_UNKNOWN` for the first — see that function's own docstring for
    why they nonetheless behave differently.
    """
    out = {}
    for nm, rec in faces.items():
        front = rec.get("front")
        side = (rec.get("sides") or {}).get(front) if front else None
        out[nm] = _cladding_material(side)[0] if side is not None else None
    return out


def _dump_material_audit(out_path=None):
    """Write `_plans/gac_building_material.json` — the classification this
    module derives from `_plans/gac_faces.json`, WITH its evidence, so the
    next reader can audit the call without re-deriving it (the acceptance
    test this exists for: `gac_props_check.py`'s material assertion checks
    the SAME live computation, not this file — this is a record, not a
    cache). Run directly: `python3 scene_gen/detail/gac_props.py`.
    """
    _, faces, _ = load()
    rows = []
    for nm in sorted(faces):
        rec = faces[nm]
        front = rec.get("front")
        side = (rec.get("sides") or {}).get(front) if front else None
        material, evidence = (None, {}) if side is None else \
            _cladding_material(side)
        rows.append({"name": nm, "front": front, "material": material,
                     "evidence_m2": evidence})
    path = out_path or os.path.join(_SG_DIR, "_plans",
                                    "gac_building_material.json")
    json.dump(rows, open(os.path.normpath(path), "w"), indent=1)
    print(f"wrote {os.path.normpath(path)}")
    for r in rows:
        print("  %-22s front=%-4s material=%-9s %s" %
              (r["name"], r["front"] or "-", r["material"] or "-",
               r["evidence_m2"]))
    return rows


_URL_SCHEMES = ("omniverse://", "airstack://", "file:")


def _place(rec, x, y, z, yaw, category, of=None):
    """One prop placement, seated on its own measured base.

    `of` names the building it belongs to. `apply_placements` ignores keys it
    does not know, and having it makes a prop AUDITABLE — without it the only
    way to tell which building an escape belongs to is proximity, and on a real
    street the neighbour is closer than the far side of the same building.

    `dress()` never routes a placement through `_normalize_usd_list`/
    `_join_asset_root` — those live in `scene_generator.py`, which this module
    does not call into for props — so a schemeless `usd` here is NOT prefixed
    with `asset_root` the way an ordinary asset-set entry would be; it is
    handed to `AddReference` exactly as written. `_plans/gac_props.json` used
    to store a path relative to `asset_root` (`.../Library/Stages/`), but
    GreatAmericanCity lives under `.../Projects/SEI-COA/GreatAmericanCity/`
    — a different tree — so every prop resolved to a path that does not
    exist and silently failed to load (`omni.client.stat` on the old value
    returns `ERROR_NOT_FOUND`). `tools/gac_props_measure.py` now writes the
    full URL; this assertion is what stops that regressing without a render
    to notice it in.
    """
    usd = rec["usd"]
    assert str(usd).startswith(_URL_SCHEMES), (
        f"gac_props: {rec.get('name', usd)!r} has a schemeless usd path "
        f"{usd!r} — dress() never prefixes asset_root, so this would "
        f"reference a path that does not exist. Re-run "
        f"tools/gac_props_measure.py and check its ROOT constant.")
    return {"of": of, "usd": usd, "x_m": float(x), "y_m": float(y),
            "z_m": float(z - rec["z0"]), "yaw_deg": float(yaw) % 360.0,
            "roll_deg": 0.0, "pitch_deg": 0.0,
            "scale": float(rec.get("mpu", 1.0)),
            "category": category, "axis_up": "Z", "raw_pivot": True}


def _seat_origin(rec, cx_want, cy_want, yaw_deg):
    """Origin XY for `rec` so its MEASURED bbox centre — not its raw pivot —
    lands at world ``(cx_want, cy_want)`` once rotated by *yaw_deg*.

    Every prop in `_plans/gac_props.json` has its own `cx`/`cy`: the bbox
    centre's offset from wherever the exporter left the pivot, in the prop's
    OWN local frame. It is rarely small — `SM_Generator_Eletric` is 3.69 m off
    centre against a 2.83 x 6.84 m footprint (bigger than the parapet margin
    that is supposed to keep it clear of the edge), and every roof/wall prop
    in this pack measures similarly.

    `_place()` sets ``raw_pivot: True``, which tells `apply_placements` to
    skip ITS OWN centroid correction — right for a kit assembled from many
    pieces already positioned relative to each other, wrong for a standalone
    prop with an arbitrary pivot (see that flag's own docstring). Every
    distance and inset `roof_props`/`wall_props` compute is a distance from
    the prop's CENTRE — the free-space radius, the parapet margin, the
    stand-off from a wall — so without this correction the placement's
    `x_m, y_m` (the pivot) is not where any of that math thinks it is, and
    the rendered footprint lands up to `|cx, cy|` away from the position that
    was just proven clear. This is the one place that offset gets undone.
    """
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    return (cx_want - (rec["cx"] * ca - rec["cy"] * sa),
            cy_want - (rec["cx"] * sa + rec["cy"] * ca))


def _name_of(usd):
    """Basename with its extension stripped, for matching against `dims`/`faces`
    and the `flat_roof` / `no_roof_props` config lists.

    `.usd` and `.usdc` both occur in this repo's asset sets (`plan_png.py`
    notes the 2026-08-26 building drop ships `.usdc` throughout) — a fixed
    `[:-4]` strips the wrong number of characters off a 5-character extension
    and leaves a trailing letter, so basenames silently never match. Split on
    the actual dot instead.
    """
    base = usd.rsplit("/", 1)[-1]
    return base.rsplit(".", 1)[0] if "." in base else base


def roof_props(bld, dims, by_kind, rng, max_props=6, of=None,
               tank_window=(30.0, 90.0), ac_min_h=0.0,
               roof_house_max_h_m=ROOF_HOUSE_MAX_H_M,
               bld_material=MATERIAL_UNKNOWN, match_material=True):
    """A STRUCTURED rooftop installation, in the order a real roof is built.
    Returns [placement].

    NOT A SCATTER. The previous version dropped plant at uniformly random
    points inset from the parapet with overlap rejection — reviewed and
    rejected: "the AC units seem random instead of structured like it would
    on an actual building" (user, 2026-08-28). `disaster/urban_fire.py`'s
    `dress_roof_urban` (which replaced `quake_flow.dress_roof` for the same
    reason, on the same review) documents the fix as a real roof's own build
    order, using primitive boxes because it authors directly onto a stage;
    this is the measured-GAC-kit counterpart, placing existing assets instead
    of drawing new geometry, but the LAYOUT is deliberately the same:

        1. a STAIR/LIFT BULKHEAD — one box against an EDGE, squared to the
           building. Tallest thing up there; it anchors everything else.
        2. a TANK on its stand, next to the bulkhead — that is where the
           riser is.
        3. a COMMS MAST, on the bulkhead's other side (an extension beyond
           `dress_roof_urban`'s own four elements — see step 3 below).
        4. VENT STACKS through the deck, small, in a line off the bulkhead.
        5. a row of CONDENSERS on the far side, on ONE axis, evenly pitched.
           A real roof has them craned in and bolted to a grid — not a
           housekeeping PAD asset in this kit, so the pad is a reserved
           clearance zone rather than a modelled slab (see `PAD_MARGIN`).

    Everything is positioned in the BUILDING's own local frame and rotated
    into world by `ca`/`sa` (the building's yaw) — the same machinery
    `_seat_origin` already turns a prop's measured centre through — so the
    whole installation stays square to the building at any yaw.

    `AC_PITCH` (1.7 m bolt spacing) and `PAD_MARGIN` (0.45 m clearance past
    the row) are `dress_roof_urban`'s own constants, borrowed rather than
    reinvented: `AC_PITCH` floors the row's spacing (raised per row when the
    drawn condenser is itself wider than that — this kit's
    `SM_Generator_Eletric` is 6.84 m long, nothing like `dress_roof_urban`'s
    synthetic 1.05 x 0.62 m packaged unit); `PAD_MARGIN` is the extra
    clearance the row's reserved footprint must clear before the parapet
    inset, standing in for the housekeeping pad this kit has no asset for.

    *tank_window* is ``(min_h, max_h)`` in metres, tested against the
    BUILDING's own measured height — a water tank is a mid-rise thing (a real
    tower plumbs its rooftop tank off a pressurised riser, not a tank sat on
    the roof slab, and `SM_Water_Tank_02` at 5.0 x 5.0 x 11.3 m reads as a
    toy on a 15 m building and is invisible from the air on a 300 m one).
    Outside the window a tank is never rolled, full stop.

    *ac_min_h* floors which buildings are even offered the condenser row and
    vent stacks — 0.0 by default, since AC genuinely belongs on almost any
    flat roof.

    *roof_house_max_h_m* is a ceiling on the bulkhead ONLY (`ROOF_HOUSE_MAX_H_M`
    by default) — "Don't add roof houses to high rises, it looks weird" (user,
    2026-08-29). A stair/lift overrun reads as the top of a real building on
    a mid-rise; on a 130-310 m tower it is a shed on a mountain. Everything
    else (tank, mast, vents, condensers) has no such ceiling — a tower's roof
    still has AC and a tank, just not a stair head sized for six storeys.

    *bld_material* is *bld*'s own cladding family (`"brick"`/`"concrete"`/
    `"glass"`/`"metal"`/`None`, from `_cladding_material`) or `MATERIAL_
    UNKNOWN` when the caller has no cladding measurement for this building at
    all (a non-GAC `flat_roof` building — `_plans/gac_faces.json` only
    covers the 31 GAC names). *match_material* (`building_props.roof_house_
    match_material`, defaulted on) gates whether the bulkhead pool is
    filtered by it at all. THE POOL IS FILTERED, NEVER SUBSTITUTED: with
    *match_material* true and a REAL material (not `MATERIAL_UNKNOWN`),
    `house_pool` below drops every `roof_house` record whose own measured
    `material` (`gac_props_measure.py`'s `_classify_material`) does not
    equal *bld_material* — including when *bld_material* is `None` (a GAC
    building whose front elevation carried no material signal at all), which
    matches NO recorded `roof_house` material and empties the pool outright.
    An empty pool means no bulkhead gets placed on this roof this call, not a
    fallback to an unfiltered pick — "a bare roof is better than a brick hut
    on a concrete tower" (user, 2026-08-29, said twice about two different
    buildings). `MATERIAL_UNKNOWN` is the one value that skips filtering
    altogether, since there the caller has no evidence to filter BY, not
    evidence that nothing matches.
    """
    W, D, H = dims["W"], dims["D"], dims["H"]
    yaw = float(bld.get("yaw_deg", 0.0))
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x0, y0 = float(bld["x_m"]), float(bld["y_m"])
    # World Z of the roof. Not just `H`: `bld["z_m"]` is this placement's own
    # lift off the ground, needed when a damaged stand-in this run's disaster
    # tilted or sank it, and `dims["z0"]` is the building's own measured
    # pivot-to-geometry offset — a PRISTINE building already carries `z_m ==
    # -z0` (the lift that puts its geometry's own bottom, not its arbitrary
    # local origin, on the ground), so adding `z_m` on top of `H` without
    # ALSO adding `z0` double-counts that lift and floats every roof prop by
    # exactly `-z0` — up to 8-13 cm, measured across the GAC library
    # (`tools/gac_props_measure.py`'s companion check). `z0` defaults to 0.0
    # for a resolver-measured non-GAC building, where `dress()` derives it
    # from the SAME `SizeResolver.get()["base"]` this formula is undoing.
    roof_z = float(bld.get("z_m", 0.0)) + H + float(dims.get("z0", 0.0))
    out, taken = [], []

    def _free(lx, ly, r, against=None):
        return all((lx - qx) ** 2 + (ly - qy) ** 2 > (r + qr) ** 2
                   for qx, qy, qr in (taken if against is None else against))

    def _put_at(rec, lx, ly, extra_yaw, kind, check_overlap=True,
               overlap_rad=None):
        """Author `rec` with its measured centre at local (lx, ly), rotated
        by the building's yaw plus `extra_yaw`.

        `check_overlap` is `True` (test against everything placed so far),
        `False` (skip the test entirely), or a LIST — a snapshot of `taken`
        to test against instead of the live list. The row uses the list
        form: two condensers spaced along their own pitch never overlap
        EACH OTHER by construction, but the conservative circular bound
        (`rad = max(W, D)/2`) is too pessimistic to prove that itself
        whenever a unit is wider across its row than along it, and would
        reject a valid neighbour — `check_overlap=False` used to paper over
        that by skipping the check altogether, which also silently stopped
        checking the row against the bulkhead/tank already on the roof and
        let a row member land inside the bulkhead's own footprint on a roof
        where the bulkhead sits on the edge the row's OWN run axis reaches
        (`gac_props_check.py` assertion 4, `SM_Building_21`/`_24`/`_26`).
        A snapshot taken BEFORE the row starts is what actually reproduces
        "immune to itself, not immune to what came before it"."""
        # The CIRCUMRADIUS (half-diagonal), not half the longer side: the
        # farthest point on a W x D rectangle from its own centre is a
        # CORNER, at `hypot(W, D)/2`, which is bigger than `max(W, D)/2`
        # whenever the rectangle isn't degenerate. `max(W, D)/2` only bounds
        # the reach along the two cardinal directions, so two boxes at
        # arbitrary relative headings (a bulkhead tried at extra_yaw 90
        # against a tank/mast/vent at extra_yaw 0 is exactly that) can pass
        # a `max(W, D)/2` circle test corner-to-corner while their true
        # rotated footprints still overlap — `gac_props_check.py` assertion
        # 4 caught this on `BG_Building_D`/`SM_Building_28`/`_26`.
        rad = overlap_rad if overlap_rad is not None else 0.5 * math.hypot(rec["W"], rec["D"])
        if check_overlap is not False:
            against = None if check_overlap is True else check_overlap
            if not _free(lx, ly, rad + 0.4, against=against):
                return None
        taken.append((lx, ly, rad))
        pyaw = yaw + extra_yaw
        wx = x0 + lx * ca - ly * sa
        wy = y0 + lx * sa + ly * ca
        # (wx, wy) is the FOOTPRINT CENTRE — seat the prop's own pivot so
        # its measured centre lands there, not its pivot (see `_seat_origin`).
        ox, oy = _seat_origin(rec, wx, wy, pyaw)
        pl = _place(rec, ox, oy, roof_z, pyaw, kind, of)
        out.append(pl)
        return pl

    def _edge_fit(rec, extra_yaw, side, margin):
        """(n_pos, run_max, along) placing `rec` (its own W/D swapped by a
        90 deg `extra_yaw`) flush against `side`'s margin-inset line, or
        None if it does not fit either axis. `n_pos` is the local offset
        FROM THE BUILDING CENTRE toward `side` (magnitude only — sign comes
        from `_edge_point`); `run_max` is how far the centre may then slide
        along the edge and still clear the inset on that axis too; `along`
        is the item's own extent along the edge, for spacing arithmetic."""
        item_x, item_y = (rec["W"], rec["D"]) if extra_yaw % 180.0 == 0.0 \
            else (rec["D"], rec["W"])
        if side in ("N", "S"):
            half_n, run_half, across, along = D / 2.0, W / 2.0, item_y, item_x
        else:
            half_n, run_half, across, along = W / 2.0, D / 2.0, item_x, item_y
        if across / 2.0 + margin > half_n or along / 2.0 + margin > run_half:
            return None
        return half_n - margin - across / 2.0, run_half - margin - along / 2.0, along

    def _edge_point(side, n_pos, run_pos):
        ex, ey = _SIDE_DIR[side]
        return (run_pos, n_pos * ey) if side in ("N", "S") else (n_pos * ex, run_pos)

    # ------------------------------------------------------------------
    # 1. THE BULKHEAD — hard against one edge, squared to the building.
    #    Tried at both 0 and a 90 deg extra yaw (the pool item's own W/D
    #    swapped), since a 24 x 40 m box only fits some edges one way
    #    round; the first candidate that clears the parapet inset on both
    #    axes wins. `ROOF_HOUSE_MIN_M2` gate is unchanged from the scatter
    #    version — these assets are 18-40 m across and only suit a big roof.
    #    `roof_house_max_h_m` is the OTHER half: too tall a building looks
    #    just as wrong as too small a roof (see the docstring above).
    #    `bld_material` narrows the pool ONE MORE TIME, after the size fit —
    #    a roof_house clad wrong for this building is not a candidate at
    #    all, not a candidate that loses a coin flip (see the docstring's
    #    *bld_material*/*match_material* paragraph for why an empty result
    #    here means no bulkhead, never an unfiltered fallback pick).
    # ------------------------------------------------------------------
    side = rng.choice(("N", "E", "S", "W"))
    bulkhead_run = None            # run-axis coordinate the tank/vents/mast anchor to
    bulkhead_lx = bulkhead_ly = None   # its actual local centre, for the row's off_sign
    house_pool = [r for r in by_kind.get("roof_house", [])
                  if r["W"] + 2 * PARAPET_M < W and r["D"] + 2 * PARAPET_M < D]
    if match_material and bld_material != MATERIAL_UNKNOWN:
        house_pool = [r for r in house_pool if r.get("material") == bld_material]
    if (house_pool and W * D >= ROOF_HOUSE_MIN_M2 and H <= roof_house_max_h_m
            and rng.random() < 0.55):
        rng.shuffle(house_pool)
        for rec in house_pool:
            fit, chosen_yaw = None, 0.0
            for extra_yaw in rng.sample((0.0, 90.0), 2):
                f = _edge_fit(rec, extra_yaw, side, PARAPET_M)
                if f is not None and f[1] > 0.0:
                    fit, chosen_yaw = f, extra_yaw
                    break
            if fit is None:
                continue
            n_pos, run_max, _along = fit
            run_pos = rng.uniform(-run_max, run_max)
            lx, ly = _edge_point(side, n_pos, run_pos)
            if _put_at(rec, lx, ly, chosen_yaw, "roof_house") is not None:
                bulkhead_run, bulkhead_lx, bulkhead_ly = run_pos, lx, ly
                break

    # An anchor for the tank/vents/mast even when no physical bulkhead got
    # placed (roof too small, or the 0.55 draw didn't fire): the riser is
    # there whether or not the stair head above it got modelled, so they
    # still cluster near the middle of this edge rather than scattering.
    anchor_run = bulkhead_run if bulkhead_run is not None else 0.0

    # ------------------------------------------------------------------
    # 2. THE TANK, beside the bulkhead — the riser is there. Tried at a
    #    handful of offsets outward from the anchor along the SAME edge, so
    #    it lands next to the bulkhead rather than merely somewhere on the
    #    roof; `_free` (via `_put_at`) is what actually keeps it off the
    #    bulkhead's own footprint.
    # ------------------------------------------------------------------
    if (by_kind.get("roof_tank") and tank_window[0] <= H <= tank_window[1]
            and rng.random() < 0.75):
        rec = rng.choice(by_kind["roof_tank"])
        f = _edge_fit(rec, 0.0, side, PARAPET_M)
        if f is not None and f[1] > 0.0:
            n_pos, run_max, along = f
            for step in (0.0, along, -along, 2 * along, -2 * along):
                run_pos = max(-run_max, min(run_max, anchor_run + step))
                lx, ly = _edge_point(side, n_pos, run_pos)
                if _put_at(rec, lx, ly, 0.0, "roof_tank") is not None:
                    break

    # ------------------------------------------------------------------
    # 3. THE COMMS MAST, on the bulkhead's OTHER side from the tank — real
    #    ones rise off the stair-head roof, not free-standing, so this only
    #    appears when a bulkhead was actually placed (nothing to anchor it
    #    to otherwise). Not one of `dress_roof_urban`'s four elements; kept
    #    from the earlier scatter version because the kit measures it, but
    #    anchored instead of scattered.
    # ------------------------------------------------------------------
    if (bulkhead_run is not None and by_kind.get("roof_mast")
            and W * D > 900.0 and rng.random() < 0.35):
        rec = rng.choice(by_kind["roof_mast"])
        f = _edge_fit(rec, 0.0, side, PARAPET_M)
        if f is not None and f[1] > 0.0:
            n_pos, run_max, along = f
            for step in (-along, along, -2 * along, 2 * along):
                run_pos = max(-run_max, min(run_max, anchor_run + step))
                lx, ly = _edge_point(side, n_pos, run_pos)
                if _put_at(rec, lx, ly, 0.0, "roof_mast") is not None:
                    break

    # ------------------------------------------------------------------
    # 4. VENT STACKS, in a line off the bulkhead — small, through the deck.
    # ------------------------------------------------------------------
    pipe_pool = by_kind.get("roof_pipe", [])
    if pipe_pool and H >= ac_min_h:
        rec = rng.choice(pipe_pool)
        f = _edge_fit(rec, 0.0, side, PARAPET_M)
        if f is not None and f[1] > 0.0:
            n_pos, run_max, along = f
            pitch = max(1.5, along + 0.4)
            for k in range(rng.randint(2, 4)):
                run_pos = max(-run_max, min(run_max,
                              anchor_run + (k + 1.5) * pitch))
                lx, ly = _edge_point(side, n_pos, run_pos)
                _put_at(rec, lx, ly, 0.0, "roof_prop")

    # ------------------------------------------------------------------
    # 5. THE CONDENSER ROW — the element the review was actually about.
    #    One axis, evenly pitched, on the FAR side from the bulkhead so the
    #    two never compete for the same stretch of roof — the row's
    #    cross-axis sign is the OPPOSITE of the bulkhead's own cross
    #    position (mirrors `dress_roof_urban`'s `off_sign`), falling back to
    #    a coin flip when there is no bulkhead to be opposite of. Checked
    #    against a SNAPSHOT of the roof taken before the row starts (see
    #    `_put_at`) — immune to the row's own conservative self-overlap,
    #    not to the bulkhead/tank/mast/vents already up there.
    # ------------------------------------------------------------------
    pre_row_taken = list(taken)
    plant_pool = by_kind.get("roof_plant", [])
    if plant_pool and H >= ac_min_h:
        rec = rng.choice(plant_pool)
        long_x = W >= D
        # `_put_at` is given `extra_yaw = 0.0 if long_x else 90.0` below —
        # chosen SPECIFICALLY so the item's own W axis always ends up along
        # the row's run direction, whichever building-local axis that is.
        # `along_dim`/`across_dim` must therefore be `rec["W"]`/`rec["D"]`
        # UNCONDITIONALLY, not swapped again by `long_x` here: a 90 deg
        # extra_yaw already performs that swap once (verified against
        # `_edge_fit`, which computes the identical item_x/item_y pair for
        # the same extra_yaw and does NOT re-swap). Swapping twice cancels
        # out and silently uses the item's ACROSS dimension as its pitch —
        # `gac_props_check.py` caught this as adjacent SM_Air_Machine units
        # (4.52 x 3.46 m) placed 3.76 m apart along their 4.52 m axis, a
        # ~0.76 m overlap on every row this drew a non-square item into.
        along_dim, across_dim = rec["W"], rec["D"]
        pitch = max(AC_PITCH, along_dim + 0.3)
        row_run = (W if long_x else D) - 2.0 * PARAPET_M
        n = 0
        if row_run > pitch:
            n = max(2, min(max_props, int(row_run / pitch)))
            n = min(n, rng.randint(3, 7))
        if n >= 2:
            cross_of_bulkhead = (bulkhead_ly if long_x else bulkhead_lx) \
                if bulkhead_lx is not None else None
            if cross_of_bulkhead:
                off_sign = -1.0 if cross_of_bulkhead > 0 else 1.0
            else:
                off_sign = rng.choice((-1.0, 1.0))
            rows = 2 if (min(W, D) > 14.0 and n >= 4 and rng.random() < 0.55) else 1
            per_row = max(1, n // rows)
            row_gap = across_dim + 1.1     # dress_roof_urban's own back-to-back gap
            reserved_half = (across_dim / 2.0 if rows == 1
                             else across_dim + row_gap / 2.0) + PAD_MARGIN
            perp_half = (D / 2.0 if long_x else W / 2.0) - PARAPET_M
            if perp_half > reserved_half:
                target = (D if long_x else W) * 0.22 * off_sign
                cross = max(-(perp_half - reserved_half),
                           min(perp_half - reserved_half, target))
                span = (per_row - 1) * pitch
                for r in range(rows):
                    row_cross = cross + (r - (rows - 1) / 2.0) * row_gap
                    for k in range(per_row):
                        a = -span / 2.0 + k * pitch
                        lx, ly = (a, row_cross) if long_x else (row_cross, a)
                        # Circumradius here too (see `_put_at`'s default) —
                        # this call only ever checks against `pre_row_taken`
                        # (the bulkhead/tank/mast/vents), never against a row
                        # sibling, so there is no self-rejection risk to
                        # trade away by being exact rather than generous.
                        _put_at(rec, lx, ly, 0.0 if long_x else 90.0,
                               "roof_prop", check_overlap=pre_row_taken,
                               overlap_rad=0.5 * math.hypot(along_dim, across_dim))
    return out


def wall_props(bld, dims, face, by_kind, rng, of=None):
    """Service runs (pipe/cable/conduit) up a BLANK elevation. [placement].

    NO FIRE ESCAPE HERE ANY MORE. An earlier version stacked
    `SM_Building_Stair` up the wall — see this module's own docstring for
    why that was removed outright rather than gated: it was built, reviewed
    against the scene, and rejected on how it looked, not on a knob's
    default.
    """
    blanks = list(face.get("blank_sides") or [])
    if not blanks:
        return []
    W, D, H = dims["W"], dims["D"], dims["H"]
    yaw = float(bld.get("yaw_deg", 0.0))
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x0, y0 = float(bld["x_m"]), float(bld["y_m"])
    # See `roof_props`'s `roof_z` for why this term exists: `bld["z_m"]` is
    # the placement's own lift off the ground and `dims["z0"]` is the
    # building's own pivot-to-geometry offset (0 for a resolver-measured
    # non-GAC building, since `_seat_origin`-style correction has nowhere to
    # put it) — a wall run's height is a loose visual range, not a seating
    # decision, but there is no reason to carry the same formula error two
    # places once it is known.
    base_z = float(bld.get("z_m", 0.0)) + float(dims.get("z0", 0.0))
    out = []
    side = rng.choice(blanks)
    lx, ly = _SIDE_DIR[side]
    # the wall plane, and how far along it we may slide
    half = (W / 2.0) if side in ("E", "W") else (D / 2.0)
    run = (D if side in ("E", "W") else W)
    # along-wall direction is the outward normal turned +90
    ax, ay = -ly, lx

    def _world(px, py):
        return (x0 + px * ca - py * sa, y0 + px * sa + py * ca)

    for r in by_kind.get("wall_run", []):
        if rng.random() > 0.35:
            continue
        t = rng.uniform(-0.42, 0.42) * run
        px = lx * (half + r["W"] / 2.0) + ax * t
        py = ly * (half + r["W"] / 2.0) + ay * t
        wx, wy = _world(px, py)
        ryaw = yaw + math.degrees(math.atan2(ly, lx))
        ox, oy = _seat_origin(r, wx, wy, ryaw)
        out.append(_place(r, ox, oy, base_z + rng.uniform(1.0, max(1.5, H - 3.0)),
                          ryaw, "wall_run", of))
    return out


def dress(config, placements, rng, by_kind=None, faces=None, dims=None,
          resolver=None, category="house"):
    """Dress every eligible building in *placements* with roof and wall kit.

    *config* is the FULL compiled scene config; this reads its own
    ``building_props`` block the way `districts.assign` reads ``districts`` —
    defensively, so a preset that never mentions it (or an older cached one)
    still runs and simply dresses nothing.

    A GAC building (`nm in dims`, the 31 entries of `_plans/gac_buildings.json`)
    is always eligible for roof props — flat-roofed by construction, and what
    this stock's own 28 curated props were measured for. Any OTHER building
    (a Muyang tower, a downtowncity filler, an AEC brownstone...) is eligible
    only when named in ``building_props.flat_roof``: nothing about a bbox says
    whether the roof under it is flat, pitched or domed, and guessing wrong
    puts a plant box on a ridge line. Its W/D/H then come from *resolver* —
    the SAME `SizeResolver` `build_city` already measured it with, so this is
    a cache hit keyed on ``(usd, scale, axis_up)``, not a re-measure.
    ``building_props.no_roof_props`` excludes a basename from roof props
    regardless of either list (GAC included).

    `wall_props` (service runs only — see its own docstring for why there is
    no fire escape any more) needs `blank_sides`, which today only
    `_plans/gac_faces.json` carries — a building absent from *faces* is
    skipped there, never guessed.

    ``building_props.roof_house_match_material`` (default ``True``) gates
    whether a building's own cladding (`building_materials(faces)`, derived
    from *faces* — see `_cladding_material`) restricts which `roof_house`
    record it can be given; see `roof_props`'s own docstring for exactly how
    the filter behaves, including why an empty result means no bulkhead
    rather than an unfiltered one. A correctness rule, not a taste knob —
    defaulted on — but a knob all the same, since the review that forced it
    is about THIS pack's specific brick/concrete split and a future asset
    set might have nothing worth matching.

    Returns the NEW prop placements only; *placements* is read, not mutated.
    """
    cfg = (config or {}).get("building_props") or {}
    if not cfg.get("enabled", False):
        return []
    if by_kind is None:
        by_kind, faces, dims = load()

    max_props = int(cfg.get("max_roof_props", 6))
    tank_window = (float(cfg.get("water_tank_min_h_m", 30.0)),
                  float(cfg.get("water_tank_max_h_m", 90.0)))
    ac_min_h = float(cfg.get("ac_min_h_m", 0.0))
    roof_house_max_h = float(cfg.get("roof_house_max_h_m", ROOF_HOUSE_MAX_H_M))
    flat_roof = {str(n) for n in (cfg.get("flat_roof") or [])}
    no_roof = {str(n) for n in (cfg.get("no_roof_props") or [])}
    match_material = bool(cfg.get("roof_house_match_material", True))
    # {building_name: material_or_None} for the 31 GAC names *faces* carries
    # — a name this loop meets that is NOT a key here (a non-GAC `flat_roof`
    # building) gets `MATERIAL_UNKNOWN` below, which `roof_props` reads as
    # "no evidence to filter by" rather than "measured, no match" (see
    # `building_materials`'s own docstring for that distinction).
    bld_material_of = building_materials(faces) if match_material else {}

    out = []
    gac_seen, other_seen, dressed = set(), set(), 0
    for p in placements:
        if p.get("category") != category:
            continue
        nm = _name_of(p.get("usd", ""))
        d = dims.get(nm)
        if d is not None:
            gac_seen.add(nm)
        else:
            other_seen.add(nm)
            if resolver is None or nm not in flat_roof:
                continue
            fp = resolver.get(p.get("usd", ""), category,
                              scale=p.get("scale", 1.0),
                              axis_up=p.get("axis_up", "Z"))
            # `z0` mirrors `_plans/gac_buildings.json`'s own field — the
            # world Z the geometry's own lowest point sits at when the
            # asset's LOCAL origin (not its bbox) is placed at world Z=0. It
            # is what `roof_props`'s `roof_z` needs to undo the ground-lift
            # `fp["base"]` already applied; see that formula's own comment.
            # `SizeResolver.get()` returns `base = -mn*scale`, so
            # `z0 = mn*scale = -base`.
            d = {"W": fp["sx"], "D": fp["sy"], "H": fp["sz"], "z0": -fp["base"]}

        tag = "%s@%.1f,%.1f" % (nm, p["x_m"], p["y_m"])
        n_before = len(out)
        if nm not in no_roof:
            bld_material = bld_material_of.get(nm, MATERIAL_UNKNOWN)
            out += roof_props(p, d, by_kind, rng, max_props=max_props,
                              tank_window=tank_window, ac_min_h=ac_min_h,
                              roof_house_max_h_m=roof_house_max_h,
                              bld_material=bld_material,
                              match_material=match_material, of=tag)
        if nm in faces:
            out += wall_props(p, d, faces[nm], by_kind, rng, of=tag)
        if len(out) > n_before:
            dressed += 1

    tally = {}
    for pl in out:
        tally[pl["category"]] = tally.get(pl["category"], 0) + 1
    other_dressed = other_seen & flat_roof
    parts = "  ".join(f"{k}={v}" for k, v in sorted(tally.items()))
    print(f"[gac_props] dressed {dressed} building(s)  "
          f"gac={len(gac_seen)}/{len(dims)}  "
          f"other={len(other_dressed)}/{len(other_seen)} (on flat_roof)  "
          f"-> {len(out)} prop(s)  {parts or '(none)'}")
    unlisted = other_seen - flat_roof
    if unlisted:
        print(f"[gac_props] {len(unlisted)} non-GAC 'house' basename(s) not on "
              f"building_props.flat_roof, so undressed: "
              f"{', '.join(sorted(unlisted))}")
    return out


# ---------------------------------------------------------------------------
# The disaster hand-off: ADOPT what is already on the roof, author nothing.
# ---------------------------------------------------------------------------
# `disaster/quake_flow.dress_roof` and `disaster/urban_fire.dress_roof_urban`
# author their OWN rooftop plant, but only for KIT-BASHED `urban_building`
# archetypes (`wreck_building`/`burn_building`, `category` strings like
# `bld_<style>_wing0_storey_corner`) — a building-generation system this
# module never touches (`dress()` filters on `category == "house"`, which a
# kit archetype's pieces never carry). There is no shared prim between the
# two, so an archetype building has nothing here to adopt; that authoring
# stays as the only source of its plant.
#
# The STANDALONE monolith buildings this module DOES dress (GAC, and
# anything on `building_props.flat_roof`) are a different story: earthquake
# damages them directly (`disaster/quake.py`'s `_mono_pass`, matched by
# `category == "house"` and a non-archetype `usd` — the exact same test
# `dress()` uses). Fire USED to have a matching whole-asset entry point,
# `disaster/urban_fire.burn_monolith`, written to the same contract
# (`usd, x, y, yaw` — a placed monolith, not a style); it was removed on
# 2026-08-29 and whole-asset buildings now go through
# `disaster/kit_substitute.route()`. The quake side is unchanged.
# Both operate on the IDENTICAL building this module dressed, addressable by
# the SAME `usd`/`x`/`y` the placement already carries — so for THIS class
# of building, "author a second set of tanks and AC units" is not just
# wasteful, it is wrong: the base layout already put one there.
ROOF_FIXED_KINDS = frozenset({"roof_house"})
ROOF_PLANT_KINDS = frozenset({"roof_tank", "roof_mast", "roof_prop"})


def roof_plant_of(placements, usd, x, y):
    """(fixed_paths, plant_paths) — the roof installation `dress()` already
    authored for the building identified by ``(usd, x, y)``, split the way a
    disaster pass's settle machinery needs it split.

    THE SPLIT, MADE EXPLICIT: `roof_house` (the bulkhead) is BUILDING, not
    plant — a stair head cast into the roof is a rigid mass too big for a
    settle solver to catch cheaply (`urban_fire.dress_roof_urban`'s own
    docstring: pulling its bulkhead+pad out of `roof_plant` is what stopped
    "the settle past its 2200-step cap with five bodies still moving").
    `roof_tank`, `roof_mast` and `roof_prop` (the condenser row, vent
    stacks, and the comms mast) stand loose on the deck and follow their
    roof — they are what `ctx["roof_plant"]` means everywhere it is
    consumed (`quake_flow._b_settle_roof_plant`).

    MATCHED BY IDENTITY, NEVER PROXIMITY: the exact `of` tag `_place()`
    writes, ``"{basename}@{x:.1f},{y:.1f}"``. Matching by nearest-position
    instead fails on a real street, where the neighbour is closer than the
    far side of the same building — this module's own docstring records the
    same failure, independently, for exactly this reason.

    A building the base layout did NOT dress — not on `flat_roof`, below a
    height gate, `building_props.enabled: false` when the scene was
    generated, or simply not in `_plans/gac_buildings.json` and never
    resolver-eligible — returns ``([], [])``. That building's disaster pass
    then runs with an empty plant list, which is exactly how it behaves
    today: nothing to adopt is not an error, it is the common case.

    *placements* must be the placement list AFTER `apply_placements` has run
    (each prop's `prim_path` set) — a path-less prop cannot be handed to a
    rigid-body solver, so one is silently dropped rather than raised on.
    """
    nm = _name_of(str(usd))
    tag = "%s@%.1f,%.1f" % (nm, float(x), float(y))
    fixed, plant = [], []
    for p in placements:
        if p.get("of") != tag:
            continue
        path = p.get("prim_path")
        if not path:
            continue
        cat = p.get("category")
        if cat in ROOF_FIXED_KINDS:
            fixed.append(path)
        elif cat in ROOF_PLANT_KINDS:
            plant.append(path)
    return fixed, plant


if __name__ == "__main__":
    # `python3 scene_gen/detail/gac_props.py` — regenerate the building
    # material audit (`_dump_material_audit`'s own docstring). No `pxr`
    # needed: this only reads the already-measured `_plans/gac_faces.json`.
    _dump_material_audit()
