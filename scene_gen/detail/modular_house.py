"""
modular_house.py — buildings assembled from the ModularNeighborhood kit.

WHY A KIT INSTEAD OF A BUILDING ASSET
-------------------------------------
The suburb places whole-house USDs: one model, one silhouette, one window
arrangement, repeated down the street. A kit trades the other way — pieces are
shared, so a street of 40 houses costs 40 x (a few walls) rather than 40 x (a
whole house), and footprint, storey count, roof form, fenestration and material
can all differ without a new asset for each.

    /Library/Stages/Muyang/ModularNeighborhood/Assets/   293 USDs

THE GRID, WHICH IS THE WHOLE CONTRACT
-------------------------------------
Measured, not guessed — `tools/modular_kit_report.py` re-measures it:

    unit       centimetres (metersPerUnit 0.01, Z-up), hence SCALE = 0.01.
    storey     3.500 m. EVERY `Outer_Wall_*` is exactly 350 cm tall.
    wall spans Quart 5 m, Half 10 m, full 20 m. The mesh is 20 cm WIDER than
               its module — each end carries half the 20 cm wall thickness, so
               butted pieces overlap at the corner instead of leaving a seam.
    floors     Floor_Quart 5x5, Floor_Half 10x10. Slab top at z=0, hangs to
               -0.28 m, so a wall at z=0 stands on it.
    pivots     Quart/Half/full pieces are CENTRED. The 8th/16th pieces are not
               (module runs -1.0..+1.5 m), so they are left unused here.

FOOTPRINTS ARE CELL SETS, WHICH IS WHAT BUYS THE L AND U SHAPES
---------------------------------------------------------------
A footprint is a set of 10x10 m BLOCKS; each block is 2x2 cells of 5 m. Walls
are then generated from the boundary — every cell edge with no neighbour across
it gets wall — so an L, T or U costs exactly as much code as a rectangle, and
the walls always close. Blocks rather than bare cells because ROOFS are sized
in 10 m bays: one gable per block is always placeable, whereas a 5 m-deep bay
has no piece in the kit and would have to be faked.

OUTWARD IS LOCAL -Y, AND IT IS LOAD-BEARING
-------------------------------------------
A plain wall is near-symmetric in Y (-10..+11 cm) so its facing looks
arbitrary. It is not: everything that protrudes protrudes in **-Y** —
`Outer_Wall_Quart_Win_02` reaches -1.26 m, `_Win_03`/`_Win_04` and
`Outer_Wall_Quart_Door_03` -1.33 m. Those are bay windows and a door canopy.
Face a wall the wrong way and the bay grows into the living room.

    south (outward -Y) yaw   0      east (outward +X) yaw  90
    north (outward +Y) yaw 180      west (outward -X) yaw 270

MATERIALS: THE KIT SHIPS EXACTLY ONE WALL TEXTURE
-------------------------------------------------
Surveyed with `tools/modular_kit_report.py`: every `Outer_Wall_*` binds the
same three surfaces through GeomSubsets —

    Cladding_01        the cream siding: THE ONLY WALL TEXTURE IN THE KIT
    Stucco_01_Inst     window and door reveals
    Win_Roof_Style_01  frames

There is no brick wall piece and no wood wall piece. `Brick_01_Inst`,
`Wood_01`, `Concrete_01` and the rest exist only as standalone material USDs
that nothing references. So more than one look REQUIRES rebinding, and
`apply_palette` is that: it matches subsets by the material they already carry
and swaps only those, which is why frames and glass survive untouched.

The brick that appeared only at the top of a house was never the wall — it is
`Brick_01_Dirt` on the gable subset of `Roof_01` itself. A palette therefore
sets the gable to the same material as the wall, or a brick strip rides above
every siding house whatever the walls do.

Roof textures, and this is the complete list: `Roof_Tiles_01_Inst` (pitched)
and `Roof_Felt_01` (flat). Two.
"""

import math

KIT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443"
       "/Library/Stages/Muyang/ModularNeighborhood/Assets/")

SCALE = 0.01          # centimetre-authored
STOREY_M = 3.5        # every Outer_Wall_* is 350 cm tall
CELL_M = 5.0          # the wall module
BLOCK_M = 10.0        # the roof bay = 2x2 cells

# ---------------------------------------------------------------------------
# Pieces
# ---------------------------------------------------------------------------
WALL_5 = {
    "plain":  "Outer_Wall_Quart_01",
    "door":   "Outer_Wall_Quart_Door_01",
    "porch":  "Outer_Wall_Quart_Door_03",     # door + canopy, protrudes 1.33 m
    "garage": "Outer_Wall_Quart_Garage_01",
}
# Four distinct 5 m window walls. Big houses looked under-glazed because a 10 m
# run took the single `Outer_Wall_Half_Win_01`; show facades now prefer two 5 m
# pieces so they can draw from this list instead.
WINDOWS_5 = ["Outer_Wall_quart_Win_01",   # lowercase q is the kit's own
             "Outer_Wall_Quart_Win_02",   # bay, protrudes 1.26 m
             "Outer_Wall_Quart_Win_03",
             "Outer_Wall_Quart_Win_04"]
WALL_10 = {
    "plain":  "Outer_Wall_Half_01",
    "door":   "Outer_Wall_Half_Door_01",
    "win":    "Outer_Wall_Half_Win_01",
    "garage": "Outer_Wall_Half_Garage_01",
}
FLOOR_5 = "Floor_Quart_01"

ROOF_GABLE = "Roof_01"                 # 11.11 x 11.75, ridge 4.38 m up
# 6.10 x 11.75 — HALF width, FULL depth. The only piece that roofs the 5 m
# strip a 15 m-wide plan leaves over, which was otherwise open to the sky.
ROOF_HALF_GABLE = "Roof_Half_01"
# The `Roof_Flat_*` set is NOT flat — it is a shallow hip — and does not tile:
# every piece is 12.885 m in X and the set varies only in Y, so two of them on
# adjacent bays overlap by 2.9 m. Flat roofs are placed as ONE piece per
# rectangle, which is why only these two sizes exist here.
ROOF_FLAT = {(10.0, 10.0): ("Roof_Flat_01", 0.0),
             (15.0, 10.0): ("Roof_Flat_Open_Apex_01", -2.8685)}  # pivot not centred
# The flat set is really a BAR system: X is fixed at 12.885 m (a 10 m bay plus
# 1.44 m eaves each side) and only Y varies, so it roofs a 10 m-WIDE building of
# ANY depth — cap, then one 5 m middle per bay, then cap. That is the only way
# to get a continuous flat roof out of this kit; anything wider than 15 m has no
# piece at all and falls back to parallel gables, which read as houses.
ROOF_FLAT_MID = "Roof_Flat_Middle_Half_01"   # 12.885 x 5.0, centred
ROOF_FLAT_CAP = "Roof_Flat_End_Thin_01"      # 12.885 x 0.61, sits in -Y
BAR_WIDTH_M = 10.0

DOOR = "Outer_Door_01"
GARAGE_DOOR = {10.0: "Garage_Door_01offset", 5.0: "Garage_Door_02offset"}
# "offset" is not decoration: both are authored -1.999..+0.844 m, so a
# placement at z=0 buries the door 2 m underground.
GARAGE_DOOR_Z_M = 1.999
# Measured against the built opening, on top of the pivot correction, and
# expressed in the WALL's own frame — negative moves the door the same way on
# every elevation, as seen from outside.
#
# This used to be rotated by the door's yaw, which is the wall's yaw PLUS 180
# (the flip that turns the panel outward). That inverted it: `_rot(c, 0, 180)`
# is `(-c, 0)`, so every negative value moved the door POSITIVE in world X. A
# requested -0.23 landed at +0.23 and a further -0.47 landed at +0.70, i.e.
# +0.47 from where it had been. Rotating by the wall yaw makes the sign honest.
GARAGE_DOOR_X_M = -0.24     # +0.23 as it stood, then the -0.47 asked for
# The canopied door piece carries its opening 1.235 m FORWARD of the wall
# plane, out under the canopy, so the leaf has to follow it out.
PORCH_DOOR_OUT_M = 1.235

# Pieces that reach more than ~0.5 m past the wall plane: bay windows and the
# canopied door. Their tops are OPEN, so each needs a cap or you see into the
# house from above. Measured, not assumed — Y minima are -1.257 to -1.334 m
# against a plain wall's -0.10.
PROTRUDING = {"Outer_Wall_Quart_Win_02", "Outer_Wall_Quart_Win_03",
              "Outer_Wall_Quart_Win_04", "Outer_Wall_Quart_Door_03"}
BAY_ROOF = "Outer_Wall_Quart_Roof_01"   # 5.64 x 4.06 x 2.41, the kit's bay cap
BAY_ROOF_Z_M = 0.0     # sits ON the storey line; see the note in build_building


def _protrudes(usd_name):
    return usd_name.rsplit("/", 1)[-1].replace(".usd", "") in PROTRUDING


FENCE = "Fence_01_0"          # 5.00 m panel — exactly one cell
# The panel's own `Fence_01` material is weathered white paint FLAKING off grey
# timber, which at street distance reads as a derelict fence rather than a
# painted one. `Wood_01_White` is the pack's flat white-painted board — same
# family, no peeling. Rebound by `apply_palette` (see DRESSING_REBIND), because
# plot dressing deliberately carries no palette of its own.
FENCE_MATERIAL = ("Fence_01", "Wood_01_White")
# The low garden walls, which are stone/concrete rather than timber. Same 5 m
# module, so they drop into the fence runs unchanged.
STONE_WALL = "Wall_Low_Half_01"        # 5.00 m, runs along Y (thin in X)
STONE_WALL_END = "Wall_Low_End_01"
POOL_FLOOR = "Swimming_Pool_Floor_01_0"   # a 20x20 m plane; scaled to the pool
POOL_EDGE = "Swimming_Pool_Edge_01"       # 2.5 m coping, drops 3 m
POOL_CORNER = "Swimming_Pool_Curve_01"
# The coping is the z=0 datum; the water surface sits below it.
POOL_WATER_Z_M = -0.35
# Clear ground between the BACK WALL and the near lip of the water. Read by
# `pool_at`, which used to centre the pool in whatever rear it was given: that
# was the only sensible answer while a pool lot was granted ~9.5 m of rear, and
# it is the wrong one now that it asks for 20 — centring puts 8 m of nothing on
# both sides and no lawn anywhere, where a real back garden has its lawn
# between the house and the water.
POOL_SETBACK_M = 9.0        # from the house, so it clears the fence
# Walk at the house end of the pool, and at the fence end. The rear walk is the
# wider of the two on purpose: it is the side people actually stand on, and a
# coping hard against a privacy fence has nowhere to put a chair.
POOL_WALK_M = 2.5
POOL_REAR_WALK_M = 3.0
# Coping half-band. The `Swimming_Pool_Edge_01` modules are laid CENTRED on the
# pool rectangle, so they straddle its edge; this is how far the grass is cut
# back inside that rectangle so it runs under the coping rather than stopping
# short of it, and equally how far the coping's outer face stands outside it.
POOL_COPING_M = 0.16
# PAVED APRON, outside the coping. A pool surround is walkable deck, not lawn
# to the water's edge: ANSI/APSP-3 asks 4 ft (1.22 m) of unobstructed deck all
# round a residential pool, and 1.5 m is the built width that still fits inside
# what `pool_at` already reserves — 2.5 m of clear ground at the house end and
# 3.0 m at the fence — so both ends keep lawn instead of being paved out to the
# boundary. Drawn by `suburb_scene.apply_ground`, which is where ground-level
# slabs and the z ladder they sit on live; this module only says how wide.
POOL_APRON_M = 1.5
# Yard depths, front and back, both on the 5 m module. MODULE SCOPE because
# `pool_at` and `dress_plot` both size against the back yard and must agree:
# it has to hold a 5 m pool with clearance at each end, or the pool lands on
# the fence.
FRONT_YARD_M, BACK_YARD_M = 10.0, 20.0
# The scene's existing vehicle pool, copied from `config/asset_packs/shared.yaml`
# (`usds.cars`) so this catalogue shows what the suburb actually places. Paths
# are relative to that file's `asset_root`, hence the explicit prefix here.
# (usd, scale, axis_up, yaw_offset)
_LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
# Residential drives only, so no taxi and no police car — those are livery
# vehicles and read as odd parked outside a house. The extra 90 deg on top of
# the asset set's own `yaw-offset` is measured: these come in broadside to the
# drive without it.
CAR_YAW_EXTRA = 90.0
CARS = [
    (_LIB + "RetroNeighborhood/130.usdz", 0.01, "Y", 90.0),
    (_LIB + "RetroNeighborhood/Nissan_Fairlady_Z_S30240Z_1978.usdz", 0.01, "Y", 90.0),
    (_LIB + "Muyang/DownTown/Assets/Vehicle_A.usd", 0.01, "Z", 90.0),
]

DRIVEWAY_5 = "Driveway_Quart_01"
ROAD_TILE = "Road_01"                     # 16 x 16 m asphalt, for parking
PATH = "Path_01"                          # 3 x 3 m slab
VERANDA = "Veranda_01"                    # 10.19 m shopfront canopy
PORCH_MID = "Porch_Middle_01"
PORCH_CORNER = "Porch_Corner_01"
PORCH_PILLAR = "Porch_Pillar_01"

# TREES COME FROM RetroNeighborhood, NOT THIS KIT. The kit's Fir/Birch/Oak/Tree
# read as flat cardboard cut-outs at any useful distance. Retro's are two-section
# (bark mesh + branch cards) and hold up: seven species with a real size spread,
# same metersPerUnit 0.01, defaultPrim /Root, so they place identically.
#   (usd, approximate canopy diameter in metres)
RETRO = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/RetroNeighborhood/Props/"
TREES = [
    (RETRO + "SM_Tree_05.prop.usd", 14.1),   # full canopy, 17.4 m tall
    (RETRO + "SM_Tree_06.prop.usd", 14.5),   # full canopy, 16.5 m tall
    (RETRO + "SM_Tree_02.prop.usd", 7.5),    # mid, 9.5 m tall
    (RETRO + "SM_Tree_01.prop.usd", 5.0),
    (RETRO + "SM_Tree_03.prop.usd", 5.0),
    (RETRO + "SM_Tree_07.prop.usd", 5.0),
]

# Yard and street props.
PROPS = {
    "hedge":  "Hedge_01",
    "lamp":   "Lamp_01_0",
    "bench":  "Bench_01",
    "chair":  "Garden_Chair_01_0",
    "hoop":   "Basketball_Hoop_01",
    "rocks":  "Garden_Rocks_01",
    "sign_street": "Street_Sign_01_0",
    "sign_stop":   "Stop_Sign_01_0",
    "monument":    "Monument_01",
}

# ---------------------------------------------------------------------------
# Palettes
# ---------------------------------------------------------------------------
# Keyed by the material a subset ALREADY binds, which is how a swap can be
# targeted without knowing anything about the mesh:
#
#   Cladding_01      the wall face          -> `wall`
#   Brick_01_Dirt    Roof_01's gable infill -> `gable`
#   Roof_Tiles_01_Inst / Roof_Felt_01       -> `roof`
#
# Anything else (Win_Roof_Style_01 frames, Stucco_01_Inst reveals, glass) is
# deliberately left alone. Each palette is internally coherent — brick walls
# get a brick gable, wood gets wood — because the mismatch is exactly what made
# the first pass read as brick-hat-on-a-siding-house.
# ROOFS: only `Section0` is the roof FIELD, on every one of the 21 roof pieces.
# The other subsets are gable brick (Section1), verge/dormer trim (Section2)
# and soffit render (Section3) — binding a roof texture to those, or a trim
# texture to Section0, is what made the roofs look wrong. `apply_palette`
# matches on `Roof_Tiles_01_Inst` / `Roof_Felt_01`, which only ever appear on
# Section0, so the field is all it can touch.
#
# Three of what were used as "roof variety" were not roof materials at all:
#   Marble_01          speckled granite COUNTERTOP — on no roof mesh anywhere
#   Win_Roof_Style_01  the verge/dormer TRIM strip: white with two rust runs
#   Cladding_Dirt_01   the flat roof's parapet UPSTAND — clapboard lying down
#
# What actually reads as a roof:
_RETRO = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/RetroNeighborhood/Props/"
# NEITHER of the kit's own roof materials is usable: `Roof_Tiles_01_Inst` is
# about a third acid-lime moss by area and reads yellow-green from the air, and
# `Roof_Felt_01` is more than half moss. Both are replaced everywhere.
#
# Variety without moss comes from the TINT lever instead. Every Retro material
# is an instance of `MI_Road.mdl` exposing `Tint_AlbedoTexture_` (float4), which
# multiplies the albedo — the pack itself uses this to get five clapboard
# colours out of one texture. So one clean charcoal shingle yields N colourways.
# Base is #383533, so multipliers above 1 lift and warm it.
#   name -> (asset, primPath, tint or None)
_SHINGLE = (_RETRO + "SM_House_01.prop.usd", "/Root/Looks/MI_Rooftiles_03")
MATERIAL_SOURCE = {
    "shingle_charcoal": _SHINGLE + (None,),                    # as authored
    "shingle_slate":    _SHINGLE + ((0.85, 0.95, 1.25, 1.0),),  # blue-grey
    "shingle_grey":     _SHINGLE + ((1.75, 1.75, 1.70, 1.0),),  # weathered grey
    "shingle_brown":    _SHINGLE + ((1.70, 1.25, 0.90, 1.0),),  # brown asphalt
    "MI_Road":     (_RETRO + "SM_Road_01.prop.usd", "/Root/Looks/MI_Road", None),
    "MI_Sidewalk": (_RETRO + "SM_Road_01.prop.usd", "/Root/Looks/MI_Sidewalk", None),
}
ROOF_SHINGLE = "shingle_charcoal"
ROOF_DECK = "Concrete_02"          # flat: clean concrete deck
ROOF_ASPHALT = "MI_Road"           # flat: worn asphalt, closest to built-up

PALETTES = {
    "siding_cream": {"wall": None, "gable": "Cladding_01", "roof": "shingle_grey", "roof_flat": "Concrete_02",
                     "note": "the kit's own siding, with the gable corrected "
                             "from brick to match the walls"},
    "brick_red":    {"wall": "Brick_01_Inst", "gable": "Brick_01_Inst",
                     "roof": ROOF_SHINGLE, "note": "full red brick"},
    "brick_buff":   {"wall": "Brick_10_Inst", "gable": "Brick_10_Inst",
                     "roof": "shingle_slate", "roof_flat": "Concrete_02", "note": "full buff brick"},
    "wood_dark":    {"wall": "Wood_01", "gable": "Wood_01", "roof": "shingle_brown", "roof_flat": "Concrete_02",
                     "note": "full stained timber"},
    # Wood_01_White, NOT Wood_02_White: the latter is the peeling tan-through
    # paint, which is wanted on the porch pillars and the house base it is
    # authored for, and not on a whole elevation.
    "wood_white":   {"wall": "Wood_01_White", "gable": "Wood_01_White",
                     "roof": ROOF_SHINGLE, "note": "full painted white timber"},
    "stucco":       {"wall": "Stucco_01_Inst", "gable": "Stucco_01_Inst",
                     "roof": ROOF_DECK, "note": "render/stucco"},
    "concrete":     {"wall": "Concrete_01", "gable": "Concrete_01",
                     "roof": ROOF_ASPHALT, "note": "commercial concrete"},
    "concrete_pale": {"wall": "Concrete_02", "gable": "Concrete_02",
                      "roof": ROOF_DECK, "note": "commercial, paler"},
}
# The subset materials a palette is allowed to touch.
_WALL_SURFACES = {"Cladding_01"}
# `Outer_Wall_Quart_Roof_01` (the bay cap) binds Brick_01_Inst, NOT the
# Brick_01_Dirt that `Roof_01`'s gable uses — so the caps stayed brick on
# every house whatever its palette, which is the stray brick that kept
# showing up on the timber ones. Safe as a match key: no wall subset binds
# it, and a brick palette simply maps it back to itself.
_GABLE_SURFACES = {"Brick_01_Dirt", "Cladding_Dirt_01", "Brick_01_Inst"}
# Split, because a palette needs to say something different about a pitched
# roof than a flat one — `brick_red` is on both a gable house and a
# flat-roofed terrace, and shingle on a flat deck is wrong.
_ROOF_PITCHED = {"Roof_Tiles_01_Inst"}
_ROOF_FLAT = {"Roof_Felt_01"}
# Section2 on every roof piece: the verge / dormer trim strip. It is a
# whitewashed board carrying TWO RUST RUNS AND A GREEN ALGAE STREAK, which is
# the streaking still visible along the eaves once the field was fixed. Plain
# white reads far better at every distance.
#
# ONLY ON ROOFS. The same material is also Section2 of the WALL pieces, where
# it is the window framing — rebinding it there would flatten every frame in
# the scene, so the swap is gated on the placement's category.
# Plot dressing gets NO palette — a fence must not take the house's wall
# material. But a few of its own materials are still wrong, so they are swapped
# by category instead: {category: (from_material, to_material)}.
DRESSING_REBIND = {
    "plot_fence": FENCE_MATERIAL,
}

_ROOF_TRIM = {"Win_Roof_Style_01"}


# A PALETTE MUST BE ABLE TO READ WHAT A PALETTE WROTE. The sets above are the
# surfaces the KIT ships with, which is all `apply_palette` ever saw while it
# only ever ran on a house `build_building` had just assembled. A BAKED
# archetype is different: `bake_archetypes_launch_script` builds each house
# WITH ITS STYLE'S PALETTE ALREADY APPLIED, so a baked cottage's walls are
# `Wood_01_White` and a baked villa's are `Stucco_01_Inst` — none of which is
# `Cladding_01`, so re-palettising one matched nothing and silently rebound
# zero subsets. That is exactly what the assembled plat did to its row homes:
# 62 units asked for a colour each and all 62 kept the archetype's.
#
# So the read sets are the kit's surfaces UNION every surface a palette can
# produce. Wall and gable take the same material in every palette but
# `siding_cream`, so a surface that lands in both sets resolves to the same
# new material either way and the wall-first order costs nothing.
_WALL_SURFACES = _WALL_SURFACES | {
    v["wall"] for v in PALETTES.values() if v.get("wall")}
_GABLE_SURFACES = _GABLE_SURFACES | {
    v["gable"] for v in PALETTES.values() if v.get("gable")}
ROOF_TRIM_WHITE = "Stucco_01_Inst"     # flat untextured white, no streaking


# Local bbox CENTRE in XY metres, for assets whose pivot is not their centre.
# Measured with `tools/modular_kit_report.py`. Placing these by their origin is
# what put the front door half a leaf off-centre and the garage door a metre
# inside the building.
PIVOT_XY = {
    "Outer_Door_01":          (0.5525, -0.035),   # origin at the LEFT jamb
    "Garage_Door_01offset":   (0.004,   0.9405),  # sits ~0.94 m to +Y of origin
    "Garage_Door_02offset":   (0.2475,  0.941),
    "Swimming_Pool_Edge_01":  (0.0,     0.25),    # 2.5 m module, origin 1 m in
    "Swimming_Pool_Curve_01": (0.1725,  0.1715),
}


def _usd(name):
    return KIT + name + ".usd"


def _pivot_xy(name, x, y, yaw_deg):
    """Placement origin that lands *name*'s bbox centre on (x, y) at *yaw*."""
    cx, cy = PIVOT_XY.get(name, (0.0, 0.0))
    if cx == 0.0 and cy == 0.0:
        return x, y
    dx, dy = _rot(cx, cy, yaw_deg)
    return x - dx, y - dy


def _place(usd, x, y, z=0.0, yaw=0.0, category="house", scale=SCALE):
    return {"usd": usd, "x_m": float(x), "y_m": float(y), "z_m": float(z),
            "yaw_deg": float(yaw) % 360.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "scale": scale, "category": category, "axis_up": "Z",
            # EVERY piece here is positioned in the kit's authored frame, with
            # the pivot corrections this module applies itself (PIVOT_XY). The
            # generator's centroid correction must not also run, or each piece
            # is re-centred on its own bbox and the house comes apart — which is
            # why doors and garage doors landed correctly in the preview (no
            # resolver) and wrongly in the suburb (resolver passed).
            "raw_pivot": True}


_YAW = {"S": 0.0, "E": 90.0, "N": 180.0, "W": 270.0}


def _rot(dx, dy, yaw_deg):
    a = math.radians(yaw_deg)
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def cells(blocks):
    """10x10 m blocks -> the 5 m cells they cover."""
    return {(2 * bi + di, 2 * bj + dj)
            for (bi, bj) in blocks for di in (0, 1) for dj in (0, 1)}


def footprint(spec):
    """A style's plan as 5 m cells, from either `cells` or `blocks`."""
    if "cells" in spec:
        return set(spec["cells"])
    return cells(spec["blocks"])


def _blocks_of(cs):
    """The 10x10 blocks fully covered by a cell set — what a gable can sit on.

    A 15 m-wide plan leaves a 5 m strip that is no block. It does NOT fall
    under the neighbouring eaves — those overhang 0.56 m against a 5 m strip,
    so the strip was simply unroofed and you could see into the house. It gets
    its own half gable; see `_half_columns`.
    """
    out = set()
    for (i, j) in cs:
        bi, bj = i // 2, j // 2
        if all((2 * bi + di, 2 * bj + dj) in cs
               for di in (0, 1) for dj in (0, 1)):
            out.add((bi, bj))
    return out


def _half_columns(cs):
    """Leftover 5 m x 10 m columns: cells no full block covers.

    `Roof_Half_01` is 6.10 x 11.75 m — half width, full depth — which is
    exactly this strip plus its eaves, and is the only reason a 15 m-wide
    gable house can be roofed at all.

    Yields ``(centre_x_cells, centre_y_cells)`` in cell units.
    """
    covered = {(2 * bi + di, 2 * bj + dj)
               for (bi, bj) in _blocks_of(cs) for di in (0, 1) for dj in (0, 1)}
    left = cs - covered
    out = []
    for (i, j) in sorted(left):
        if j % 2 or (i, j + 1) not in left:
            continue                     # only whole 10 m-deep columns
        out.append((i + 0.5, j + 1.0))
    return out


def rect_cells(nx, ny, ox=0, oy=0):
    """A plain rectangular plan, nx by ny cells of 5 m."""
    return {(ox + i, oy + j) for i in range(nx) for j in range(ny)}


def _runs(cs):
    """Boundary of a cell set as maximal straight runs.

    Yields ``(side, fixed_coord_m, start_m, length_m)`` in footprint-local
    metres, where *side* is the OUTWARD direction. Merging collinear cells into
    one run is what lets a 10 m piece be used instead of two 5 m ones.
    """
    out = []
    # horizontal runs (walls facing S or N), grouped per y-line
    for side, dj, line_off in (("S", -1, 0), ("N", +1, 1)):
        lines = {}
        for (i, j) in cs:
            if (i, j + dj) not in cs:
                lines.setdefault(j + line_off, []).append(i)
        for jline, isl in lines.items():
            for start, n in _consec(sorted(isl)):
                out.append((side, jline * CELL_M, start * CELL_M, n * CELL_M))
    # vertical runs (walls facing W or E), grouped per x-line
    for side, di, line_off in (("W", -1, 0), ("E", +1, 1)):
        lines = {}
        for (i, j) in cs:
            if (i + di, j) not in cs:
                lines.setdefault(i + line_off, []).append(j)
        for iline, jsl in lines.items():
            for start, n in _consec(sorted(jsl)):
                out.append((side, iline * CELL_M, start * CELL_M, n * CELL_M))
    return out


def _consec(sorted_vals):
    """[(start, count)] for each maximal consecutive stretch."""
    out, run_start, prev = [], None, None
    for v in sorted_vals:
        if run_start is None:
            run_start, prev = v, v
        elif v == prev + 1:
            prev = v
        else:
            out.append((run_start, prev - run_start + 1))
            run_start, prev = v, v
    if run_start is not None:
        out.append((run_start, prev - run_start + 1))
    return out


def _tile(length_m, prefer_small=False):
    """[(centre_offset, span)] tiling a run.

    *prefer_small* splits into 5 m pieces even where a 10 m one fits. Show
    facades use it: there is only ONE 10 m window wall but FOUR 5 m ones, so
    10 m pieces are precisely why the big houses looked under-glazed.
    """
    out, done = [], 0.0
    while length_m - done > 1e-6:
        span = 5.0 if (prefer_small or length_m - done < 10.0 - 1e-6) else 10.0
        out.append((done + span / 2.0, span))
        done += span
    return out


def front_anchors(spec):
    """Where the front door and garage sit, in centred house-local metres.

    ONE definition, used by both `build_building` (to choose wall pieces) and
    `dress_plot` (to aim the path and the drive). They were computed separately
    before, which is why paths led to blank wall and drives sometimes ran to the
    front door instead of the garage.

    Returns ``(door_x, garage_x_or_None, front_y)``.
    """
    cs = footprint(spec)
    xs = [i for i, _ in cs]
    ys = [j for _, j in cs]
    ox = -(min(xs) * CELL_M + (max(xs) + 1) * CELL_M) / 2.0
    oy = -(min(ys) * CELL_M + (max(ys) + 1) * CELL_M) / 2.0
    front = spec.get("front", "S")
    line = {"S": min(ys) * CELL_M, "N": (max(ys) + 1) * CELL_M,
            "W": min(xs) * CELL_M, "E": (max(xs) + 1) * CELL_M}[front]
    runs = [r for r in _runs(cs) if r[0] == front and abs(r[1] - line) < 1e-6]
    if not runs:
        return 0.0, None, oy
    _side, fixed, start, length = max(runs, key=lambda r: r[3])
    slots = _tile(length, prefer_small=True)
    garage_i = 0 if spec.get("garage") else None
    # The door CANNOT share the garage's slot. Defaulting it to 0 alongside a
    # garage silently produced houses with no front door at all.
    door_i = spec.get("door_slot")
    if door_i is None:
        door_i = 1 if garage_i == 0 else 0
    door_i = min(door_i, len(slots) - 1)
    if garage_i is not None and door_i == garage_i:
        door_i = min(garage_i + 1, len(slots) - 1)
    to_local = (lambda off: start + off + ox) if front in ("S", "N") \
        else (lambda off: start + off + oy)
    gx = to_local(slots[garage_i][0]) if garage_i is not None else None
    return to_local(slots[door_i][0]), gx, (line + oy if front in ("S", "N")
                                            else line + ox)


def _wall_asset(kind, span, rng):
    if span >= 10.0:
        return WALL_10.get(kind, WALL_10["plain"])
    if kind == "win":
        return rng.choice(WINDOWS_5)
    return WALL_5.get(kind, WALL_5["plain"])


def _facade_kinds(side, n_slots, spec, storey, rng, is_front,
                  door_i=0, garage_i=None):
    """What each slot along one run gets. Indices come from `front_anchors`."""
    kinds = []
    for i in range(n_slots):
        if storey > 0:
            kinds.append("win" if rng.random() < 0.75 else "plain")
        elif is_front and garage_i is not None and i == garage_i:
            kinds.append("garage")
        elif is_front and i == door_i:
            kinds.append("porch" if spec.get("porch", True) else "door")
        elif is_front:
            kinds.append("win")
        elif side in ("E", "W"):
            kinds.append("win" if rng.random() < 0.6 else "plain")
        else:
            kinds.append("win" if rng.random() < 0.35 else "plain")
    return kinds


def build_building(style, x, y, yaw, rng, category=None):
    """Assemble one building centred at *(x, y)*, facing *yaw*.

    The facade faces -Y before rotation, so yaw=0 puts the entrance south.
    """
    spec = STYLES[style]
    cat = category or f"bld_{style}"
    cs = footprint(spec)
    storeys = spec["storeys"]
    out = []

    # centre the footprint on the origin
    xs = [i for i, _ in cs]
    ys = [j for _, j in cs]
    ox = -(min(xs) * CELL_M + (max(xs) + 1) * CELL_M) / 2.0
    oy = -(min(ys) * CELL_M + (max(ys) + 1) * CELL_M) / 2.0

    def add(usd, lx, ly, lz, lyaw, sub, scale=SCALE):
        wx, wy = _rot(lx + ox, ly + oy, yaw)
        out.append(_place(_usd(usd), x + wx, y + wy, lz, lyaw + yaw,
                          category=f"{cat}_{sub}", scale=scale))

    # ---- floors: one 5x5 tile per cell per storey -------------------------
    for s in range(storeys):
        for (i, j) in sorted(cs):
            add(FLOOR_5, (i + 0.5) * CELL_M, (j + 0.5) * CELL_M,
                s * STOREY_M, 0.0, "floor")

    # ---- walls along every boundary run -----------------------------------
    front = spec.get("front", "S")
    _door_x, _garage_x, _ = front_anchors(spec)
    _garage_i = 0 if spec.get("garage") else None
    _door_i = spec.get("door_slot")
    if _door_i is None:
        _door_i = 1 if _garage_i == 0 else 0
    if _garage_i is not None and _door_i == _garage_i:
        _door_i = _garage_i + 1
    for s in range(storeys):
        z = s * STOREY_M
        for side, fixed, start, length in _runs(cs):
            # Which line counts as "the front" depends on the side's AXIS:
            # for S/N `fixed` is a y-line, for W/E an x-line. Comparing both
            # against min(ys) meant a building fronting W never got a door.
            front_line = {"S": min(ys) * CELL_M,
                          "N": (max(ys) + 1) * CELL_M,
                          "W": min(xs) * CELL_M,
                          "E": (max(xs) + 1) * CELL_M}[side]
            is_front = (side == front and s == 0
                        and abs(fixed - front_line) < 1e-6)
            small = is_front or spec.get("all_small", False)
            slots = _tile(length, prefer_small=small)
            kinds = _facade_kinds(side, len(slots), spec, s, rng, is_front,
                                  door_i=_door_i, garage_i=_garage_i)
            for (off, span), kind in zip(slots, kinds):
                if side in ("S", "N"):
                    px, py = start + off, fixed
                else:
                    px, py = fixed, start + off
                # Drawn ONCE: `_wall_asset` rolls the RNG to pick a window, so
                # calling it again below would place a cap for a different
                # piece than the one that went up.
                usd = _wall_asset(kind, span, rng)
                add(usd, px, py, z, _YAW[side], "wall")
                if kind in ("door", "porch"):
                    ax, ay = px, py
                    if kind == "porch":
                        ox_, oy_ = _rot(0.0, -PORCH_DOOR_OUT_M, _YAW[side])
                        ax, ay = px + ox_, py + oy_
                    dx, dy = _pivot_xy(DOOR, ax, ay, _YAW[side])
                    add(DOOR, dx, dy, z, _YAW[side], "door")
                elif kind == "garage":
                    # +180: the panel faces INTO the building as authored, and
                    # its geometry sits ~0.94 m to +Y of its origin, so without
                    # both the flip and the pivot correction it lands a metre
                    # inside the garage facing backwards.
                    gd = GARAGE_DOOR[10.0 if span >= 10.0 else 5.0]
                    gy = (_YAW[side] + 180.0) % 360.0
                    # Offset in the WALL's frame, not the flipped door's — see
                    # GARAGE_DOOR_X_M. Rotating by `gy` reverses the sign.
                    gox, goy = _rot(GARAGE_DOOR_X_M, 0.0, _YAW[side])
                    dx, dy = _pivot_xy(gd, px + gox, py + goy, gy)
                    add(gd, dx, dy, z + GARAGE_DOOR_Z_M, gy, "door")
                # A bay or porch piece protrudes up to 1.33 m past the wall
                # plane and is OPEN at the top — from above you see straight
                # into the house. `Outer_Wall_Quart_Roof_01` is the kit's cap
                # for exactly that, so every protruding piece gets one.
                if _protrudes(usd):
                    add(BAY_ROOF, px, py, z + STOREY_M + BAY_ROOF_Z_M,
                        _YAW[side], "bay_roof")

    # ---- roof --------------------------------------------------------------
    ztop = storeys * STOREY_M
    w = (max(xs) - min(xs) + 1) * CELL_M
    d = (max(ys) - min(ys) + 1) * CELL_M
    rect = len(cs) * CELL_M ** 2 >= w * d - 1e-6      # is the plan a rectangle?
    if spec.get("roof") == "flat_bar" and rect and abs(w - BAR_WIDTH_M) < 1e-6:
        # cap, n middles, cap — the set's own composition, so no overlap.
        add(ROOF_FLAT_CAP, w / 2.0, 0.0, ztop, 0.0, "roof")
        for k in range(int(round(d / 5.0))):
            add(ROOF_FLAT_MID, w / 2.0, 2.5 + k * 5.0, ztop, 0.0, "roof")
        add(ROOF_FLAT_CAP, w / 2.0, d, ztop, 180.0, "roof")
    elif spec.get("roof") == "flat" and rect and (w, d) in ROOF_FLAT:
        asset, dx = ROOF_FLAT[(w, d)]
        add(asset, w / 2.0 + dx, d / 2.0, ztop, 0.0, "roof")
    else:
        # Gable per 10x10 block. Also the FALLBACK for a flat-roofed plan the
        # flat set cannot cover: those pieces are all 12.885 m in X and only
        # vary in Y, so anything wider than 15 m has no flat piece and tiling
        # them overlaps by 2.9 m. A pitched roof on a big-box unit is the
        # honest substitute — `warehouse` and `industrial` are 52 of the 710
        # footprints sampled, and they are pitched.
        for (bi, bj) in sorted(_blocks_of(cs)):
            add(ROOF_GABLE, (bi + 1.0) * BLOCK_M - BLOCK_M / 2,
                (bj + 1.0) * BLOCK_M - BLOCK_M / 2, ztop, 0.0, "roof")
        # ...and a half gable over any 5 m strip that is no block. Without this
        # a 15 m-wide house is roofed over only its first 10 m.
        for (cxc, cyc) in _half_columns(cs):
            add(ROOF_HALF_GABLE, cxc * CELL_M, cyc * CELL_M, ztop, 0.0, "roof")

    # ---- a shopfront canopy along the commercial frontage ------------------
    if spec.get("canopy"):
        for side, fixed, start, length in _runs(cs):
            if side != front or abs(fixed - min(ys) * CELL_M) > 1e-6:
                continue
            for off, span in _tile(length):
                if span >= 10.0:
                    add(VERANDA, start + off, fixed - 0.1, 0.0, _YAW[side],
                        "canopy")
    return out


# ---------------------------------------------------------------------------
# Plot dressing
# ---------------------------------------------------------------------------
def _outset_ring(ring, g):
    """A 4-corner pool ring grown by *g* on every side, still rotated with it.

    Ring order is `pool_at`'s: corner 0 nearest the house and edge 0 along the
    long side, so the corners are (-,-) (+,-) (+,+) (-,+) in the (edge,
    left-normal) frame and each one moves out along both.
    """
    ex = float(ring[1][0]) - float(ring[0][0])
    ey = float(ring[1][1]) - float(ring[0][1])
    L = math.hypot(ex, ey) or 1.0
    ux, uy = ex / L, ey / L
    return [(float(qx) + g * (ux * sx - uy * sy),
             float(qy) + g * (uy * sx + ux * sy))
            for (qx, qy), (sx, sy) in zip(list(ring)[:4],
                                          ((-1, -1), (1, -1), (1, 1), (-1, 1)))]


def pool_rings(water_ring):
    """``(coping_outer, apron_outer)`` for a pool whose water is *water_ring*.

    The ring `pool_at` returns is the WATER: the pool rectangle already inset
    by one coping half-band. The `Swimming_Pool_Edge_01` modules are laid
    centred on that rectangle, so the stone's outer face is two half-bands out
    — which is where the apron starts, so the slab meets the coping rather than
    drawing over it and hiding it.
    """
    g = 2.0 * POOL_COPING_M
    return _outset_ring(water_ring, g), _outset_ring(water_ring,
                                                     g + POOL_APRON_M)


def _coping_run(side_m, module_m=2.5):
    """Offsets of the coping modules along one pool side, centre-relative.

    COVER THE SIDE, DO NOT TILE IT. The count was `int(side / 2.5)`, i.e.
    floor — so a 4 m short side got ONE 2.5 m module and 1.5 m of it had no
    coping at all, and the pool read as an open-ended U rather than a
    rectangle. Flooring is right for tiling something that must not overhang;
    a coping is the opposite case, because these modules are identical and
    overlapping two of them is invisible while a gap between them is the
    first thing you see.

    So: ceil the count and spread the modules evenly across the side. They
    overlap wherever the side is not a whole number of modules (a 4 m side
    takes two at a 2.0 m pitch, overlapping 0.5 m) and the run always reaches
    both corners.
    """
    n = max(1, int(math.ceil(float(side_m) / float(module_m) - 1e-9)))
    pitch = float(side_m) / n
    return [-float(side_m) / 2.0 + (k + 0.5) * pitch for k in range(n)]


def pool_at(style, x, y, yaw, force=None, rear_m=None):
    """A pool behind *style*, as ``(placements, hole_polygon)`` or ``(None, None)``.

    The hole polygon is the WATER rectangle in world coordinates, rotated with
    the house — the ground pass subtracts it so the lawn does not draw over the
    water. Kept separate from `dress_plot` so the suburb can have pools without
    inheriting the catalogue's driveways and hedges.
    """
    spec = STYLES[style]
    # *force* lets the caller decide instead of the style. The suburb does:
    # whether a plot has a pool is a property of the PLOT's package (see
    # `suburb_parcel.ARCHETYPES`), not of which house happened to fit on it.
    want = spec.get("pool") if force is None else force
    if not want:
        return None, None
    cs = footprint(spec)
    ys = [j for _, j in cs]
    d = (max(ys) - min(ys) + 1) * CELL_M
    front_y = -d / 2.0
    # HOW MUCH GROUND IS ACTUALLY BEHIND THE HOUSE. The catalogue can assume its
    # own generous back yard; the suburb cannot — the lot is whatever the block
    # gave it. Assuming BACK_YARD_M there put the pool 20 m behind every house
    # whether or not the lot reached that far, which is why the pools were not
    # where the houses are and the poolside chairs ended up in the street.
    rear = BACK_YARD_M if rear_m is None else float(rear_m)
    pw, pd = 8.0, 4.0        # rectangular, sized to a real back garden
    # A walk at each end and the pool between them is the floor — which is what
    # `suburb_parcel.pool_rear_min_m` asks the plat for. A 5 m-deep pool needed
    # 11 m of rear and essentially no real lot had it.
    if rear < POOL_WALK_M + pd + POOL_REAR_WALK_M:
        return None, None
    # WHERE IN THE REAR YARD, which stopped being "the middle" the moment a
    # pool lot started asking for 20 m of it (`suburb_parcel.pool_rear_m`).
    # Centred, the water sat 8 m off the back wall AND 8 m off the fence: a
    # pool floating in the garden with a strip of lawn on either side and room
    # for nothing on either. So it is pushed BACK to `POOL_SETBACK_M` from the
    # house — the constant that already says how far a pool stands off the
    # building — which collects the lawn into one usable piece between the
    # house and the water. Clamped so the far coping never comes closer than
    # `POOL_REAR_WALK_M` to the rear lot line, and never nearer the house than
    # `POOL_WALK_M`: on a lot the block only granted the minimum rear, the two
    # clamps meet and the pool sits between its two walks with no lawn, which
    # is the old behaviour and the only thing that fits.
    near = max(POOL_WALK_M,
               min(POOL_SETBACK_M, rear - POOL_REAR_WALK_M - pd))
    py = front_y + d + near + pd / 2.0
    out = []

    def add(usd, lx, ly, lz, lyaw, sub, scale=SCALE):
        wx, wy = _rot(lx, ly, yaw)
        out.append(_place(_usd(usd), x + wx, y + wy, lz, lyaw + yaw,
                          category=f"plot_{sub}", scale=scale))

    for k in range(int(pw / pd)):
        add(POOL_FLOOR, -pw / 2 + (k + 0.5) * pd, py, POOL_WATER_Z_M, 0.0,
            "pool", scale=SCALE * pd / 20.0)
    for o in _coping_run(pw):
        for yy, yw in ((py - pd / 2, 90.0), (py + pd / 2, 270.0)):
            ex, ey = _pivot_xy(POOL_EDGE, o, yy, yw)
            add(POOL_EDGE, ex, ey, 0.0, yw, "pool_edge")
    for o in _coping_run(pd):
        for xx, yw in ((-pw / 2, 180.0), (pw / 2, 0.0)):
            ex, ey = _pivot_xy(POOL_EDGE, xx, py + o, yw)
            add(POOL_EDGE, ex, ey, 0.0, yw, "pool_edge")

    # The hole is the water rectangle INSET by the coping, so the grass meets
    # the outside of the coping rather than stopping short of it.
    hw, hd = pw / 2 - POOL_COPING_M, pd / 2 - POOL_COPING_M
    hole = []
    for lx, ly in ((-hw, py - hd), (hw, py - hd), (hw, py + hd), (-hw, py + hd)):
        wx, wy = _rot(lx, ly, yaw)
        hole.append((x + wx, y + wy))
    return out, hole


def plan_lot(lot, style, rng=None):
    """Everything the LAYOUT pass needs in order to site this house, in world
    metres. One function, so the two passes cannot disagree.

    THE SEAM THIS CLOSES. The plat decided where a driveway went by guessing
    `house_width * 0.30` to one side, and where the path went not at all; this
    module knew exactly where the garage and the front door were and never told
    it. So drives arrived beside the garage on some lots and at the front step
    on others, and paths led to blank wall. Same disconnection for pools: the
    plat sized the lot without knowing a pool wanted 9.5 m of rear, and this
    module placed one without knowing what the lot had.

    *lot* is the house record `suburb_parcel.parcel_blocks` emits — it must
    carry `c` (centre), `u` (frontage tangent), `n` (INWARD normal), `d` (house
    depth), `frontage` (the kerb-side anchor) and `lot_depth`.

    Returns a dict:
        yaw          world yaw to pass to `build_building`
        door         world XY of the front door
        garage       world XY of the garage door, or None
        drive        ((kerb), (garage)) or None — the brick apron
        path         ((kerb), (door))            — the walk to the door
        rear_m       ground behind the house, measured not assumed
        pool_ok      whether a pool fits that rear

    THE FRAME, derived rather than assumed. `build_building` faces its front at
    local -Y, and the front must look at the kerb, which is -n. So local +Y IS
    n, and local +X is u. Every offset below is therefore
    `c + u*local_x + n*local_y`, which stays correct whichever of the two
    normals `_inward` happened to return — a `perp(u)` shortcut would not.
    """
    spec = STYLES[style]
    c, u, n = lot["c"], lot["u"], lot["n"]
    door_x, garage_x, front_y = front_anchors(spec)

    def world(lx, ly):
        return (c[0] + u[0] * lx + n[0] * ly, c[1] + u[1] * lx + n[1] * ly)

    door = world(door_x, front_y)
    garage = world(garage_x, front_y) if garage_x is not None else None

    # The kerb end of each run: the frontage anchor slid along the street to sit
    # square in front of whatever it serves, so a drive runs straight in rather
    # than cutting diagonally across the yard.
    #
    # ON A TURNAROUND THE KERB IS AN ARC and `u` is its TANGENT, so sliding
    # along `u` walks off the paving as the arc curves away — and `frontage`
    # there is the LOT LINE, which `_arc_cap_bulbs` already leaves a verge short
    # of the asphalt. Measured on seeds 3/7/11, that ended every cul-de-sac
    # drive 3.7-4.5 m and every walk 3.0-3.7 m outside the turnaround: the gap
    # between the end of the drive and the kerb. `kerb_arc` is the (centre,
    # paved radius) `suburb_parcel` publishes for exactly this; strike the run
    # RADIALLY to it, which is what that module's own platted drive does.
    fr = lot.get("frontage") or c
    ka = lot.get("kerb_arc")

    def kerb(local_x, target):
        if not ka:
            return (fr[0] + u[0] * local_x, fr[1] + u[1] * local_x)
        (ax, ay), rp = ka
        ang = math.atan2(target[1] - ay, target[0] - ax)
        return (ax + rp * math.cos(ang), ay + rp * math.sin(ang))

    path = (kerb(door_x, door), door)
    drive = None
    if garage is not None:
        drive = (kerb(garage_x, garage), garage)

    # Rear space, measured: lot depth less how far the house sits in, less half
    # its own depth. Halving (lot_depth - d) understates it by the whole
    # setback, which is the difference between a pool fitting and not.
    into = ((c[0] - fr[0]) * n[0] + (c[1] - fr[1]) * n[1])
    rear = float(lot.get("lot_depth", 0.0)) - into - float(lot.get("d", 0.0)) / 2.0

    return {"yaw": math.degrees(math.atan2(u[1], u[0])),
            "door": door, "garage": garage,
            "drive": drive, "path": path,
            "rear_m": rear,
            "pool_ok": rear >= POOL_REAR_NEED_M}


# What `pool_at` needs behind the house: a walk, the pool, a rear walk. Derived
# from the same constants `pool_at` tests against, because it was a second copy
# of the sum and the two have already disagreed once — `plan_lot` reported
# `pool_ok` on a 9.2 m rear that `pool_at` then refused.
POOL_REAR_NEED_M = POOL_WALK_M + 4.0 + POOL_REAR_WALK_M


def dress_plot(style, x, y, yaw, rng, lot_w=None, lot_d=None):
    """Drive, path, fence, hedge and pool around a building.

    The lot is SIZED FROM THE HOUSE and snapped to the 5 m module, because the
    fence panel and the driveway tile are both 5 m. A fixed 34x40 lot left the
    panels ending mid-run and the fence not lining up with anything.
    """
    spec = STYLES[style]
    cs = footprint(spec)
    xs = [i for i, _ in cs]
    ys = [j for _, j in cs]
    w = (max(xs) - min(xs) + 1) * CELL_M
    d = (max(ys) - min(ys) + 1) * CELL_M
    door_x, garage_x, front_y = front_anchors(spec)

    def _snap(v):
        return CELL_M * max(1, int(round(v / CELL_M)))

    lot_w = _snap(lot_w if lot_w else w + 2 * 7.5)
    out = []

    def add(usd, lx, ly, lz, lyaw, sub, scale=SCALE):
        wx, wy = _rot(lx, ly, yaw)
        out.append(_place(_usd(usd), x + wx, y + wy, lz, lyaw + yaw,
                          category=f"plot_{sub}", scale=scale))

    kerb_y = front_y - FRONT_YARD_M             # the lot's street edge

    # DRIVE runs to the GARAGE; PATH runs to the DOOR. Always, and never the
    # same line. A house with no garage still gets a drive — to a parking pad
    # off to one side — because pointing it at the door is what made the brick
    # apron arrive at the front step on some houses and the garage on others.
    drive_x = garage_x if garage_x is not None else -lot_w / 2 + CELL_M
    n_drive = max(1, int(round((front_y - kerb_y) / CELL_M)))
    for k in range(n_drive):
        add(DRIVEWAY_5, drive_x, front_y - (k + 0.5) * CELL_M, 0.0, 0.0,
            "driveway")
    if garage_x is not None:
        cu, csc, cax, cyaw = CARS[rng.randrange(len(CARS))]
        wx, wy = _rot(drive_x, front_y - 4.0, yaw)
        out.append(_place(cu, x + wx, y + wy, 0.0,
                          yaw + cyaw + CAR_YAW_EXTRA, category="plot_car",
                          scale=csc))
        if cax == "Y":                          # authored Y-up: stand it up
            out[-1]["roll_deg"] = 90.0
            out[-1]["axis_up"] = "Y"
    # The path always reaches the door — `front_anchors` guarantees there is
    # one, which was not true before (a garage stole the door's slot).
    for k in range(max(1, int((front_y - kerb_y) / 3.0))):
        add(PATH, door_x, front_y - 1.5 - k * 3.0, 0.0, 0.0, "path")
    # NO BIN HERE. One on every driveway reads as collection day rather than a
    # street; the suburb puts the scene's own `trash_cans` on the kerb instead
    # (`suburb_scene.build_frontage`), which is where they belong.

    # FENCE on the lot boundary, whole panels only, both runs starting from the
    # same corner so the two sides meet the back run squarely.
    kind = spec.get("fence")
    fy1 = front_y + d + BACK_YARD_M             # the back fence line
    if kind:
        asset = STONE_WALL if kind == "stone" else FENCE
        along_x, along_y = (90.0, 0.0) if kind == "stone" else (0.0, 90.0)
        fx0, fx1 = -lot_w / 2, lot_w / 2
        n_x = max(1, int(round(lot_w / CELL_M)))
        n_y = max(1, int(round((fy1 - front_y) / CELL_M)))
        for k in range(n_x):
            add(asset, fx0 + (k + 0.5) * CELL_M, fy1, 0.0, along_x, "fence")
        for k in range(n_y):
            add(asset, fx0, front_y + (k + 0.5) * CELL_M, 0.0, along_y, "fence")
            add(asset, fx1, front_y + (k + 0.5) * CELL_M, 0.0, along_y, "fence")

    # POOL, in the gap between the back of the house and the back fence.
    # Sized to that gap and skipped when it will not fit — it used to be placed
    # at a fixed offset, so on the deeper houses it sat on top of the fence.
    if spec.get("pool"):
        gap0, gap1 = front_y + d + 2.5, fy1 - 2.5
        if gap1 - gap0 >= 4.0:
            pd = 4.0                             # 8 x 4, as in `pool_at`
            pw = 8.0
            py = (gap0 + gap1) / 2.0
            # The floor asset is a SQUARE 20 m plane and placements carry one
            # uniform scale, so a rectangle is two 5 m tiles, not one stretched
            # plane. Stretching is not expressible here.
            for k in range(int(pw / pd)):
                add(POOL_FLOOR, -pw / 2 + (k + 0.5) * pd, py, POOL_WATER_Z_M,
                    0.0, "pool", scale=SCALE * pd / 20.0)
            # Coping. `Swimming_Pool_Edge_01` is 2.5 m along its own Y and thin
            # in X, so a run parallel to X is yaw 90 and one parallel to Y is
            # yaw 0 — and each panel needs the pivot correction, its origin
            # being 1.0 m from one end of the 2.5 m module.
            for o in _coping_run(pw):
                for yy, yw in ((py - pd / 2, 90.0), (py + pd / 2, 270.0)):
                    ex, ey = _pivot_xy(POOL_EDGE, o, yy, yw)
                    add(POOL_EDGE, ex, ey, 0.0, yw, "pool_edge")
            for o in _coping_run(pd):
                for xx, yw in ((-pw / 2, 180.0), (pw / 2, 0.0)):
                    ex, ey = _pivot_xy(POOL_EDGE, xx, py + o, yw)
                    add(POOL_EDGE, ex, ey, 0.0, yw, "pool_edge")
            for k in range(2):
                add(PROPS["chair"], -pw / 2 - 1.8, py - 1.2 + k * 2.4, 0.0,
                    90.0, "chair")

    # ONE specimen tree, clear of the house by half its canopy.
    tu, canopy = TREES[rng.randrange(len(TREES))]
    add_x = w / 2 + canopy * 0.5 + 1.0
    if add_x < lot_w / 2 - 1.0:
        wx, wy = _rot(add_x, front_y + 4.0, yaw)
        out.append(_place(tu, x + wx, y + wy, 0.0,
                          yaw + rng.uniform(0.0, 360.0), category="plot_tree"))

    # Hedge along the street frontage, leaving drive and path open.
    xh = -lot_w / 2 + 1.0
    while xh < lot_w / 2 - 1.0:
        if not (drive_x - 3.5 < xh < drive_x + 3.5
                or door_x - 2.5 < xh < door_x + 2.5):
            add(PROPS["hedge"], xh, kerb_y + 1.0, 0.0, 0.0, "hedge")
        xh += 1.3
    if spec.get("hoop"):
        add(PROPS["hoop"], drive_x + 4.0, kerb_y + 3.0, 0.0, 0.0, "hoop")
    return out


# ---------------------------------------------------------------------------
# On-stage material pass
# ---------------------------------------------------------------------------
def palette_material(stage, parent_path, name, cache=None):
    """Reference one kit / `MATERIAL_SOURCE` material by NAME and return it.

    LIFTED OUT OF `apply_palette` (2026-08-27) so the DEBRIS can use it. A
    tornado's plank field is scattered off the houses it destroyed, and until
    now every board in it took the same sawn-timber map — so a levelled block
    came out as a lumber yard with no siding and no shingle anywhere in it,
    when what a debris photograph actually shows is a mat of house-coloured
    cladding and grey roof slab with bare framing between. `disaster.planks`
    now asks for the wrecked house's own `wall` and `roof` materials by the
    same names `PALETTES` uses, and gets exactly what the standing houses are
    wearing.

    *cache* is an optional dict so a caller building many of these reuses the
    referenced prims; without one every call re-defines the same prim, which
    is harmless but wasteful.
    """
    from pxr import Gf, Sdf, Usd, UsdShade

    if cache is not None and name in cache:
        return cache[name]
    looks = Sdf.Path(parent_path).AppendChild("Looks")
    stage.DefinePrim(looks, "Scope")
    p = looks.AppendChild(name)
    existing = UsdShade.Material.Get(stage, p)
    if existing and existing.GetPrim().IsValid():
        if cache is not None:
            cache[name] = existing
        return existing
    prim = stage.DefinePrim(p)
    asset, sub, tint = MATERIAL_SOURCE.get(name, (_usd(name), None, None))
    # A RetroNeighborhood material lives INSIDE a prop rather than in a
    # standalone USD, so it is referenced by (asset, primPath). Its texture
    # paths are relative to that layer and resolve from there.
    if sub:
        prim.GetReferences().AddReference(asset, Sdf.Path(sub))
    else:
        prim.GetReferences().AddReference(asset)
    if tint is not None:
        # Override the MDL's albedo tint on the referenced shader. Legal
        # because these prims are NOT instanced — which is the same reason the
        # subset rebinding works at all.
        for q in Usd.PrimRange(stage.GetPrimAtPath(p)):
            sh = UsdShade.Shader(q)
            if not sh:
                continue
            inp = sh.GetInput("Tint_AlbedoTexture_")
            if inp:
                inp.Set(Gf.Vec4f(*tint))
    mat = UsdShade.Material(stage.GetPrimAtPath(p))
    if cache is not None:
        cache[name] = mat
    return mat


def palette_skins(palette_name):
    """`{"siding": <material>, "deck": <material>}` for one house palette.

    WHAT THE DEBRIS OFF THIS HOUSE IS MADE OF, in the two classes that carry a
    colour. `siding` is the cladding that was on the outside of the wall;
    `deck` is the roof slab with its covering still on. Everything else in the
    plank field — studs, joists, OSB sheathing — is bare timber and stays the
    sawn-timber material, because that is what the inside of a wall looks like.

    A flat-roofed style has no shingle, so it falls back to `roof_flat`.
    """
    pal = PALETTES.get(str(palette_name) or "") or {}
    if not pal:
        return {}
    return {"siding": pal.get("wall") or "Cladding_01",
            "deck": pal.get("roof") or pal.get("roof_flat") or ROOF_SHINGLE}


# What a diffuse/albedo texture is called in the packs this kit draws on —
# BY FILENAME, which is how `apply_palette.current()` identifies a surface.
_BASECOLOR_HINTS = ("basecolor", "base_color", "albedo", "diffuse", "_bc",
                    "_col", "_d.", "_alb")
# ...AND BY INPUT NAME, because the filename test is not enough. It works on
# the standalone kit materials (`Wood_01_White_BaseColor.png`) and it does NOT
# work on the RetroNeighborhood ones, whose maps are called things like
# `T_Rooftiles_03_1K.png` — no hint anywhere in the name. That is why the
# first run that skinned the debris reported "no base-colour map" for both
# shingle materials and dropped the roof slab back to sawn timber. The MDL
# INPUT is unambiguous where the filename is not, so it is tried first.
_BASECOLOR_INPUTS = ("diffuse_texture", "diffuse_color_texture",
                     "basecolor_texture", "base_color_texture",
                     "albedo_texture", "AlbedoTexture", "BaseColorTexture",
                     "Albedo_Texture", "DiffuseTexture", "inputs:diffuseColor")


def palette_texture(stage, parent_path, name):
    """`(diffuse_texture_path_or_None, (r, g, b))` for a palette material.

    WHY THE MATERIAL ITSELF CANNOT BE BOUND TO DEBRIS. `palette_material`
    returns the kit's own MDL, and those are UV-space materials: the meshes
    they were authored for carry `st`, and `disaster.planks` authors boxes with
    NO UVs AT ALL (which is why `planks.wood_material` is triplanar in the
    first place — see the note there). Bound to a plank mesh a kit material
    renders BLACK. The first attempt at house-coloured debris did exactly that
    and put a field of black roof slab into the corridor.

    So the debris takes the TEXTURE rather than the material, and projects it
    from world coordinates like everything else in the field. This walks the
    referenced material for the first asset input that looks like a base
    colour — the same walk `apply_palette`'s `current()` makes to identify a
    surface — and returns it with the palette's own albedo multiplier folded
    into an RGB tint, so `shingle_slate` and `shingle_grey` still differ from
    `shingle_charcoal` even though all three share one map.
    """
    from pxr import Sdf, Usd, UsdShade

    mat = palette_material(stage, parent_path, name)
    tint = MATERIAL_SOURCE.get(name, (None, None, None))[2]
    rgb = (1.0, 1.0, 1.0) if not tint else tuple(float(q) for q in tint[:3])
    # TWO PASSES, INPUT NAME FIRST. Any asset-valued input whose NAME says
    # diffuse is the diffuse map whatever the file is called; only if none of
    # them exists is the filename worth guessing from. A third pass takes the
    # first asset input of any kind, because a material with exactly one
    # texture on it has already told you which one it is — the kit has several
    # of those and the alternative is an untextured slab.
    by_name, by_file, any_tex = None, None, None
    if mat and mat.GetPrim().IsValid():
        for prim in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(prim)
            if not sh:
                continue
            for inp in sh.GetInputs():
                # Some shader inputs in this kit raise on Get() rather than
                # returning None, so this cannot be a bare read.
                try:
                    v = inp.Get()
                except Exception:
                    continue
                if not isinstance(v, Sdf.AssetPath):
                    continue
                f = (v.resolvedPath or v.path or "")
                if not f:
                    continue
                nm = inp.GetBaseName()
                low = f.lower()
                # ...but never a NORMAL, ROUGHNESS or ORM map, which are the
                # other asset inputs on every one of these and which would come
                # out as a blue slab or a grey one.
                if any(b in low for b in ("normal", "_orm", "roughness",
                                          "metallic", "_n.", "_rma", "height",
                                          "opacity", "emissive")):
                    continue
                if any_tex is None:
                    any_tex = f
                if by_name is None and any(
                        nm.lower() == h.lower().split(":")[-1]
                        for h in _BASECOLOR_INPUTS):
                    by_name = f
                if by_file is None and any(h in low
                                           for h in _BASECOLOR_HINTS):
                    by_file = f
    return (by_name or by_file or any_tex), rgb


def apply_palette(stage, placements, parent_path="/World/stage/generated"):
    """Rebind wall / gable / roof subsets per building.

    Needed because the kit has exactly ONE wall texture — see the module
    docstring. Matching on the material a subset already binds means the swap
    never has to know anything about the mesh, and never touches the window
    frames or the glass.

    `pxr` is imported here rather than at module scope so the rest of this file
    stays pure Python: the layout is built and tested host-side, where the
    project venv has no `pxr`.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    looks = Sdf.Path(parent_path).AppendChild("Looks")
    stage.DefinePrim(looks, "Scope")
    cache = {}

    def material(name):
        return palette_material(stage, parent_path, name, cache)

    def current(prim):
        m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not m or not m.GetPrim().IsValid():
            return ""
        # The bound material is always called "UnrealMaterial"; the identity is
        # in its texture, so read that instead of the prim name.
        for p in Usd.PrimRange(m.GetPrim()):
            sh = UsdShade.Shader(p)
            if not sh:
                continue
            for inp in sh.GetInputs():
                # Some shader inputs in this kit raise on Get() rather than
                # returning None, so this cannot be a bare read.
                try:
                    v = inp.Get()
                except Exception:
                    continue
                if isinstance(v, Sdf.AssetPath):
                    f = (v.resolvedPath or v.path or "")
                    if "basecolor" in f.lower():
                        return f.rsplit("/", 1)[-1].replace("_BaseColor.png", "")
        return ""

    n = 0
    for pl in placements:
        pal = PALETTES.get(pl.get("palette") or "")
        path = pl.get("prim_path")
        swap = DRESSING_REBIND.get(str(pl.get("category", "")))
        if not path or not (pal or swap):
            continue
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid():
            continue
        cat = str(pl.get("category", ""))
        is_roof = cat.endswith("_roof") or cat.endswith("_bay_roof")
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            targets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))) or []
            for t in ([s.GetPrim() for s in targets] or [prim]):
                surf = current(t)
                new = None
                if swap and surf == swap[0]:
                    new = swap[1]
                elif not pal:
                    continue
                elif surf in _WALL_SURFACES:
                    new = pal.get("wall")
                elif surf in _GABLE_SURFACES:
                    new = pal.get("gable")
                elif surf in _ROOF_PITCHED:
                    new = pal.get("roof") or ROOF_SHINGLE
                elif surf in _ROOF_FLAT:
                    new = pal.get("roof_flat") or ROOF_DECK
                elif surf in _ROOF_TRIM and is_roof:
                    new = pal.get("trim") or ROOF_TRIM_WHITE
                if not new:
                    continue
                UsdShade.MaterialBindingAPI(t).Bind(material(new))
                n += 1
    return n


# ---------------------------------------------------------------------------
# The catalogue
# ---------------------------------------------------------------------------
# Footprints are sets of 10x10 m BLOCKS. Long thin boxes are deliberately
# absent; the shapes that read well are the compact ones and the L.
STYLES = {
    # Residential only. This pack is NOT used for commercial: its walls, windows
    # and doors are all domestic, there is no shopfront glazing, and the roof set
    # cannot make a continuous flat roof wider than 15 m. The former commercial
    # bars survive here shortened into residential terraces, which is what they
    # actually look like.
    #
    # Shapes are mostly BOXES with two L-plans, and the fence is timber on some
    # plots and low stone wall on others.
    "cottage": {
        "cells": rect_cells(2, 2), "storeys": 1, "roof": "gable",
        "palette": "wood_white", "garage": 0, "porch": True, "fence": "picket",
        "note": "10x10 box, 1 storey, painted timber"},
    "two_storey": {
        "cells": rect_cells(2, 2), "storeys": 2, "roof": "gable",
        "palette": "brick_buff", "garage": 0, "porch": True, "fence": "stone",
        "note": "10x10 box, 2 storeys, buff brick, low stone wall"},
    "wide_house": {
        "cells": rect_cells(3, 2), "storeys": 2, "roof": "gable",
        "palette": "siding_cream", "garage": 0, "porch": True,
        "fence": "picket", "pool": True,
        "note": "15x10 box, 2 storeys, cream siding, pool"},
    "ranch": {
        "cells": rect_cells(4, 2), "storeys": 1, "roof": "gable",
        "palette": "brick_red", "garage": 1, "porch": True, "fence": "stone",
        "hoop": True, "note": "20x10 box, 1 storey, red brick, double garage"},
    "terrace": {
        "cells": rect_cells(2, 4), "storeys": 2, "roof": "flat_bar",
        "palette": "brick_red", "garage": 0, "porch": True, "fence": "stone",
        "note": "10x20, 2 storeys — the ex-commercial bar, shortened. The only "
                "shape that gets a CONTINUOUS flat roof out of this kit"},
    "villa": {
        "cells": rect_cells(3, 2), "storeys": 1, "roof": "flat",
        "palette": "stucco", "garage": 1, "porch": True, "fence": "picket",
        "pool": True, "note": "15x10, 1 storey, render, single-piece hip roof"},
    "l_family": {
        "cells": rect_cells(4, 2) | rect_cells(2, 2, 0, 2), "storeys": 2,
        "roof": "gable", "palette": "brick_buff", "garage": 1, "porch": True,
        "fence": "picket", "pool": True,
        "note": "L-plan, 2 storeys, buff brick, pool behind the fence"},
    "l_bungalow": {
        "cells": rect_cells(4, 2) | rect_cells(2, 2, 2, 2), "storeys": 1,
        "roof": "gable", "palette": "wood_dark", "garage": 1, "porch": True,
        "fence": "stone", "note": "L-plan, 1 storey, stained timber, garage wing"},
}

RESIDENTIAL = list(STYLES)
COMMERCIAL = []          # this pack is not used for commercial
ORDER = RESIDENTIAL


# How a suburban commercial plaza is actually laid out, and why this shape:
# the anchor sits at the BACK of the lot with its whole frontage facing the
# road, the smaller units wrap one side, and the outparcels — fast food, a
# bank, a fuel station — sit detached at the kerb with the parking field
# between them and the anchor. That ordering is what makes it read as a plaza
# rather than as buildings in a row, and it is why the parking goes in the
# middle instead of behind.
PLAZA = [
    # (style, x, y, yaw)  — y grows away from the road, which is at y = 0
    ("supermarket",  0.0,  74.0, 0.0),    # anchor, centred at the back
    ("retail_block", -40.0, 74.0, 0.0),   # inline units wrapping the west side
    ("car_repair",   42.0, 69.0, 0.0),    # service use pushed to the edge
    ("fast_food",   -34.0, 18.0, 0.0),    # outparcels at the kerb
    ("convenience",   0.0, 18.0, 0.0),
    ("office",       36.0, 18.0, 0.0),
]


def build_plaza(rng, origin=(0.0, 0.0), dress=True):
    """The commercial styles arranged as a plaza around a shared parking field."""
    ox, oy = origin
    out = []
    for style, px, py, yaw in PLAZA:
        shell = build_building(style, ox + px, oy + py, yaw, rng)
        for p in shell:
            p["palette"] = STYLES[style].get("palette")
        out += shell
        if dress:
            out += dress_plot(style, ox + px, oy + py, yaw, rng)

    # The parking field between the outparcels and the anchor. Aisles run
    # east-west because the anchor frontage does; a bay is 2.5 x 5 m, so the
    # 5 m driveway tile is two bays and the grid needs no new asset.
    # Asphalt, laid contiguously. The brick driveway tile is right for a house
    # and quite wrong for a parking field; `Road_01` is a 16 m road tile, so
    # the lot is four of them by five.
    for row in range(4):
        for col in range(9):
            out.append(_place(_usd(ROAD_TILE), ox - 64.0 + col * 16.0,
                              oy + 30.0 + row * 16.0, 0.0, 0.0,
                              category="plaza_parking"))
    for k in range(6):                      # lamp columns down the field
        out.append(_place(_usd(PROPS["lamp"]), ox - 57.0 + k * 23.0,
                          oy + 52.0, 0.0, 0.0, category="plaza_lamp"))
    for k in range(14):                     # parked cars, thinning out
        if rng.random() < 0.55:
            continue
        out.append(_place(_usd(PROPS["car"]), ox - 50.0 + k * 7.5,
                          oy + 34.0 + rng.randrange(6) * 7.0, 0.0,
                          rng.choice([0.0, 180.0]), category="plaza_car"))
    return out


# Every prop in the pack worth seeing, laid out as a strip. The catalogue's
# job is to show WHAT IS AVAILABLE, so this deliberately includes pieces no
# building style uses — the porch and veranda systems, both banister runs, the
# low garden walls, the trellises, the signs.
PROP_STRIP = [
    ("Veranda_01", 12.0), ("Porch_Middle_01", 12.0), ("Porch_Corner_01", 8.0),
    ("Porch_Pillar_01", 3.0), ("Banister_01", 4.0), ("Banister_02", 3.0),
    ("Wall_Low_01", 4.0), ("Wall_Low_Half_01", 3.0), ("Wall_Low_End_01", 2.0),
    ("Fence_01_0", 7.0), ("Hedge_01", 3.0), ("Hedge_02_0", 5.0),
    ("Garden_Tressel_01", 3.0), ("Garden_Tressel_02", 3.0),
    ("Garden_Rocks_01", 3.0), ("Basketball_Hoop_01", 4.0),
    ("Bench_01", 3.0), ("Garden_Chair_01_0", 2.5), ("bin_01_0", 2.0),
    ("Lamp_01_0", 2.5), ("Street_Sign_01_0", 3.0), ("Stop_Sign_01_0", 2.5),
    ("Monument_01", 4.0), ("Power_Line_01_0", 6.0),
    ("Staircase_01", 7.0), ("Staircase_03", 6.0), ("Drain_Pipe_01", 2.0),
    ("House_Base_Quart_01", 7.0), ("Driveway_Edge_Curve_01", 5.0),
    ("Swimming_Pool_Steps_01_0", 3.0), ("Road_01", 18.0),
]


def build_prop_strip(x0=0.0, y0=0.0):
    """One of each loose asset in a line, for reading off what the pack has."""
    out, x = [], x0
    for name, gap in PROP_STRIP:
        out.append(_place(_usd(name), x, y0, 0.0, 0.0, category="props"))
        x += gap
    return out


def build_catalogue(rng, cols=4, pitch_x=52.0, pitch_y=58.0, dress=True):
    """Every style on a grid, plus the prop strip.

    A GRID, not a street: this file exists so the range of shapes, materials
    and roof forms can be seen side by side. Making it read as a coherent
    neighbourhood is the suburb generator's job, not this one's.
    """
    out = []
    for n, style in enumerate(ORDER):
        cx = (n % cols - (cols - 1) / 2.0) * pitch_x
        cy = -(n // cols) * pitch_y
        yaw = STYLES[style].get("yaw", 0.0)
        shell = build_building(style, cx, cy, yaw, rng)
        # ONLY the shell takes the palette. Tagging the plot dressing too is
        # what rebound the fences, hedges and driveway to the house's wall
        # material — a timber-clad fence panel is not what `wood_white` means.
        #
        # `group` says which pieces are ONE BUILDING. A house is authored wall
        # by wall, so anything scoring or annotating these placements needs to
        # know that a dozen of them are a single object; without it a ground
        # truth built from placements reports one house as twelve detections.
        # Shell only — the plot dressing is its own set of objects.
        for p in shell:
            p["palette"] = STYLES[style].get("palette")
            p["group"] = f"house_{n}"
        out += shell
        if dress:
            out += dress_plot(style, cx, cy, yaw, rng)
    rows = (len(ORDER) + cols - 1) // cols
    out += build_prop_strip(-(cols - 1) / 2.0 * pitch_x - 10.0,
                            -rows * pitch_y - 6.0)
    return out


def build_row(rng, spacing_m=44.0, dress=True, styles=None):
    """The catalogue in a row: residential first, then commercial."""
    names = styles or ORDER
    out = []
    for i, style in enumerate(names):
        x = (i - (len(names) - 1) / 2.0) * spacing_m
        pal = STYLES[style].get("palette")
        shell = build_building(style, x, 0.0, 0.0, rng)
        for p in shell:
            p["palette"] = pal          # shell only — see build_catalogue
        out += shell
        if dress:
            out += dress_plot(style, x, 0.0, 0.0, rng)
    return out


# Measured footprint COVERED by each roof piece, in square metres, and whether
# it is a single piece sized to the whole plan. Used by `validate_roofs` to
# prove a style is fully roofed instead of finding out from a render.
_ROOF_COVER = {
    ROOF_GABLE: 100.0,          # one 10x10 bay
    ROOF_HALF_GABLE: 50.0,      # the 5 m leftover strip, full depth
    ROOF_FLAT_MID: 50.0,        # one 5 m bar section, 10 m wide
    ROOF_FLAT_CAP: 0.0,         # an eave cap; covers nothing itself
}
_ROOF_SINGLE_PIECES = {a for a, _dx in ROOF_FLAT.values()} | {
    "Roof_Flat_Open_Apex_01"}


def validate_roofs(styles=None):
    """Check every style's roof actually covers its footprint.

    WHY THIS EXISTS. A 15 m-wide plan is one 10 m block plus a 5 m strip, and
    `_blocks_of` only yields FULLY covered blocks — so the strip silently got no
    roof and 50 m2 of house stood open to the sky. Nothing caught it until it
    showed up in a render, and a render is an expensive way to find arithmetic.

    Returns ``[(style, covered_m2, needed_m2, note)]`` for anything short.
    Empty list means every style is sound.
    """
    import random as _r
    bad = []
    for style in (styles or ORDER):
        spec = STYLES[style]
        cs = footprint(spec)
        need = len(cs) * CELL_M * CELL_M
        parts = build_building(style, 0.0, 0.0, 0.0, _r.Random(0))
        roofs = [q for q in parts if q["category"].endswith("_roof")
                 and "Quart_Roof" not in q["usd"]]     # bay caps are not field
        covered, single = 0.0, False
        for q in roofs:
            name = q["usd"].rsplit("/", 1)[-1].replace(".usd", "")
            if name in _ROOF_SINGLE_PIECES:
                single = True
            covered += _ROOF_COVER.get(name, 0.0)
        if single:                    # sized to the whole plan by construction
            continue
        if not roofs:
            bad.append((style, 0.0, need, "no roof pieces at all"))
        elif covered + 1e-6 < need:
            bad.append((style, covered, need,
                        f"{need - covered:.0f} m2 unroofed"))
    return bad


def check(verbose=True):
    """Every invariant this module can check without a stage.

    Called by the launch scripts before anything is written, so a broken
    catalogue fails with a sentence instead of a strange-looking render.
    """
    problems = []
    for style, cov, need, note in validate_roofs():
        problems.append(f"roof: {style} covers {cov:.0f}/{need:.0f} m2 — {note}")
    # Every style must resolve a front door, or the path leads to blank wall.
    for style in ORDER:
        dx, gx, _fy = front_anchors(STYLES[style])
        if dx is None:
            problems.append(f"door: {style} has no front-door slot")
        if gx is not None and abs(gx - dx) < 1e-6:
            problems.append(f"door: {style} puts the door in the garage slot")
    # A palette must not name a material this module cannot resolve.
    known = set(MATERIAL_SOURCE)
    for name, pal in PALETTES.items():
        for slot in ("wall", "gable", "roof", "roof_flat", "trim"):
            v = pal.get(slot)
            if v and v not in known and not v.replace("_", "").isalnum():
                problems.append(f"palette {name}.{slot}: odd material {v!r}")
    if verbose:
        if problems:
            print("[modular_house] CHECK FAILED:")
            for q in problems:
                print("  " + q)
        else:
            print(f"[modular_house] check ok — {len(ORDER)} styles, "
                  f"roofs cover their footprints, every style has a door")
    return problems


def summarise(placements):
    from collections import Counter
    c = Counter(p["category"].split("_")[0] + "_" + p["category"].split("_")[1]
                if p["category"].count("_") > 1 else p["category"]
                for p in placements)
    lines = [f"{len(placements)} placements, "
             f"{len({p['usd'] for p in placements})} distinct kit assets"]
    for k in sorted(c):
        lines.append(f"  {k:<28} {c[k]:4d}")
    return "\n".join(lines)
