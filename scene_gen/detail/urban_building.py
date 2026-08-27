"""
urban_building.py — urban buildings assembled from modular FAÇADE kits.

THE GRAMMAR: BANDS x SIDES x CORNERS, THEN A ROOF, THEN WINGS
--------------------------------------------------------------
A suburban house (`modular_house.py`) is a footprint of cells with walls
generated from the boundary. An urban block is simpler and taller: a rectangle
whose four sides are each a RUN of façade modules between two CORNER pieces,
stacked in vertical BANDS (a tall ground floor, N identical storeys, a parapet
or top-floor band), then a flat roof — and, for the block-scale buildings,
WINGS: further rectangles standing on that roof (a podium carrying several
slabs of different heights, which is how the pack's own 91 x 96 x 69 m merged
block is massed).

FAÇADES ARE PLANNED ONCE PER BAND, NOT ROLLED PER PIECE
-------------------------------------------------------
Picking a random piece for every slot produced façades with no rhythm — a
balcony here, a blank there — which is not how any building is built. A
band's wall plan is therefore drawn ONCE (`_plan_band`): a short MOTIF of
pieces (uniform / alternating / paired), mirrored about the centre, with the
entrance in the centre slot of the street side and balconies in a fixed set
of columns. Every storey of the band then reuses that plan, so windows stack
in columns and balconies run as vertical stripes, storey over storey.

THE KITS
--------
`/Projects/SEI-COA/ProceduralBuildingGenerator/` — the one asked for — is an
Unreal 4.22 pack (772 `.uasset`, zero USDs) and cannot be read outside
Unreal. The kits in USD, each measured with `tools/modular_kit_report.py`:

  ModernCityEnvironment01/Meshes   5 families, 3 m storey, 5 m / 4 m module,
      pivot at the LEFT end on the wall line, outward -Y. Corner pieces are
      Ls with legs L on the wall line, authored "SE" (01/03/04: 1 m legs,
      02 ground: 8 m) or "SW" (05: whole 5 m bays). Roof tiles 5x5 / 4.8x4.8
      corner-pivoted, 8x8 / 4x4 / 23x23 centred.
  Downtown_West/Assets/building_b  Dmytro's storefront kit: pieces run along
      +Y, CENTRED, outward +X, 3 m / 5 m wide — frame "dw" below turns them
      into the canonical frame. Storefront 4.3 m, trim 1.2 m, window bands
      6 m (two storeys per piece), top cornice 1.6 m: the z-levels come from
      the artist's own blocks in `Library/Stages/Dmytro/downtown_edited_v2.usd`.
  CivilianArea/Assets              a 2.5 m-module generic kit (PlainWall /
      SingleWindow / DoubleWindow / Door, 2.5 m storeys, outward -Y) plus a
      government portico set (3/6/9 m columns with base + capital, 6 m and
      12 m pediments, trims with "SE" corner Ls of 1.5/2/3 m legs) and a
      church set (2.75 m module, 5.5 m windows, tower top, cross).

HARVESTED BLOCKS
----------------
`config/harvested/downtown_west_blocks.json` holds the five city blocks the
artist assembled in `downtown_edited_v2.usd` (380-640 pieces each, ~90 x 40 m),
as piece -> 4x4 matrix relative to the block centre. `build_harvested` places
one verbatim; `apply_placements` honours the `matrix` key.

Nothing here imports `pxr` except the on-stage pass at the bottom
(`apply_glass_tint`): the layout is plain Python and `check()` proves the
tables host-side.
"""

import json
import math
import os

SEI = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
KIT = SEI + "ModernCityEnvironment01/Meshes/"
DW_B = SEI + "Downtown_West/Assets/building_b/"
CIV = SEI + "CivilianArea/Assets/"
SCALE = 1.0
STOREY_M = 3.0
HARVESTED_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                              "config", "harvested", "downtown_west_blocks.json")

# Measured extents of every piece used below: (sx, sy, sz, xmin, ymin, zmin).
# `check()` refuses a band whose pieces do not share the band's height or
# module, which is how a typo in a name fails as a sentence, not a render.
PIECES = {
    # family 01 — mid-rise, 5 m module, 1 m corners, 6 m ground floor
    "SM_MBuilding01_FirstFloor_A": (5.00, 1.14, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_FirstFloor_B": (5.00, 1.14, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_FirstFloor_C": (5.00, 2.12, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_FirstFloor_D": (5.00, 1.12, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_FirstFloor_E": (5.00, 1.14, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_Corner_A":     (1.55, 1.55, 6.00, 0.00, -0.55, 0.00),
    "SM_MBuilding01_Facade_A":     (5.07, 0.47, 3.00, -0.03, -0.22, 0.00),
    "SM_MBuilding01_Facade_B":     (5.07, 0.47, 3.00, -0.03, -0.22, 0.00),
    "SM_MBuilding01_Facade_C":     (5.00, 0.40, 3.00, 0.00, -0.15, 0.00),
    "SM_MBuilding01_Facade_D":     (5.00, 0.40, 3.00, 0.00, -0.15, 0.00),
    "SM_MBuilding01_Corner_B":     (1.25, 1.25, 3.00, -0.03, -0.22, 0.00),
    "SM_MBuilding01_Corner_C":     (1.15, 1.15, 3.00, 0.00, -0.15, 0.00),
    "SM_MBuilding01_Rooftop":      (5.12, 0.75, 2.17, -0.06, -0.75, -0.17),
    "SM_MBuilding01_Corner_D":     (1.81, 1.81, 2.17, -0.06, -0.75, -0.17),
    "SM_MBuilding01_Roof":         (5.00, 5.00, 0.00, 0.00, 0.00, 0.00),
    "SM_MBuilding01_Wall":         (5.00, 0.00, 6.00, 0.00, 0.00, 0.00),
    # family 02 — office, 4 m module (8 m ground), no thin corners
    "SM_MBuilding02_FirstFloor_A": (8.21, 8.21, 8.20, 0.00, -0.21, 0.00),
    "SM_MBuilding02_FirstFloor_B": (8.00, 0.51, 7.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_FirstFloor_C": (8.00, 0.50, 7.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_FirstFloor_E": (8.00, 0.60, 7.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_FirstFloor_G": (8.00, 0.10, 7.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_Facade_A":     (4.00, 0.40, 3.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_Facade_B":     (4.00, 0.40, 3.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_Facade_C":     (4.00, 0.27, 3.00, 0.00, -0.10, 0.00),
    "SM_MBuilding02_Balcony_A":    (4.00, 1.53, 3.00, 0.00, -0.30, 0.00),
    "SM_MBuilding02_Balcony_B":    (4.00, 1.13, 3.00, 0.00, -0.90, 0.00),
    "SM_MBuilding02_Balcony_C":    (4.00, 3.08, 3.00, 0.00, -2.85, 0.00),
    "SM_MBuilding02_RoofLedge_A":  (4.00, 0.51, 1.00, 0.00, -0.21, 0.00),
    "SM_MBuilding02_Roof_A":       (8.00, 8.00, 0.00, -4.00, -4.00, 0.00),
    # family 03 — brownstone, 4 m module, 1 m corners, 5 m ground floor
    "SM_MBuilding03_FirstFloor_A":    (4.97, 1.62, 5.00, -0.48, -1.50, 0.00),
    "SM_MBuilding03_FirstFloor_B":    (4.00, 0.34, 5.00, 0.00, -0.16, 0.00),
    "SM_MBuilding03_FirstFloor_Corner": (1.16, 1.16, 5.00, 0.00, -0.16, 0.00),
    "SM_MBuilding03_Facade_A":        (4.00, 0.68, 3.00, 0.00, -0.38, 0.00),
    "SM_MBuilding03_Facade_B_Bottom": (4.00, 0.30, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding03_Facade_B_Middle": (4.00, 0.30, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding03_Facade_B_Upper":  (4.00, 0.68, 3.00, 0.00, -0.38, 0.00),
    "SM_MBuilding03_Facade_C":        (4.00, 0.25, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding03_Facade_B_Corner": (1.00, 1.00, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding03_Facade_AB_Corner": (1.38, 1.38, 3.00, 0.00, -0.38, 0.00),
    "SM_MBuilding03_RoofLedge":       (4.00, 0.78, 2.05, 0.00, -0.78, 0.00),
    "SM_MBuilding03_RoofLedge_Corner": (1.78, 1.78, 2.43, 0.00, -0.78, 0.00),
    "SM_MBuilding03_Roof_A":          (4.00, 4.00, 0.00, -2.00, -2.00, 0.00),
    # family 04 — brick commercial, 4 m module, 1 m corners, 7 m ground floor
    "SM_MBuilding04_FirstFloor_A":     (4.00, 1.07, 7.00, 0.00, -0.37, 0.00),
    "SM_MBuilding04_FirstFloor_B":     (4.00, 1.07, 7.00, 0.00, -0.37, 0.00),
    "SM_MBuilding04_FirstFloor_C":     (4.00, 1.07, 7.00, 0.00, -0.37, 0.00),
    "SM_MBuilding04_FirstFloor_Corner": (1.37, 1.37, 7.00, 0.00, -0.37, 0.00),
    "SM_MBuilding04_Facade_A":         (4.00, 0.74, 3.00, 0.00, -0.04, 0.00),
    "SM_MBuilding04_Facade_B":         (4.00, 0.70, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding04_Facade_Corner":    (1.00, 1.00, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding04_TopFloor_A":       (4.00, 1.41, 4.00, 0.00, -0.71, 0.00),
    "SM_MBuilding04_TopFloor_B":       (4.00, 1.41, 4.00, 0.00, -0.71, 0.00),
    "SM_MBuilding04_TopFloor_Corner":  (1.71, 1.71, 4.00, 0.00, -0.71, 0.00),
    "SM_Roof":                         (23.08, 23.08, 0.00, -11.54, -11.54, 0.00),
    # family 05 — tower, 5 m module, 5 m corner bays, 7 m ground floor
    "SM_MBuilding05_FirstFloor_A":       (5.00, 5.00, 7.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_FirstFloor_B":       (5.00, 5.00, 7.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_FirstFloor_C":       (5.00, 5.00, 7.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_FirstFloor_D":       (5.00, 0.30, 7.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_FirstFloorCorner_B": (5.00, 5.00, 7.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_FirstFloorCorner_A": (5.00, 5.00, 7.00, -5.00, 0.00, 0.00),
    "SM_MBuilding05_Facade_A":           (5.00, 0.40, 3.00, 0.00, -0.30, 0.00),
    "SM_MBuilding05_Facade_B":           (5.00, 0.20, 3.00, 0.00, -0.20, 0.00),
    "SM_MBuilding05_FacadeCorner_A":     (5.30, 5.30, 3.00, -0.30, -0.30, 0.00),
    "SM_MBuilding05_RoofTop_A":          (5.00, 0.50, 4.41, 0.00, -0.30, 0.00),
    "SM_MBuilding05_RoofTop_B":          (5.00, 0.40, 4.00, 0.00, -0.20, 0.00),
    "SM_MBuilding05_RoofTopCorner_A":    (5.30, 5.30, 4.41, -0.30, -0.30, 0.00),
    "SM_MBuilding05_Roof":               (4.80, 4.80, 0.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperFacade_A": (5.00, 0.00, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperCorner_A": (5.00, 5.00, 3.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperFacade_B": (5.00, 1.00, 3.00, 0.00, -1.00, 0.00),
    "SM_MBuilding05_SkyscraperCorner_B": (6.00, 6.00, 3.00, -1.00, -1.00, 0.00),
    "SM_MBuilding05_SkyscraperRoof":     (5.00, 5.00, 0.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperRoofTopFacade": (5.00, 0.50, 1.00, 0.00, -0.50, 0.00),
    "SM_MBuilding05_SkyscraperRoofTopCorner": (5.50, 5.50, 1.00, -0.50, -0.50, 0.00),
    # Downtown_West building_b — along +Y, centred, outward +X (frame "dw")
    "SM_build_b_mod_lvl1_storefront_b_wall3m": (0.10, 3.01, 4.30, -0.10, -1.51, 0.00),
    "SM_build_b_mod_lvl1_storefront_b_wall5m": (0.28, 5.10, 4.31, -0.16, -2.58, -0.01),
    "SM_build_b_mod_lvl1_storefront_a_3_5m":   (0.24, 3.04, 4.31, -0.18, -1.50, -0.01),
    "SM_build_b_mod_lvl1_doubleblackdoor":     (0.36, 3.97, 3.98, -0.19, -1.98, 0.00),
    "SM_build_b_mod_lvl1_pillar_a":            (0.81, 0.81, 4.30, -0.41, -0.41, 0.00),
    "SM_build_b_mod_lvl1_trim_3m":             (0.98, 3.00, 1.28, -0.66, -1.50, 0.00),
    "SM_build_b_mod_lvl1_trim_5m":             (0.98, 5.00, 1.28, -0.66, -2.50, 0.00),
    "SM_build_b_mod_lvl2_singlewindow":        (0.48, 3.00, 6.00, -0.46, -1.50, 0.00),
    "SM_build_b_mod_lvl2_doublewindow":        (0.48, 5.00, 6.00, -0.46, -2.50, 0.00),
    "SM_build_b_mod_lvl2_widewindow":          (0.50, 5.00, 6.00, -0.42, -2.50, 0.00),
    "SM_build_b_mod_lvl3_singlewindow":        (0.55, 3.00, 6.00, -0.46, -1.50, 0.00),
    "SM_build_b_mod_lvl3_doublewindow":        (0.57, 5.00, 6.00, -0.46, -2.50, 0.00),
    "SM_build_b_mod_lvl2_trim_long":           (1.03, 5.00, 1.00, -0.29, -2.50, 0.00),
    "SM_build_b_mod_top_trim":                 (1.18, 5.00, 1.61, -0.17, -2.50, 0.00),
    # CivilianArea — generic 2.5 m kit, outward -Y, pivot left end
    "SM_PlainWall":        (2.50, 0.20, 2.50, 0.00, 0.00, 0.00),
    "SM_SingleWindow_01a": (2.50, 0.23, 2.50, 0.00, -0.03, 0.00),
    "SM_SingleWindow_01b": (2.50, 0.23, 2.50, 0.00, -0.03, 0.00),
    "SM_DoubleWindow_01a": (2.50, 0.25, 2.50, 0.00, -0.05, 0.00),
    "SM_DoubleWindow_01b": (2.50, 0.25, 2.50, 0.00, -0.05, 0.00),
    "SM_Door":             (2.50, 0.24, 2.50, 0.00, 0.00, 0.00),
    "SM_Door_Wall":        (2.50, 0.20, 2.50, 0.00, 0.00, 0.00),
    "SM_CornerPillar":     (0.20, 0.20, 2.50, 0.00, 0.00, 0.00),
    # CivilianArea — government set
    "SM_SetGoverment_Trim_01":              (3.00, 0.91, 1.20, 0.00, -0.91, 0.00),
    "SM_SetGoverment_Trim_Corner_01_Large": (3.91, 3.91, 1.20, 0.00, -0.91, 0.00),
    "SM_SetGoverment_Roof_01":              (6.05, 0.91, 3.16, -0.05, -0.91, 0.00),
    "SM_SetGoverment_Roof_02":              (12.00, 0.91, 3.10, 0.00, -0.91, 0.00),
    "SM_SetGoverment_Pillar_Base_02_Large": (1.40, 1.40, 0.50, -0.70, -0.70, 0.00),
    "SM_SetGoverment_Pillar_02_Large":      (1.60, 1.60, 9.00, -0.80, -0.80, 0.00),
    "SM_SetGoverment_Pillar_Top_02_Large":  (1.60, 1.60, 1.50, -0.80, -0.80, 0.00),
    "SM_SetGoverment_Pillar_Base_01_Medium": (1.00, 1.00, 0.50, -0.50, -0.50, 0.00),
    "SM_SetGoverment_Pillar_01_Medium":     (1.00, 1.00, 6.00, -0.50, -0.50, 0.00),
    "SM_SetGoverment_Pillar_Top_01_Medium": (0.85, 0.85, 0.60, -0.42, -0.42, 0.00),
    # CivilianArea — church set, 2.75 m module
    "SM_Church_Window_02": (2.75, 0.26, 5.50, 0.00, 0.00, 0.00),
    "SM_Church_Window_03": (2.75, 0.26, 2.75, 0.00, 0.00, 0.00),
    "SM_Church_Door_01":   (2.75, 0.26, 2.75, 0.00, 0.00, 0.00),
    "SM_Church_Beam_Corner": (0.15, 0.15, 2.75, -0.15, -0.03, 0.00),
    "SM_Church_Tower_Top": (2.90, 1.46, 3.72, -1.45, -1.46, 0.00),
    "SM_Church_Cross_02":  (1.30, 1.30, 4.06, -0.65, -0.65, 0.00),
}

# Which server folder a piece lives in, its authoring frame, and the scale
# that makes it metric (CivilianArea is authored in centimetres: metersPerUnit
# 0.01, so 0.01; the PIECES table above is already in metres).
#   None   canonical: along +X from the pivot, outward -Y
#   "dw"   along +Y, centred on the pivot, outward +X  (Downtown_West)
_KITS = [("SM_build_b_", DW_B, "dw", 1.0), ("SM_SetGoverment_", CIV, None, 0.01),
         ("SM_Church_", CIV, None, 0.01), ("SM_Roof", KIT, None, 1.0),
         ("SM_MBuilding", KIT, None, 1.0)]
_CIV_GENERIC = {"SM_PlainWall", "SM_SingleWindow_01a", "SM_SingleWindow_01b",
                "SM_DoubleWindow_01a", "SM_DoubleWindow_01b", "SM_Door",
                "SM_Door_Wall", "SM_CornerPillar"}


def _kit(name):
    if name in _CIV_GENERIC:
        return CIV, None, 0.01
    for prefix, url, frame, scale in _KITS:
        if name.startswith(prefix):
            return url, frame, scale
    return KIT, None, 1.0


def _usd(name):
    return _kit(name)[0] + name + ".usd"


def _scale(name):
    return _kit(name)[2]


def _rot(dx, dy, yaw_deg):
    a = math.radians(yaw_deg)
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def _place(usd, x, y, z=0.0, yaw=0.0, category="bld", scale=SCALE, stretch=None):
    p = {"usd": usd, "x_m": float(x), "y_m": float(y), "z_m": float(z),
         "yaw_deg": float(yaw) % 360.0, "roll_deg": 0.0, "pitch_deg": 0.0,
         "scale": scale, "category": category, "axis_up": "Z",
         # Positioned in the kit's own frame; the generator's bbox-centroid
         # correction would pull the pieces apart (see modular_house._place).
         "raw_pivot": True}
    if stretch is not None:
        p["stretch"] = (float(stretch[0]), float(stretch[1]))
    return p


# ---------------------------------------------------------------------------
# Corners: where the piece's pivot goes and how it turns, per building corner.
#   "SE": pivot = P - R(yaw)*(L, 0)   yaw SE 0, NE 90, NW 180, SW 270
#   "SW": pivot = P                    yaw SW 0, SE 90, NE 180, NW 270
#   "C" : a post centred on the corner point, leg 0, yaw 0
# ---------------------------------------------------------------------------
_CORNER_YAW = {
    "SE": {"SE": 0.0, "NE": 90.0, "NW": 180.0, "SW": 270.0},
    "SW": {"SW": 0.0, "SE": 90.0, "NE": 180.0, "NW": 270.0},
}


def _corner_pivot(name, authored, which, W, D, L):
    P = {"SW": (0.0, 0.0), "SE": (W, 0.0), "NE": (W, D), "NW": (0.0, D)}[which]
    if authored == "C":
        sx, sy, _sz, xmin, ymin, _zmin = PIECES[name]
        return P[0] - xmin - sx / 2.0, P[1] - ymin - sy / 2.0, 0.0
    yaw = _CORNER_YAW[authored][which]
    if authored == "SE":
        dx, dy = _rot(L, 0.0, yaw)
        return P[0] - dx, P[1] - dy, yaw
    return P[0], P[1], yaw


def _side_slot(side, s, m, W, D):
    """(pivot_x, pivot_y, yaw) for the run slot [s, s+m] on *side*; the pivot
    is the left end as seen from outside, which for N and W is the far end."""
    if side == "S":
        return s, 0.0, 0.0
    if side == "E":
        return W, s, 90.0
    if side == "N":
        return s + m, D, 180.0
    return 0.0, s + m, 270.0


def _piece_pose(name, px, py, yaw, span):
    """Map a slot pivot onto the piece's own authoring frame."""
    frame = _kit(name)[1]
    if frame == "dw":
        # along +Y centred, outward +X: a yaw of -90 turns local +Y onto the
        # canonical +X and local +X onto -Y; then shift half a span so the
        # centred piece starts at the slot.
        dx, dy = _rot(span / 2.0, 0.0, yaw)
        return px + dx, py + dy, yaw - 90.0
    xmin = PIECES[name][3]
    if xmin < -0.5:                          # right-end pivot (shop-unit set)
        dx, dy = _rot(-xmin, 0.0, yaw)
        return px + dx, py + dy, yaw
    return px, py, yaw


def _width(name):
    """Along-wall extent, whichever axis the kit authored it on."""
    sx, sy, *_ = PIECES[name]
    return sy if _kit(name)[1] == "dw" else sx


def _span(name, m):
    """Modules a piece consumes (03_FirstFloor_A is 4.97 on 4 m: 1; a 5 m
    storefront on a 1 m module: 5)."""
    return m * max(1, int(round(_width(name) / m)))


# ---------------------------------------------------------------------------
# Footprint and height
# ---------------------------------------------------------------------------
def footprint(spec):
    """(W, D) on the wall line: corner legs + bays x module of the FIRST band."""
    b0 = spec["bands"][0]
    L = b0["corner"][2] if b0.get("corner") else 0.0
    m = b0["module"]
    nx, ny = spec["bays"]
    return 2 * L + nx * m, 2 * L + ny * m


def height(spec):
    """Top of the tallest roof, metres above the building's base."""
    z = sum(b["h"] * b.get("repeat", 1) for b in spec["bands"]
            if not b.get("parapet"))
    wings = list(spec.get("wings", []))
    if spec.get("tower"):
        wings.append((spec["tower"], None))
    return z + max([height(w) for w, _off in wings] or [0.0])


def _pick(rng, lst):
    return lst[rng.randrange(len(lst))]


# ---------------------------------------------------------------------------
# Planning a band: the rhythm
# ---------------------------------------------------------------------------
def _motif(pool, mode, rng):
    pool = list(pool)
    a = _pick(rng, pool)
    others = [q for q in pool if q != a] or [a]
    b = _pick(rng, others)
    return {"uniform": [a], "alternate": [a, b], "pairs": [a, a, b, b],
            "triplet": [a, a, b]}[mode]


def _fill(run, motif, m, pool):
    """[(start, span, name)] covering *run* with the motif cycled, then
    mirrored about the centre when every slot has the same span."""
    out, s, k = [], 0.0, 0
    while run - s > 1e-6:
        name = motif[k % len(motif)]
        span = _span(name, m)
        if span > run - s + 1e-6:
            fits = [q for q in pool if _span(q, m) <= run - s + 1e-6]
            if not fits:
                break
            name = min(fits, key=lambda q: _span(q, m))
            span = _span(name, m)
        out.append([s, span, name])
        s += span
        k += 1
    if out and len({o[1] for o in out}) == 1:
        n = len(out)
        for i in range(n // 2):
            out[n - 1 - i][2] = out[i][2]
    return out


_BALCONY_MODES = ["every_other", "ends", "centre", "inner", "none"]


def _balcony_columns(n, mode):
    if mode == "every_other":
        return {i for i in range(n) if i % 2 == 1}
    if mode == "ends":
        return {0, n - 1} if n > 1 else set()
    if mode == "centre":
        return {n // 2} if n % 2 else {n // 2 - 1, n // 2}
    if mode == "inner":
        return set(range(1, n - 1))
    return set()


def _plan_band(band, W, D, rng):
    """One wall plan per side, drawn once and reused for every storey."""
    m = band["module"]
    corner = band.get("corner")
    L = corner[2] if corner else 0.0
    mode = band.get("rhythm") or _pick(rng, ["uniform", "alternate", "pairs", "uniform"])
    motif_long = _motif(band["walls"], mode, rng)
    plan = {}
    for side in ("S", "E", "N", "W"):
        length = (W if side in ("S", "N") else D) - 2 * L
        pool = band["walls"]
        if side == "N" and band.get("back"):
            pool = band["back"]
            slots = _fill(length, _motif(pool, "uniform", rng), m, pool)
        else:
            # the short sides take the same motif so the corner reads as one
            slots = _fill(length, motif_long, m, pool)
        # the entrance: centre slot of the street side, when it fits
        if side == "S" and band.get("front") and slots:
            c = len(slots) // 2
            ent = [q for q in band["front"] if _span(q, m) == slots[c][1]]
            if ent:
                slots[c][2] = _pick(rng, ent)
        plan[side] = slots
    extra = band.get("front_extra")
    if extra:
        mode_b = _pick(rng, _BALCONY_MODES)
        cols = {side: _balcony_columns(len(plan[side]), mode_b) for side in ("S", "N")}
        plan["_extra"] = (extra[0], cols)
    return plan


def _place_band(add, band, plan, z, W, D):
    corner = band.get("corner")
    L = corner[2] if corner else 0.0
    sides = band.get("sides", "SENW")
    extra = plan.get("_extra")
    for side in ("S", "E", "N", "W"):
        if side not in sides:
            continue
        for i, (s, span, name) in enumerate(plan[side]):
            px, py, yaw = _side_slot(side, L + s, span, W, D)
            qx, qy, qyaw = _piece_pose(name, px, py, yaw, span)
            add(name, qx, qy, z, qyaw, band["sub"])
            if extra and side in extra[1] and i in extra[1][side]:
                ex, ey, eyaw = _piece_pose(extra[0], px, py, yaw, span)
                add(extra[0], ex, ey, z, eyaw, band["sub"] + "_extra")
    if corner:
        name, authored, _L = corner
        for which in ("SW", "SE", "NE", "NW"):
            px, py, yaw = _corner_pivot(name, authored, which, W, D, L)
            add(name, px, py, z, yaw, band["sub"] + "_corner")


def _roof(add, roof, z, W, D):
    """Whole tiles plus one stretched remainder per axis, pivot-aware."""
    name, (a, b), pivot = roof
    nx, rx = int(W // a), W - int(W // a) * a
    ny, ry = int(D // b), D - int(D // b) * b
    cols = [(k * a, a) for k in range(nx)] + ([(nx * a, rx)] if rx > 1e-6 else [])
    rows = [(k * b, b) for k in range(ny)] + ([(ny * b, ry)] if ry > 1e-6 else [])
    for (x0, wx) in cols:
        for (y0, wy) in rows:
            sx, sy = wx / a, wy / b
            if pivot == "corner":
                px, py = x0, y0
            else:
                px, py = x0 + wx / 2.0, y0 + wy / 2.0
            st = None if abs(sx - 1) < 1e-6 and abs(sy - 1) < 1e-6 else (sx, sy)
            add(name, px, py, z, 0.0, "roof", stretch=st)


def _portico(add, portico, W):
    """A colonnade along the street side with a pediment band above it."""
    base, shaft, cap = portico["column"]
    hb, hs, hc = PIECES[base][2], PIECES[shaft][2], PIECES[cap][2]
    sp, back = portico["spacing"], -portico["setback"]
    n = int(W // sp)
    x0 = (W - (n - 1) * sp) / 2.0
    for k in range(n):
        x = x0 + k * sp
        add(base, x, back, 0.0, 0.0, "portico")
        add(shaft, x, back, hb, 0.0, "portico")
        add(cap, x, back, hb + hs, 0.0, "portico")
    ped, pw = portico["pediment"]
    z = hb + hs + hc
    for k in range(int(round(W / pw))):
        add(ped, k * pw, back, z, 0.0, "pediment")


def build_building(style, x, y, yaw, rng, z0=0.0, category=None, spec=None):
    """Assemble one building centred at *(x, y)*, front facing -Y at yaw 0.

    Bands stack from *z0*; a band marked `parapet` does not raise the roof,
    which sits at its base and is ringed by it. `wings` (and the older single
    `tower`) are whole sub-buildings standing on that roof.
    """
    spec = spec or STYLES[style]
    cat = category or f"bld_{style}"
    W, D = footprint(spec)
    ox, oy = -W / 2.0, -D / 2.0
    out = []
    tint = spec.get("glass")

    def add(name, lx, ly, lz, lyaw, sub, stretch=None):
        wx, wy = _rot(lx + ox, ly + oy, yaw)
        p = _place(_usd(name), x + wx, y + wy, lz, lyaw + yaw,
                   category=f"{cat}_{sub}", stretch=stretch, scale=_scale(name))
        if tint and name.startswith("SM_MBuilding05_Skyscraper"):
            p["glass"] = tint
        out.append(p)

    z = z0
    for band in spec["bands"]:
        plan = _plan_band(band, W, D, rng)
        for _ in range(band.get("repeat", 1)):
            _place_band(add, band, plan, z, W, D)
            if not band.get("parapet"):
                z += band["h"]
    if spec.get("portico"):
        _portico(add, spec["portico"], W)
    _roof(add, spec["roof"], z, W, D)
    for name, fx, fy, dz in spec.get("ornaments", []):
        add(name, W * fx, D * fy, z + dz, 0.0, "ornament")

    wings = list(spec.get("wings", []))
    if spec.get("tower"):
        wings.append((spec["tower"], None))
    for k, (wing, off) in enumerate(wings):
        tw, td = footprint(wing)
        tx, ty = off if off is not None else ((W - tw) / 2.0, (D - td) / 2.0)
        cx, cy = _rot(ox + tx + tw / 2.0, oy + ty + td / 2.0, yaw)
        sub = dict(wing)
        if not sub.get("glass"):
            sub["glass"] = tint
        out += build_building(style, x + cx, y + cy, yaw, rng, z0=z,
                              category=f"{cat}_wing{k}", spec=sub)
    return out


# ---------------------------------------------------------------------------
# Family builders: a style is a builder call, so a size or height is one line
# ---------------------------------------------------------------------------
def _c(name, authored, leg):
    return (name, authored, float(leg))


def _F(n):
    return "SM_MBuilding0" + n


def fam01(bays, storeys, facade="AB", note="", ground=True):
    """Stone mid-rise: 6 m shopfront ground, pilaster storeys, cornice parapet.
    5 m module, 1 m corners -> W = 2 + 5*bays."""
    walls, corner = {
        "AB": ([_F("1_Facade_A"), _F("1_Facade_B")], _F("1_Corner_B")),
        "CD": ([_F("1_Facade_C"), _F("1_Facade_D")], _F("1_Corner_C")),
    }[facade]
    bands = []
    if ground:
        bands.append({"sub": "ground", "h": 6.0, "module": 5.0, "rhythm": "uniform",
                      "walls": [_F("1_FirstFloor_A"), _F("1_FirstFloor_B"),
                                _F("1_FirstFloor_D"), _F("1_FirstFloor_E")],
                      "front": [_F("1_FirstFloor_C")],
                      "corner": _c(_F("1_Corner_A"), "SE", 1.0)})
    if storeys:
        bands.append({"sub": "storey", "h": 3.0, "module": 5.0, "repeat": storeys,
                      "walls": walls, "corner": _c(corner, "SE", 1.0)})
    bands.append({"sub": "parapet", "h": 2.0, "module": 5.0, "parapet": True,
                  "rhythm": "uniform", "walls": [_F("1_Rooftop")],
                  "corner": _c(_F("1_Corner_D"), "SE", 1.0)})
    return {"family": "01", "bays": bays, "note": note, "bands": bands,
            "roof": (_F("1_Roof"), (5.0, 5.0), "corner")}


def fam02(bays, storeys, corner_bays=True, balconies=True, note="", ground=True):
    """Modern office/apartments: 7 m glazed ground, 4 m storeys with balcony
    columns, roof ledge. With *corner_bays* the 8 m curved corner piece takes
    the ground corners (W = 16 + 8*bays); without, W = 8*bays with a ground
    floor or 4*bays without one (a wing on a podium)."""
    bands = []
    if ground:
        g = {"sub": "ground", "h": 7.0, "module": 8.0, "rhythm": "uniform",
             "walls": [_F("2_FirstFloor_B"), _F("2_FirstFloor_E")],
             "front": [_F("2_FirstFloor_C")], "back": [_F("2_FirstFloor_G")]}
        if corner_bays:
            g["corner"] = _c(_F("2_FirstFloor_A"), "SE", 8.0)
        bands.append(g)
    if storeys:
        st = {"sub": "storey", "h": 3.0, "module": 4.0, "repeat": storeys,
              "walls": [_F("2_Facade_A"), _F("2_Facade_B"), _F("2_Facade_C")]}
        if balconies:
            st["front_extra"] = (_F("2_Balcony_B"), 1.0)
        bands.append(st)
    bands.append({"sub": "ledge", "h": 1.0, "module": 4.0, "parapet": True,
                  "rhythm": "uniform", "walls": [_F("2_RoofLedge_A")]})
    return {"family": "02", "bays": bays, "note": note, "bands": bands,
            "roof": (_F("2_Roof_A"), (8.0, 8.0), "center")}


def fam03(bays, storeys, note=""):
    """Brownstone: 5 m ground with entrance, Bottom / Middle / Upper storeys,
    heavy cornice. 4 m module, 1 m corners -> W = 2 + 4*bays."""
    bands = [{"sub": "ground", "h": 5.0, "module": 4.0, "rhythm": "uniform",
              "walls": [_F("3_FirstFloor_B")], "front": [_F("3_FirstFloor_A")],
              "corner": _c(_F("3_FirstFloor_Corner"), "SE", 1.0)}]
    if storeys >= 2:
        bands.append({"sub": "storey_bottom", "h": 3.0, "module": 4.0,
                      "walls": [_F("3_Facade_B_Bottom")],
                      "corner": _c(_F("3_Facade_B_Corner"), "SE", 1.0)})
    if storeys >= 3:
        bands.append({"sub": "storey_middle", "h": 3.0, "module": 4.0,
                      "repeat": storeys - 2, "walls": [_F("3_Facade_B_Middle")],
                      "corner": _c(_F("3_Facade_B_Corner"), "SE", 1.0)})
    if storeys >= 1:
        bands.append({"sub": "storey_upper", "h": 3.0, "module": 4.0,
                      "rhythm": "uniform",
                      "walls": [_F("3_Facade_B_Upper"), _F("3_Facade_A")],
                      "corner": _c(_F("3_Facade_AB_Corner"), "SE", 1.0)})
    bands.append({"sub": "cornice", "h": 2.05, "module": 4.0, "parapet": True,
                  "walls": [_F("3_RoofLedge")],
                  "corner": _c(_F("3_RoofLedge_Corner"), "SE", 1.0)})
    return {"family": "03", "bays": bays, "note": note, "bands": bands,
            "roof": (_F("3_Roof_A"), (4.0, 4.0), "center")}


def fam04(bays, storeys, top=True, note=""):
    """Brick commercial: 7 m stone-arcade ground, brick storeys, an
    overhanging 4 m top floor. 4 m module, 1 m corners -> W = 2 + 4*bays."""
    bands = [{"sub": "ground", "h": 7.0, "module": 4.0,
              "walls": [_F("4_FirstFloor_A"), _F("4_FirstFloor_B"), _F("4_FirstFloor_C")],
              "corner": _c(_F("4_FirstFloor_Corner"), "SE", 1.0)}]
    if storeys:
        bands.append({"sub": "storey", "h": 3.0, "module": 4.0, "repeat": storeys,
                      "walls": [_F("4_Facade_A"), _F("4_Facade_B")],
                      "corner": _c(_F("4_Facade_Corner"), "SE", 1.0)})
    if top:
        bands.append({"sub": "top", "h": 4.0, "module": 4.0,
                      "walls": [_F("4_TopFloor_A"), _F("4_TopFloor_B")],
                      "corner": _c(_F("4_TopFloor_Corner"), "SE", 1.0)})
    return {"family": "04", "bays": bays, "note": note, "bands": bands,
            "roof": ("SM_Roof", (23.08, 23.08), "center")}


def tower05(bays, storeys, thick=True, glass=None):
    """Curtain-wall tower bands: 1 m-deep glazing (B) with its 1 m corner, or
    the zero-thickness glass plane (A) with its two-quad corner. W = 10 + 5*bays."""
    if thick:
        walls, corner = [_F("5_SkyscraperFacade_B")], _F("5_SkyscraperCorner_B")
    else:
        walls, corner = [_F("5_SkyscraperFacade_A")], _F("5_SkyscraperCorner_A")
    return {"bays": bays, "glass": glass, "bands": [
        {"sub": "storey", "h": 3.0, "module": 5.0, "repeat": storeys,
         "rhythm": "uniform", "walls": walls, "corner": _c(corner, "SW", 5.0)},
        {"sub": "parapet", "h": 1.0, "module": 5.0, "parapet": True,
         "rhythm": "uniform", "walls": [_F("5_SkyscraperRoofTopFacade")],
         "corner": _c(_F("5_SkyscraperRoofTopCorner"), "SW", 5.0)},
    ], "roof": (_F("5_SkyscraperRoof"), (5.0, 5.0), "corner")}


def fam05(bays, storeys, tower=None, wings=None, screen=True, glass=None, note=""):
    """Concrete-frame podium (7 m lobby, 3 m storeys, 4.4 m rooftop screen)
    carrying curtain-wall towers. 5 m module, 5 m corner BAYS -> W = 10 + 5*bays."""
    bands = [{"sub": "ground", "h": 7.0, "module": 5.0, "rhythm": "uniform",
              "walls": [_F("5_FirstFloor_A"), _F("5_FirstFloor_B"), _F("5_FirstFloor_C")],
              "back": [_F("5_FirstFloor_D")],
              "corner": _c(_F("5_FirstFloorCorner_B"), "SW", 5.0)}]
    if storeys:
        bands.append({"sub": "storey", "h": 3.0, "module": 5.0, "repeat": storeys,
                      "walls": [_F("5_Facade_A"), _F("5_Facade_B")],
                      "corner": _c(_F("5_FacadeCorner_A"), "SW", 5.0)})
    if screen:
        bands.append({"sub": "rooftop", "h": 4.41, "module": 5.0, "parapet": True,
                      "rhythm": "uniform", "walls": [_F("5_RoofTop_A"), _F("5_RoofTop_B")],
                      "corner": _c(_F("5_RoofTopCorner_A"), "SW", 5.0)})
    spec = {"family": "05", "bays": bays, "note": note, "bands": bands,
            "glass": glass, "roof": (_F("5_Roof"), (4.8, 4.8), "corner")}
    if tower:
        spec["tower"] = tower
    if wings:
        spec["wings"] = wings
    return spec


def skyscraper05(bays, storeys, thick=True, glass=None, note=""):
    """A straight tower from the ground: 7 m lobby bays, then curtain wall."""
    spec = fam05(bays, 0, screen=False, glass=glass, note=note)
    t = tower05(bays, storeys, thick)
    spec["bands"] += t["bands"]
    spec["roof"] = t["roof"]
    return spec


def dw_b(bays, upper=2, note=""):
    """Dmytro's Downtown_West storefront terrace: 4.3 m storefront under a
    1.2 m trim, then 6 m two-storey window bands, then the top cornice. The
    kit has 3 m and 5 m pieces, so the module is 1 m and widths are free;
    corners are a 0.8 m pillar on the storefront and butt joints above."""
    B = "SM_build_b_mod_"
    bands = [
        {"sub": "storefront", "h": 4.3, "module": 1.0, "rhythm": "alternate",
         "walls": [B + "lvl1_storefront_b_wall5m", B + "lvl1_storefront_b_wall3m",
                   B + "lvl1_storefront_a_3_5m"],
         "front": [B + "lvl1_doubleblackdoor"],
         "corner": _c(B + "lvl1_pillar_a", "C", 0.0)},
        {"sub": "trim", "h": 1.2, "module": 1.0, "rhythm": "uniform",
         "walls": [B + "lvl1_trim_5m", B + "lvl1_trim_3m"]},
    ]
    for k in range(upper):
        lvl = "lvl2" if k == 0 else "lvl3"
        bands.append({"sub": f"windows{k}", "h": 6.0, "module": 1.0,
                      "rhythm": "alternate",
                      "walls": [B + lvl + "_doublewindow", B + lvl + "_singlewindow"]
                      + ([B + "lvl2_widewindow"] if k == 0 else [])})
    bands.append({"sub": "cornice", "h": 1.61, "module": 1.0, "parapet": True,
                  "rhythm": "uniform", "walls": [B + "top_trim"]})
    return {"family": "dw_b", "bays": bays, "note": note, "bands": bands,
            "roof": ("SM_Roof", (23.08, 23.08), "center")}


def civic(bays, storeys, portico=False, note=""):
    """CivilianArea generic kit: 2.5 m module and storeys, corner posts.
    With *portico*: the government set's trim band, a colonnade of 9 m
    columns along the front and 6 m pediment pieces above it."""
    bands = [{"sub": "ground", "h": 2.5, "module": 2.5, "rhythm": "alternate",
              "walls": ["SM_DoubleWindow_01a", "SM_PlainWall", "SM_SingleWindow_01a"],
              "front": ["SM_Door"], "corner": _c("SM_CornerPillar", "C", 0.0)}]
    if storeys > 1:
        bands.append({"sub": "storey", "h": 2.5, "module": 2.5, "repeat": storeys - 1,
                      "walls": ["SM_DoubleWindow_01b", "SM_SingleWindow_01b", "SM_PlainWall"],
                      "corner": _c("SM_CornerPillar", "C", 0.0)})
    spec = {"family": "civ", "bays": bays, "note": note, "bands": bands,
            "roof": ("SM_Roof", (23.08, 23.08), "center")}
    if portico:
        bands.append({"sub": "trim", "h": 1.2, "module": 3.0, "parapet": True,
                      "rhythm": "uniform", "walls": ["SM_SetGoverment_Trim_01"],
                      "corner": _c("SM_SetGoverment_Trim_Corner_01_Large", "SE", 3.0)})
        spec["portico"] = {
            "column": ("SM_SetGoverment_Pillar_Base_02_Large",
                       "SM_SetGoverment_Pillar_02_Large",
                       "SM_SetGoverment_Pillar_Top_02_Large"),
            "spacing": 6.0, "setback": 3.0,
            "pediment": ("SM_SetGoverment_Roof_01", 6.0)}
    return spec


def church(bays, note=""):
    """CivilianArea church set: 2.75 m module, 5.5 m windows over a door
    band, corner beams, flat roof with the tower top and cross at the front."""
    C = "SM_Church_"
    return {"family": "church", "bays": bays, "note": note, "bands": [
        {"sub": "ground", "h": 2.75, "module": 2.75, "rhythm": "uniform",
         "walls": [C + "Window_03"], "front": [C + "Door_01"],
         "corner": _c(C + "Beam_Corner", "C", 0.0)},
        {"sub": "nave", "h": 5.5, "module": 2.75, "rhythm": "uniform",
         "walls": [C + "Window_02"]},        # the 2.75 m beam is half this band
    ], "roof": ("SM_Roof", (23.08, 23.08), "center"),
        "ornaments": [(C + "Tower_Top", 0.5, 0.1, 0.0), (C + "Cross_02", 0.5, 0.1, 3.72)]}


# Glass tints for the family-05 curtain wall, applied as a multiplier on the
# window texture (`apply_glass_tint`). The kit ships ONE glass texture, so
# this is the only lever for telling towers apart.
GLASS = {
    "clear":  (1.00, 1.00, 1.00),
    "blue":   (0.55, 0.72, 1.00),
    "steel":  (0.60, 0.66, 0.72),
    "bronze": (0.95, 0.62, 0.35),
    "green":  (0.55, 0.90, 0.72),
    "dark":   (0.30, 0.32, 0.38),
}


def _with(spec, **kw):
    out = dict(spec)
    out.update(kw)
    return out


STYLES = {
    # --- the original five (one per family), at a larger scale ------------
    "apartment":  fam01((4, 3), 4, "AB", "stone apartment block, 22x17, 18 m"),
    "office":     fam02((2, 0), 4, note="office with curved corner bays, 32x16, 19 m"),
    "brownstone": fam03((5, 3), 3, "brownstone with entrance and cornice, 22x14, 14 m"),
    "commercial": fam04((5, 4), 3, True, "brick commercial, arcade ground, 22x18, 20 m"),
    "tower":      fam05((3, 2), 2, tower=tower05((1, 1), 10), glass="blue",
                        note="podium + 10-storey tower, 25x20, 43 m"),
    # --- skyscrapers / high-rises ------------------------------------------
    "skyscraper_a":  skyscraper05((3, 3), 32, glass="dark", note="32-storey dark-glass tower, 25x25, 103 m"),
    "skyscraper_b":  fam05((4, 3), 3, tower=tower05((2, 2), 24), glass="bronze",
                           note="podium + 24-storey bronze tower, 30x25, 88 m"),
    "skyscraper_c":  skyscraper05((2, 2), 26, thick=False, glass="steel",
                                  note="slim steel-glass tower, 20x20, 85 m"),
    "highrise_step": fam05((5, 3), 4, tower=tower05((3, 1), 14), glass="green",
                           note="wide podium, 14-storey green tower, 35x25, 61 m"),
    "highrise_01":   fam01((4, 3), 16, "CD", "16-storey stone high-rise, 22x17, 54 m"),
    "highrise_02":   fam02((1, 0), 18, note="18-storey balcony high-rise, 24x16, 61 m"),
    "highrise_04":   fam04((5, 4), 10, True, "10-storey brick high-rise, 22x18, 41 m"),
    # --- offices -----------------------------------------------------------
    "office_wide":  fam02((3, 1), 5, note="wide office, 40x24, 22 m"),
    "office_slab":  fam02((6, 2), 7, corner_bays=False, note="slab office, 48x16, 28 m"),
    "office_plain": fam02((2, 1), 6, balconies=False, note="office without balconies, 32x24, 25 m"),
    # --- apartments / residential -----------------------------------------
    "apartment_tall": fam01((5, 3), 9, "CD", "9-storey apartments, plain façades, 27x17, 33 m"),
    "apartment_long": fam01((8, 2), 5, "AB", "long apartment block, 42x12, 21 m"),
    "brownstone_row": fam03((9, 3), 4, "row of brownstones, 38x14, 17 m"),
    "walkup":         fam03((5, 4), 6, "6-storey walk-up, 22x18, 23 m"),
    # --- commercial --------------------------------------------------------
    "department_store": fam04((10, 7), 2, True, "department store, 42x30, 17 m"),
    "commercial_mid":   fam04((7, 4), 5, True, "mid-rise commercial, 30x18, 26 m"),
    # --- city-block massings: a podium carrying several slabs --------------
    "block_residential": _with(
        fam02((9, 5), 0, note="full block: 7 m podium carrying three 16-20-storey "
                              "balcony slabs (the pack's merged block, rebuilt), 88x56, 67 m"),
        wings=[(fam02((6, 4), 16, corner_bays=False, ground=False), (0.0, 0.0)),
               (fam02((6, 4), 16, corner_bays=False, ground=False), (64.0, 0.0)),
               (fam02((10, 4), 10, corner_bays=False, ground=False), (24.0, 0.0)),
               (fam02((22, 4), 20, corner_bays=False, ground=False), (0.0, 40.0))]),
    "block_office": fam05((12, 8), 2, glass="steel", wings=[
                        (tower05((2, 2), 18, glass="steel"), (5.0, 5.0)),
                        (tower05((1, 1), 26, glass="blue"), (45.0, 30.0)),
                        (tower05((3, 1), 12, glass="dark"), (40.0, 5.0))],
                    note="full block: podium + three towers 54/78/36 m, 70x50, 91 m"),
    "block_stone": _with(
        fam01((12, 9), 0, "AB", "full block: 6 m stone podium, two 8-storey wings, 62x47, 30 m"),
        wings=[(fam01((12, 3), 8, "AB", ground=False), (0.0, 0.0)),
               (fam01((6, 3), 8, "CD", ground=False), (0.0, 30.0))]),
    "block_commercial": fam04((13, 8), 4, True, "full block department store, 54x34, 23 m"),
    # --- Downtown_West storefront terraces (Dmytro's kit) -------------------
    "dw_terrace":      dw_b((25, 15), 2, "storefront terrace, 2 window bands, 25x15, 18 m"),
    "dw_terrace_long": dw_b((40, 15), 1, "long storefront terrace, 40x15, 12 m"),
    # --- CivilianArea -----------------------------------------------------
    "civic_hall":    civic((24, 12), 4, portico=True, note="government hall with 9 m colonnade, 60x30, 13 m"),
    "civic_offices": civic((12, 8), 5, note="civic offices, 30x20, 13 m"),
    "church":        church((6, 10), "church: 5.5 m nave windows, tower top, 17x28, 8 m"),
}

ORDER = ["apartment", "office", "brownstone", "commercial", "tower"]
SHOWCASE = list(STYLES)


# ---------------------------------------------------------------------------
# Harvested blocks
# ---------------------------------------------------------------------------
_harvested = None


def load_harvested(path=HARVESTED_JSON):
    global _harvested
    if _harvested is None:
        with open(path) as f:
            _harvested = json.load(f)
    return _harvested


def harvested_names():
    return [b["name"] for b in load_harvested()["blocks"]]


def harvested_size(name):
    b = next(b for b in load_harvested()["blocks"] if b["name"] == name)
    return tuple(b["size"])


def build_harvested(name, x, y, yaw=0.0):
    """One of the artist's Downtown_West blocks, verbatim, centred at (x, y)."""
    data = load_harvested()
    b = next(b for b in data["blocks"] if b["name"] == name)
    a = math.radians(yaw)
    c, s = math.cos(a), math.sin(a)
    out = []
    for ai, m in b["pieces"]:
        rel = data["assets"][ai].replace("Assets/Game/Downtown_West/", "Downtown_West/")
        # row-vector convention: world = local * M_piece * R(yaw) * T(x, y)
        r = [[c, s, 0.0, 0.0], [-s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, 0.0, 1.0]]
        mm = [[sum(m[i][k] * r[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        out.append({"usd": SEI + rel, "x_m": mm[3][0], "y_m": mm[3][1], "z_m": mm[3][2],
                    "yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0, "scale": 1.0,
                    "axis_up": "Z", "raw_pivot": True, "matrix": mm,
                    "category": f"harvested_{name}"})
    return out


# ---------------------------------------------------------------------------
# Layouts
# ---------------------------------------------------------------------------
def build_street(rng, gap_m=8.0, styles=None, y=0.0):
    """The styles in a row along +X, fronts facing -Y, *gap_m* apart."""
    names = styles or ORDER
    widths = [footprint(STYLES[s])[0] for s in names]
    total = sum(widths) + gap_m * (len(names) - 1)
    out, where = [], []
    x = -total / 2.0
    for s, w in zip(names, widths):
        cx = x + w / 2.0
        out += build_building(s, cx, y, 0.0, rng)
        where.append((s, cx, w, footprint(STYLES[s])[1], height(STYLES[s])))
        x += w + gap_m
    return out, where


def build_showcase(rng, styles=None, row_w=220.0, gap_m=12.0, row_gap_m=40.0,
                   y0=0.0, harvested=True):
    """Every style packed into rows from the south, shortest first, plus the
    harvested Downtown_West blocks. Returns (placements, where, n_rows) with
    where = [(name, x, y, W, D, H), ...]."""
    items = [(s, footprint(STYLES[s]) + (height(STYLES[s]),), False)
             for s in (styles or SHOWCASE)]
    if harvested:
        items += [(n, harvested_size(n), True) for n in harvested_names()]
    items.sort(key=lambda it: it[1][2])
    out, where = [], []
    x, y, row_d, row = 0.0, y0, 0.0, 0
    for name, (W, D, H), is_h in items:
        if x > 0.0 and x + W > row_w:
            y += row_d + row_gap_m
            x, row_d, row = 0.0, 0.0, row + 1
        cx, cy = x + W / 2.0, y + D / 2.0
        out += build_harvested(name, cx, cy) if is_h else build_building(name, cx, cy, 0.0, rng)
        where.append((name, cx, cy, W, D, H))
        x += W + gap_m
        row_d = max(row_d, D)
    return out, where, row + 1


# ---------------------------------------------------------------------------
# On-stage pass (needs pxr)
# ---------------------------------------------------------------------------
def apply_glass_tint(stage, placements):
    """Multiply the curtain-wall window texture by each tower's `glass` tint.

    The kit's materials are UsdPreviewSurface networks; the BaseColor is a
    UsdUVTexture whose `scale` input multiplies the sample. Setting it on the
    shader INSIDE each placed reference is legal because placements are not
    instanced, and it leaves the mullion pattern intact — which is why this is
    done instead of binding a flat-coloured material.
    """
    from pxr import Gf, Sdf, Usd, UsdShade
    n = 0
    for p in placements:
        tint = p.get("glass")
        path = p.get("prim_path")
        if not tint or not path:
            continue
        rgb = GLASS[tint] if isinstance(tint, str) else tint
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid():
            continue
        for prim in Usd.PrimRange(root):
            sh = UsdShade.Shader(prim)
            if not sh:
                continue
            sid = sh.GetIdAttr().Get()
            if sid == "UsdPreviewSurface":
                # The kit's curtain wall is a PERFECT MIRROR (metallic 1.0,
                # roughness 0.0): from the street a pristine tower shows
                # hard-edged black triangles and diagonal bars — reflections
                # of the podium posts and the next tower — which read as
                # damage (earthquake round 3, agent G). Real IGU glass is a
                # slightly soft reflector; GLASS_ROUGHNESS is the knob.
                r = sh.GetInput("roughness")
                rv = r.Get() if r else None
                if rv is not None and float(rv) < 0.05:
                    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(GLASS_ROUGHNESS))
                continue
            if sid != "UsdUVTexture":
                continue
            f = sh.GetInput("file")
            v = f.Get() if f else None
            if not isinstance(v, Sdf.AssetPath) or "BaseColor" not in (v.path or ""):
                continue
            sh.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*rgb, 1.0))
            n += 1
    return n


GLASS_ROUGHNESS = 0.22   # curtain-wall roughness when the kit's shader is a perfect mirror


# ---------------------------------------------------------------------------
# Checks — everything provable without a stage
# ---------------------------------------------------------------------------
def _fits(run, spans):
    """Can pieces of these spans tile *run* exactly? (small DP)"""
    if run < -1e-6:
        return False
    steps = int(round(run * 4))
    ok = [False] * (steps + 1)
    ok[0] = True
    units = [int(round(s * 4)) for s in spans]
    for i in range(1, steps + 1):
        ok[i] = any(u <= i and ok[i - u] for u in units)
    return ok[steps]


def _check_spec(name, spec, problems):
    W, D = footprint(spec)
    for bi, band in enumerate(spec["bands"]):
        m = band["module"]
        corner = band.get("corner")
        L = corner[2] if corner else 0.0
        names = list(band["walls"]) + list(band.get("front", [])) + list(band.get("back", []))
        if corner:
            names.append(corner[0])
        if band.get("front_extra"):
            names.append(band["front_extra"][0])
        missing = [n for n in names if n not in PIECES]
        for n in missing:
            problems.append(f"{name} band {bi}: unmeasured piece {n}")
        if missing:
            continue
        spans = sorted({_span(n, m) for n in band["walls"]})
        for side_len, axis in ((W, "W"), (D, "D")):
            if not _fits(side_len - 2 * L, spans):
                problems.append(f"{name} band {bi} ({band['sub']}): {axis}="
                                f"{side_len} is not 2*{L} + a tiling of {spans}")
        for n in names:
            sx, sy, sz, *_ = PIECES[n]
            is_corner = corner and n == corner[0]
            if not is_corner and abs(_width(n) - _span(n, m)) > 1.0:
                problems.append(f"{name} band {bi}: {n} is {_width(n)} m wide, "
                                f"module is {m}")
            tol = 1.5 if is_corner else 0.5
            if abs(sz - band["h"]) > tol and not band.get("parapet"):
                problems.append(f"{name} band {bi}: {n} is {sz} m tall, "
                                f"band is {band['h']}")
    roof, (a, b), pivot = spec["roof"]
    if roof not in PIECES:
        problems.append(f"{name}: unmeasured roof {roof}")
    for n, _fx, _fy, _dz in spec.get("ornaments", []):
        if n not in PIECES:
            problems.append(f"{name}: unmeasured ornament {n}")
    if spec.get("portico"):
        for n in list(spec["portico"]["column"]) + [spec["portico"]["pediment"][0]]:
            if n not in PIECES:
                problems.append(f"{name}: unmeasured portico piece {n}")
    wings = list(spec.get("wings", []))
    if spec.get("tower"):
        wings.append((spec["tower"], None))
    for k, (w, off) in enumerate(wings):
        tw, td = footprint(w)
        ox, oy = off if off is not None else ((W - tw) / 2.0, (D - td) / 2.0)
        if ox < -1e-6 or oy < -1e-6 or ox + tw > W + 1e-6 or oy + td > D + 1e-6:
            problems.append(f"{name}: wing {k} {tw}x{td} at ({ox},{oy}) overhangs {W}x{D}")
        _check_spec(f"{name}.wing{k}", w, problems)


def check(verbose=True):
    problems = []
    for name in STYLES:
        _check_spec(name, STYLES[name], problems)
    if verbose:
        if problems:
            print("[urban_building] CHECK FAILED:")
            for q in problems:
                print("  " + q)
        else:
            print(f"[urban_building] check ok — {len(STYLES)} styles, "
                  f"{min(height(v) for v in STYLES.values()):.0f}-"
                  f"{max(height(v) for v in STYLES.values()):.0f} m tall")
    return problems


def summarise(placements):
    from collections import Counter
    c = Counter(p["category"] for p in placements)
    lines = [f"{len(placements)} placements, "
             f"{len({p['usd'] for p in placements})} distinct kit assets"]
    for k in sorted(c):
        lines.append(f"  {k:<44} {c[k]:4d}")
    return "\n".join(lines)


if __name__ == "__main__":
    import random
    if check():
        raise SystemExit(1)
    pl, where, n_rows = build_showcase(random.Random(7))
    print(f"{len(pl)} placements, {len(where)} buildings in {n_rows} rows")
    for s, x, y, w, d, h in where:
        note = STYLES[s]["note"] if s in STYLES else "harvested Downtown_West block"
        print(f"  {s:<20} ({x:6.1f},{y:6.1f})  {w:3.0f} x {d:3.0f} m  {h:3.0f} m  {note}")
