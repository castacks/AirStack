"""
urban_building.py — urban buildings assembled from a modular FAÇADE kit.

THE GRAMMAR: BANDS x SIDES x CORNERS, THEN A ROOF
--------------------------------------------------
A suburban house (`modular_house.py`) is a footprint of cells with walls
generated from the boundary. An urban block is simpler and taller: a rectangle
whose four sides are each a RUN of façade modules between two CORNER pieces,
stacked in vertical BANDS (a tall ground floor, N identical storeys, a parapet
or top-floor band), then a flat roof. Every family in a commercial kit is
authored this way, so a building here is a list of bands, each naming the
pieces for its walls and its corner, and `build_building` does the rest.

THE KIT IN USE, AND THE ONE THAT WAS ASKED FOR
----------------------------------------------
`/Projects/SEI-COA/ProceduralBuildingGenerator/` is an Unreal Engine 4.22 pack
(772 `.uasset`, 2 `.umap`, zero USDs; its "generator" is a 28 MB Blueprint).
Nothing outside Unreal can read `.uasset`, and there is no Unreal on this host,
so it cannot be referenced from Isaac Sim until it is exported to USD — the
other packs under `/Projects/SEI-COA/` were (the Omniverse UE Connector's
`*_payload.usd` / `.prop.usd` layout is its signature). This module therefore
targets the one modular urban kit that IS in USD:

    /Projects/SEI-COA/ModernCityEnvironment01/Meshes/    318 USDs, 5 families

and describes it as DATA (`FAMILIES` / `STYLES`) so that the PBG kit drops in
once exported: re-measure with `tools/modular_kit_report.py`, write its table.

THE GRID, MEASURED (`tools/modular_kit_report.py` re-measures it)
----------------------------------------------------------------
    unit      metres (metersPerUnit 1.0, Z-up), hence SCALE = 1.0.
    storey    3.0 m. Every `*_Facade_*` is exactly 3.00 m tall.
    module    5 m (families 01, 05) or 4 m (02, 03, 04); ground floors run
              8 m / 12 m in family 02.
    pivot     at the LEFT end of the piece, ON the wall line, at the floor:
              a façade spans local x in [0, module], its body straddles y=0
              (-0.22..+0.25 for 01_Facade_A) and stands on z=0.
    outward   local -Y. Measured from area-weighted face normals, which sum
              to -0.7..-0.8 of total area in Y on every façade; balconies
              (02_Balcony_C reaches y=-2.85) and cornices (03_RoofLedge
              y=-0.78) protrude that way. Same convention as modular_house:
              yaw 0 puts the front on the south.
    corners   an L whose two legs are L_m long ON THE WALL LINE, plus the
              same protrusion as the band's façades. Two authoring frames:
                "SE"  legs 1 m (01/03/04) or 8 m (02 ground): the X-leg runs
                      from the pivot +X along the south wall, the wall-line
                      corner is at local (L, 0), the Y-leg runs +Y from there
                      along the EAST wall. Faces -Y and +X (normals +0.35/-0.35).
                "SW"  legs 5 m (05): the pivot IS the wall-line corner; legs
                      run +X along the south wall and +Y along the WEST wall.
                      Faces -Y and -X.
              A side's run is therefore W - 2L long and must tile with the
              band's module: W = 2L + n*module. `check()` enforces it.
    roofs     flat tiles: 01_Roof 5x5 and 05_Roof 4.8x4.8 with the pivot at
              a corner (0..a, 0..b); 02_Roof_A 8x8, 03_Roof_A 4x4 and SM_Roof
              23x23 CENTRED. Laid as whole tiles plus one stretched remainder
              per axis (`stretch` is a per-axis scale `apply_placements`
              understands), so the texture keeps its scale over most of the
              roof and nothing pokes past the parapet.
    payloads  every piece is `SM_X.usd` -> payload `SM_X_payload.usd`;
              `apply_placements` calls `prim.Load()`, which is what makes
              them draw. 104 of 185 have the Mesh AS the root prim.

Nothing here imports `pxr`: the layout is plain Python and is checked host-side
by `check()`; the launch script writes it with `scene_generator.apply_placements`.
"""

import math

KIT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443"
       "/Projects/SEI-COA/ModernCityEnvironment01/Meshes/")
SCALE = 1.0
STOREY_M = 3.0

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
    "SM_MBuilding02_Balcony_B":    (4.00, 1.13, 3.00, 0.00, -0.90, 0.00),
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
    # family 04 — glass commercial, 4 m module, 1 m corners, 7 m ground floor
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
    "SM_MBuilding05_Facade_A":           (5.00, 0.40, 3.00, 0.00, -0.30, 0.00),
    "SM_MBuilding05_Facade_B":           (5.00, 0.20, 3.00, 0.00, -0.20, 0.00),
    "SM_MBuilding05_FacadeCorner_A":     (5.30, 5.30, 3.00, -0.30, -0.30, 0.00),
    "SM_MBuilding05_RoofTop_A":          (5.00, 0.50, 4.41, 0.00, -0.30, 0.00),
    "SM_MBuilding05_RoofTop_B":          (5.00, 0.40, 4.00, 0.00, -0.20, 0.00),
    "SM_MBuilding05_RoofTopCorner_A":    (5.30, 5.30, 4.41, -0.30, -0.30, 0.00),
    "SM_MBuilding05_Roof":               (4.80, 4.80, 0.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperFacade_B": (5.00, 1.00, 3.00, 0.00, -1.00, 0.00),
    "SM_MBuilding05_SkyscraperCorner_B": (6.00, 6.00, 3.00, -1.00, -1.00, 0.00),
    "SM_MBuilding05_SkyscraperRoof":     (5.00, 5.00, 0.00, 0.00, 0.00, 0.00),
    "SM_MBuilding05_SkyscraperRoofTopFacade": (5.00, 0.50, 1.00, 0.00, -0.50, 0.00),
    "SM_MBuilding05_SkyscraperRoofTopCorner": (5.50, 5.50, 1.00, -0.50, -0.50, 0.00),
}

# Pieces whose authored frame is ambiguous from a bbox alone (L-shapes whose
# second leg reads as double-sided in the normal sum). Shown by
# `build_prop_strip` so they can be read off a render before a style uses them.
UNPLACED = [
    "SM_MBuilding02_Facade_D", "SM_MBuilding02_Facade_E",
    "SM_MBuilding02_Facade_F", "SM_MBuilding02_RoofLedge_Corner",
    "SM_MBuilding05_FirstFloorCorner_A", "SM_MBuilding05_SkyscraperCorner_A",
    "SM_MBuilding05_SkyscraperFacade_A", "SM_MBuilding01_Wall",
    "SM_Ground_Floor_Entrance", "SM_Seam_Corner",
]


def _usd(name):
    return KIT + name + ".usd"


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
#
# Each entry maps the building corner to (yaw, pivot offset from the wall-line
# corner point, in the piece's OWN leg length). Derived, not guessed, from the
# two authoring frames in the module docstring:
#   "SE": pivot = P - R(yaw)*(L, 0)   yaw SE 0, NE 90, NW 180, SW 270
#   "SW": pivot = P                    yaw SW 0, SE 90, NE 180, NW 270
# ---------------------------------------------------------------------------
_CORNER_YAW = {
    "SE": {"SE": 0.0, "NE": 90.0, "NW": 180.0, "SW": 270.0},
    "SW": {"SW": 0.0, "SE": 90.0, "NE": 180.0, "NW": 270.0},
}


def _corner_pivot(authored, which, W, D, L):
    P = {"SW": (0.0, 0.0), "SE": (W, 0.0), "NE": (W, D), "NW": (0.0, D)}[which]
    yaw = _CORNER_YAW[authored][which]
    if authored == "SE":
        dx, dy = _rot(L, 0.0, yaw)
        return P[0] - dx, P[1] - dy, yaw
    return P[0], P[1], yaw


# Side -> (yaw, pivot for a run slot [s, s+m] on that side). The pivot is the
# LEFT end of the piece as seen from outside, which for N and W is the far end
# of the slot because yaw 180/270 send local +X to world -X/-Y.
def _side_slot(side, s, m, W, D):
    if side == "S":
        return s, 0.0, 0.0
    if side == "E":
        return W, s, 90.0
    if side == "N":
        return s + m, D, 180.0
    return 0.0, s + m, 270.0


def _runs(length, m):
    n = int(round(length / m))
    return [k * m for k in range(n)]


# ---------------------------------------------------------------------------
# One building
# ---------------------------------------------------------------------------
def footprint(spec):
    """(W, D) on the wall line, from bays x module + corner legs.

    `bays` counts the run modules per side at the FIRST band; every band's
    (module, corner leg) pair must then divide W - 2L, which `check()` proves.
    """
    b0 = spec["bands"][0]
    L = b0["corner"][2] if b0.get("corner") else 0.0
    m = b0["module"]
    nx, ny = spec["bays"]
    return 2 * L + nx * m, 2 * L + ny * m


def _pick(rng, lst):
    return lst[rng.randrange(len(lst))]


def _band(out, add, spec, band, z, W, D, rng):
    """All four sides of one band at height *z*."""
    m = band["module"]
    corner = band.get("corner")          # (name, authored, leg_m) or None
    L = corner[2] if corner else 0.0
    for side in ("S", "E", "N", "W"):
        length = (W if side in ("S", "N") else D) - 2 * L
        pool = band["walls"]
        if side == "S" and band.get("front"):
            pool = band["front"]
        elif side == "N" and band.get("back"):
            pool = band["back"]
        for s in _runs(length, m):
            px, py, yaw = _side_slot(side, L + s, m, W, D)
            add(_pick(rng, pool), px, py, z, yaw, band["sub"])
            # An occasional balcony/extra on the street side, in the same slot.
            extra = band.get("front_extra")
            if side == "S" and extra and rng.random() < extra[1]:
                add(extra[0], px, py, z, yaw, band["sub"] + "_extra")
    if corner:
        name, authored, _L = corner
        for which in ("SW", "SE", "NE", "NW"):
            px, py, yaw = _corner_pivot(authored, which, W, D, L)
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
            else:                                   # centred tile
                px, py = x0 + wx / 2.0, y0 + wy / 2.0
            st = None if abs(sx - 1) < 1e-6 and abs(sy - 1) < 1e-6 else (sx, sy)
            add(name, px, py, z, 0.0, "roof", stretch=st)


def build_building(style, x, y, yaw, rng, z0=0.0, category=None):
    """Assemble one building centred at *(x, y)*, front facing -Y at yaw 0.

    Returns placements. Bands stack from *z0*; a band marked `parapet` does
    not raise the roof, which sits at its base and is ringed by it.
    """
    spec = STYLES[style]
    cat = category or f"bld_{style}"
    W, D = footprint(spec)
    ox, oy = -W / 2.0, -D / 2.0
    out = []

    def add(name, lx, ly, lz, lyaw, sub, stretch=None):
        wx, wy = _rot(lx + ox, ly + oy, yaw)
        out.append(_place(_usd(name), x + wx, y + wy, lz, lyaw + yaw,
                          category=f"{cat}_{sub}", stretch=stretch))

    z = z0
    for band in spec["bands"]:
        for _ in range(band.get("repeat", 1)):
            _band(out, add, spec, band, z, W, D, rng)
            if not band.get("parapet"):
                z += band["h"]
    _roof(add, spec["roof"], z, W, D)

    tower = spec.get("tower")
    if tower:
        tw, td = footprint(tower)
        tx, ty = tower.get("offset", ((W - tw) / 2.0, (D - td) / 2.0))
        # The tower is a building in its own right, standing on the roof.
        sub = _TowerStyle(tower)
        STYLES[sub.key] = tower
        try:
            cx, cy = _rot(ox + tx + tw / 2.0, oy + ty + td / 2.0, yaw)
            out += build_building(sub.key, x + cx, y + cy, yaw, rng, z0=z,
                                  category=f"{cat}_tower")
        finally:
            del STYLES[sub.key]
    return out


class _TowerStyle:
    _n = 0

    def __init__(self, spec):
        _TowerStyle._n += 1
        self.key = f"__tower_{_TowerStyle._n}"


# ---------------------------------------------------------------------------
# The catalogue: one style per kit family
# ---------------------------------------------------------------------------
def _c(name, authored, leg):
    return (name, authored, float(leg))


STYLES = {
    # 17 x 12 m, 6 m ground + 3 storeys + parapet = 15 m to the roof.
    "apartment": {
        "family": "01", "bays": (3, 2),
        "bands": [
            {"sub": "ground", "h": 6.0, "module": 5.0,
             "walls": ["SM_MBuilding01_FirstFloor_A", "SM_MBuilding01_FirstFloor_B",
                       "SM_MBuilding01_FirstFloor_D", "SM_MBuilding01_FirstFloor_E"],
             "front": ["SM_MBuilding01_FirstFloor_C", "SM_MBuilding01_FirstFloor_A",
                       "SM_MBuilding01_FirstFloor_E"],
             "corner": _c("SM_MBuilding01_Corner_A", "SE", 1.0)},
            {"sub": "storey", "h": 3.0, "module": 5.0, "repeat": 3,
             "walls": ["SM_MBuilding01_Facade_A", "SM_MBuilding01_Facade_B"],
             "corner": _c("SM_MBuilding01_Corner_B", "SE", 1.0)},
            {"sub": "parapet", "h": 2.0, "module": 5.0, "parapet": True,
             "walls": ["SM_MBuilding01_Rooftop"],
             "corner": _c("SM_MBuilding01_Corner_D", "SE", 1.0)},
        ],
        "roof": ("SM_MBuilding01_Roof", (5.0, 5.0), "corner"),
        "note": "mid-rise apartment block: 6 m shopfront ground floor, "
                "3 storeys, cornice parapet"},
    # 24 x 16 m: the 8 m ground corners fill the whole short side, the 4 m
    # storeys above simply butt at the corners (the family has no thin corner).
    "office": {
        "family": "02", "bays": (1, 0),
        "bands": [
            {"sub": "ground", "h": 7.0, "module": 8.0,
             "walls": ["SM_MBuilding02_FirstFloor_B", "SM_MBuilding02_FirstFloor_E"],
             "front": ["SM_MBuilding02_FirstFloor_C", "SM_MBuilding02_FirstFloor_E"],
             "back": ["SM_MBuilding02_FirstFloor_G"],
             "corner": _c("SM_MBuilding02_FirstFloor_A", "SE", 8.0)},
            {"sub": "storey", "h": 3.0, "module": 4.0, "repeat": 3,
             "walls": ["SM_MBuilding02_Facade_A", "SM_MBuilding02_Facade_B",
                       "SM_MBuilding02_Facade_C"],
             "front_extra": ("SM_MBuilding02_Balcony_B", 0.35)},
            {"sub": "ledge", "h": 1.0, "module": 4.0, "parapet": True,
             "walls": ["SM_MBuilding02_RoofLedge_A"]},
        ],
        "roof": ("SM_MBuilding02_Roof_A", (8.0, 8.0), "center"),
        "note": "office block: 7 m glazed ground floor with 8 m corner bays, "
                "3 storeys with balconies, roof ledge"},
    # 14 x 10 m, 5 m ground + bottom/middle/upper storeys (the family names
    # its storeys as a vertical sequence) + a 2 m cornice.
    "brownstone": {
        "family": "03", "bays": (3, 2),
        "bands": [
            {"sub": "ground", "h": 5.0, "module": 4.0,
             "walls": ["SM_MBuilding03_FirstFloor_B"],
             "front": ["SM_MBuilding03_FirstFloor_A", "SM_MBuilding03_FirstFloor_B"],
             "corner": _c("SM_MBuilding03_FirstFloor_Corner", "SE", 1.0)},
            {"sub": "storey1", "h": 3.0, "module": 4.0,
             "walls": ["SM_MBuilding03_Facade_B_Bottom"],
             "corner": _c("SM_MBuilding03_Facade_B_Corner", "SE", 1.0)},
            {"sub": "storey2", "h": 3.0, "module": 4.0,
             "walls": ["SM_MBuilding03_Facade_B_Middle"],
             "corner": _c("SM_MBuilding03_Facade_B_Corner", "SE", 1.0)},
            {"sub": "storey3", "h": 3.0, "module": 4.0,
             "walls": ["SM_MBuilding03_Facade_B_Upper", "SM_MBuilding03_Facade_A"],
             "corner": _c("SM_MBuilding03_Facade_AB_Corner", "SE", 1.0)},
            {"sub": "cornice", "h": 2.05, "module": 4.0, "parapet": True,
             "walls": ["SM_MBuilding03_RoofLedge"],
             "corner": _c("SM_MBuilding03_RoofLedge_Corner", "SE", 1.0)},
        ],
        "roof": ("SM_MBuilding03_Roof_A", (4.0, 4.0), "center"),
        "note": "brownstone: 5 m ground floor with entrance, three storeys "
                "of stone façade, cornice"},
    # 18 x 14 m, 7 m ground + 2 glass storeys + a 4 m overhanging top floor.
    "glass_commercial": {
        "family": "04", "bays": (4, 3),
        "bands": [
            {"sub": "ground", "h": 7.0, "module": 4.0,
             "walls": ["SM_MBuilding04_FirstFloor_A", "SM_MBuilding04_FirstFloor_B",
                       "SM_MBuilding04_FirstFloor_C"],
             "corner": _c("SM_MBuilding04_FirstFloor_Corner", "SE", 1.0)},
            {"sub": "storey", "h": 3.0, "module": 4.0, "repeat": 2,
             "walls": ["SM_MBuilding04_Facade_A", "SM_MBuilding04_Facade_A",
                       "SM_MBuilding04_Facade_B"],
             "corner": _c("SM_MBuilding04_Facade_Corner", "SE", 1.0)},
            {"sub": "top", "h": 4.0, "module": 4.0,
             "walls": ["SM_MBuilding04_TopFloor_A", "SM_MBuilding04_TopFloor_B"],
             "corner": _c("SM_MBuilding04_TopFloor_Corner", "SE", 1.0)},
        ],
        "roof": ("SM_Roof", (23.08, 23.08), "center"),
        "note": "glass commercial: 7 m lobby, two curtain-wall storeys, "
                "overhanging top floor"},
    # 20 x 15 m podium (7 m ground, 2 storeys, 4.4 m rooftop plant screen)
    # carrying a 15 x 10 m tower of 8 storeys. Corners are whole 5 m bays.
    "tower": {
        "family": "05", "bays": (2, 1),
        "bands": [
            {"sub": "ground", "h": 7.0, "module": 5.0,
             "walls": ["SM_MBuilding05_FirstFloor_A", "SM_MBuilding05_FirstFloor_B",
                       "SM_MBuilding05_FirstFloor_C"],
             "back": ["SM_MBuilding05_FirstFloor_D"],
             "corner": _c("SM_MBuilding05_FirstFloorCorner_B", "SW", 5.0)},
            {"sub": "storey", "h": 3.0, "module": 5.0, "repeat": 2,
             "walls": ["SM_MBuilding05_Facade_A", "SM_MBuilding05_Facade_B"],
             "corner": _c("SM_MBuilding05_FacadeCorner_A", "SW", 5.0)},
            {"sub": "rooftop", "h": 4.41, "module": 5.0, "parapet": True,
             "walls": ["SM_MBuilding05_RoofTop_A", "SM_MBuilding05_RoofTop_B"],
             "corner": _c("SM_MBuilding05_RoofTopCorner_A", "SW", 5.0)},
        ],
        "roof": ("SM_MBuilding05_Roof", (4.8, 4.8), "corner"),
        "tower": {
            "bays": (1, 0),
            "bands": [
                {"sub": "storey", "h": 3.0, "module": 5.0, "repeat": 8,
                 "walls": ["SM_MBuilding05_SkyscraperFacade_B"],
                 "corner": _c("SM_MBuilding05_SkyscraperCorner_B", "SW", 5.0)},
                {"sub": "parapet", "h": 1.0, "module": 5.0, "parapet": True,
                 "walls": ["SM_MBuilding05_SkyscraperRoofTopFacade"],
                 "corner": _c("SM_MBuilding05_SkyscraperRoofTopCorner", "SW", 5.0)},
            ],
            "roof": ("SM_MBuilding05_SkyscraperRoof", (5.0, 5.0), "corner"),
        },
        "note": "podium-and-tower: 7 m lobby, 2 storeys, rooftop screen, "
                "8-storey curtain-wall tower set back on the roof"},
}

ORDER = ["apartment", "office", "brownstone", "glass_commercial", "tower"]


# ---------------------------------------------------------------------------
# Layouts
# ---------------------------------------------------------------------------
def build_street(rng, gap_m=8.0, styles=None, y=0.0):
    """The styles in a row along +X, fronts facing -Y, *gap_m* apart.

    Returns ``(placements, [(style, x, W, D), ...])`` so a launcher can aim a
    camera and print where each building stands.
    """
    names = styles or ORDER
    widths = [footprint(STYLES[s])[0] for s in names]
    total = sum(widths) + gap_m * (len(names) - 1)
    out, where = [], []
    x = -total / 2.0
    for s, w in zip(names, widths):
        cx = x + w / 2.0
        out += build_building(s, cx, y, 0.0, rng)
        where.append((s, cx, w, footprint(STYLES[s])[1]))
        x += w + gap_m
    return out, where


def build_prop_strip(x0=0.0, y0=0.0, gap_m=12.0, names=None):
    """One of each ambiguous piece in a line, for reading its frame off a render."""
    out, x = [], x0
    for n in (names or UNPLACED):
        out.append(_place(_usd(n), x, y0, 0.0, 0.0, category="strip"))
        x += gap_m
    return out


# ---------------------------------------------------------------------------
# Checks — everything provable without a stage
# ---------------------------------------------------------------------------
def _check_spec(name, spec, problems):
    W, D = footprint(spec)
    for bi, band in enumerate(spec["bands"]):
        m = band["module"]
        corner = band.get("corner")
        L = corner[2] if corner else 0.0
        for side_len, axis in ((W, "W"), (D, "D")):
            run = side_len - 2 * L
            if run < -1e-6 or abs(run / m - round(run / m)) > 1e-6:
                problems.append(f"{name} band {bi} ({band['sub']}): {axis}="
                                f"{side_len} is not 2*{L} + n*{m}")
        names = list(band["walls"]) + list(band.get("front", [])) \
            + list(band.get("back", []))
        if corner:
            names.append(corner[0])
        if band.get("front_extra"):
            names.append(band["front_extra"][0])
        for n in names:
            if n not in PIECES:
                problems.append(f"{name} band {bi}: unmeasured piece {n}")
                continue
            sx, sy, sz, *_ = PIECES[n]
            is_corner = corner and n == corner[0]
            if not is_corner and abs(sx - m) > 1.0:
                problems.append(f"{name} band {bi}: {n} is {sx} m wide, "
                                f"module is {m}")
            # A corner may carry its own ledge (02_FirstFloor_A is 8.2 m on a
            # 7 m band); a wall piece may not.
            tol = 1.5 if is_corner else 0.5
            if abs(sz - band["h"]) > tol and not band.get("parapet"):
                problems.append(f"{name} band {bi}: {n} is {sz} m tall, "
                                f"band is {band['h']}")
    roof, (a, b), pivot = spec["roof"]
    if roof not in PIECES:
        problems.append(f"{name}: unmeasured roof {roof}")
    if pivot not in ("corner", "center"):
        problems.append(f"{name}: roof pivot {pivot!r}")
    if spec.get("tower"):
        tw, td = footprint(spec["tower"])
        if tw > W + 1e-6 or td > D + 1e-6:
            problems.append(f"{name}: tower {tw}x{td} overhangs podium {W}x{D}")
        _check_spec(name + ".tower", spec["tower"], problems)


def check(verbose=True):
    problems = []
    for name in ORDER:
        _check_spec(name, STYLES[name], problems)
    if verbose:
        if problems:
            print("[urban_building] CHECK FAILED:")
            for q in problems:
                print("  " + q)
        else:
            sizes = ", ".join(f"{s} {footprint(STYLES[s])[0]:.0f}x"
                              f"{footprint(STYLES[s])[1]:.0f}" for s in ORDER)
            print(f"[urban_building] check ok — {len(ORDER)} styles: {sizes}")
    return problems


def summarise(placements):
    from collections import Counter
    c = Counter(p["category"] for p in placements)
    lines = [f"{len(placements)} placements, "
             f"{len({p['usd'] for p in placements})} distinct kit assets"]
    for k in sorted(c):
        lines.append(f"  {k:<36} {c[k]:4d}")
    return "\n".join(lines)


if __name__ == "__main__":
    import random
    if check():
        raise SystemExit(1)
    pl, where = build_street(random.Random(7))
    print(summarise(pl))
    for s, x, w, d in where:
        print(f"  {s:<18} x={x:+7.1f}  {w:.0f} x {d:.0f} m")
