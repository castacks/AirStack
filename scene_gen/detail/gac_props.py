"""gac_props — dress a GreatAmericanCity building with its own roof and wall kit.

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
    wall_stair   SM_Building_Stair    1.6 x 3.3 x  4.2   ONE STOREY of external
                                                         stair; a fire escape is
                                                         this, stacked

There is no asset with "escape" in its name in the whole pack — the fire-escape
role is filled by stacking `SM_Building_Stair`, and the measurement is what says
so: 4.18 m tall is a storey, and 1.61 m deep against 3.33 m wide is a thing that
hangs off a wall rather than stands on a roof. `SM_Stair` is NOT a candidate
however much its name suggests it: at 0.06 x 0.57 x 1.92 m it is a single step.

FIRE ESCAPES GO ON A BLANK ELEVATION. That is both how it is done — a service
face, never the show front — and what this stock needs anyway, since most of
these buildings are blank on two or three sides (`_plans/gac_faces.json`) and
those faces are exactly where something is wanted to break up the slab.

FRAME. A placement's `yaw_deg` already includes the per-asset `yaw-offset` that
normalises the pack's mixed fronts, so a measured side maps to world by that one
rotation and nothing else has to be unwound. Props are seated by their measured
`z0`, because several have a pivot that is not on their base —
`SM_Steel_Pipe_Plastic` sits at -6.16.
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
ESCAPE_PITCH_M = 4.18  # SM_Building_Stair's own height — one storey
ESCAPE_BASE_M = 3.0    # first landing above the pavement
ROOF_HOUSE_MIN_M2 = 1400.0
STAIR = "SM_Building_Stair"


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


def _place(rec, x, y, z, yaw, category, of=None):
    """One prop placement, seated on its own measured base.

    `of` names the building it belongs to. `apply_placements` ignores keys it
    does not know, and having it makes a prop AUDITABLE — without it the only
    way to tell which building an escape belongs to is proximity, and on a real
    street the neighbour is closer than the far side of the same building.
    """
    return {"of": of, "usd": rec["usd"], "x_m": float(x), "y_m": float(y),
            "z_m": float(z - rec["z0"]), "yaw_deg": float(yaw) % 360.0,
            "roll_deg": 0.0, "pitch_deg": 0.0,
            "scale": float(rec.get("mpu", 1.0)),
            "category": category, "axis_up": "Z", "raw_pivot": True}


def _name_of(usd):
    return usd.rsplit("/", 1)[-1][:-4]


def roof_props(bld, dims, by_kind, rng, max_props=6, of=None):
    """Plant, tanks and masts on the roof. Returns [placement]."""
    W, D, H = dims["W"], dims["D"], dims["H"]
    yaw = float(bld.get("yaw_deg", 0.0))
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x0, y0 = float(bld["x_m"]), float(bld["y_m"])
    area = W * D
    out, taken = [], []

    def _free(lx, ly, r):
        return all((lx - qx) ** 2 + (ly - qy) ** 2 > (r + qr) ** 2
                   for qx, qy, qr in taken)

    def _put(rec, kind):
        rad = 0.5 * max(rec["W"], rec["D"])
        hw = max(1.0, W / 2.0 - PARAPET_M - rad)
        hd = max(1.0, D / 2.0 - PARAPET_M - rad)
        for _ in range(14):
            lx, ly = rng.uniform(-hw, hw), rng.uniform(-hd, hd)
            if not _free(lx, ly, rad + 0.8):
                continue
            taken.append((lx, ly, rad))
            # the prop's own centre offset has to turn with the building
            wx = x0 + lx * ca - ly * sa
            wy = y0 + lx * sa + ly * ca
            out.append(_place(rec, wx, wy, H,
                              yaw + rng.choice((0.0, 90.0, 180.0, 270.0)),
                              "roof_prop", of))
            return True
        return False

    # A BIG ROOF GETS ITS OVERRUN FIRST, because it is the one prop that will
    # not fit later and it is what makes a roof read as a real building rather
    # than a lid with boxes on it.
    house = [r for r in by_kind.get("roof_house", [])
             if r["W"] + 2 * PARAPET_M < W and r["D"] + 2 * PARAPET_M < D]
    if house and area >= ROOF_HOUSE_MIN_M2 and rng.random() < 0.55:
        _put(rng.choice(house), "roof_house")
    if by_kind.get("roof_tank") and rng.random() < 0.75:
        _put(rng.choice(by_kind["roof_tank"]), "roof_tank")
    if by_kind.get("roof_mast") and area > 900.0 and rng.random() < 0.35:
        _put(rng.choice(by_kind["roof_mast"]), "roof_mast")
    # plant scales with roof area — one unit per ~250 m2, capped
    pool = by_kind.get("roof_plant", []) + by_kind.get("roof_pipe", [])
    n = max(1, min(max_props - len(out), int(area / 250.0)))
    for _ in range(n):
        if pool:
            _put(rng.choice(pool), "roof_prop")
    return out


def wall_props(bld, dims, face, by_kind, rng, of=None):
    """A fire escape up a BLANK elevation, plus service runs. [placement]."""
    blanks = list(face.get("blank_sides") or [])
    if not blanks:
        return []
    W, D, H = dims["W"], dims["D"], dims["H"]
    yaw = float(bld.get("yaw_deg", 0.0))
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x0, y0 = float(bld["x_m"]), float(bld["y_m"])
    stair = next((r for r in by_kind.get("wall_stair", [])
                  if r["name"] == STAIR), None)
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

    if stair is not None and H >= ESCAPE_BASE_M + 2 * ESCAPE_PITCH_M:
        t = rng.uniform(-0.28, 0.28) * run
        # sit the run just proud of the wall: half its depth outboard
        px = lx * (half + stair["W"] / 2.0 - 0.05) + ax * t
        py = ly * (half + stair["W"] / 2.0 - 0.05) + ay * t
        wx, wy = _world(px, py)
        # the stair's thin axis is X (1.61 vs 3.33), so its local +X is the
        # outward direction; align that with the wall normal
        syaw = yaw + math.degrees(math.atan2(ly, lx))
        z = ESCAPE_BASE_M
        while z + ESCAPE_PITCH_M <= H - 1.5:
            out.append(_place(stair, wx, wy, z, syaw, "fire_escape", of))
            z += ESCAPE_PITCH_M
    for r in by_kind.get("wall_run", []):
        if rng.random() > 0.35:
            continue
        t = rng.uniform(-0.42, 0.42) * run
        px = lx * (half + r["W"] / 2.0) + ax * t
        py = ly * (half + r["W"] / 2.0) + ay * t
        wx, wy = _world(px, py)
        out.append(_place(r, wx, wy, rng.uniform(1.0, max(1.5, H - 3.0)),
                          yaw + math.degrees(math.atan2(ly, lx)),
                          "wall_run", of))
    return out


def dress(placements, rng, by_kind=None, faces=None, dims=None,
          category="house"):
    """Dress every GreatAmericanCity building in *placements*.

    Returns the NEW prop placements only, so the caller appends them and the
    building list is untouched.
    """
    if by_kind is None:
        by_kind, faces, dims = load()
    out = []
    for p in placements:
        if p.get("category") != category:
            continue
        nm = _name_of(p.get("usd", ""))
        if nm not in dims or nm not in faces:
            continue
        tag = "%s@%.1f,%.1f" % (nm, p["x_m"], p["y_m"])
        out += roof_props(p, dims[nm], by_kind, rng, of=tag)
        out += wall_props(p, dims[nm], faces[nm], by_kind, rng, of=tag)
    return out
