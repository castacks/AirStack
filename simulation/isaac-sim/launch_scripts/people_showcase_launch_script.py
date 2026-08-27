#!/usr/bin/env python
"""
People bench — every survivor placement, side by side, on one small plate.

    ISAAC_SIM_SCRIPT_NAME=people_showcase_launch_script.py \
    SCENE_CONFIG=suburb_wildfire airstack up isaac-sim

WHY THIS EXISTS
---------------
`disaster.people` puts ~60 survivors into a 1600 x 1200 m plat, which is the
right thing for the dataset and the wrong thing for JUDGING it: the six
situations end up kilometres apart, and checking that a seated figure sits at
the right height means flying to it. This bench is the same six situations at
the same code path, laid out in a row 60 m apart, plus a pose row that stands
every authored pose shoulder to shoulder for comparison.

It is deliberately NOT a small suburb. There is no street network, no lots and
no fire field — those decide WHERE a group goes, which the plat already
demonstrates and which cannot be judged from a photograph anyway. What this
bench isolates is the part that can only be judged on sight: the POSE, the
height it sits at, which way it faces, and whether you can see a person
through a car's windows once the glass is stripped.

THE ROW, LEFT TO RIGHT
----------------------
  x=-180  A  refuge: parking lot   asphalt slab with painted bays, three cars
                                   nosed in, a group standing / sitting /
                                   crouching between them. 14 of the Camp Fire's
                                   31 refuge areas were car parks.
  x=-100  B  refuge: open ground   the same group on bare lawn, nothing to
                                   burn within 15 m. The control for A: same
                                   poses, no asphalt, so the lot is not doing
                                   the reading for them.
  x= -20  C  pool                  a pool at its authored 8 x 4 m with the
                                   water plane at -0.35 m, its coping and its
                                   paved apron at the plat's own widths, and
                                   three figures standing chest-deep in it.
                                   The Lahaina seawall case, in a suburb.
  x= +60  D  gridlock queue        five cars nose to tail in a lane, the head
                                   stopped short of a fallen bole laid across
                                   it, somebody seated IN a glass-stripped car,
                                   people beside the cars, one walking out
                                   past the blockage, one sitting on the kerb.
  x=+150  E  at home, outside      a house with a car on the drive and people
                                   in the front yard — the last-minute case.
  x=+225  F  exposed interior      a `roof_collapsed` archetype with a figure
                                   on the floor plate, visible from above
                                   BECAUSE the roof fell in.

  y=+70   POSE ROW                 one character per pose at 6 m spacing:
                                   idle, walk, crouch, sit_ground,
                                   sit_edge (on a low wall), seated_car (on a
                                   seat-height block). Same rig, same scale, so
                                   any pose that is wrong is wrong HERE, next
                                   to six that are right.

WHAT TO LOOK FOR, IN ORDER
--------------------------
  1. Does every figure's SUPPORT touch the thing it is on — feet on grass,
     seat on the coping, hips on the ground? `_POSE_Z_OFFSET` is the table
     that decides this and it was derived, not measured, so it is the first
     thing that will be wrong.
  2. Can you see the person inside the car in unit D? If not, `strip_glass`
     missed a mesh on that asset (the Muyang cars cannot be fixed — their
     windows are painted into a single-mesh texture — so D uses a Retro car).
  3. Do the seated figures read as seated from 40 m up, which is the altitude
     the dataset is captured from? A pose that only works at eye level is not
     worth authoring.

The bench builds nothing procedural and references the same archetypes the
plat does, so it loads in well under a minute.
"""

import math
import os
import random
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                    # noqa: E402
import suburb_scene as ss                                       # noqa: E402
from scene_prep import get_stage_meters_per_unit                # noqa: E402
from compile_disaster import load_scene_config                  # noqa: E402
from disaster import people as ppl                              # noqa: E402
from detail import vehicles as veh                              # noqa: E402
from detail import modular_house as mh                          # noqa: E402

PARENT = "/World/stage/generated"
SEED = int(os.environ.get("PEOPLE_SEED", "11"))
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
ARCH_DIR = os.environ.get(
    "ARCH_DIR", "/isaac-sim/AirStack/scene_gen/assets/archetypes")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)
# UNITS=A,D runs a subset while iterating on one vignette.
ONLY = [s.strip().upper() for s in os.environ.get("UNITS", "").split(",")
        if s.strip()]

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Cut.usda"
ROUGH_MAT = "airstack://scene_gen/assets/materials/Grass_Countryside.usda"
ASPHALT_MAT = ("airstack://scene_gen/assets/materials/megascans/"
               "Road_Asphalt.usda")
# The pool apron, and the same key the plat binds it with — `usds.materials`
# `pool_apron` in `config/asset_sets/suburban.yaml`.
PAVEMENT_MAT = ("airstack://scene_gen/assets/materials/megascans/"
                "Worn_Pavement.usda")

# Station centres. Wide enough apart that one unit's props never read as part
# of its neighbour, close enough that the whole row fits one overview frame.
STATIONS = [
    ("A", "refuge: parking lot", -180.0),
    ("B", "refuge: open ground", -100.0),
    ("C", "pool",                 -20.0),
    ("D", "gridlock queue",        60.0),
    ("E", "at home, outside",     150.0),
    ("F", "exposed interior",     225.0),
]
# The pool station's water is below grade, so the lawn is cut around it — and
# around the WATER only, which is what `apply_ground` cuts `pool_rects` to.
# The coping band and the apron are laid OVER the lawn like a driveway, not
# cut into it. `modular_house.pool_at` authors 8 x 4 m and hands on the water
# ring, that rectangle inset by one coping half-band; the same inset here.
POOL_W_M, POOL_D_M = 8.0, 4.0
_POOL_HL = POOL_W_M / 2.0 - mh.POOL_COPING_M
_POOL_HD = POOL_D_M / 2.0 - mh.POOL_COPING_M
POOL_HOLE = (-20.0 - _POOL_HL, -_POOL_HD, -20.0 + _POOL_HL, _POOL_HD)

POSE_ROW_Y = 70.0
# No `wave`: removed from scene_generator._HUMAN_POSES on review.
POSE_ROW = ["idle", "walk", "crouch", "sit_ground", "sit_edge",
            "seated_car"]


# ---------------------------------------------------------------------------
# stage furniture
# ---------------------------------------------------------------------------

def _mat(stage, key, url):
    path = "/World/ground/materials/" + key
    prim = stage.DefinePrim(Sdf.Path(path))
    prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
    prim.Load()
    return path


def build_ground_and_light(stage, ssf):
    """Lawn under the row and even daylight over it.

    Plain grass, no burn scar. The scar is the plat's problem and it is a
    translucent overlay that would sit between the camera and every figure
    here; a pose is judged against the ground it stands on, and that ground
    should be the least interesting thing in frame.
    """
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground/materials"))
    rough = _mat(stage, "grass_rough", ROUGH_MAT)
    grass = _mat(stage, "grass", GRASS_MAT)
    asphalt = _mat(stage, "asphalt", ASPHALT_MAT)
    pavement = _mat(stage, "pavement", PAVEMENT_MAT)

    def _sheet(prefix, x0, y0, x1, y1, z, col, mat):
        """A ground sheet with the pool cut out of it.

        BOTH sheets need the hole, not just the top one. The lawn sits on a
        wider rough-grass backdrop, and cutting only the lawn leaves the
        backdrop — 33 cm above the water — drawing straight over the pool, so
        the surface still reads as grass inside a grey ring. `suburb_scene`
        makes the same point about its own backdrop plane: the block meshes
        subtract the pool rects and the base plane did not, "so the hole
        revealed an opaque plane and the pool was never visible".
        """
        hx0, hy0, hx1, hy1 = POOL_HOLE
        for name, (ax, ay, bx, by) in {
                "w": (x0, y0, min(hx0, x1), y1),
                "e": (max(hx1, x0), y0, x1, y1),
                "s": (max(hx0, x0), y0, min(hx1, x1), min(hy0, y1)),
                "n": (max(hx0, x0), max(hy1, y0), min(hx1, x1), y1)}.items():
            if bx - ax <= 1e-6 or by - ay <= 1e-6:
                continue
            sg._make_plane_mesh(stage, prefix + "_" + name, ax, ay, bx, by, z,
                                3.0, ssf, display_color=col,
                                mat_prim_path=mat)

    _sheet("/World/ground/base", -320.0, -160.0, 360.0, 160.0, -0.02,
           (0.21, 0.31, 0.15), rough)
    # THE LAWN HAS A HOLE IN IT, because the pool's water sits BELOW grade.
    # `modular_house.pool_at` puts the surface at -0.35 m and the suburb cuts
    # the block mesh around it for exactly this reason ("without a hole the
    # grass simply draws over it and the pool is never visible"). A bench that
    # lays one unbroken quad reproduces that bug, and the pool comes out as a
    # grey ring with lawn inside it — which is what it did.
    _sheet("/World/ground/lawn", -260.0, -60.0, 300.0, 110.0, 0.0,
           (0.24, 0.36, 0.17), grass)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(950.0)
    dome.CreateColorAttr(Gf.Vec3f(0.74, 0.78, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2400.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 32.0))
    return {"grass": grass, "rough": rough, "asphalt": asphalt,
            "pavement": pavement}


def _slab(stage, path, x0, y0, x1, y1, z, ssf, mat, col):
    # `mat_prim_path` is typed as a str and an unbound surface must pass "",
    # not None — the water is display-colour only.
    sg._make_plane_mesh(stage, path, x0, y0, x1, y1, z, 3.0, ssf,
                        display_color=col, mat_prim_path=(mat or ""))


def _frame(stage, prefix, inner, outer, z, ssf, mat, col):
    """The band between two axis-aligned rings, as four slabs.

    A FRAME, NOT A SLAB, for the reason `suburb_scene.apply_ground` gives on
    the same geometry: filled, it draws over the water below grade and the
    pool disappears.
    """
    ix0, ix1 = min(q[0] for q in inner), max(q[0] for q in inner)
    iy0, iy1 = min(q[1] for q in inner), max(q[1] for q in inner)
    ox0, ox1 = min(q[0] for q in outer), max(q[0] for q in outer)
    oy0, oy1 = min(q[1] for q in outer), max(q[1] for q in outer)
    for name, (ax, ay, bx, by) in {
            "n": (ox0, iy1, ox1, oy1),
            "s": (ox0, oy0, ox1, iy0),
            "e": (ix1, iy0, ox1, iy1),
            "w": (ox0, iy0, ix0, iy1)}.items():
        _slab(stage, f"{prefix}_{name}", ax, ay, bx, by, z, ssf, mat, col)


def _box(stage, path, cx, cy, w, d, h, ssf, col=(0.62, 0.60, 0.57)):
    """A plain block — a kerb, a low wall, a car seat to sit on."""
    hx, hy = w / 2.0, d / 2.0
    pts, counts, idx = [], [], []
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    for (dx, dy) in corners:
        pts.append(Gf.Vec3f((cx + dx) * ssf, (cy + dy) * ssf, 0.0))
    for (dx, dy) in corners:
        pts.append(Gf.Vec3f((cx + dx) * ssf, (cy + dy) * ssf, h * ssf))
    faces = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    for f in faces:
        counts.append(4)
        idx += list(f)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    return path


def _label(stage, path, text_x, text_y, ssf):
    """A stake at each station, so a capture can be told apart from its
    neighbour without counting along the row."""
    _box(stage, path, text_x, text_y, 0.35, 0.35, 3.2, ssf,
         col=(0.86, 0.24, 0.20))


# ---------------------------------------------------------------------------
# the vignettes
# ---------------------------------------------------------------------------

class Bench:
    """Collects placements; each `unit_*` adds one station's worth."""

    def __init__(self, ctx, rng):
        self.ctx = ctx
        self.rng = rng
        self.cars = []
        self.humans = []
        self.notes = []
        self.refs = []          # (archetype key, x, y, yaw)
        self._tree_xy = []      # so two stands never plant on each other
        self.car_row_xy = []    # (label, x, y) for the per-car close-ups
        self.hdg_rows = []      # (label, x, y) for the heading test
        # (label, x, y, radius) — a POSE cannot be judged from the station
        # frame: at 62 m a figure is 40 px tall and a seated one is a smudge.
        # Each unit names the patch of ground its people stand on and the
        # capture pass shoots it plan, oblique and at EYE LEVEL, which is the
        # only angle that shows where a limb actually is.
        self.close = []

    # -- helpers ---------------------------------------------------------
    def human(self, x, y, z_ground, yaw, pose, tag="rigged", usd=None):
        usd = usd or self.ctx["pick"](tag)
        p = ppl._human_placement(self.ctx, usd, x, y, z_ground, yaw, pose)
        self.humans.append(p)
        return p

    def car(self, x, y, heading, tag="residential", glass=True, usd=None):
        usd = usd or self.ctx["pick_car"](tag, glass)
        p = ppl._car_placement(self.ctx, usd, x, y, heading, "bench")
        # KEEP THE HEADING, NOT JUST THE COMPOSED YAW. `_car_placement` returns
        # `yaw_deg = heading + the CAR's yaw-offset`, and an occupant needs the
        # heading: `_human_placement` will add the HUMAN's own offset on top,
        # so passing the car's composed yaw double-counts an offset that is not
        # the passenger's and lands them sideways in the seat.
        p["heading_deg"] = float(heading)
        self.cars.append(p)
        return p

    def seat_in(self, car, x_off=0.0, y_off=0.0):
        """Put somebody in the driver's seat of *car*, if they fit.

        The seat pan is a third of the roof height on a saloon, but a seated
        adult needs about 0.85 m of head room above it — so on anything low the
        two constraints fight and the pan has to drop. Returns the note.
        """
        ap = self.ctx["asset_pools"]
        fp = self.ctx["resolver"].get(car["usd"], "car",
                                      scale=ap.scale_of(car["usd"]),
                                      axis_up=ap.axis_of(car["usd"]))
        roof = float(fp.get("sz", 1.5))
        head = 0.85
        seat = max(0.18, min(roof / 3.0, roof - head - 0.04))
        fits = (roof - seat) >= head
        hd = float(car.get("heading_deg", 0.0))
        rad = math.radians(hd)
        self.human(car["x_m"] + math.cos(rad) * x_off - math.sin(rad) * y_off,
                   car["y_m"] + math.sin(rad) * x_off + math.cos(rad) * y_off,
                   seat, hd, "seated_car")
        return "{0}: roof {1:.2f} m, seat {2:.2f} m, {3}".format(
            os.path.basename(car["usd"]), roof, seat,
            "fits" if fits else "HEAD THROUGH THE ROOF")

    # -- context ---------------------------------------------------------
    def burnt_trees(self, arch, cx, cy, n, r_in, r_out, levels=None):
        """A burnt stand around a station, from the same archetypes the plat
        references.

        CONTEXT IS NOT DECORATION HERE. Every one of these vignettes is a
        wildfire scene and the thing that says so is the vegetation: a group
        standing on asphalt with green lawn around it reads as a car park, and
        the same group with black poles behind it reads as a refuge. The stand
        is drawn as a RING with an inner radius so it frames the vignette
        instead of standing in it — `r_in` should clear whatever the station
        built, since these archetypes carry their own ground debris out to
        about 10 m and debris under a car looks like a collision.
        """
        levels = levels or ("torched", "snag", "scorched", "torched")
        species = ("Black_Oak", "Shumard_Oak", "Douglas_Fir",
                   "Largetooth_Aspen", "American_Beech")
        placed = 0
        for i in range(n):
            for _try in range(12):
                a = self.rng.uniform(0.0, 2.0 * math.pi)
                rr = math.sqrt(self.rng.uniform(0.0, 1.0))
                r = r_in + (r_out - r_in) * rr
                x, y = cx + math.cos(a) * r, cy + math.sin(a) * r
                if all((x - px) ** 2 + (y - py) ** 2 > 81.0
                       for (px, py) in self._tree_xy):
                    break
            else:
                continue
            key = "tree_{0}_{1}".format(
                species[self.rng.randrange(len(species))],
                levels[self.rng.randrange(len(levels))])
            if key not in arch:
                continue
            self._tree_xy.append((x, y))
            self.refs.append((key, x, y, self.rng.uniform(0.0, 360.0)))
            placed += 1
        return placed

    # -- A: parking-lot refuge -------------------------------------------
    def unit_a(self, stage, ssf, mats, cx, arch):
        """Asphalt, bays, cars, and a group between them."""
        w, d = 44.0, 26.0
        _slab(stage, f"{PARENT}/A_lot", cx - w / 2, -d / 2, cx + w / 2, d / 2,
              0.04, ssf, mats["asphalt"], (0.21, 0.23, 0.26))
        # Bay stripes, drawn the way the park lot draws them.
        # PAINT SITS ON THE SLAB, so its box has to START above it: `_box`
        # builds from z=0 up, and at 0.01 m tall on a 0.04 m slab every stripe
        # was buried and the lot came out blank.
        # `r`, not the coordinate: a USD prim name cannot contain '-', so
        # f"bay_{int(-9.4)}" is an illegal path and Sdf.Path rejects it.
        for r, row_y in enumerate((-d / 2 + 3.6, d / 2 - 3.6)):
            for i in range(13):
                bx = cx - w / 2 + 3.0 + i * 2.7
                _box(stage, f"{PARENT}/A_bay_{r}_{i}", bx, row_y,
                     0.12, 5.4, 0.055, ssf, col=(0.93, 0.93, 0.90))
        for i, (ox, oy, hd) in enumerate(((-12.0, -6.0, 90.0),
                                          (-6.6, -6.0, 90.0),
                                          (7.0, 4.0, 200.0))):
            self.car(cx + ox, oy, hd)
        # The group: two clusters, mixed posture.
        grp = [(-2.0, -1.0, "idle"), (0.4, 0.6, "idle"), (1.9, -1.6, "idle"),
               (-1.2, 2.2, "sit_ground"), (2.8, 1.4, "sit_ground"),
               (-3.4, 1.0, "crouch")]
        for (ox, oy, pose) in grp:
            self.human(cx + ox, oy, 0.04, self.rng.uniform(0, 360), pose)
        self.close.append(("A_group", cx, 0.0, 7.0))
        # The apron out to a kerb, so the lot reads as reachable rather than
        # as a slab dropped on a lawn — the park's lot has one for the same
        # reason and publishes it in `info["park"]["parking"]["apron"]`.
        _slab(stage, f"{PARENT}/A_apron", cx - 4.0, d / 2, cx + 4.0,
              d / 2 + 9.0, 0.04, ssf, mats["asphalt"], (0.21, 0.23, 0.26))
        self.burnt_trees(arch, cx, 0.0, 16, max(w, d) / 2 + 3.0,
                         max(w, d) / 2 + 20.0)
        _label(stage, f"{PARENT}/A_label", cx, d / 2 + 12.0, ssf)

    # -- B: open ground --------------------------------------------------
    def unit_b(self, stage, ssf, mats, cx, arch):
        """The same postures on bare lawn — A's control."""
        grp = [(-2.2, -0.8, "idle"), (0.0, 0.0, "idle"), (2.1, 0.9, "idle"),
               (-0.9, 2.4, "sit_ground"), (1.6, -2.2, "sit_ground"),
               (3.2, -0.4, "crouch")]
        for (ox, oy, pose) in grp:
            self.human(cx + ox, oy, 0.0, self.rng.uniform(0, 360), pose)
        # Open ground IS the scenario, so the stand is held well back: the
        # 15 m clearance `people._open_ground` enforces is the whole point.
        self.burnt_trees(arch, cx, 0.0, 10, 17.0, 30.0)
        _label(stage, f"{PARENT}/B_label", cx, 14.0, ssf)

    # -- C: pool ---------------------------------------------------------
    def unit_c(self, stage, ssf, mats, cx, arch):
        """An 8 x 4 m pool at the authored water height, with standers in it.

        Built from plain quads rather than `modular_house.pool_at` on purpose:
        that function sites a pool behind a HOUSE on a lot, and the lot is the
        one thing this bench does not have. The numbers are its numbers, and
        the two bands around the water come from `modular_house.pool_rings`,
        so the deck cannot drift from the one the plat ships.
        """
        water = mh.POOL_WATER_Z_M
        x0, y0, x1, y1 = POOL_HOLE
        ring = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        coping, apron = mh.pool_rings(ring)
        # TWO BANDS, AS ON THE PLAT: the coping straddling the pool rectangle,
        # and the paved apron outside it — `apply_ground` binds that one to
        # `pool_apron`, the worn pavement, and so does this. The plat sets the
        # kit's stone `Swimming_Pool_Edge_01` in the inner band; there is no
        # lot to place it on here, so it stays a slab.
        #
        # Both sit on the bench's own "just above the lawn" rung. The plat's
        # ladder is derived from the plate span (`ground_z_scale`) and this
        # bench has no plate; the two bands are disjoint in plan, so being
        # coplanar costs nothing and is how a deck meets its coping anyway.
        _frame(stage, f"{PARENT}/C_coping", ring, coping, 0.02, ssf,
               mats["asphalt"], (0.78, 0.76, 0.72))
        _frame(stage, f"{PARENT}/C_apron", coping, apron, 0.02, ssf,
               mats["pavement"], (0.62, 0.61, 0.58))
        # The water, and a floor under it so the pool does not read as a hole
        # cut through the world when you look in from above.
        _slab(stage, f"{PARENT}/C_floor", x0, y0, x1, y1,
              water - 1.55, ssf, "", (0.36, 0.52, 0.55))
        _slab(stage, f"{PARENT}/C_water", x0, y0, x1, y1,
              water, ssf, "", (0.09, 0.36, 0.53))
        # CHEST-DEEP STANDERS AND NOTHING ELSE, which is what `people._pools`
        # now places: the floater read as debris from capture altitude and the
        # coping-sitter's legs vanished into the deck. Offsets stay inside the
        # 0.55 / 0.45 of half-extent box `_pool_people` draws in, and the sink
        # is its arithmetic — the water plane less `CHEST_FRAC` of the rig's
        # MEASURED height, not a hand-copied 1.25 m.
        ap = self.ctx["asset_pools"]
        for (ox, oy, hd) in ((-2.0, 0.4, 200.0), (0.4, -0.5, 25.0),
                             (2.1, 0.7, 310.0)):
            usd = self.ctx["pick"]("rigged")
            fp = self.ctx["resolver"].get(usd, "human", scale=ap.scale_of(usd),
                                          axis_up=ap.axis_of(usd))
            self.human(cx + ox, oy,
                       water - ppl.CHEST_FRAC * float(fp.get("sz", 1.8)),
                       hd, "idle", usd=usd)
        self.burnt_trees(arch, cx, 0.0, 8, 12.0, 24.0)
        _label(stage, f"{PARENT}/C_label", cx, 12.0, ssf)

    # -- D: the gridlock queue -------------------------------------------
    def unit_d(self, stage, ssf, mats, cx, arch):
        """Cars nose to tail, a bole across the lane, people in and around."""
        lane_w = 10.7
        _slab(stage, f"{PARENT}/D_road", cx - 46.0, -lane_w / 2,
              cx + 46.0, lane_w / 2, 0.03, ssf, mats["asphalt"],
              (0.20, 0.21, 0.23))
        _box(stage, f"{PARENT}/D_kerb", cx, lane_w / 2 + 0.25, 92.0, 0.5,
             0.14, ssf, col=(0.68, 0.67, 0.64))
        # The blocker: a fallen bole ACROSS the carriageway. Referenced from
        # the same archetype the plat uses, yawed 90 to the lane.
        key = "tree_Douglas_Fir_fallen"
        if key in arch:
            self.refs.append((key, cx + 16.0, 0.0, 90.0))
        else:
            self.notes.append("D: %s missing, using a plain barrier" % key)
            _box(stage, f"{PARENT}/D_barrier", cx + 16.0, 0.0, 1.1, 11.0,
                 1.1, ssf, col=(0.24, 0.20, 0.17))
        # Five cars backed up from 5 m short of it, nose toward +x (outbound).
        seat = None
        for i in range(5):
            x = cx + 11.0 - i * 6.4
            y = -2.4 + self.rng.uniform(-0.35, 0.35)
            c = self.car(x, y, 0.0 + self.rng.uniform(-4.0, 4.0),
                         tag="residential", glass=True)
            if i == 1:
                seat = c
        if seat is not None:
            self.notes.append("D: " + self.seat_in(seat, x_off=-0.15))
        # Beside the cars, on the kerb, and walking out past the blockage.
        self.human(cx + 6.0, 1.6, 0.03, 300.0, "idle")
        self.human(cx - 2.0, 2.2, 0.03, 250.0, "idle")
        self.human(cx - 8.5, -5.4, 0.03, 60.0, "idle")
        self.human(cx - 14.0, lane_w / 2 + 0.6, 0.14, 180.0, "sit_edge")
        for k in range(3):
            self.human(cx + 26.0 + k * 7.0, -1.0 + k * 1.3, 0.03,
                       0.0 + self.rng.uniform(-12.0, 12.0), "walk")
        # Stand back from the carriageway on both sides. The fallen bole that
        # blocks the road came from one of these.
        self.burnt_trees(arch, cx, 14.0, 9, 9.0, 22.0)
        self.burnt_trees(arch, cx, -14.0, 9, 9.0, 22.0)
        _label(stage, f"{PARENT}/D_label", cx, lane_w / 2 + 16.0, ssf)

    # -- E: at home, outside ---------------------------------------------
    def unit_e(self, stage, ssf, mats, cx, arch):
        """A house, a car on the drive, people in the front yard."""
        key = "house_ranch_scorched"
        if key not in arch:
            key = next((k for k in arch if k.startswith("house_")), None)
        if key:
            self.refs.append((key, cx, 12.0, 180.0))
        _slab(stage, f"{PARENT}/E_drive", cx + 4.0, -4.0, cx + 7.2, 6.0,
              0.03, ssf, mats["asphalt"], (0.35, 0.31, 0.28))
        self.car(cx + 5.6, 2.0, 90.0)
        self.human(cx + 3.2, -0.4, 0.02, 250.0, "idle")
        self.human(cx + 1.2, 1.6, 0.02, 200.0, "idle")
        self.human(cx - 1.6, 0.2, 0.02, 150.0, "sit_ground")
        self.burnt_trees(arch, cx, 6.0, 8, 16.0, 28.0)
        _label(stage, f"{PARENT}/E_label", cx, -14.0, ssf)

    # -- F: exposed interior ---------------------------------------------
    def unit_f(self, stage, ssf, mats, cx, arch):
        """A roof-collapsed house with somebody on the floor plate."""
        key = next((k for k in ("house_ranch_roof_collapsed",
                                "house_cottage_roof_collapsed")
                    if k in arch), None)
        if key is None:
            key = next((k for k in arch if k.endswith("_roof_collapsed")),
                       None)
        if key:
            self.refs.append((key, cx, 0.0, 0.0))
            self.notes.append("F: %s" % key)
        else:
            self.notes.append("F: no roof_collapsed archetype found")
        self.human(cx + 0.6, 1.2, 0.02, 210.0, "sit_ground")
        self.human(cx - 1.8, -0.8, 0.02, 40.0, "crouch")
        self.burnt_trees(arch, cx, 0.0, 8, 15.0, 27.0)
        _label(stage, f"{PARENT}/F_label", cx, -16.0, ssf)

    # -- the car row -----------------------------------------------------
    def car_row(self, stage, ssf, all_cars, y=-70.0):
        """Every car in the pool, each with somebody in the driver's seat.

        THE POINT IS THE COMPARISON. Whether an occupant is visible depends on
        two per-asset facts that no amount of placement logic can fix: how
        TALL the car is (a seated adult needs ~0.85 m of head room over the
        seat pan) and whether its glass is separable geometry or painted into
        a single-mesh texture. Both are properties of the art, both vary
        wildly across this pool, and the only way to know which cars can carry
        a passenger is to sit one in each and look.
        """
        x = -70.0
        for i, usd in enumerate(all_cars):
            c = self.car(x, y, 0.0, usd=usd)
            self.car_row_xy.append(
                (os.path.splitext(os.path.basename(usd))[0][:18], x, y))
            self.notes.append("car row: " + self.seat_in(c, x_off=-0.15))
            _box(stage, f"{PARENT}/CR_tick_{i}", x, y - 6.0, 0.3, 0.3, 1.4,
                 ssf, col=(0.86, 0.24, 0.20))
            # A standing figure beside each, as the scale reference that made
            # the undersized cars obvious in the first place.
            self.human(x + 2.6, y + 1.2, 0.0, 250.0, "idle")
            x += 12.0

    # -- the heading test ------------------------------------------------
    def heading_row(self, stage, ssf, cars, y=-105.0):
        """The same car and a human at four headings, to READ the offsets off.

        `yaw-offset` in the asset set is a per-asset correction for art that is
        not authored facing +X, and it is the one number in a placement that
        cannot be derived — a bounding box says which axis is LONGEST, never
        which end is the nose. So it is measured here instead: at heading 0
        every vehicle and every person in this row should point along +X, i.e.
        to the RIGHT in a top-down capture with north up. Anything pointing
        along +Y is 90 degrees out and its `yaw-offset` wants -90 from what it
        has; anything pointing at -X is 180 out.
        """
        for ci, usd in enumerate(cars):
            base_y = y - ci * 26.0
            for i, h in enumerate((0.0, 90.0, 180.0, 270.0)):
                x = -70.0 + i * 22.0
                self.car(x, base_y, h, usd=usd)
                self.human(x, base_y + 8.0, 0.0, h, "idle")
                _box(stage, f"{PARENT}/H_tick_{ci}_{i}", x - 7.0, base_y,
                     0.3, 0.3, 1.6, ssf, col=(0.86, 0.24, 0.20))
            self.hdg_rows.append(
                (os.path.splitext(os.path.basename(usd))[0][:18], -26.0,
                 base_y + 4.0))

    # -- the pose row ----------------------------------------------------
    def pose_row(self, stage, ssf):
        """Every pose in a line, on supports at the heights they assume."""
        x0 = -60.0
        for i, pose in enumerate(POSE_ROW):
            x = x0 + i * 8.0
            z = 0.0
            if pose == "sit_edge":
                # A low wall to sit on, at coping height.
                _box(stage, f"{PARENT}/P_wall", x + 0.55, POSE_ROW_Y, 2.4,
                     0.5, 0.45, ssf, col=(0.78, 0.76, 0.72))
                z = 0.45
            elif pose == "seated_car":
                # A seat pan at a sedan's h-point.
                _box(stage, f"{PARENT}/P_seat", x + 0.5, POSE_ROW_Y, 1.6,
                     1.4, 0.50, ssf, col=(0.30, 0.29, 0.30))
                z = 0.50
            self.human(x, POSE_ROW_Y, z, 180.0, pose)
            _box(stage, f"{PARENT}/P_tick_{i}", x, POSE_ROW_Y - 3.2, 0.3, 0.3,
                 1.2, ssf, col=(0.86, 0.24, 0.20))


# ---------------------------------------------------------------------------

def main():
    omni.timeline.get_timeline_interface().stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    if stage is None:
        raise RuntimeError("Failed to create a new stage")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))

    _mpu, ssf = get_stage_meters_per_unit(stage)
    config = load_scene_config(SCENE_CONFIG)
    mats = build_ground_and_light(stage, ssf)

    # The same pools and the same resolver the plat uses, so an asset
    # correction that is wrong here is wrong there.
    pools = ss.AssetPools(config)
    resolver = sg._make_resolver(config)
    raw_h = ss._raw_pool(config, "humans")
    raw_c = ss._raw_pool(config, "cars")
    humans_rigged = pools.load_tagged(raw_h, "rigged") or pools.load(raw_h)
    cars_res = pools.load_tagged(raw_c, "residential") or pools.load(raw_c)
    glassy = frozenset(pools.load_tagged(raw_c, "glass_separable"))
    # EVERY car, in a stable order, for the car row — livery and commercial
    # included. They are wrong OUTSIDE A HOUSE, which is why `build_cars` will
    # not draw them there, and they are exactly right in an evacuation queue.
    # CIVILIAN CARS ONLY in the rows. The pool also carries a taxi, a police
    # car, a construction truck and a motorhome, and they are all real and all
    # wrong here: a livery or commercial vehicle outside a house reads as an
    # incident rather than as a resident's car, and the bench exists to judge
    # the RESIDENT case. They stay in the asset set under their own tags for
    # the evacuation queue, which is the one place they belong.
    all_cars = pools.load_tagged(raw_c, "residential") or pools.load(raw_c)
    if not humans_rigged:
        raise RuntimeError("no `humans` pool in asset set %s" % SCENE_CONFIG)
    print("[bench] pools: {0} rigged human(s), {1} residential car(s), "
          "{2} glass-separable".format(len(humans_rigged), len(cars_res),
                                       len(glassy)))

    rng = random.Random(SEED)

    def pick(tag="rigged"):
        return humans_rigged[rng.randrange(len(humans_rigged))]

    def pick_car(tag="residential", glass=True):
        pool = [u for u in cars_res if (u in glassy) == bool(glass)] or cars_res
        return pool[rng.randrange(len(pool))]

    ctx = {"asset_pools": pools, "resolver": resolver, "glassy": glassy,
           "pick": pick, "pick_car": pick_car}

    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in os.listdir(ARCH_DIR) if f.endswith(".usd")} \
        if os.path.isdir(ARCH_DIR) else {}
    print("[bench] {0} archetype(s) available".format(len(arch)))

    bench = Bench(ctx, rng)
    for (uid, label, cx) in STATIONS:
        if ONLY and uid not in ONLY:
            continue
        if uid == "A":
            bench.unit_a(stage, ssf, mats, cx, arch)
        elif uid == "B":
            bench.unit_b(stage, ssf, mats, cx, arch)
        elif uid == "C":
            bench.unit_c(stage, ssf, mats, cx, arch)
        elif uid == "D":
            bench.unit_d(stage, ssf, mats, cx, arch)
        elif uid == "E":
            bench.unit_e(stage, ssf, mats, cx, arch)
        elif uid == "F":
            bench.unit_f(stage, ssf, mats, cx, arch)
        print("[bench] unit {0}: {1}  at x={2:.0f}".format(uid, label, cx))
    if not ONLY:
        bench.pose_row(stage, ssf)
        bench.car_row(stage, ssf, all_cars)
        bench.heading_row(stage, ssf, all_cars)

    # Archetype references, at their instance poses.
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    for i, (key, x, y, yaw) in enumerate(bench.refs):
        path = "{0}/inst/ref_{1}_{2}".format(PARENT, key, i)
        prim = stage.DefinePrim(Sdf.Path(path))
        prim.GetReferences().AddReference(arch[key])
        prim.Load()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
        xf.AddRotateZOp().Set(float(yaw))
        print("[bench] referenced {0} at ({1:.0f}, {2:.0f}) yaw {3:.0f}"
              .format(key, x, y, yaw))

    # CARS FIRST, THEN THE GLASS, THEN THE PEOPLE — the plat's order. A person
    # authored into a car before its windows come off is a person you cannot
    # see, and that is the exact failure this bench is here to catch.
    if bench.cars:
        sg.apply_placements(stage, bench.cars, PARENT + "/bench_cars", ssf,
                            resolver=resolver, instance_categories=set())
        n_glass = 0
        for q in bench.cars:
            if q.get("glass_separable") and q.get("prim_path"):
                n_glass += veh.strip_glass(stage, q["prim_path"])
        print("[bench] {0} car(s), {1} glass mesh(es) stripped"
              .format(len(bench.cars), n_glass))
    if bench.humans:
        sg.apply_placements(stage, bench.humans, PARENT + "/bench_people", ssf,
                            resolver=resolver, instance_categories=set())

    tally = {}
    for p in bench.humans:
        tally[p.get("pose") or "none"] = tally.get(p.get("pose") or "none",
                                                   0) + 1

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    app = omni.kit.app.get_app()
    for _ in range(30):
        app.update()

    print("\n" + "=" * 72)
    print("PEOPLE BENCH")
    print("  {0} people, {1} car(s), {2} archetype ref(s)".format(
        len(bench.humans), len(bench.cars), len(bench.refs)))
    print("  poses: " + ", ".join("%s=%d" % kv for kv in sorted(tally.items())))
    for (uid, label, cx) in STATIONS:
        if ONLY and uid not in ONLY:
            continue
        print("    {0}  x={1:>6.0f}   {2}".format(uid, cx, label))
    if not ONLY:
        print("    POSE ROW y={0:.0f}: {1}".format(POSE_ROW_Y,
                                                   ", ".join(POSE_ROW)))
    for n in bench.notes:
        print("  ! " + n)
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            _snaps.overview(stage, (20.0, 20.0), 300.0,
                            os.path.join(SNAP_DIR, "row.png"), ssf)
            pts = {uid: (cx, 0.0) for (uid, _l, cx) in STATIONS
                   if not ONLY or uid in ONLY}
            if not ONLY:
                pts["poses"] = (-36.0, POSE_ROW_Y)
                pts["cars"] = (-28.0, -70.0)
            # Framed to include the STAND around each vignette, not just the
            # figures: the burnt trees are what say "wildfire", and a shot
            # tight enough to miss them is a shot of a car park.
            _snaps.views_around(stage, pts, SNAP_DIR, ssf, top_h=62.0,
                                obl_dist=52.0, obl_h=26.0)
            # POSE CLOSE-UPS, three angles per group. The station frames above
            # are for LAYOUT; a limb angle and a ground contact need the
            # subject filling the frame, and the EYE-LEVEL shot is the one
            # that decides it — a figure whose seat floats 20 cm reads as
            # "standing" from 62 m and as broken from 1.6 m.
            for (n, gx, gy, r) in bench.close:
                _snaps.place_camera(
                    stage, (gx * ssf, gy * ssf, r * 2.0 * ssf),
                    (gx * ssf, gy * ssf, 0.0), focal_mm=24.0)
                _snaps.snapshot(os.path.join(SNAP_DIR, "%s_plan.png" % n))
                dd = r * 1.7 / math.sqrt(2.0)
                _snaps.place_camera(
                    stage, ((gx - dd) * ssf, (gy - dd) * ssf, r * 0.9 * ssf),
                    (gx * ssf, gy * ssf, 1.0 * ssf), focal_mm=24.0)
                _snaps.snapshot(os.path.join(SNAP_DIR, "%s_near.png" % n))
                ed = r * 1.9 / math.sqrt(2.0)
                _snaps.place_camera(
                    stage, ((gx - ed) * ssf, (gy - ed) * ssf, 1.6 * ssf),
                    (gx * ssf, gy * ssf, 1.0 * ssf), focal_mm=35.0)
                _snaps.snapshot(os.path.join(SNAP_DIR, "%s_eye.png" % n))
            # PER-CAR CLOSE-UPS. Whether a passenger fits is a question about
            # centimetres of head room, and it cannot be answered from a shot
            # that frames the whole row — every earlier judgement about these
            # was made from too far away and was wrong.
            for (n, hx, hy) in bench.hdg_rows:
                _snaps.place_camera(stage, (hx * ssf, hy * ssf, 46.0 * ssf),
                                    (hx * ssf, hy * ssf, 0.0))
                _snaps.snapshot(os.path.join(SNAP_DIR, "heading_%s.png" % n))
            if bench.car_row_xy:
                close = {"car_" + n: (cx_, cy_)
                         for (n, cx_, cy_) in bench.car_row_xy}
                _snaps.views_around(stage, close, SNAP_DIR, ssf, top_h=9.0,
                                    obl_dist=7.5, obl_h=2.4)
            print("[bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            print("[bench] snapshots FAILED: {0}".format(exc))

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
