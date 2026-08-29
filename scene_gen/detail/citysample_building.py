"""citysample_building — compose a City Sample tower out of its façade kit.

WHY THIS EXISTS: THE PACK SHIPS THE PARTS, NOT THE BUILDINGS
-------------------------------------------------------------
`CitySample/All_Buildings_Lineup_Hero.usd` looks like 20 assembled buildings
and is not one. Each `BPP_Bldg_Hero_*` prim holds its building's façade
modules correctly GROUPED, but every child is an `InstancedStaticMesh` prim
carrying exactly ONE copy of its source module — measured by comparing point
counts against the source files, the ratio is 1.00 — and all of them share a
single component transform. Unreal's per-instance transform arrays did not
survive the USD export, so every wall, column and corner of a twenty-storey
tower sits at the same origin. It shows in the bounds: every hero building
measures 4-17 m tall, and `BPP_Bldg_Hero_Tower_CHJ_B01_N1` comes back
78.7 x 57.2 x 4.0 m. Opening that file gives 20 heaps of overlapping panels.

The kit underneath is sound, so the placement is authored here instead. This
is the same job `detail/urban_building` does for the ModernCityEnvironment01
modules, against a different naming convention.

THE CONVENTION, READ OUT OF `_plans/citysample_kit.json`
---------------------------------------------------------
Every module's local origin sits ON THE FAÇADE LINE at the level's floor:

    +Y      the bay, running 0 .. bay_width (3.25 m on CHC, 4.5 on SFB,
            9.25 on NYG — it is per family, and per module WITHIN a family,
            which is what lets a wall run be tiled to an arbitrary length)
    -X      into the building; the module's thickness
    +Z      0 at the floor, z1 at the ceiling — so a level's height is read
            off its own modules, not assumed

`CornerEx` is the corner quadrant, spanning -2..0 in both X and Y about the
corner point. `Entrance` is a ground-floor wall of its own width. So a level
is a ring: corner, then walls tiled along the side, at every vertex.

WALKING THE RING COUNTER-CLOCKWISE, the interior is on the left of travel,
which is exactly where -X has to point. With travel direction phi that is

    yaw = phi - 90

and the check is that local -X, rotated by yaw, lands on (-sin phi, cos phi)
— the left of travel. Every side of the rectangle then costs the same code.

WEIGHT, AND THE TWO THINGS THAT MAKE IT AFFORDABLE
---------------------------------------------------
These are Nanite source meshes: the 20 hero buildings are 83.2M triangles,
median 4.74M each, against 8.0M for all 31 GreatAmericanCity buildings put
together. A ring of 40 bays over 10 levels is ~400 module placements, and
placed naively that is tens of millions of points per building.

  1. INSTANCEABLE. The 400 placements reference perhaps a dozen distinct
     modules, so each placement is marked `instanceable` and the renderer
     keeps ONE copy of each. This is the whole reason the kit is usable at
     city scale at all.
  2. A POINT BUDGET. Within one level and one kind the pack ships several
     designs at wildly different densities — CHC L01's 3.25 m wall comes as
     both a 16k-point module and a 180k-point one, and its entrance is 731k.
     `max_points` takes the cheapest module that clears the budget, and falls
     back to the cheapest available when nothing does.

NO ROOFS IN THE KIT. `Kit_Hero_Bldg/MeshRoof` holds roofs cut for specific
hero buildings, not a generic one, so `build` caps the stack with a slab. For
a drone dataset that is not cosmetic: nadir is the view we are actually
flying, and an open-topped tower reads straight down into its own interior.
"""

import json
import math
import os

_SG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KIT_JSON = os.path.join(_SG_DIR, "_plans", "citysample_kit.json")
# the decimated, locally-authored kit — metres, plain meshes, plain
# UsdPreviewSurface. `CS_KIT=local` selects it.
_KIT_LOCAL = os.path.join(_SG_DIR, "_plans", "citysample_kit_local.json")

# levels numbered past this are the pack's specials (SFB L301, CHD L201) and
# are not part of the ground-up stack
_SPECIAL_LEVEL = 100
# The catalogue stores module paths RELATIVE to the project root, so a
# reference has to be joined against it. `_join_asset_root(path, "")`
# returns the path untouched, and a relative reference resolves against
# the anonymous authoring layer — every module then silently fails to
# open and the building bounds come back as nothing but its roof slab.
ASSET_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
              "SEI-COA/")
MAX_POINTS = 60000
ROOF_T = 0.45
# HOW FAR A MODULE MAY STAND PROUD OF THE FAÇADE LINE. Most are within a
# centimetre or two of it, but the pack also ships hero pieces that project
# metres — `SM_BLDG_SFB_L11_A_Wall_01_N1` reaches 16.0 m outboard, a wing
# rather than a bay, and tiling it round a ring grew the building's footprint
# by 32 m on both axes. Entrances get more room because a canopy is real.
# how far past `max_points` the last-resort module may go
FALLBACK_X = 2.0
OUTBOARD_M = 1.5
OUTBOARD_ENTRANCE_M = 3.0


def load_kit(path=None):
    """The module catalogue. `CS_KIT=local` picks the decimated local one."""
    if path is None:
        want = (os.environ.get("CS_KIT") or "").strip().lower()
        path = _KIT_LOCAL if want in ("local", "1", "decimated") else _KIT_JSON
    return json.load(open(os.path.normpath(path)))


def families(kit):
    """[(family, variant)] with enough levels to make a building."""
    out = []
    for fam in sorted(kit):
        for var in sorted(kit[fam]):
            lv = [int(k) for k in kit[fam][var] if int(k) < _SPECIAL_LEVEL]
            if len(lv) >= 2:
                out.append((fam, var))
    return out


def _pick(mods, kind, max_points, want_w=None, outboard=OUTBOARD_M):
    """Cheapest module of `kind` inside the budget, nearest `want_w` wide."""
    c = [m for m in mods if m["kind"] == kind and m["x1"] <= outboard]
    if not c:
        return None
    ok = [m for m in c if m["points"] <= max_points] or c
    if want_w is not None:
        ok = sorted(ok, key=lambda m: (abs((m["y1"] - m["y0"]) - want_w),
                                       m["points"]))
    else:
        ok = sorted(ok, key=lambda m: (-(m["y1"] - m["y0"]), m["points"]))
    return ok[0]


def _walls(mods, max_points):
    """Wall modules by DESCENDING bay width, cheapest at each width."""
    best = {}
    for m in mods:
        if m["kind"] not in ("Wall", "Column"):
            continue
        if m["x1"] > OUTBOARD_M:
            continue
        w = round(m["y1"] - m["y0"], 2)
        if w < 0.4:
            continue
        if w not in best or m["points"] < best[w]["points"]:
            best[w] = m
    # A WIDTH WHOSE CHEAPEST MODULE BUSTS THE BUDGET IS DROPPED, not kept at
    # full weight. The first cut fell back per width and so still admitted
    # CHC_A L6's 9.50 m bay at 682k points — four of those per side, sixteen
    # per level, 11M points for one storey — while the same level offers a
    # 3.25 m bay at 44k that tiles the run just as well. Only when NO width
    # is affordable does the cheapest module get used regardless.
    ok = {w: m for w, m in best.items() if m["points"] <= max_points}
    if ok:
        return [ok[w] for w in sorted(ok, reverse=True)]
    # THE FALLBACK NEEDS A CEILING OF ITS OWN. Taking the cheapest module
    # regardless of budget let `CHG_B` tile a 232k-point wall 491 times and
    # draw 114M points for ONE building — the budget was doing nothing on
    # exactly the levels it was meant to protect. Past this multiple the level
    # is simply not buildable, and `plan_building` stacks around it.
    if best:
        cheap = min(best.values(), key=lambda m: m["points"])
        if cheap["points"] <= max_points * FALLBACK_X:
            return [cheap]
    return []


def plan_building(kit, fam, var, target_h, max_points=MAX_POINTS):
    """An ordered stack of levels reaching about `target_h` metres.

    L1 is the ground floor and is used once; the levels above it are used in
    order, and the tallest mid-level repeats to make up the height. Repeating
    a MID level rather than the ground one is the point — a building with two
    entrance storeys reads as a mistake from the street.
    """
    lv = kit[fam][var]
    # ONLY LEVELS `build` CAN ACTUALLY LAY. A level whose walls are all over
    # the point budget or all standing proud of the façade line contributes
    # no ring, and `build` skips it — but the first cut still counted its
    # height when planning the stack, so `CHG_B` planned 18 levels to 73.3 m
    # and came out a 7.5 m stub. Planning and building have to agree on what
    # a usable level is.
    nums = sorted(int(k) for k in lv
                  if int(k) < _SPECIAL_LEVEL and _walls(lv[k], max_points))
    if not nums:
        return None

    def _h(n):
        ms = lv[str(n)]
        z = max((m["z1"] for m in ms), default=0.0)
        return z if z > 0.4 else 3.2

    ground = nums[0]
    mids = nums[1:] or [ground]
    stack = [ground]
    h = _h(ground)
    fill = max(mids, key=_h)
    i = 0
    while h < target_h and len(stack) < 60:
        n = mids[i] if i < len(mids) else fill
        stack.append(n)
        h += _h(n)
        i += 1
    return {"family": fam, "variant": var, "levels": stack,
            "H": h, "max_points": max_points}


def footprint(kit, fam, var):
    """A plausible (W, D) for this family, from its widest ground-floor bay.

    A building whose side is not a whole number of bays leaves a gap at the
    end of every run, so the packer is given a footprint the ring can
    actually close on.
    """
    lv = kit[fam][var]
    nums = sorted(int(k) for k in lv if int(k) < _SPECIAL_LEVEL)
    ws = _walls(lv[str(nums[0])], 10 ** 9)
    bay = (ws[0]["y1"] - ws[0]["y0"]) if ws else 3.25
    return (round(bay * 8, 1), round(bay * 6, 1))


def _run(side_len, walls, corner_w):
    """Bay widths tiling one side, greedily widest-first. Returns [w]."""
    room = side_len - 2.0 * corner_w
    out = []
    for m in walls:
        w = m["y1"] - m["y0"]
        while room >= w - 1e-6 and len(out) < 200:
            out.append(m)
            room -= w
    return out, room


def placements(kit, spec, x, y, W, D, yaw=0.0, category=None,
               asset_root=ASSET_ROOT):
    """The building as a list of PLACEMENT DICTS, ready for
    `scene_generator.apply_placements`. Returns (placements, height_m).

    THE SAME CONTRACT `detail/urban_building.build_building` USES, and for the
    same reason. The first version of this module authored its own USD —
    `AddReference`, `SetInstanceable`, `stage.Load` — which is a second asset
    spawning path in a project that already has one, and it failed exactly
    where a hand-rolled path does. `apply_placements` carries hard-won
    handling this kit needs and I rediscovered the hard way:

      * a TYPELESS def takes the reference, so an asset whose root prim is a
        Mesh keeps its own type instead of being overridden into an Xform that
        composes attributes but draws nothing;
      * `prim.Load()` PER PRIM, because a prim composed into an already-running
        stage does not auto-load nested payloads the way `Usd.Stage.Open`
        does — the reference lands with a correct bbox and transform and no
        visible geometry. That is precisely the failure this module shipped
        with: every family measured its full 70-82 m and the viewport drew
        nothing but the roof slabs, and the offline check missed it because
        the check called `Load` itself.

    `raw_pivot` is set for the same reason `modular_house` sets it: these
    modules are positioned in the KIT'S own frame, each bay already sitting
    where it belongs relative to its neighbours, so re-centring each one on
    its own bbox would pull the ring apart.

    `scale` is the module's OWN `metersPerUnit`. Every CitySample module is
    authored in centimetres and USD converts nothing on reference — measured,
    not assumed: a 1.00 m bay referenced unscaled bounds 100.00.
    """
    # IMPORTED HERE, NOT AT MODULE SCOPE. `scene_generator` pulls in `pxr`,
    # which does not exist on the host — and the planning half of this module
    # (families / plan_building / footprint / _walls) is checked host-side
    # without a container. A module-level import makes all of that unusable.
    import scene_generator as sg

    lv = kit[spec["family"]][spec["variant"]]
    mp = spec["max_points"]
    cat = category or "cs_{0}_{1}".format(spec["family"], spec["variant"])
    out, z, n_pts = [], 0.0, 0
    ca, sa = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))

    def _world(lx, ly):
        return (x + lx * ca - ly * sa, y + lx * sa + ly * ca)

    def _add(mod, px, py, ang, pz):
        # A LOCAL MODULE IS A PATH ON DISK, NOT A NUCLEUS KEY. The decimated
        # catalogue stores paths relative to `scene_gen/`; joining those onto
        # the project root would silently produce an unreachable URL.
        usd = (os.path.normpath(os.path.join(_SG_DIR, mod["usd"]))
               if mod.get("local") else
               sg._join_asset_root(mod["usd"], asset_root))
        out.append({"usd": usd,
                    "x_m": float(px), "y_m": float(py), "z_m": float(pz),
                    "yaw_deg": float(ang) % 360.0, "roll_deg": 0.0,
                    "pitch_deg": 0.0, "scale": float(mod.get("mpu", 1.0)),
                    "category": cat, "axis_up": "Z", "raw_pivot": True})

    for n in spec["levels"]:
        mods = lv[str(n)]
        walls = _walls(mods, mp)
        if not walls:
            continue
        corner = _pick(mods, "CornerEx", mp)
        cw = (corner["y1"] - corner["y0"]) if corner else 0.0
        ent = (_pick(mods, "Entrance", mp, outboard=OUTBOARD_ENTRANCE_M)
               if n == spec["levels"][0] else None)
        lh = max((m["z1"] for m in mods), default=3.2)
        # W and D are the footprint a packer reserved, and the façade skin
        # stands up to `OUTBOARD_M` proud of the line it is laid on, so the
        # ring is inset by that much
        hw = max(2.0, W / 2.0 - OUTBOARD_M)
        hd = max(2.0, D / 2.0 - OUTBOARD_M)
        # the four sides, walked COUNTER-CLOCKWISE so the interior stays on
        # the left of travel and -X points into it
        verts = [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]
        for i in range(4):
            ax, ay = verts[i]
            bx, by = verts[(i + 1) % 4]
            phi = math.degrees(math.atan2(by - ay, bx - ax))
            ang = yaw + phi - 90.0
            L = math.hypot(bx - ax, by - ay)
            ux, uy = (bx - ax) / L, (by - ay) / L
            if corner is not None:
                # offset by its own overhang: a corner's body runs BACKWARDS
                # along the outgoing edge, and backwards from a vertex is
                # outside the rectangle
                co = -corner["y0"]
                wx, wy = _world(ax + ux * co, ay + uy * co)
                _add(corner, wx, wy, ang, z)
                n_pts += corner["points"]
            run, _rest = _run(L, walls, cw)
            if ent is not None and i == 0 and run:
                run = [ent] + run[1:]
            t = cw
            for m in run:
                bw = m["y1"] - m["y0"]
                if t + bw > L - cw + 1e-6:
                    break
                # THE BAY IS NOT ALWAYS y IN [0, WIDTH] — the pack authors
                # modules in both directions, so offsetting by -y0 makes every
                # one occupy [t, t + bw] whichever way it was authored
                off = t - m["y0"]
                wx, wy = _world(ax + ux * off, ay + uy * off)
                _add(m, wx, wy, ang, z)
                n_pts += m["points"]
                t += bw
        z += lh
    return out, z, n_pts


def build(stage, root, kit, spec, x, y, W, D, yaw=0.0, tag="cs",
          material=None, asset_root=ASSET_ROOT, ssf=1.0, instance=True):
    """Author the building through `apply_placements`. Returns a summary.

    INSTANCED, AND IT HAS TO BE. `apply_placements` does NOT instance by
    default — its own docstring records why, and records the precedent: 89.1M
    points OOM-killed Isaac Sim on the urban scene. A composed tower is 200 to
    800 placements of 2 to 48 DISTINCT Nanite modules, so uninstanced it pays
    full geometry for every bay: the 18-family lineup drew ~125M points over
    6467 placements and the process was Killed part-way through `CHG_A`.
    Instanced, the same 18 buildings hold 5.7M points of unique geometry.

    Instancing is opt-in there because `generate_scene.prune_prims`
    deactivates sub-prims inside placed assets and USD forbids editing inside
    an instance. Nothing prunes these façade modules — they are referenced
    whole and never reached into — so the category is safe to instance.
    """
    import scene_generator as sg
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    pls, z, n_pts = placements(kit, spec, x, y, W, D, yaw=yaw,
                               category="cs_" + tag, asset_root=asset_root)
    cats = {p["category"] for p in pls} if instance else None
    if pls:
        sg.apply_placements(stage, pls, root, ssf,
                            instance_categories=cats)

    # THE CAP, authored inline because the kit has no generic roof —
    # `Kit_Hero_Bldg/MeshRoof` holds roofs cut for specific hero buildings.
    # For a drone dataset that is not cosmetic: nadir is the view being flown,
    # and an uncapped stack reads straight down its own shaft.
    top = UsdGeom.Cube.Define(stage, Sdf.Path(root + "/roof"))
    top.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(top.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x * ssf), float(y * ssf),
                                     float((z - ROOF_T / 2.0) * ssf)))
    xf.AddRotateZOp().Set(float(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(float(W * ssf), float(D * ssf),
                                 float(ROOF_T * ssf)))
    top.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    if material is not None:
        UsdShade.MaterialBindingAPI(top.GetPrim()).Bind(material)
    return {"W": W, "D": D, "H": z, "prims": len(pls) + 1, "points": n_pts,
            "levels": len(spec["levels"])}
