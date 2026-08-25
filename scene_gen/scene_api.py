#!/usr/bin/env python
"""
ONE entry point for authoring a generated scene onto a USD stage.

    from scene_api import build_scene
    stats = build_scene(stage, "suburb_wildfire", ssf, poles=True)

`build_scene` is everything `suburb_assemble_launch_script.py` used to do
inline: layout + cheap detail, the fire field, a baked damage archetype per
house and tree, the survivor plan, the burnable/scorch pass, the ground scar
and the people. Both the standalone assembly launcher and the drone launcher
(`example_multi_drone_scene_import.py` with `SCENE_CONFIG` set) call it, so
there is one implementation of the plat and not two.

THE INTERNAL ORDER IS LOAD-BEARING and is the reason this is a function rather
than a pile of helpers a caller sequences itself:

    plan survivors -> author the evacuation QUEUE'S CARS -> burnable/scorch
    pass -> ground scar + Damaged_Asphalt re-bind -> glades -> the PEOPLE

The queue's cars are cars and belong to the fire, so they must exist before the
scorch pass chars everything the front reached; the people must come after it,
because a survivor is not scorched. Moving either across that pass is a silent
visual regression, not a crash.

SIM-AGNOSTIC AT IMPORT. Module level is stdlib + numpy + pxr, the same bar the
rest of `scene_gen` holds, so this file imports under a bare Kit python with no
`SimulationApp`. The two things that genuinely need the app —
`scene_prep.add_sky` and `omni.client` for a Nucleus archetype listing — are
imported inside the functions that use them.
"""

import json
import math
import os
import random
import time

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

import scene_generator as sg
import suburb_scene as ss
from compile_disaster import load_scene_config
from detail import modular_house as mh
from disaster import damage, fire, ground
from disaster import people as ppl
from disaster import vegetation as veg
from scene_generator import resolve_sky
from suburb_scene import generate_suburb_on_stage

_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
LOCAL_ARCH_DIR = os.path.join(_SCENE_GEN_DIR, "assets", "archetypes")

PARENT_DEFAULT = "/World/stage/generated"
POLE_SCOPE = "/_people_poles"
ROW_POLE_SCOPE = "/_rowhome_poles"
# 25 m clears every house and every tree on the plat; 0.35 m still reads at
# 400 m of altitude.
POLE_H_M = 25.0
POLE_R_M = 0.35

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# `Burnt_Forest_Floor` IS DELIBERATELY NOT USED ON TREES OR LOGS. It is a
# photographed GROUND surface; wrapped round a trunk or a log it reads as
# ground standing up — `veg.char_bole` records the same finding on a standing
# bole and rejected it there too. Log debris takes `veg.bark_material`; the
# ground scar still uses the surface, which is what it is for.


# ---------------------------------------------------------------------------
# Archetypes
# ---------------------------------------------------------------------------

def default_arch_dir():
    """Where the damage bake lives when the caller does not say.

    `scene_gen/assets/archetypes/` is UNTRACKED, so a fresh clone on a pod has
    no such directory. The bake sits under `AIRSTACK_ASSET_ROOT` on Nucleus at
    the same relative path it has in the repo, so fall back there — and only
    when the local bake is genuinely absent, so a workstation run is unchanged.
    """
    if os.path.isdir(LOCAL_ARCH_DIR):
        return LOCAL_ARCH_DIR
    root = os.environ.get("AIRSTACK_ASSET_ROOT", "").strip().rstrip("/")
    if "://" in root:
        return root + "/scene_gen/assets/archetypes"
    return LOCAL_ARCH_DIR


def _arch_from_nucleus(base):
    """Enumerate the archetype bake at a URL. Returns {name: url}.

    `os.listdir` cannot enumerate an `omniverse://` path and raises before a
    single prim is authored. Only DISCOVERY is filesystem-bound: `_ref` hands
    the value straight to `GetReferences().AddReference`, which takes a URL
    happily.

    `omni.client.list` is the direct analogue of `os.listdir`. `archetypes.json`
    (the bake manifest) is the fallback for a server that refuses a listing;
    its `usd` fields carry the BAKE MACHINE's absolute paths, so only the
    basename is usable and it is re-joined onto *base*.
    """
    import omni.client                      # app-side; see the module docstring

    base = base.rstrip("/")
    res, entries = omni.client.list(base + "/")
    if res == omni.client.Result.OK:
        names = [e.relative_path for e in entries]
        if any(n.endswith(".usd") for n in names):
            print("[scene] archetypes: omni.client.list on {0}".format(base))
            return {os.path.splitext(n)[0]: base + "/" + n
                    for n in names if n.endswith(".usd")}
    res, _ver, content = omni.client.read_file(base + "/archetypes.json")
    if res != omni.client.Result.OK:
        raise RuntimeError(
            "arch_dir is a URL but neither listable nor carrying "
            "archetypes.json: {0} ({1})".format(base, res))
    print("[scene] archetypes: archetypes.json manifest on {0}".format(base))
    raw = (content if isinstance(content, (bytes, bytearray))
           else bytes(memoryview(content)))
    manifest = json.loads(raw.decode("utf-8"))
    out = {}
    for rec in manifest:
        f = os.path.basename(str(rec.get("usd", "")))
        if f.endswith(".usd"):
            out[os.path.splitext(f)[0]] = base + "/" + f
    return out


def load_archetypes(arch_dir):
    """The bake, from wherever *arch_dir* points. Local path or URL, same dict."""
    if "://" in arch_dir:
        arch = _arch_from_nucleus(arch_dir)
    elif os.path.isdir(arch_dir):
        print("[scene] archetypes: os.listdir on {0}".format(arch_dir))
        arch = {os.path.splitext(f)[0]: os.path.join(arch_dir, f)
                for f in os.listdir(arch_dir) if f.endswith(".usd")}
    else:
        raise RuntimeError(
            "arch_dir does not exist: {0}\nThe plat is assembled BY REFERENCE "
            "to pre-baked damage archetypes; without them every house and tree "
            "is missing, and the bake is untracked so a fresh clone has none. "
            "Point ARCH_DIR at the Nucleus copy "
            "(omniverse://<host>:443/Projects/SEI-COA/scene_gen/assets/"
            "archetypes) or bake locally with "
            "bake_archetypes_launch_script.py.".format(arch_dir))
    # LOUD, because zero archetypes is not an error anywhere downstream — it
    # builds a plat with roads and no houses, which reads as a bad scene rather
    # than as a broken path.
    if not arch:
        raise RuntimeError("arch_dir holds no .usd archetypes: " + arch_dir)
    print("[scene] archetypes: {0} found ({1} house, {2} tree)".format(
        len(arch),
        sum(1 for k in arch if k.startswith("house_")),
        sum(1 for k in arch if k.startswith("tree_"))))
    return arch


# ---------------------------------------------------------------------------
# Authoring helpers
# ---------------------------------------------------------------------------

def _sanitize(name):
    return "".join(c if c.isalnum() else "_" for c in str(name))


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT. The same archetype is referenced many times across the plat;
    # without instancing, N copies cost N x the geometry and the trees alone
    # OOM'd Isaac at ~186M points. The transform ops sit on the instance ROOT,
    # which instancing still allows — only editing INSIDE the referenced
    # content is forbidden, which is exactly what a palette rebind does, so a
    # caller that means to recolour a building has to opt out.
    if instance:
        prim.SetInstanceable(True)
    return True


def build_row_poles(stage, clusters, ssf, parent=PARENT_DEFAULT):
    """One CYAN pole at each row-home court. Returns the count.

    62 row units on a 1600 x 1200 m plat are perfectly present and completely
    unfindable, and "I don't see any row homes" is what that looks like from
    the cockpit. Its own scope, so it switches off independently of the
    survivor markers.
    """
    root = parent + ROW_POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for i, c in enumerate(clusters or ()):
        pk = (c or {}).get("parking") or {}
        ctr = pk.get("centre") or c.get("centre")
        if not ctr:
            continue
        cyl = UsdGeom.Cylinder.Define(
            stage, Sdf.Path("{0}/row_{1}".format(root, i)))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        cyl.AddTranslateOp().Set(Gf.Vec3d(float(ctr[0]) * ssf,
                                          float(ctr[1]) * ssf,
                                          POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(0.0, 0.95, 1.0)])
        n += 1
    return n


def build_people_poles(stage, recs, ssf, parent=PARENT_DEFAULT):
    """One magenta pole at each survivor group's centroid. Returns the count.

    Sixty people over 1600 x 1200 m are individually invisible until you are
    almost on top of them. One pole per GROUP (a group is the thing you fly
    to), magenta because nothing in a burn scar is. All under one scope so the
    set switches off with a single prim, and carrying NO semantic label, so a
    run that forgets to disable it still produces no annotation for it.
    """
    groups = {}
    for r in recs:
        groups.setdefault((r["scenario"], int(r.get("group", 0))), []).append(r)
    root = parent + POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for (scenario, gi), members in sorted(groups.items()):
        cx = sum(float(m["x"]) for m in members) / len(members)
        cy = sum(float(m["y"]) for m in members) / len(members)
        path = "{0}/pole_{1}_{2}".format(root, _sanitize(scenario), gi)
        cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(path))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        # Cylinders are authored centred on their origin.
        cyl.AddTranslateOp().Set(
            Gf.Vec3d(cx * ssf, cy * ssf, POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 0.85)])
        # An unlit-looking marker reads at any time of day and in smoke.
        UsdGeom.Gprim(cyl).CreateDisplayOpacityAttr([1.0])
        n += 1
    return n


def _load_burnt_wood(stage, parent):
    """The material for generated log debris: BARK, not burnt ground.

    A flat dark PBR has no normal or ORM map, so a cylinder lit by one sun
    reads as painted pipe — which is why this is a photographed surface. It is
    not `Burnt_Forest_Floor`, though: that is GROUND, and ground wrapped round
    a log reads as ground. `veg.bark_material` is real oak bark at 4K with a
    normal map, tinted dark because a charred log is charred BARK.
    """
    return veg.bark_material(stage, parent + "/BurnLooks/log_bark",
                             tile_m=1.7, tint=(0.30, 0.26, 0.23))


def _tube(stage, path, p0, p1, r0, r1, ssf, sides=8):
    """One log or limb of a blockage — see `veg.log_mesh`.

    `log_mesh` caps both ends and jitters the girth. Bare barrel quads gave an
    open tube you could see down the inside of, on a mathematically exact
    cylinder — the one shape nothing in a forest has.
    """
    return veg.log_mesh(stage, path, p0, p1, r0, r1, ssf,
                        sides=max(7, int(sides) + 1),
                        rng=random.Random(abs(hash(path)) % 99991))


def _place_debris(stage, spec, ssf, i, mat, parent):
    """The strewn field that makes a blockage impassable.

    `people._blocker_debris` decides WHERE (it is a planner and touches no
    stage); this authors it.
    """
    n = 0
    for j, d in enumerate(spec.get("debris") or ()):
        prim = _tube(stage, "{0}/inst/blockdeb_{1}_{2}".format(parent, i, j),
                     d["p0"], d["p1"], d["r0"], d["r1"], ssf,
                     sides=8 if d["kind"] == "log" else 6)
        if mat is not None:
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
        n += 1
    return n


def _place_blocker(stage, spec, ssf, i, parent):
    """Put a blockage across the carriageway. See `people._add_blocker`.

    TWO KINDS AND TWO MECHANISMS. A fallen tree is a baked archetype and lands
    the same way every other tree does — one reference, instanceable. A
    toppled streetlight is a prim that is ALREADY ON THE STAGE, so it is
    re-authored rather than added: `ClearXformOpOrder` + fresh ops on the
    instance ROOT, which USD allows.
    """
    if spec["kind"] == "fallen_tree":
        return _ref(stage, "{0}/inst/blocker_{1}".format(parent, i),
                    spec["usd"], spec["x"], spec["y"], spec["yaw_deg"], ssf)
    if spec["kind"] != "streetlight" or not spec.get("prim_path"):
        return False
    prim = stage.GetPrimAtPath(spec["prim_path"])
    if not (prim and prim.IsValid()):
        return False
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return False
    sc = float(spec.get("scale", 1.0))
    xf.ClearXformOpOrder()
    sg._set_xform_ops(xf, prim,
                      translate=(spec["x"] * ssf, spec["y"] * ssf,
                                 spec["z"] * ssf),
                      rotate=(spec["roll_deg"], 0.0, spec["yaw_deg"]),
                      scale=(sc, sc, sc))
    return True


def _apply_sky(stage, config, sky_fn):
    """Sky from the config, not from any loaded stage.

    `scene_generator` deliberately only RESOLVES the path (it stays
    sim-agnostic), so the dispatch on HDRI-vs-borrowed-stage lives in
    `scene_prep.add_sky`, which is app-side. Imported here rather than at
    module level so this file still imports under a bare python.
    """
    if sky_fn is None:
        try:
            from scene_prep import add_sky as sky_fn
        except ImportError as exc:
            raise RuntimeError(
                "scene_api needs simulation/isaac-sim/utils on sys.path for "
                "scene_prep.add_sky, or an explicit sky_fn=: {0}".format(exc))
    sky_fn(stage, resolve_sky(config))


# ---------------------------------------------------------------------------
# The scene
# ---------------------------------------------------------------------------

def build_scene(stage, scene_config, scene_scale_factor, *,
                arch_dir=None, seed=11, burn_frac=0.45, elapsed=None,
                poles=False, parent_path=PARENT_DEFAULT, people_json=None,
                sky_fn=None, info_out=None):
    """Author a whole generated plat into *stage*. Returns a stats dict.

    Args:
        stage:              the composed USD stage to author into.
        scene_config:       preset name for `compile_disaster.load_scene_config`
                            (e.g. "suburb_wildfire"), or a config dict.
        scene_scale_factor: stage units per metre, from
                            `scene_prep.get_stage_meters_per_unit`. Everything
                            is authored in metres x this — the plat is NEVER
                            rescaled afterwards.
        arch_dir:           damage-archetype bake, local path or `omniverse://`
                            URL. None -> `default_arch_dir()`.
        seed:               drives layout, damage, vegetation and survivors.
        burn_frac:          share of houses inside the burn. The front is run
                            until that quantile of house arrival times, NOT
                            until the last house — `max(arrival)` burns almost
                            the whole plat and leaves nothing unburnt to fly
                            toward.
        elapsed:            seconds of fire, overriding *burn_frac* outright.
                            None / 0 -> derive from *burn_frac*.
        poles:              author the magenta survivor + cyan row-home locator
                            markers. A LOOKING aid: unlabelled, own scopes.
        parent_path:        stage path the plat is authored under.
        people_json:        survivor ground truth. Written with plain `open()`,
                            so it must be a FILESYSTEM path even when
                            *arch_dir* is a URL. None -> `<local
                            archetypes>/humans_<seed>.json`.
        sky_fn:             `fn(stage, sky_path)`; None -> `scene_prep.add_sky`.
        info_out:           optional dict, filled with the raw internals a
                            caller may want afterwards (`binfo`, `placements`,
                            `records`, `blockers`, `config`, `arch`).

    THE ORDER BELOW IS LOAD-BEARING — see the module docstring. The step
    numbering matches what `suburb_assemble_launch_script.py` used to carry
    inline so the two stay diffable against git history.
    """
    t0 = time.time()
    parent = parent_path.rstrip("/")
    ssf = scene_scale_factor
    config = (scene_config if isinstance(scene_config, dict)
              else load_scene_config(scene_config))
    cfg_name = (scene_config if isinstance(scene_config, str)
                else str(config.get("name", "<dict>")))
    arch_dir = arch_dir or default_arch_dir()
    people_json = people_json or os.path.join(
        LOCAL_ARCH_DIR, "humans_{0}.json".format(seed))

    # 1) LAYOUT + CHEAP DETAIL, houses/trees returned as instances
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=parent,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    _apply_sky(stage, config, sky_fn)
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[scene] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    arch = load_archetypes(arch_dir)

    # 2) FIRE FIELD
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((config.get("disaster") or {}).get("fire") or {})
    ox, oy = fcfg["origin_m"]
    th = math.radians(float(fcfg["heading_deg"]))
    ct, stt = math.cos(th), math.sin(th)
    head, flank, back = (float(fcfg["head_mps"]), float(fcfg["flank_mps"]),
                         float(fcfg["back_mps"]))

    def arrival(x, y):
        dx, dy = x - ox, y - oy
        return fire._ignition_time(dx * ct + dy * stt, -dx * stt + dy * ct,
                                   head, flank, back)

    arr = [arrival(h["x"], h["y"]) for h in houses]
    fin = sorted(t for t in arr if math.isfinite(t))
    elapsed = float(elapsed or 0.0) or (
        fin[int(min(0.999, max(0.0, burn_frac)) * (len(fin) - 1))]
        if fin else 300.0)
    span = max(1.0, elapsed - (min(fin) if fin else 0.0))
    phases = dict(ignition_s=0.03 * span, flame_s=0.35 * span,
                  smoulder_s=0.25 * span, ash_after_s=0.45 * span)

    def age(x, y):
        t = arrival(x, y)
        return -1.0 if not math.isfinite(t) else elapsed - t

    # 3) REFERENCE A HOUSE ARCHETYPE PER INSTANCE
    UsdGeom.Scope.Define(stage, Sdf.Path(parent + "/inst"))
    n_h = miss_h = 0
    htally = {}
    hlevels = []
    pal_jobs = []
    for i, h in enumerate(houses):
        d = age(h["x"], h["y"])
        level = "pristine" if d < 0 else damage.level_for_age(d, **phases)[0]
        # CAPTURED, because it is computed here and nowhere else. The people
        # pass needs it to tell a house that sheltered nobody from one that
        # came through — that is how the pool refuges and the exposed-interior
        # figures are chosen.
        hlevels.append(level)
        htally[level] = htally.get(level, 0) + 1
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            miss_h += 1
            continue
        # ROW HOMES ARE RECOLOURED, so they cannot be instanced. A terrace
        # whose units are all the same colour reads as one long building rather
        # than as eight houses, and the whole point of the morphology is that
        # they are eight houses.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        _hp = "{0}/inst/h_{1}".format(parent, i)
        if _ref(stage, _hp, usd, h["x"], h["y"], h["yaw"], ssf,
                instance=not _recolour):
            n_h += 1
            if _recolour:
                pal_jobs.append({"prim_path": _hp, "palette": _pal,
                                 "category": "house"})

    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, parent)
            print("[scene] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[scene] row-home palette FAILED: {0}".format(_exc))

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE (green species usd if pristine)
    n_t = miss_t = 0
    ttally = {}
    tree_prims = []
    trng = random.Random(seed + 71)
    for i, t in enumerate(trees):
        d = age(t["x"], t["y"])
        sp = t["species"]
        if d < 0:
            level = "pristine"
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01
        else:
            # SAME SCALED PHASES AS THE HOUSES. With the default (fixed) phases
            # every tree in a 1600 m burn is past `ash_after` and comes out
            # `snag` — one look for the whole plat. The scaled phases spread the
            # ladder across the real arrival range.
            level = veg.level_for_age(d, **phases)[0]
            if level != "pristine":
                level = veg.stand_outcome(level, trng)
            if level == "pristine":
                usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
                scale = 0.01
            else:
                usd = arch.get("tree_{0}_{1}".format(sp, level))
                scale = 1.0
        ttally[level] = ttally.get(level, 0) + 1
        if not usd:
            miss_t += 1
            continue
        _tpath = "{0}/inst/t_{1}".format(parent, i)
        if _ref(stage, _tpath, usd, t["x"], t["y"], t["yaw"], ssf, scale=scale):
            n_t += 1
            tree_prims.append((_tpath, float(t["x"]), float(t["y"])))

    # 4a) SURVIVORS, PLANNED. Nothing is authored here except the EVACUATION
    # QUEUE and its blockage: those are cars and they belong to the fire, so
    # they have to exist before 4b scorches everything the front reached. The
    # people themselves are held back until 4c.
    resolver = sg._make_resolver(config)
    apools = ss.AssetPools(config)
    pcfg = ppl.resolve_cfg(config)
    pctx = ppl.build_ctx(config, binfo, placements, resolver, apools,
                         age, elapsed, span, arch=arch, levels=hlevels)
    p_cars, p_blockers, p_humans, p_recs = ppl.plan_people(
        pcfg, pctx, random.Random(int(pcfg.get("seed", 91)) + seed))
    n_blocked = n_deb = 0
    _deb_mat = None
    try:
        _deb_mat = _load_burnt_wood(stage, parent)
    except Exception as _exc:
        print("[scene] blockage material unavailable: {0}".format(_exc))
    for _i, _spec in enumerate(p_blockers):
        n_blocked += 1 if _place_blocker(stage, _spec, ssf, _i, parent) else 0
        n_deb += _place_debris(stage, _spec, ssf, _i, _deb_mat, parent)
    if n_deb:
        print("[scene] blockage: {0} strewn piece(s) on the carriageway"
              .format(n_deb))

    # FIRE ON THE BLOCKAGE. A pile of charred timber across a road is ambiguous
    # from the air — it could be a woodpile, it could be a shadow. A plume over
    # it is not, and it is the one cue that says the road is closed NOW rather
    # than at some point in the past.
    n_flow = 0
    if p_blockers:
        try:
            fire.setup_flow_stack(stage, density_cell_size_m=0.16,
                                  max_blocks=16384, scene_scale_factor=ssf,
                                  root="/World/flow_blockage")
            for _i, _spec in enumerate(p_blockers):
                # NOT EVERY BLOCKAGE BURNS. A tree across a road is a blockage
                # whether or not it is alight, and a plat where every one of
                # them is on fire reads as staged.
                if not _spec.get("fire", True):
                    continue
                _bx = float(_spec.get("road_x", _spec.get("x", 0.0)))
                _by = float(_spec.get("road_y", _spec.get("y", 0.0)))
                # AHEAD OF THE BLOCKAGE, NOT ON THE QUEUE. The cars back up
                # BEHIND the blockage, so an emitter centred on it puts flame
                # among them. `out_bear` is the direction the queue is trying to
                # travel, so +bearing is past the blockage and away from the
                # cars — which is also where the timber fell from.
                _ob = math.radians(float(_spec.get("out_bear_deg", 0.0)))
                _fx, _fy = math.cos(_ob), math.sin(_ob)
                for _k, (_st, _along, _hz) in enumerate(
                        (("flame", 7.0, 1.1),
                         ("smoke", 15.0, 0.9))):
                    _dx, _dy = _fx * _along, _fy * _along
                    _pr = fire._flow_create(
                        stage, "/World/flow_blockage/em_{0}_{1}".format(_i, _k),
                        "FlowEmitterBox")
                    if not _pr or not _pr.IsValid():
                        continue
                    fire._set(_pr, "layer", Sdf.ValueTypeNames.Int,
                              fire.FLOW_LAYER)
                    fire._set(_pr, "position", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f((_bx + _dx) * ssf, (_by + _dy) * ssf,
                                       _hz * ssf))
                    fire._set(_pr, "halfSize", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f(3.2 * ssf, 2.6 * ssf, 1.3 * ssf))
                    fire.set_emission(_pr, _st, scale=1.0)
                    n_flow += 1
            print("[scene] blockage: {0} Flow emitter(s) burning".format(n_flow))
        except Exception as _exc:
            print("[scene] blockage fire FAILED: {0}".format(_exc))

    if p_cars:
        # UN-INSTANCED, like every other car on the plat: a car has to be
        # authorable to be scorched, and to have its GLASS removed — which is
        # the only way the camera ever sees the people sitting in one.
        sg.apply_placements(stage, p_cars, parent + "/people_cars", ssf,
                            resolver=resolver, instance_categories=set())
        from detail import vehicles as _veh
        _n_mesh = sum(_veh.open_cabin(stage, q["prim_path"], q.get("usd", ""))
                      for q in p_cars if q.get("prim_path"))
        print("[scene] people: {0} car(s) placed ({1} glass mesh(es) "
              "stripped), {2} blocker(s)".format(len(p_cars), _n_mesh,
                                                 n_blocked))

    # 4b) FENCES + BURNABLE FURNITURE. A timber fence is a line of dry fuel on
    # the ground and one of the first things a wildfire takes: runs VANISH deep
    # in the burn and stand SCORCHED at its edge. No fracture — there is no
    # settle in the assembly, so consumed-or-scorched is the whole vocabulary.
    def coverage_at(x, y):
        d = age(x, y)
        if d < 0.0:
            return 0.0
        return min(1.0, 0.45 + 0.55 * min(1.0, d / max(1e-6, elapsed)))

    # FENCES take one shared dark OmniPBR when they survive, NOT per-subset
    # soot: there are ~7k of them and compositing a texture each blew VRAM to
    # 17 GB. EVERYTHING ELSE the fire reached is SCORCHED IN PLACE with
    # `soot_materials` and never consumed, so nothing "disappears".
    char = damage._pbr(stage, parent + "/BurnLooks/fence_char",
                       (0.05, 0.045, 0.04), 0.9)
    # `car` IS DELIBERATELY NOT IN THIS LIST. Car materials are glossy, tightly
    # mapped paint over a small UV layout, and a soot wash built for a brick
    # wall comes out as grey blotches. A car in a burn scar is also not usually
    # charred — the ones that burn are consumed outright and the rest are
    # ordinary cars standing in a black landscape, which is exactly what an
    # abandoned evacuation queue looks like.
    BURNABLE = ("fence", "bench", "chair", "picnic_table", "table",
                "trash_can", "play_structure", "planter", "bin", "cafe_set",
                "swing", "seesaw", "bike_rack", "mailbox", "bus_stop",
                "park_feature", "goal", "basket", "hoop")
    brng = random.Random(seed + 5)
    n_gone = n_char = 0
    scorch_props = []
    for q in placements:
        path = q.get("prim_path")
        cat = str(q.get("category", ""))
        if not path or not any(k in cat for k in BURNABLE):
            continue                                   # only combustibles
        if damage.is_incombustible(cat):
            continue
        d = age(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)))
        if d <= 0.0:
            continue                                   # front never reached it
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if "fence" in cat:                             # line fuel
            if d / max(1e-6, span) > 0.55 and brng.random() < 0.90:
                if prim.SetActive(False):
                    n_gone += 1
            else:
                for m in Usd.PrimRange(prim):
                    if m.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(m).Bind(char)
                n_char += 1
        else:                                          # cars, park props, ...
            scorch_props.append(q)
    n_soot = damage.soot_materials(stage, scorch_props, parent,
                                   random.Random(seed), coverage_at=coverage_at)
    print("[scene] burnable: {0} fence(s) consumed, {1} fence(s) charred, "
          "{2} prop subset(s) scorched (cars/park/furniture)"
          .format(n_gone, n_char, n_soot))

    # PARK GROUND SURFACES scorched. The court slabs, pitch, line markings and
    # paths are drawn geometry (not placements), so they miss the soot pass
    # above; re-bind the ones the front reached to the dark char material.
    gnd_prim = stage.GetPrimAtPath(parent + "/ground")
    _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_park = 0
    if gnd_prim and gnd_prim.IsValid():
        for prim in gnd_prim.GetChildren():
            _nm = prim.GetName()
            if not _nm.startswith("park_"):
                continue
            # ASPHALT DOES NOT CHAR, and charring the refuge lot black would
            # delete the feature the survivors are standing on: a parking lot
            # reads as a refuge BECAUSE it is bare pavement.
            if _nm.startswith("park_parking"):
                continue
            r = _bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) < 0.0:
                continue
            for m in Usd.PrimRange(prim):
                if m.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI(m).Bind(char)
            n_park += 1
    print("[scene] park ground: {0} surface(s) scorched".format(n_park))

    # 5) GROUND SCAR (built on the fly, cheap)
    region = tuple(binfo.get("region") or (-800, -600, 800, 600))
    zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
    burn_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
    kn = ground.knobs_from_env(max(region[2] - region[0], region[3] - region[1]))
    cov = ground.feathered_coverage(
        arrival, elapsed, (ox, oy), region, np.random.default_rng(seed + 23),
        edge_m=kn["edge_m"], finger_m=kn["finger_m"], islands=kn["islands"])
    made = ground.build_overlay(
        stage, cov, region, ssf, burn_z, material_parent=parent,
        cell_m=kn["cell_m"], bands=kn["bands"], tile_m=kn["tile_m"],
        op_range=kn["op_range"],
        skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))

    # FIRE-DAMAGED PAVING -> Damaged_Asphalt. `apply_ground` builds the road
    # and drive ribbons before any fire field exists, so re-bind here. Brick
    # drives are left alone — worn brick already reads as damaged.
    dmg_url = sg._join_asset_root(
        "airstack://scene_gen/assets/materials/megascans/Damaged_Asphalt.usda", "")
    dmg_path = parent + "/ground/materials/damaged_asphalt"
    dprim = stage.DefinePrim(Sdf.Path(dmg_path))
    dprim.GetReferences().AddReference(dmg_url)
    dprim.Load()
    dmat = UsdShade.Material(stage.GetPrimAtPath(dmg_path))
    gnd = stage.GetPrimAtPath(parent + "/ground")
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_dmg = 0
    if gnd and gnd.IsValid() and dmat:
        for prim in gnd.GetChildren():
            nm = prim.GetName()
            if not nm.startswith(("road_", "bulb_", "drive_")):
                continue
            bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bound and "Brick" in bound.GetPrim().GetPath().pathString:
                continue                       # worn-brick drive stays brick
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) >= 0.0:   # front reached it
                UsdShade.MaterialBindingAPI(prim).Bind(dmat)
                n_dmg += 1
    print("[scene] {0} road/drive ribbon(s) re-bound to Damaged_Asphalt "
          "in the burn".format(n_dmg))

    # 4b-2) CLEAR A GLADE ROUND EACH OPEN-GROUND GROUP. `_open_ground` already
    # requires 15 m from any structure or tree TRUNK, but a trunk keep-out is
    # not a clearing: these crowns are 10-25 m across, so a group can satisfy
    # the rule and still sit under a closed canopy where a drone sees leaves.
    # The scenario IS "people on open ground", so the ground is opened.
    n_glade = 0
    _glade_r = float((pcfg.get("scenarios", {}).get("open_ground") or {})
                     .get("glade_r_m", 16.0))
    if _glade_r > 0.0 and tree_prims:
        _seeds = [(r["x"], r["y"]) for r in p_recs
                  if r.get("scenario") == "open_ground"]
        if _seeds:
            _r2 = _glade_r * _glade_r
            for _tp, _tx, _ty in tree_prims:
                for (_sx, _sy) in _seeds:
                    if (_tx - _sx) ** 2 + (_ty - _sy) ** 2 <= _r2:
                        _pr = stage.GetPrimAtPath(_tp)
                        if _pr and _pr.IsValid() and _pr.SetActive(False):
                            n_glade += 1
                        break
            print("[scene] clearings: {0} tree(s) removed within {1:.0f} m "
                  "of {2} open-ground group(s)".format(
                      n_glade, _glade_r, len(_seeds)))

    # 4c) THE PEOPLE, AFTER THE SCORCH PASS. `"human"` deliberately matches
    # nothing in BURNABLE above, and holding them back until here is the
    # belt-and-braces version of that: a survivor is not scorched. Authored
    # through `apply_placements` because that is what binds each `pose` onto
    # the character's own UsdSkel rig, and NOT instanced, because the pose
    # animation is authored inside each prim.
    n_people = n_poles = n_rowp = 0
    if p_humans:
        sg.apply_placements(stage, p_humans, parent + "/people", ssf,
                            resolver=resolver, instance_categories=set())
        n_people = len(p_humans)
        ppl.write_records(people_json, p_recs, meta={
            "seed": seed, "scene_config": cfg_name,
            "elapsed_s": round(elapsed, 1), "span_s": round(span, 1),
            "burn_frac": burn_frac,
            "fire_origin_m": [ox, oy],
            "fire_heading_deg": float(fcfg["heading_deg"]),
            "blockers": p_blockers,
        })
        print("[scene] people: {0} authored, ground truth -> {1}"
              .format(n_people, people_json))
        if poles:
            n_poles = build_people_poles(stage, p_recs, ssf, parent)
            n_rowp = build_row_poles(stage, binfo.get("clusters"), ssf, parent)
            print("[scene] poles: {0} survivor (magenta, {1}{2}) + {3} "
                  "row-home (cyan, {1}{4}) — deactivate the scope to hide"
                  .format(n_poles, parent, POLE_SCOPE, n_rowp, ROW_POLE_SCOPE))

    if info_out is not None:
        info_out.update({"binfo": binfo, "placements": placements,
                         "records": p_recs, "blockers": p_blockers,
                         "cars": p_cars, "config": config, "arch": arch,
                         "parent": parent})

    _ptally = {}
    for _r in p_recs:
        _ptally[_r["scenario"]] = _ptally.get(_r["scenario"], 0) + 1
    return {
        "scene_config": cfg_name, "seed": seed, "seconds": time.time() - t0,
        "region": region, "elapsed_s": elapsed, "span_s": span,
        "burn_frac": burn_frac, "arch_dir": arch_dir,
        "houses": n_h, "houses_missing": miss_h, "house_tally": htally,
        "trees": n_t, "trees_missing": miss_t, "tree_tally": ttally,
        "bands": len(made), "flow": n_flow, "glades": n_glade,
        "fences_consumed": n_gone, "fences_charred": n_char,
        "props_scorched": n_soot, "park_surfaces": n_park,
        "roads_damaged": n_dmg,
        "people": n_people, "people_alive": sum(1 for _r in p_recs
                                                if _r.get("alive")),
        "people_tally": _ptally, "people_json": people_json,
        "cars": len(p_cars), "blockers": len(p_blockers),
        "poles": n_poles + n_rowp,
    }
