#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow"]
# ///
"""rubble_preview.py — an OFFLINE look check for one rubble pile.

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/rubble_preview.py \
        --type rc --W 30 --D 30 --H 55 --kind dome --seed 3 \
        --out ~/scorch_previews/rubble_r4/

Builds a plan (via `disaster.quake_rubble.plan_pile` if that module exists
yet — agent B's file, built in parallel against the same round-4 contract —
else a SYNTHETIC fallback heightfield built right here, clearly logged as
such), authors it with `disaster.quake_rubble_usd.author()` against the
LOCAL asset mirror (`scene_gen/assets/`, `flatten_instances=True` since
Blender's `bpy.ops.wm.usd_import` does not bring in `UsdGeom.PointInstancer`
instances — see the module docstring below), drops in a 120x120 m grey
ground plane and a short standing wall stub for scale, saves the USD next to
the PNGs, and renders three views through headless Blender/Cycles: two
oblique corners and a near-nadir look-down. `render_usd.py`'s import/render
idioms are reused (its `setup_engine`/`scene_bounds`/`place_camera` shapes),
copied rather than imported since that file is owned by another agent this
round and is not to be touched.

FALLBACK PLAN CAVEAT: `disaster/quake_rubble.py` did not exist yet when this
tool was written. Its stand-in shapes a dome by radial falloff and low-
frequency noise, and gives `windrow`/`fan` a directionally different profile
purely as a placeholder for readability — it is NOT the round-4 morphology
model (repose angle, run-out, crown/footprint ratios per §7 of the research
doc are not modelled here). Once `quake_rubble.py` lands this fallback stops
being used automatically (this script imports it first and only falls back
on `ImportError`).
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from pathlib import Path

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import quake_rubble_usd as qru        # noqa: E402

LOCAL_ASSET_ROOT = os.path.join(_SCENE_GEN, "assets")

REPOSE_DEG = 35.0
CROWN_FRAC = {"urm": 0.28, "rc": 0.28, "rc_glass": 0.22}

# compass side -> unit vector, matching quake_flow's S/E/N/W convention
# (also used by `_shape_windrow`/`_shape_fan` below for the fallback plan).
_FALL_DIR = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}

# A small hand-picked subset of the mirrored debris catalogue (measured by
# this agent's `tools/nucleus_fetch.py` pull), used ONLY by the synthetic
# fallback plan below — `quake_rubble.CATALOGUE`, once it exists, replaces
# this entirely (quake_rubble_usd.author() already prefers plan["catalogue"]
# / the real CATALOGUE over anything this script provides).
_FALLBACK_CATALOGUE = {
    "chunk_01": {"url": "standalone/debris/pieces/chunk_01/chunk_01.usdc", "size": (0.971, 0.985, 0.786)},
    "chunk_02": {"url": "standalone/debris/pieces/chunk_02/chunk_02.usdc", "size": (0.881, 1.004, 0.813)},
    "chunk_03": {"url": "standalone/debris/pieces/chunk_03/chunk_03.usdc", "size": (0.971, 0.984, 0.786)},
    "chunk_05": {"url": "standalone/debris/pieces/chunk_05/chunk_05.usdc", "size": (0.925, 0.735, 0.892)},
    "chunk_07": {"url": "standalone/debris/pieces/chunk_07/chunk_07.usdc", "size": (0.656, 0.965, 0.83)},
    "lump_01": {"url": "standalone/debris/pieces/lump_01/lump_01.usdc", "size": (0.361, 0.37, 0.062)},
    "lump_02": {"url": "standalone/debris/pieces/lump_02/lump_02.usdc", "size": (0.223, 0.24, 0.055)},
    "lump_04": {"url": "standalone/debris/pieces/lump_04/lump_04.usdc", "size": (0.199, 0.196, 0.066)},
    "lump_06": {"url": "standalone/debris/pieces/lump_06/lump_06.usdc", "size": (0.156, 0.146, 0.045)},
    "slab_01": {"url": "standalone/debris/pieces/slab_01/slab_01.usdc", "size": (4.189, 3.03, 0.373)},
    "slab_05": {"url": "standalone/debris/pieces/slab_05/slab_05.usdc", "size": (3.636, 3.835, 0.521)},
    "rebar_02": {"url": "standalone/debris/pieces/rebar_02/rebar_02.usdc", "size": (4.412, 1.449, 0.212)},
    "sheet_02": {"url": "standalone/debris/pieces/sheet_02/sheet_02.usdc", "size": (4.522, 2.851, 0.559)},
    "concrete_slabs": {"url": "concrete_rubble_debris/split/concrete_slabs/concrete_slabs.usdc", "size": (3.639, 2.459, 0.669)},
    "concrete_debris_elements": {"url": "concrete_rubble_debris/split/concrete_debris_elements/concrete_debris_elements.usdc", "size": (3.53, 2.579, 0.379)},
    "huge_concrete_rubble_pile": {"url": "concrete_rubble_debris/split/huge_concrete_rubble_pile/huge_concrete_rubble_pile.usdc", "size": (8.046, 7.68, 1.476)},
    "brick_debris_pile": {"url": "concrete_rubble_debris/split/brick_debris_pile/brick_debris_pile.usdc", "size": (6.071, 4.897, 1.196)},
}

_CHUNK_NAMES = ["chunk_01", "chunk_02", "chunk_03", "chunk_05", "chunk_07"]
_LUMP_NAMES = ["lump_01", "lump_02", "lump_04", "lump_06"]


# --------------------------------------------------------------------------- #
# fallback plan (used only while disaster.quake_rubble does not exist)
# --------------------------------------------------------------------------- #
def _grid(W, D, pad, n):
    hx, hy = W / 2.0, D / 2.0
    xs = np.linspace(-hx * pad, hx * pad, n)
    ys = np.linspace(-hy * pad, hy * pad, n)
    X, Y = np.meshgrid(xs, ys, indexing="xy")
    return X, Y, hx, hy


def _heightfield_from_grid(X, Y, Z, look):
    n = X.shape[0]
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).astype(np.float64)
    idx = np.arange(n * n).reshape(n, n)
    faces = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b = idx[j, i], idx[j, i + 1]
            c, d = idx[j + 1, i + 1], idx[j + 1, i]
            faces.append((a, b, c))
            faces.append((a, c, d))
    faces = np.asarray(faces, dtype=np.int64)
    return {
        "points": pts, "faces": faces, "look": look,
        "crown_z": float(Z.max()), "volume_m3": float(Z.sum() * 0.1),
        "reach_m": float(max(X.max(), Y.max())),
        "footprint": {"cx": 0.0, "cy": 0.0},
    }


def _shape_dome(X, Y, hx, hy, crown, pad):
    r = np.sqrt((X / (hx * pad)) ** 2 + (Y / (hy * pad)) ** 2)
    return crown * np.clip(1.0 - r, 0.0, 1.0) ** 1.2


def _shape_windrow(X, Y, hx, hy, crown, sides, depth_m, offset_m):
    along_y = sides in ("S", "N")
    if sides == "S":
        wall, sign = -hy, 1.0
    elif sides == "N":
        wall, sign = hy, -1.0
    elif sides == "E":
        wall, sign = hx, -1.0
    else:
        wall, sign = -hx, 1.0
    d = (Y - wall) * sign if along_y else (X - wall) * sign
    ridge_c = offset_m + depth_m
    half_w = max(depth_m * 1.6, 1.0)
    prof = np.clip(1.0 - np.abs(d - ridge_c) / half_w, 0.0, 1.0)
    # keep the ridge only alongside the chosen wall, not wrapped all the way
    # around the footprint
    along_coord = X if along_y else Y
    along_half = hx if along_y else hy
    edge_fade = np.clip(1.0 - (np.abs(along_coord) / (along_half * 1.05)) ** 6, 0.0, 1.0)
    return crown * 0.6 * prof * edge_fade


def _shape_fan(X, Y, hx, hy, crown, sides):
    # wider at the toe, out from the wall named by `sides`
    if sides in ("E", "W"):
        t = (X + hx) / (2 * hx) if sides == "E" else (hx - X) / (2 * hx)
        spread = 0.30 + 1.0 * np.clip(t, 0.0, 1.0)
        r = np.sqrt((X / (hx * spread + 1e-6)) ** 2 + (Y / (hy * 0.9 + 1e-6)) ** 2)
    else:
        t = (Y + hy) / (2 * hy) if sides == "N" else (hy - Y) / (2 * hy)
        spread = 0.30 + 1.0 * np.clip(t, 0.0, 1.0)
        r = np.sqrt((Y / (hy * spread + 1e-6)) ** 2 + (X / (hx * 0.9 + 1e-6)) ** 2)
    return crown * np.clip(1.0 - r, 0.0, 1.0) ** 1.3


def _fallback_plan(W, D, H, btype, kind, seed, sides="S", depth_m=1.2, offset_m=1.5):
    """A synthetic heightfield + a handful of large/instanced debris, standing
    in for `quake_rubble.plan_pile` until agent B's module exists. See the
    module docstring's FALLBACK PLAN CAVEAT."""
    print("[rubble_preview] disaster.quake_rubble not found -> using the "
          "SYNTHETIC fallback plan (dome/windrow/fan heuristics, not the "
          "round-4 morphology model)")
    rng = random.Random(seed)
    look = "urm" if btype == "urm" else "rc"
    n = 51
    pad = 1.25
    X, Y, hx, hy = _grid(W, D, pad, n)

    crown_frac = CROWN_FRAC.get(btype, 0.28)
    crown = min(crown_frac * H, math.tan(math.radians(REPOSE_DEG)) * min(hx, hy) * 0.95)
    crown = max(crown, 1.2)

    if kind == "windrow":
        Z = _shape_windrow(X, Y, hx, hy, crown, sides, depth_m, offset_m)
    elif kind == "fan":
        Z = _shape_fan(X, Y, hx, hy, crown, sides)
    else:
        Z = _shape_dome(X, Y, hx, hy, crown, pad)

    noise = np.zeros_like(X)
    for wlen, amp in ((max(W, D) * 0.28, 0.35), (max(W, D) * 0.12, 0.18), (max(W, D) * 0.05, 0.08)):
        ph1, ph2 = rng.uniform(0, 6.28), rng.uniform(0, 6.28)
        noise = noise + amp * np.sin(X / wlen + ph1) * np.cos(Y / wlen * 1.3 + ph2)
    Z = np.clip(Z + noise * max(crown * 0.12, 0.15), 0.0, None)
    Z[Z < 0.06] = 0.0
    mound = _heightfield_from_grid(X, Y, Z, look)

    # a thin toe apron: same footprint, lower and wider, only where the
    # mound itself has gone to ~0 (fines spread beyond the pile toe)
    Xa, Ya, hxa, hya = _grid(W, D, pad * 1.5, n)
    if kind == "windrow":
        Za = _shape_windrow(Xa, Ya, hxa, hya, crown * 0.18, sides, depth_m * 2.2, offset_m)
    elif kind == "fan":
        Za = _shape_fan(Xa, Ya, hxa, hya, crown * 0.18, sides)
    else:
        Za = _shape_dome(Xa, Ya, hxa, hya, crown * 0.18, pad * 1.5)
    apron = _heightfield_from_grid(Xa, Ya, Za, look)

    def _z_at(x, y):
        gx = np.interp(x, X[0, :], np.arange(n))
        gy = np.interp(y, Y[:, 0], np.arange(n))
        i0, j0 = int(np.clip(gx, 0, n - 2)), int(np.clip(gy, 0, n - 2))
        return float(Z[j0, i0])

    large = []
    big_names = ["huge_concrete_rubble_pile", "concrete_slabs", "slab_01", "slab_05"] \
        if btype == "rc" else ["brick_debris_pile", "concrete_debris_elements"]
    for k in range(3):
        ang = rng.uniform(0, 6.2832)
        rad = rng.uniform(0.15, 0.55) * min(hx, hy)
        x, y = rad * math.cos(ang), rad * math.sin(ang)
        name = rng.choice(big_names)
        large.append({
            "asset": name, "prim_path": None, "kind": "raft",
            "pos": (x, y, _z_at(x, y) * 0.4),
            "rot_deg": (rng.uniform(15, 45), rng.uniform(-10, 10), rng.uniform(0, 360)),
            "scale": rng.uniform(0.7, 1.0), "size": _FALLBACK_CATALOGUE[name]["size"],
            "bury": rng.uniform(0.1, 0.4),
        })
    for k in range(2):
        ang = rng.uniform(0, 6.2832)
        rad = rng.uniform(0.2, 0.6) * min(hx, hy)
        x, y = rad * math.cos(ang), rad * math.sin(ang)
        large.append({
            "asset": None, "prim_path": None,
            "kind": "timber_joist" if btype == "urm" else "concrete",
            "pos": (x, y, _z_at(x, y) * 0.5),
            "rot_deg": (0.0, 0.0, rng.uniform(0, 360)),
            "scale": 1.0, "size": (0.2, 0.2, rng.uniform(2.0, 3.4)),
            "bury": rng.uniform(0.0, 0.15),
        })

    def _scatter(names, count, z_scale, size_range):
        pos, ori, scl, pidx = [], [], [], []
        for _ in range(count):
            ang = rng.uniform(0, 6.2832)
            rad = (rng.betavariate(1.6, 1.4)) * max(hx, hy) * pad
            x, y = rad * math.cos(ang), rad * math.sin(ang)
            z = _z_at(x, y) * z_scale
            yaw = rng.uniform(0, 6.2832)
            pos.append((x, y, z))
            ori.append((math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
            scl.append(rng.uniform(*size_range))
            pidx.append(rng.randrange(len(names)))
        return {"protos": list(names), "proto_index": pidx, "positions": pos,
                "orientations": ori, "scales": scl}

    instances = {
        "chunk": _scatter(_CHUNK_NAMES, 70, 0.85, (0.6, 1.1)),
        "flake": _scatter(_LUMP_NAMES, 50, 1.0, (0.7, 1.3)),
    }

    stats = {"n_large": len(large), "n_instances": sum(len(v["positions"]) for v in instances.values()),
             "crown_m": crown, "volume_m3": mound["volume_m3"]}
    return {"mound": mound, "apron": apron, "large": large, "instances": instances,
            "catalogue": _FALLBACK_CATALOGUE, "stats": stats}


def build_plan(W, D, H, btype, kind, seed, sides, depth_m, offset_m, crown_m=None,
              elem_h_m=None):
    # Broad except: agent B's `quake_rubble.py` is being written in parallel
    # this round and may exist as a file but not yet expose `plan_pile` (or
    # may raise for any other reason while it is mid-write) — any failure
    # here should fall back to the synthetic plan, not crash the preview.
    try:
        from disaster import quake_rubble as qr
        rng = random.Random(seed)
        # `levels` is a LIST of storey floor z-values, not a storey count —
        # `quake_flow.py` always treats `m["levels"]` that way (`len(...)`,
        # `m["levels"][i]`) even though the round-4 plan doc's one-line
        # summary of the mass dict does not say so. See the note this agent
        # left in `tests/test_quake_rubble_usd.py`.
        n_storeys = max(1, round(H / 3.3))
        levels = [0.0 + i * H / n_storeys for i in range(n_storeys)]
        m = {"cx": 0.0, "cy": 0.0, "W": W, "D": D, "yaw": 0.0, "z0": 0.0, "top": H, "levels": levels}
        kwargs = {}
        # `sides` is a plain string (e.g. "S" or "NW") — `tuple(sides)` splits
        # it into per-character compass letters, exactly the tuple form every
        # `plan_pile` kind expects (dome: any number of fall sides via
        # `set(sides)`; windrow/fan: one row per side in `sides`).
        side_tuple = tuple(sides) if sides else None
        if kind == "dome":
            kwargs.update(sides=side_tuple, crown_m=crown_m)
        elif kind == "windrow":
            kwargs.update(sides=side_tuple, along=(0.1, 0.9), depth_m=depth_m, offset_m=offset_m)
        elif kind == "fan":
            kwargs.update(sides=side_tuple, depth_m=depth_m, elem_h_m=elem_h_m)
        plan = qr.plan_pile(m, btype, rng, kind=kind, **kwargs)
        print("[rubble_preview] used disaster.quake_rubble.plan_pile (the real planner)")
        return plan
    except Exception as exc:
        print("[rubble_preview] disaster.quake_rubble.plan_pile unavailable ({0}: {1}) "
              "-> falling back to the synthetic plan".format(type(exc).__name__, exc))
        return _fallback_plan(W, D, H, btype, kind, seed, sides, depth_m, offset_m)


# --------------------------------------------------------------------------- #
# scene furniture: ground plane + a standing stub, authored with bare pxr
# --------------------------------------------------------------------------- #
def _add_ground_and_stub(stage, parent, W, D, H):
    from pxr import Gf, Sdf, UsdGeom, Vt
    from disaster import damage

    # z = -0.02, not 0.0: the mound/apron rim and any authored "large" box
    # both sit at z = 0 (their own bottom-centre / footprint convention —
    # see `quake_rubble_usd._box`), so a ground plane AT z = 0 is coplanar
    # with the rim and z-fights in every wide (obl/top) render — a BLACK
    # band hugging the mound toe (round-4 v5 review: `rc_dome_s3_contact.png`,
    # `urm_dome_s1_contact.png`). The planner's own winding/normals were
    # checked and are fine; this is a preview-furniture-only fix, dropping
    # the ground 2 cm below everything the plan ever puts at z = 0 so the
    # two surfaces can never share a depth value. 2 cm is far below Cycles'
    # shadow-bias/z-fighting threshold at these render distances but not
    # visually different from flush ground in any shot.
    GROUND_Z = -0.02
    hx, hy = 60.0, 60.0
    gpath = "{0}/ground".format(parent)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(gpath))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(-hx, -hy, GROUND_Z), Gf.Vec3f(hx, -hy, GROUND_Z),
                                       Gf.Vec3f(hx, hy, GROUND_Z), Gf.Vec3f(-hx, hy, GROUND_Z)]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateExtentAttr([Gf.Vec3f(-hx, -hy, GROUND_Z), Gf.Vec3f(hx, hy, GROUND_Z)])
    ground_rgb = (0.42, 0.42, 0.41)
    gmat = damage._pbr(stage, "{0}/QuakeLooks/ground_grey".format(parent),
                       ground_rgb, 0.96)
    qru._add_preview_fallback(stage, "{0}/QuakeLooks/ground_grey".format(parent),
                              ground_rgb, 0.96)
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(gmat)

    # a short stub of the original wall, still standing at the footprint's
    # south edge, 30% of the building's height — scale reference for how
    # tall this was before it came down.
    stub_path = "{0}/standing_stub".format(parent)
    sx, sy, sz = W * 0.28, 0.3, 0.30 * H
    stub = qru._box(stage, stub_path, sx, sy, sz)
    stub_rgb = (0.16, 0.15, 0.15)
    smat = damage._pbr(stage, "{0}/QuakeLooks/stub_dark".format(parent),
                       stub_rgb, 0.88)
    qru._add_preview_fallback(stage, "{0}/QuakeLooks/stub_dark".format(parent),
                              stub_rgb, 0.88)
    UsdShade.MaterialBindingAPI.Apply(stub.GetPrim()).Bind(smat)
    xf = UsdGeom.Xformable(stub)
    # base at GROUND_Z (the stub mesh's own local origin is bottom-centre,
    # z=0 — see `_box`), so it sits flush on the dropped ground instead of
    # floating 2 cm above it or (if left at z=0) resuming the same
    # coplanar-with-ground z-fighting this function exists to avoid.
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -D / 2.0 - sy / 2.0 + 0.05, GROUND_Z))


# --------------------------------------------------------------------------- #
# bpy render (idioms copied from render_usd.py — that file is owned by
# another agent this round and is not modified)
# --------------------------------------------------------------------------- #
def _setup_engine(samples, use_gpu):
    import bpy
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    device = "CPU"
    if use_gpu:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
            except Exception:
                continue
            gpus = [d for d in prefs.devices if d.type == backend]
            if gpus:
                for d in prefs.devices:
                    d.use = d.type == backend
                scene.cycles.device = "GPU"
                device = "GPU/{0} ({1})".format(backend, gpus[0].name)
                break
    if device == "CPU":
        scene.cycles.device = "CPU"
    return device


def _add_lighting(bg):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    bg_node = world.node_tree.nodes["Background"]
    bg_node.inputs[0].default_value = (bg, bg, bg, 1.0)
    bg_node.inputs[1].default_value = 0.9
    bpy.context.scene.world = world

    def sun(name, rot, energy):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(3)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    sun("key", (math.radians(55), 0, math.radians(40)), 4.0)
    sun("fill", (math.radians(65), 0, math.radians(210)), 1.5)


def _place_camera(center, dist, az_deg, el_deg, fov=math.radians(42)):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = 0.05
    cam_data.clip_end = dist * 20.0
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    az, el = math.radians(az_deg), math.radians(el_deg)
    direction = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = Vector(center) + direction * dist
    cam.rotation_euler = (Vector(center) - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _place_camera_lookat(location, target, fov=math.radians(50), clip_end=500.0):
    """A camera at an explicit `location` aimed at `target` — for the
    close-up "standing near the crown" shot, which is not a point on a
    sphere around one centre the way the oblique/nadir views are."""
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = 0.03
    cam_data.clip_end = clip_end
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    loc = Vector(location)
    cam.location = loc
    cam.rotation_euler = (Vector(target) - loc).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _render_to(path, w, h):
    import bpy
    scene = bpy.context.scene
    scene.render.resolution_x = w
    scene.render.resolution_y = h
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def _contact_sheet(tiles, out, title):
    from PIL import Image, ImageDraw, ImageFont
    imgs = [Image.open(t).convert("RGB") for t in tiles]
    cols = len(imgs)
    tw, th = imgs[0].size
    pad, header = 10, 40
    W = cols * tw + (cols + 1) * pad
    H = header + th + 2 * pad
    sheet = Image.new("RGB", (W, H), (20, 20, 19))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 22)
    except Exception:
        font = ImageFont.load_default()
    draw.text((pad, 10), title, fill=(235, 235, 230), font=font)
    for i, im in enumerate(imgs):
        sheet.paste(im, (pad + i * (tw + pad), header + pad))
    sheet.save(out)


def render_views(usd_path, out_dir, tag, pile_center, pile_dist, crown_pt, fall_dir,
                 res=(1280, 720), samples=64, cpu=False, close_dist=12.0):
    """Three views on a sphere around `pile_center` at `pile_dist`, plus a
    4th close-up standing `close_dist` m from `crown_pt` looking down
    `fall_dir` — "what a drone at 40 m sees" (research memo §7) for the
    overall silhouette (obl1/obl2/top), and a conversational-range check of
    whether the CROWN reads as rubble and the large elements read as large,
    which a 40 m frame cannot resolve.

    `pile_dist`/`close_dist` (round-5): the tool's own auto-framing
    (`pile_dist = 1.6 x pile bbox diagonal` in `main()`, `close_dist` here
    defaulting to 12 m) does not land at a requested fixed distance for
    every pile size — `--dist`/`--close-dist` let a caller override both
    explicitly (e.g. "~35 m oblique, ~6 m close-up") instead of only ever
    getting whatever the bbox-relative formula produces (v6 README's
    "Distance note": the auto-framed obliques landed at 76-142 m for the
    documented rc/urm domes, not the ~35 m that write-up asked for, because
    at the time changing this file was out of scope for that agent)."""
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))

    device = _setup_engine(samples, use_gpu=not cpu)
    print("Rendering on {0}".format(device))
    _add_lighting(0.55)

    views = [
        ("obl1", 35.0, 35.0),
        ("obl2", 35.0 + 180.0, 35.0),
        ("top", 10.0, 82.0),
    ]
    tiles = []
    t0 = time.time()
    for name, az, el in views:
        _place_camera(pile_center, pile_dist, az, el)
        fp = Path(out_dir) / "{0}_{1}.png".format(tag, name)
        _render_to(fp, *res)
        tiles.append(fp)
        print("  {0} -> {1}  ({2:.1f}s elapsed)".format(name, fp, time.time() - t0))

    fdx, fdy = fall_dir
    look_ahead = close_dist * (8.0 / 12.0)     # keep the v4-v6 framing ratio at any close_dist
    cam_pos = (crown_pt[0] - fdx * close_dist, crown_pt[1] - fdy * close_dist, crown_pt[2] + 2.0)
    look_at = (crown_pt[0] + fdx * look_ahead, crown_pt[1] + fdy * look_ahead, max(crown_pt[2] * 0.10, 0.3))
    _place_camera_lookat(cam_pos, look_at, fov=math.radians(55))
    fp = Path(out_dir) / "{0}_close.png".format(tag)
    _render_to(fp, *res)
    tiles.append(fp)
    print("  close -> {0}  ({1:.1f}s elapsed)".format(fp, time.time() - t0))

    sheet = Path(out_dir) / "{0}_contact.png".format(tag)
    _contact_sheet(tiles, sheet, tag)
    print("wrote {0}".format(sheet))
    return tiles, sheet


def _pile_bbox(stage, paths):
    """World-space centre + diagonal of the union bbox of `paths` (the
    MOUND and the "large" elements only — not the ground plane, apron, or
    the flattened instancer scatter, which would drag the frame out to the
    full stage furniture instead of the actual pile)."""
    from pxr import Gf, Usd, UsdGeom

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    lo = [math.inf, math.inf, math.inf]
    hi = [-math.inf, -math.inf, -math.inf]
    found = False
    for p in paths:
        if not p:
            continue
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            continue
        rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        found = True
        mn, mx = rng.GetMin(), rng.GetMax()
        for i in range(3):
            lo[i] = min(lo[i], mn[i])
            hi[i] = max(hi[i], mx[i])
    if not found:
        return (0.0, 0.0, 2.0), 12.0
    center = tuple((lo[i] + hi[i]) / 2.0 for i in range(3))
    diag = math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(3)))
    return center, diag


def _crown_point(mound):
    """The mound's own highest point (its literal crown), not the footprint
    centre at some assumed height — exact, and correct for an asymmetric
    (windrow/fan) pile whose crown is not at the footprint's geometric
    middle."""
    pts = np.asarray(mound["points"], dtype=np.float64).reshape(-1, 3)
    i = int(np.argmax(pts[:, 2]))
    return tuple(float(v) for v in pts[i])


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--type", choices=("rc", "urm"), required=True)
    ap.add_argument("--W", type=float, required=True)
    ap.add_argument("--D", type=float, required=True)
    ap.add_argument("--H", type=float, required=True)
    ap.add_argument("--kind", choices=("dome", "windrow", "fan"), default="dome")
    ap.add_argument("--sides", default="S",
                    help="wall(s), one letter each, no separator: N/E/S/W, e.g. NW "
                         "for two dome fall sides")
    ap.add_argument("--crown", type=float, default=None, help="dome: crown height override (m)")
    ap.add_argument("--depth", type=float, default=1.2, help="windrow/fan depth at the wall (m)")
    ap.add_argument("--elem-h", type=float, default=None,
                    help="fan/windrow: the fallen element's own height (m) — overrides "
                         "--depth's role in picking a default depth if --depth is not given")
    ap.add_argument("--offset", type=float, default=1.5, help="windrow offset out from the wall (m)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cpu", action="store_true", help="force CPU rendering")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--dist", type=float, default=None,
                    help="override the oblique-view camera distance (m); "
                         "default: 1.6x the pile's own bbox diagonal")
    ap.add_argument("--close-dist", type=float, default=12.0,
                    help="close-up camera distance from the crown (m)")
    ap.add_argument("--rw", type=int, default=1280, help="render width (px)")
    ap.add_argument("--rh", type=int, default=720, help="render height (px)")
    args = ap.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = "{0}_{1}_s{2}".format(args.type, args.kind, args.seed)

    plan = build_plan(args.W, args.D, args.H, args.type, args.kind, args.seed,
                      args.sides, args.depth, args.offset, crown_m=args.crown,
                      elem_h_m=args.elem_h)
    stats = plan.get("stats") or {}
    print("[rubble_preview] plan stats: {0}".format(stats))

    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    parent = "/World/Bldg"
    stage.DefinePrim(parent, "Xform")

    result = qru.author(stage, parent, plan, tag=args.type, asset_root=LOCAL_ASSET_ROOT,
                        flatten_instances=True)
    print("[rubble_preview] authored: mound={0} apron={1} large={2} instancer-xforms={3}"
          .format(bool(result["mound"]), bool(result["apron"]),
                  len(result["large"]), len(result["instancers"])))

    _add_ground_and_stub(stage, parent, args.W, args.D, args.H)

    # frame the PILE (mound + large elements), not the 120x120 m ground —
    # and pick the crown / fall direction for the close-up view, straight
    # off the plan (exact) rather than re-deriving them from the mesh.
    pile_center, pile_diag = _pile_bbox(stage, [result["mound"]] + list(result["large"]))
    pile_dist = args.dist if args.dist else 1.6 * max(pile_diag, 4.0)
    if plan.get("mound"):
        crown_pt = _crown_point(plan["mound"])
    else:
        crown_pt = (pile_center[0], pile_center[1], pile_center[2] + 2.0)
    fall_side = (stats.get("fall_sides") or [args.sides])[0]
    fall_dir = _FALL_DIR.get(fall_side, _FALL_DIR["S"])
    print("[rubble_preview] framing: center={0} dist={1:.1f} crown={2} fall_dir={3} ({4})"
          .format(tuple(round(c, 2) for c in pile_center), pile_dist,
                  tuple(round(c, 2) for c in crown_pt), fall_dir, fall_side))

    usd_path = out_dir / "{0}.usda".format(tag)
    stage.GetRootLayer().Export(str(usd_path))
    print("wrote {0}".format(usd_path))

    tiles, sheet = render_views(usd_path, out_dir, tag, pile_center, pile_dist,
                                crown_pt, fall_dir, res=(args.rw, args.rh),
                                samples=args.samples, cpu=args.cpu,
                                close_dist=args.close_dist)
    print("[rubble_preview] done: {0}".format([str(t) for t in tiles] + [str(sheet)]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
