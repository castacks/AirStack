"""Repair the live deadline-review scene and capture every quake casualty.

Run inside the already-open Isaac Sim Script Editor::

    exec(open("/isaac-sim/AirStack/scene_gen/tools/live_quake_casualty_review.py").read())

This intentionally does not rebuild damage.  It performs only the deadline
gates requested for the earthquake baseline review: install the selected sky,
hide the known red/incompatible Office fit-out references, and take two usable
damage-facing photographs of every authored casualty.
"""

import importlib.util
import json
import math
import os
import subprocess
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade


OUT = os.environ.get(
    "SNAP_DIR", "/isaac-sim/.nvidia-omniverse/logs/eq500_casualties_r17")
PEOPLE_JSON = os.path.join(OUT, "quake_people.json")
BUILDINGS_JSON = os.path.join(OUT, "quake_buildings.json")
VIEW_DIR = os.path.join(OUT, "casualty_views")
CIRRUS = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
          "NVIDIA/Assets/Skies/Dynamic/Cirrus.usd")
HDR_FALLBACK = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                "NVIDIA/Assets/Skies/2022_1/Skies/Clear/mealie_road.hdr")
OFFICE_MARKER = "/NVIDIA/Assets/Isaac/5.1/Isaac/Environments/Office/Props/"


def _vram():
    try:
        raw = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total",
             "--format=csv,noheader,nounits"], text=True, timeout=10)
        return tuple(float(v.strip()) for v in raw.splitlines()[0].split(",")[:2])
    except Exception:
        return (None, None)


def _reference_paths(prim):
    paths = []
    for spec in prim.GetPrimStack():
        try:
            refs = spec.referenceList.GetAddedOrExplicitItems()
        except Exception:
            refs = ()
        for ref in refs:
            if ref.assetPath:
                paths.append(str(ref.assetPath))
    return paths


def _has_usable_surface(material):
    """Whether Isaac has a surface implementation it can actually render.

    Several baked fit-out meshes retain only an ``unreal:surface`` output.
    That is metadata from the source asset, not a supported shader in Kit,
    and RTX displays the mesh as the bright-red error material.  A universal,
    MDL, or MaterialX source is usable here; Unreal-only is not.
    """
    if not material:
        return True
    for context in ("", "mdl", "mtlx"):
        try:
            source = (material.ComputeSurfaceSource(context)
                      if context else material.ComputeSurfaceSource())
            if source and source[0] and source[0].GetPrim().IsValid():
                return True
        except Exception:
            continue
    return False


def _hide_incompatible_office_props(stage):
    """Remove only fit-out props that visibly render as solid red.

    Their USD layers resolve, but their legacy material has no usable USD
    surface source in this Isaac build.  Keeping a red error proxy is worse
    for evaluation than omitting optional room dressing.  Building, rubble,
    people, vehicles and street props are outside this narrowly scoped pass.
    """
    hidden = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.GetName().startswith("prop_"):
            continue
        refs = _reference_paths(prim)
        material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        incompatible_reference = any(OFFICE_MARKER in ref for ref in refs)
        incompatible_baked_look = bool(material) and not _has_usable_surface(
            material)
        if incompatible_reference or incompatible_baked_look:
            if prim.IsActive() and prim.SetActive(False):
                hidden.append({
                    "prim": path,
                    "references": refs,
                    "material": (str(material.GetPath())
                                 if material else None),
                    "reason": ("office_reference" if incompatible_reference
                               else "unsupported_baked_material"),
                })
    return hidden


def _install_sky(stage, app):
    before = _vram()
    for old in ("/World/Environment", "/World/DomeLight", "/World/QuakeSky"):
        prim = stage.GetPrimAtPath(old)
        if prim.IsValid() and prim.IsActive():
            prim.SetActive(False)

    sky = stage.DefinePrim(Sdf.Path("/World/QuakeSky"), "Xform")
    sky.SetActive(True)
    sky.GetReferences().ClearReferences()
    sky.GetReferences().AddReference(CIRRUS, "/World")
    for _ in range(80):
        app.update()
    # The compass is an authoring aid, not part of the sky.
    compass = stage.GetPrimAtPath(
        "/World/QuakeSky/AxisNorth/ArrowsNWSE")
    if compass.IsValid():
        compass.SetActive(False)
    for _ in range(20):
        app.update()

    after = _vram()
    selected = CIRRUS
    # Preserve at least six percent of the card for the render product and
    # baseline process.  Only then take the user's requested HDR fallback.
    if after[0] is not None and after[1] and after[0] > 0.94 * after[1]:
        stage.GetPrimAtPath("/World/QuakeSky").SetActive(False)
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/QuakeHDR"))
        dome.CreateIntensityAttr(2000.0)
        dome.CreateExposureAttr(0.0)
        dome.CreateTextureFileAttr(Sdf.AssetPath(HDR_FALLBACK))
        dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
        selected = HDR_FALLBACK
        for _ in range(80):
            app.update()
        after = _vram()
    # Cirrus provides the sky/background, but the quake canyon has deep
    # shadowed elevations.  A sun plus a weak, shadowless opposite fill keeps
    # casualties readable without flattening the outdoor damage geometry.
    for path, intensity, rotation, shadows in (
            ("/World/QuakeReviewSun", 2600.0, (48.0, -18.0, 132.0), True),
            ("/World/QuakeReviewFill", 650.0, (58.0, 12.0, -48.0), False)):
        old = stage.GetPrimAtPath(path)
        if old.IsValid():
            old.SetActive(False)
        light = UsdLux.DistantLight.Define(stage, Sdf.Path(path))
        light.CreateIntensityAttr(float(intensity))
        light.CreateAngleAttr(2.0 if shadows else 5.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.955, 0.88))
        if not shadows:
            light.GetPrim().CreateAttribute(
                "inputs:shadow:enable", Sdf.ValueTypeNames.Bool).Set(False)
        xf = UsdGeom.Xformable(light.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    for _ in range(20):
        app.update()
    return {"selected": selected,
            "vram_before_mib": before[0], "vram_after_mib": after[0],
            "vram_total_mib": after[1],
            "review_lighting": "Cirrus + 2600 key + 650 shadowless fill"}


def _load_snapshots():
    loaded = sys.modules.get("snapshots")
    if loaded is not None and hasattr(loaded, "snapshot"):
        return loaded
    path = "/isaac-sim/AirStack/simulation/isaac-sim/utils/snapshots_rp.py"
    spec = importlib.util.spec_from_file_location("quake_live_snapshots", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _outward(building, side):
    local = {"S": (0.0, -1.0), "E": (1.0, 0.0),
             "N": (0.0, 1.0), "W": (-1.0, 0.0)}.get(side, (1.0, 0.0))
    a = math.radians(float(building.get("yaw_deg", 0.0)))
    return (math.cos(a) * local[0] - math.sin(a) * local[1],
            math.sin(a) * local[0] + math.cos(a) * local[1])


def _subject_center(stage, rec):
    """Composed human centre, rather than the rig's placement pivot."""
    prim = stage.GetPrimAtPath(str(rec.get("prim") or ""))
    if prim and prim.IsValid():
        try:
            bc = UsdGeom.BBoxCache(
                0.0, [UsdGeom.Tokens.default_], useExtentsHint=True)
            rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if not rng.IsEmpty():
                lo, hi = rng.GetMin(), rng.GetMax()
                centre = tuple(0.5 * (float(lo[i]) + float(hi[i]))
                               for i in range(3))
                if all(math.isfinite(v) for v in centre):
                    return centre
        except Exception:
            pass
    return (float(rec["x"]), float(rec["y"]),
            float(rec.get("render_z", rec.get("z", 0.0))) + 0.25)


def _capture_people(stage, app, people, buildings):
    snaps = _load_snapshots()
    by_prim = {str(row.get("prim")): row for row in buildings}
    os.makedirs(VIEW_DIR, exist_ok=True)
    rows = []
    for rec in people:
        ident = str(rec["id"])
        root_x, root_y = float(rec["x"]), float(rec["y"])
        root_z = float(rec.get("render_z", rec.get("z", 0.0)))
        x, y, subject_z = _subject_center(stage, rec)
        z = root_z
        building = by_prim.get(str(rec.get("building_prim")), {})
        nx, ny = _outward(building, str(rec.get("side", "E")))
        target = (x, y, subject_z)
        # Current placement records carry the exact two rays checked against
        # final composed geometry.  This applies both to people just inside a
        # broken facade and to people on the exposed rubble apron.
        checked = rec.get("review_eyes") or ()
        checked_target = rec.get("review_target")
        if len(checked) == 2 and checked_target:
            eye1, eye2 = (tuple(float(v) for v in q) for q in checked)
            # The verified point establishes the clear opening. Aim at the
            # composed rig centre in XY, but do not aim above that opening
            # when a skeleton extent is unexpectedly tall.
            target = (x, y, min(subject_z,
                                float(checked_target[2]) + 0.45))
        elif rec.get("state") == "interior_casualty":
            setback = float(rec.get("setback_m", 2.0))
            d1 = setback + 3.0
            d2 = setback + 4.0
            eye1 = (x + nx * d1, y + ny * d1, z + 1.2)
            eye2 = (x + nx * d2, y + ny * d2, z + 1.85)
        else:
            # Nadir establishes which rubble covers the body; the oblique is
            # from the outward/runout side so the building cannot fill frame.
            eye1 = (x, y, subject_z + 7.0)
            eye2 = (x + nx * 6.5, y + ny * 6.5, subject_z + 5.0)

        results = []
        for number, eye in enumerate((eye1, eye2), 1):
            path = os.path.join(
                VIEW_DIR, "person_{0}_view{1}.png".format(ident, number))
            snaps.place_camera(stage, tuple(v for v in eye), target,
                               focal_mm=(48.0 if rec.get("state") ==
                                         "interior_casualty" else 35.0))
            ok = bool(snaps.snapshot(stage, path, res=(1280, 1280),
                                     subframes=20, target=(root_x, root_y)))
            results.append({"view": number, "path": path, "ok": ok,
                            "eye": [round(v, 3) for v in eye]})
        rows.append({"id": ident, "state": rec.get("state"),
                     "building": rec.get("building_prim"),
                     "side": rec.get("side"), "views": results})
        print("[quake_live_review] {0}: {1}/2 views".format(
            ident, sum(v["ok"] for v in results)), flush=True)
    return rows


stage = omni.usd.get_context().get_stage()
app = omni.kit.app.get_app()
if stage is None:
    raise RuntimeError("No live stage")
with open(PEOPLE_JSON) as fh:
    people_doc = json.load(fh)
with open(BUILDINGS_JSON) as fh:
    buildings = json.load(fh)
people = list(people_doc.get("people") or ())

hidden = _hide_incompatible_office_props(stage)
sky = _install_sky(stage, app)
omni.timeline.get_timeline_interface().play()
views = _capture_people(stage, app, people, buildings)
report = {
    "schema": "airstack.quake-casualty-review/1",
    "casualties": len(people),
    "expected_views": 2 * len(people),
    "successful_views": sum(v["ok"] for r in views for v in r["views"]),
    "failed_views": [v["path"] for r in views for v in r["views"]
                     if not v["ok"]],
    "hidden_incompatible_office_props": hidden,
    "sky": sky,
    "people": views,
}
with open(os.path.join(OUT, "casualty_review_done.json"), "w") as fh:
    json.dump(report, fh, indent=2)
print("[quake_live_review] DONE: {0}/{1} views, {2} red Office prop refs "
      "hidden, sky={3}, VRAM={4}/{5} MiB".format(
          report["successful_views"], report["expected_views"], len(hidden),
          sky["selected"], sky["vram_after_mib"], sky["vram_total_mib"]),
      flush=True)
