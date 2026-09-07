#!/usr/bin/env python3
"""edit_usd — open a USD in an Isaac Sim window so you can edit AND SAVE it.

    docker exec -it isaac-sim bash -lc \\
      'PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/edit_usd.py <file.usd>'

Then edit in the viewport / Stage tree and press **Ctrl+S** (or File > Save).
Closing the window exits.

WHY NOT `view_usd.py`
---------------------
That one REFERENCES each file onto a fresh stage so it can lay several out on a
grid. USD does not author across a reference arc: every change you make there
lands as an override on the VIEWER's root layer, and the file you were looking
at is never touched. Ctrl+S saves the wrapper, the asset stays exactly as it
was, and the edits are gone when the window closes — measured the hard way on
`SM_MERGED_BP_MBuilding01_soft_storey.usd`, whose mtime never moved.

So this opens the file AS THE ROOT LAYER (`open_stage`). The edit target is
that layer, which is what makes Ctrl+S write back to the path you named.

LIGHTS AND GROUND GO IN THE SESSION LAYER
-----------------------------------------
An archetype is meshes and materials — no light, no floor — so opening one
straight gives you a black viewport. Adding a dome light to the root layer
would fix the view and then SAVE ITSELF INTO THE ASSET, which is how a
review rig ends up shipped inside a building. They are authored into
`GetSessionLayer()` instead: composed for looking at, never written by a save.

PROVENANCE
----------
A hand-edited archetype is no longer what the bake produced, and the manifest
would go on claiming it was. On exit this compares the file's hash with the one
taken at startup and, if it changed, writes `hand_edited_at` (and the
fingerprint it was baked under) onto that archetype's manifest record — so
`archetypes/version.py` and anyone reading the library can tell.

It also keeps a `.orig.usd` beside the file unless `--no-backup`, because the
first thing anyone does with a hand-edit is want the original back.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _gizmo_at_mesh_centre():
    """Put the move/rotate/scale gizmo on each object's own geometry.

    Kit defaults this to "Authored Pivot", and a baked fragment's authored
    pivot sits at the object origin -- which for a wreck recentred on the
    world is nowhere near the piece you clicked. The gizmo then appears
    detached from the mesh, and dragging it feels like moving something else.

    "Bounding Box Center" places it at the centre of the selected object's own
    bounds instead. Set explicitly on every launch rather than trusting the
    persisted value, so an edit session behaves the same on any machine.
    """
    try:
        import carb
        carb.settings.get_settings().set(
            "/persistent/exts/omni.kit.manipulator.prim.core"
            "/manipulator/placement", "Bounding Box Center")
    except Exception as exc:                                     # noqa: BLE001
        print(f"[edit] could not set gizmo placement: {exc}")


def _digest(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def _parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("usd", help="the USD to open and edit IN PLACE")
    p.add_argument("--no-backup", action="store_true",
                   help="skip the <name>.orig.usd snapshot")
    p.add_argument("--no-ground", action="store_true",
                   help="skip the reference ground plane")
    p.add_argument("--headless", action="store_true",
                   help="no window (only useful for testing this script)")
    return p.parse_args(argv)


def _mark_hand_edited(usd_path: str, baked_fp: str = "") -> None:
    """Record on the manifest that this archetype is no longer pure bake output."""
    import datetime

    lib_dir = os.path.dirname(os.path.abspath(usd_path))
    man = os.path.join(lib_dir, "manifest.json")
    if not os.path.isfile(man):
        return
    try:
        with open(man) as fh:
            doc = json.load(fh)
        name = os.path.basename(usd_path)
        hit = False
        for rec in doc.get("archetypes", ()):
            if rec.get("usd") == name:
                rec["hand_edited_at"] = datetime.datetime.now().astimezone(
                    ).isoformat(timespec="seconds")
                if baked_fp:
                    rec["hand_edited_from_fingerprint"] = baked_fp
                hit = True
        if hit:
            with open(man, "w") as fh:
                json.dump(doc, fh, indent=2, sort_keys=True)
            print(f"[edit] manifest: marked {name} hand-edited")
    except Exception as exc:                                     # noqa: BLE001
        print(f"[edit] could not mark the manifest: {exc}")


def main(argv=None) -> int:
    args = _parse(argv)
    path = os.path.abspath(args.usd)
    if not os.path.isfile(path):
        print(f"[edit] no such file: {path}", file=sys.stderr)
        return 2

    orig = ""
    if not args.no_backup:
        orig = os.path.splitext(path)[0] + ".orig" + os.path.splitext(path)[1]
        if os.path.exists(orig):
            print(f"[edit] backup already exists, keeping it: {orig}")
        else:
            shutil.copy2(path, orig)
            print(f"[edit] backup -> {orig}")

    before = _digest(path)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": args.headless})
    _gizmo_at_mesh_centre()

    import omni.kit.app
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

    ctx = omni.usd.get_context()
    # THE FILE ITSELF AS ROOT LAYER. This is the whole point — see the module
    # docstring. `open_stage` makes the edit target this layer, so Ctrl+S
    # writes back here instead of into a throwaway wrapper.
    ctx.open_stage(path)
    for _ in range(60):
        omni.kit.app.get_app().update()
    stage = ctx.get_stage()
    if stage is None:
        print("[edit] stage failed to open", file=sys.stderr)
        simulation_app.close()
        return 1

    # Look-at rig, in the SESSION layer so a save cannot pick it up.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        rig = Sdf.Path("/_review")
        UsdGeom.Scope.Define(stage, rig)
        UsdLux.DomeLight.Define(
            stage, rig.AppendChild("dome")).CreateIntensityAttr(900.0)
        key = UsdLux.DistantLight.Define(stage, rig.AppendChild("key"))
        key.CreateIntensityAttr(2200.0)
        UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(
            Gf.Vec3f(-45.0, 0.0, 30.0))
        if not args.no_ground:
            cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                          UsdGeom.Tokens.render])
            rng = cache.ComputeWorldBound(
                stage.GetPseudoRoot()).ComputeAlignedRange()
            e = 200.0
            z = 0.0
            if not rng.IsEmpty():
                mn, mx = rng.GetMin(), rng.GetMax()
                e = max(40.0, 2.0 * max(mx[0] - mn[0], mx[1] - mn[1]))
                z = float(mn[2])
            g = UsdGeom.Mesh.Define(stage, rig.AppendChild("ground"))
            g.CreatePointsAttr([Gf.Vec3f(-e, -e, z), Gf.Vec3f(e, -e, z),
                                Gf.Vec3f(e, e, z), Gf.Vec3f(-e, e, z)])
            g.CreateFaceVertexCountsAttr([4])
            g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
            g.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.25, 0.15)])
    # Belt and braces: put the edit target back on the root layer, so anything
    # authored from the UI or the Script Editor lands in the file.
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))

    print("\n" + "=" * 70)
    print(f"EDITING  {path}")
    print(f"  backup   : {orig or 'none (--no-backup)'}")
    print("  save     : Ctrl+S, or File > Save   (writes back to the path above)")
    print("  lighting : /_review, in the session layer — never saved")
    print("  exit     : close the window")
    print("=" * 70 + "\n")
    sys.stdout.flush()

    while simulation_app.is_running():
        omni.kit.app.get_app().update()

    # DO NOT `simulation_app.close()` HERE. It hard-exits the process
    # (`/app/fastShutdown`), so everything below — the change check and the
    # provenance mark — would never run. `bake_cli.py` documents the same trap
    # for its exit code; here it meant a hand-edited archetype was saved and
    # then never marked, so the manifest went on claiming pure bake output.
    after = _digest(path)
    if after == before:
        print("[edit] file unchanged — nothing was saved")
        sys.stdout.flush()
        os._exit(0)
    print(f"[edit] SAVED: {path} changed ({before} -> {after})")
    fp = ""
    try:
        from archetypes import version as V
        fp = V.source_fingerprint()
    except Exception:                                            # noqa: BLE001
        pass
    _mark_hand_edited(path, fp)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
