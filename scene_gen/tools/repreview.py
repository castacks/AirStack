#!/usr/bin/env python3
"""repreview — re-take an archetype's bake previews after it changed.

    /isaac-sim/python.sh tools/repreview.py <archetype.usd> [more.usd ...]

`archetypes/preview.py` shoots every rung DURING the bake, which is the cheap
moment: the wreck is already on a lit stage. A file that changed AFTERWARDS —
hand-posed through `edit_usd.py` / `edit_usds.py`, or re-baked on its own —
keeps its old pictures, so the contact sheet quietly shows the previous
geometry and the library looks fine when it is not.

This re-shoots the same two frames, into the same `previews/` names the
manifest already points at, so `tools/preview_sheet.py` picks them up with no
manifest edit at all.

Same camera as the bake (`preview.capture_cell`), so a re-shot tile is
comparable with the ones beside it rather than being a different picture of the
same thing.
"""

from __future__ import annotations

import argparse
import os
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("usds", nargs="+")
    p.add_argument("--gui", action="store_true")
    return p.parse_args(argv)


def _png_complete(path: str) -> bool:
    """Is *path* a PNG whose final IEND chunk has been written?

    Waiting on mtime alone is NOT enough: the timestamp updates when the file
    is OPENED for writing, so a capture that has created the file but not yet
    flushed it looks finished. `os._exit` then truncates it. Measured
    2026-08-30 -- `house_04_pancaked_top.png` landed at 679,936 bytes and
    `PIL` refused it with "image file is truncated", which took the whole
    contact sheet down with it.

    The IEND chunk is the last 12 bytes of a well-formed PNG, so its presence
    is the cheap, dependency-free proof that the writer got to the end.
    """
    try:
        with open(path, "rb") as fh:
            if fh.read(8) != b"\x89PNG\r\n\x1a\n":
                return False
            fh.seek(-12, os.SEEK_END)
            return fh.read()[4:8] == b"IEND"
    except OSError:
        return False


def _await_writes(before: dict, tries: int = 600) -> list:
    """Pump until every path in *before* has actually CHANGED on disk.

    `preview._capture` returns its path WITHOUT confirming the write --
    `capture_viewport_to_file` is asynchronous, and its own docstring says a
    caller that must know should stat the file rather than trust the return
    value. This tool then calls `os._exit`, which kills the process outright:
    any capture still in flight is lost, and the run prints the names of files
    it never wrote. Measured 2026-08-30 on a single-file re-shot -- reported
    `1/1 re-shot`, and the PNG on disk was untouched from the original bake.

    Comparing against the PRIOR mtime rather than mere existence is the point:
    every file being re-shot here already exists, so an existence check passes
    instantly on the stale picture.
    """
    import omni.kit.app
    app = omni.kit.app.get_app()
    done = []
    for _ in range(tries):
        done = [q for q, t0 in before.items()
                if os.path.exists(q) and os.path.getmtime(q) > t0
                and _png_complete(q)]
        if len(done) == len(before):
            return done
        app.update()
    return done


def main(argv=None) -> int:
    args = _parse(argv)
    files = [os.path.abspath(f) for f in args.usds if os.path.isfile(f)]
    if not files:
        print("[repreview] nothing to do", file=sys.stderr)
        return 2

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": not args.gui})

    done = 0
    try:
        import omni.kit.app
        import omni.usd
        from pxr import Sdf, Usd, UsdGeom
        from archetypes import bake as A
        from archetypes import library as lib
        from archetypes import preview as PV

        for f in files:
            ctx = omni.usd.get_context()
            ctx.new_stage()
            stage = ctx.get_stage()
            A.prepare_stage(stage)
            holder = UsdGeom.Xform.Define(stage, Sdf.Path("/World/subject"))
            holder.GetPrim().GetReferences().AddReference(f)
            for _ in range(40):
                omni.kit.app.get_app().update()

            paths = [str(p.GetPath()) for p in Usd.PrimRange(holder.GetPrim())
                     if p.IsA(UsdGeom.Mesh)]
            if not paths:
                print(f"[repreview] no meshes in {os.path.basename(f)}")
                continue
            out = PV.preview_dir(os.path.dirname(f))
            stem = os.path.join(out, os.path.splitext(os.path.basename(f))[0])
            expect = [stem + "_obl.png", stem + "_top.png"]
            before = {q: (os.path.getmtime(q) if os.path.exists(q) else 0.0)
                      for q in expect}
            PV.capture_cell(stage, paths, stem)
            landed = _await_writes(before)
            print(f"[repreview] {os.path.basename(f)} -> "
                  f"{sorted(os.path.basename(q) for q in landed) or 'FAILED'}")
            done += 1 if len(landed) == len(expect) else 0
    except BaseException:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        app.close()
        raise

    print(f"\n[repreview] {done}/{len(files)} re-shot")
    # Before close(): it hard-exits (`/app/fastShutdown`) and nothing after it
    # would run — the trap `bake_cli.py` documents.
    sys.stdout.flush()
    os._exit(0 if done == len(files) else 1)


if __name__ == "__main__":
    sys.exit(main())
