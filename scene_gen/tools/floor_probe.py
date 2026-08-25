#!/usr/bin/env python3
"""floor_probe.py — is `settle.prepare`'s floor actually a collider?

A settle that reports fragments at rest ELEVEN METRES BELOW an infinite
half-space has two possible explanations and they want completely different
fixes:

    the floor is INERT     nothing collides with it, everything falls for the
                           whole step budget, and a box floor (or whatever
                           makes it real) is the fix. CCD is irrelevant.
    the floor TUNNELS      it collides, but a body moving far enough in one
                           1/60 s step passes through without a contact ever
                           being generated. CCD is the fix; a thicker floor
                           only raises the speed at which it fails.

One drop separates them, and the separation is the SPEED at impact — so this
drops the same unit cube from several heights onto the real
`settle.prepare` floor and prints where each one comes to rest:

    rests at ~0.5 m from every height        the floor works, at these speeds
    rests at ~0.5 m from low, falls from high tunnelling, and the threshold
                                             is between the two heights
    falls from every height                  the floor is inert

Impact speed is `sqrt(2 g h)`, and travel per 1/60 s step is that over 60 —
printed alongside, because that distance against a zero-thickness collider is
the whole question.

    cd AirStack
    uv run --env-file .env.host python scene_gen/tools/floor_probe.py

RUNS ON THE HOST, like `quake_preview.py`: `AirStack/.venv` carries Isaac Sim
5.1, and `.env.host` supplies OMNI_KIT_ACCEPT_EULA.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS)
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

#: Drop heights, metres. Chosen to bracket the regime the tornado lab runs in
#: (a 6.5 m house) and the one the scene-wide path runs in (a 66 m tower).
HEIGHTS = (1.0, 5.0, 15.0, 40.0, 70.0)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--heights", default=",".join(str(h) for h in HEIGHTS))
    p.add_argument("--steps", type=int, default=2000)
    p.add_argument("--size", type=float, default=1.0, help="cube edge, metres")
    p.add_argument("--cpu", action="store_true", help="CPU dynamics")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    heights = [float(h) for h in args.heights.split(",") if h.strip()]

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})

    import omni.kit.app                                         # noqa: E402
    import omni.usd                                             # noqa: E402
    from pxr import Gf, Sdf, UsdGeom                            # noqa: E402

    from disaster import settle                                 # noqa: E402

    half = float(args.size) * 0.5
    print("\n[floor_probe] cube edge %.2f m -> resting centre should be "
          "%.2f m if the floor is real\n" % (args.size, half), flush=True)
    print("%8s %10s %12s %10s %10s  %s" % (
        "drop_m", "impact_m/s", "per_step_m", "rest_z", "moved", "verdict"),
        flush=True)

    rows = []
    for h in heights:
        # A NEW CONTEXT STAGE PER DROP. `settle` drives `SimulationContext`,
        # which attaches PhysX to whatever stage the USD context holds — a
        # standalone `Usd.Stage.CreateNew` is not that stage.
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        # A MESH, NOT A `UsdGeom.Cube`. `settle._apply_collider` skips
        # anything that is not `IsA(UsdGeom.Mesh)`, so a Cube gets no
        # collider, no rigid body, and `settle.run` returns "nothing to
        # settle" having simulated precisely nothing — which looks exactly
        # like a body that fell through the floor if you only read its final
        # z. That false negative is what the first version of this probe
        # reported.
        cube = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/box"))
        h2 = float(args.size) * 0.5
        corners = [(-h2, -h2, -h2), (h2, -h2, -h2), (h2, h2, -h2),
                   (-h2, h2, -h2), (-h2, -h2, h2), (h2, -h2, h2),
                   (h2, h2, h2), (-h2, h2, h2)]
        quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        cube.CreatePointsAttr([Gf.Vec3f(*c) for c in corners])
        cube.CreateFaceVertexCountsAttr([4] * 6)
        cube.CreateFaceVertexIndicesAttr([i for q in quads for i in q])
        cube.CreateExtentAttr([Gf.Vec3f(-h2, -h2, -h2), Gf.Vec3f(h2, h2, h2)])
        op = UsdGeom.Xformable(cube).AddTranslateOp()
        op.Set(Gf.Vec3d(0.0, 0.0, float(h)))
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        for _ in range(4):
            omni.kit.app.get_app().update()

        # No kick, no blast: gravity and the floor, nothing else.
        info = settle.run(stage, ["/World/box"], [], steps=int(args.steps),
                          kick=0.0, blast=0.0, gpu=not args.cpu,
                          settle_note=False, bake_result=True)

        pos = UsdGeom.Xformable(stage.GetPrimAtPath("/World/box"))
        rest = None
        for o in pos.GetOrderedXformOps():
            if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                rest = float(o.Get()[2])
        speed = math.sqrt(2.0 * 9.81 * h)
        per_step = speed / 60.0
        # A run with no bodies proves nothing either way — say so rather
        # than scoring it as a fall.
        n_bodies = int(info.get("bodies") or 0) if not isinstance(
            info.get("bodies"), list) else len(info["bodies"])
        ok = (n_bodies > 0 and rest is not None
              and abs(rest - half) < 0.25)
        if n_bodies <= 0:
            verdict = "NO BODY — nothing simulated, result meaningless"
        elif ok:
            verdict = "floor held"
        elif rest is not None:
            verdict = "FELL THROUGH (%.1f m below rest)" % (half - rest)
        else:
            verdict = "no transform"
        print("%8.1f %10.1f %12.3f %10.2f %10s  %s" % (
            h, speed, per_step, rest if rest is not None else float("nan"),
            info.get("still_moving", "?"), verdict), flush=True)
        rows.append((h, rest, ok, n_bodies))

    if any(n <= 0 for _h, _r, _ok, n in rows):
        print("\n[floor_probe] ABORT: some drops simulated no body at all "
              "(see NO BODY above). Fix the probe before reading a verdict.")
        app.close()
        return 2
    held = [h for h, _r, ok, _n in rows if ok]
    fell = [h for h, _r, ok, _n in rows if not ok]
    print("\n[floor_probe] held from: %s" % (held or "nothing"))
    print("[floor_probe] fell from: %s" % (fell or "nothing"))
    if not held and fell:
        print("[floor_probe] VERDICT: the floor is INERT — nothing collides "
              "with it at any speed. A box floor is the fix; CCD is not.")
    elif held and not fell:
        print("[floor_probe] VERDICT: the floor COLLIDES at every speed "
              "tested. Whatever puts fragments underground is not this.")
    elif held and fell:
        print("[floor_probe] VERDICT: TUNNELLING, threshold between %.1f m "
              "and %.1f m of drop. CCD is the fix." % (max(held), min(fell)))
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
