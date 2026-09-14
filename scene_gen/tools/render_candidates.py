#!/usr/bin/env python
"""render_candidates.py — thumbnail every candidate `match_assets.py` proposed.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/render_candidates.py \
        --candidates scene_gen/_plans/dc_candidates.json \
        --root omniverse://.../Projects/SEI-COA/ \
        --out scene_gen/_plans/candidates/

The matcher ranks on a name-derived tag set and a bounding box, which is enough
to shortlist and never enough to choose: two 20 x 17 m things tagged `building`
can be a clapboard house and a burnt-out shell. This is the step that closes
that gap — one image per candidate, contact-sheeted per query, so the pick is
made by looking.

RUNS ON THE HOST, in Kit. Blender is the faster renderer and cannot be used:
its USD importer has no Omniverse resolver, so every `omniverse://` reference
comes in empty. Kit resolves them, and the repo venv carries a full Isaac Sim
install, so no container is involved.

Each asset is opened on its OWN stage and framed off its own bounding box —
assets in this library range from a 0.5 m hydrant to a 45 m warehouse, and one
fixed camera would render most of them as a dot or a wall.
"""

import argparse
import json
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--candidates", required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--res", type=int, default=420)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True,
                                       "width": args.res, "height": args.res})
    import omni.kit.app
    import omni.usd
    from pxr import Usd, UsdGeom, Gf, UsdLux

    import importlib.util as ilu
    sp = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "..", "simulation", "isaac-sim", "utils", "snapshots.py")
    sp = os.path.normpath(sp)
    spec = ilu.spec_from_file_location("snapshots", sp)
    snaps = ilu.module_from_spec(spec)
    spec.loader.exec_module(snaps)

    cands = json.load(open(args.candidates))
    uniq = {}
    for q, lst in cands.items():
        for c in lst:
            uniq.setdefault(c["path"], c)
    paths = list(uniq)
    if args.limit:
        paths = paths[:args.limit]
    os.makedirs(args.out, exist_ok=True)
    root = args.root if args.root.endswith("/") else args.root + "/"
    print(f"[cand] {len(paths)} unique assets", flush=True)

    ctx = omni.usd.get_context()
    done, failed = 0, []
    for i, rel in enumerate(paths):
        png = os.path.join(args.out, rel.replace("/", "__") + ".png")
        if os.path.exists(png):
            continue
        try:
            ctx.new_stage()
            stage = ctx.get_stage()
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)
            ref = UsdGeom.Xform.Define(stage, "/World/asset")
            # A reference targets the layer's `defaultPrim`, and some packs
            # root at a bare Mesh. Composing that onto an Xform brings the
            # mesh's ATTRIBUTES and no geometry -- empty bound, black frame,
            # no error. Hanging it on a TYPELESS child instead lets the
            # referenced type win. Every FactoryDistrict building is authored
            # this way and all eleven rendered as "empty bound" before this.
            sub = Usd.Stage.Open(root + rel)
            dp = sub.GetDefaultPrim() if sub is not None else None
            if dp and dp.IsA(UsdGeom.Gprim):
                stage.DefinePrim("/World/asset/geo").GetReferences(
                    ).AddReference(root + rel)
            else:
                ref.GetPrim().GetReferences().AddReference(root + rel)
            # A dome light, because half these packs ship no lighting at all
            # and an unlit capture is a black square that says nothing.
            UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1200.0)
            for _ in range(30):
                omni.kit.app.get_app().update()
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_], useExtentsHint=True)
            r = bc.ComputeWorldBound(ref.GetPrim()).ComputeAlignedRange()
            if r.IsEmpty():
                failed.append((rel, "empty bound"))
                continue
            mn, mx = r.GetMin(), r.GetMax()
            c = [(mn[k] + mx[k]) / 2.0 for k in range(3)]
            span = max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], 1.0)
            snaps.place_camera(stage,
                               (c[0] + span * 1.1, c[1] - span * 1.1,
                                c[2] + span * 0.8),
                               (c[0], c[1], c[2]))
            snaps.snapshot(png, frames=18)
            done += 1
            if done % 10 == 0:
                print(f"[cand] {done}/{len(paths)}", flush=True)
        except Exception as exc:
            failed.append((rel, str(exc)[:90]))
    print(f"[cand] rendered {done}, failed {len(failed)} -> {args.out}", flush=True)
    for rel, why in failed[:15]:
        print(f"   FAIL {rel}: {why}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
