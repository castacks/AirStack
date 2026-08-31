#!/usr/bin/env python
"""fake_interior_census — HOW MANY BREAK FRAGMENTS IN A SHIPPED BAKE ARE
WEARING AN INTERIOR MATERIAL?

    "For the accidental interior material that was applied to the building.
     Let's make sure that NEVER happens to any building."   (user, row 5)

`fire_collapse.is_fake_interior` is the single predicate every chooser in
that module consults, and `fire_collapse.fake_interior_audit` reports it into
the notes of each new bake. This is the same question asked of bakes that
already exist on disk — the fleet check the audit cannot do retroactively.

Per USD given (a directory expands to its `*.usd`), for every `brk_*/frag_*`
Mesh: the material bound on the PRIM (the façade half — the `core` subset is
the char and is not a façade choice), matched by prim path AND base-map url.
Prints per file: the fragment count, the offender count, and the offending
materials by name.

    usd_python.sh fake_interior_census.py /isaac-sim/.cache/fire_bakes_dtc
    usd_python.sh fake_interior_census.py a.usd b.usd

READ-ONLY: stages are opened, never saved.
"""
import glob
import os
import sys
from collections import Counter

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
from disaster import fire_collapse as fc                      # noqa: E402


def basecolor(stage, mpath):
    """The base-colour url a material by PATH feeds, or None. By path, never
    by handle: a material handed out inside a traversal expires."""
    mp = stage.GetPrimAtPath(mpath) if mpath else None
    if not mp or not mp.IsValid():
        return None
    for c in mp.GetChildren():
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for nm in ("diffuseColor", "diffuse_texture", "base_color"):
            i = sh.GetInput(nm)
            if i is None:
                continue
            try:
                if i.HasConnectedSource():
                    src = i.GetConnectedSource()[0].GetPrim()
                    f = UsdShade.Shader(src).GetInput("file")
                    v = f.Get() if f else None
                    return str(v)
                v = i.Get()
                if v is not None and nm == "diffuse_texture":
                    return str(v)
            except Exception:
                continue
    return None


def census(path):
    stage = Usd.Stage.Open(path)
    if stage is None:
        print("%-46s  (could not open)" % os.path.basename(path))
        return 0, 0
    rows = []
    for pr in stage.Traverse():
        p = pr.GetPath().pathString
        if "/brk_" not in p or not pr.IsA(UsdGeom.Mesh):
            continue
        try:
            m = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
        except Exception:
            continue
        rows.append((p, str(m.GetPrim().GetPath())
                     if (m and m.GetPrim().IsValid()) else None))
    bad = Counter()
    for p, mp in rows:
        if mp and fc.is_fake_interior(mp, basecolor(stage, mp)):
            bad[mp.rsplit("/", 1)[-1]] += 1
    n_bad = sum(bad.values())
    print("%-46s  %5d frag(s)  %5d on a fake-interior material%s"
          % (os.path.basename(path), len(rows), n_bad,
             ("   <-- " + ", ".join("%s x%d" % kv for kv in bad.most_common(4)))
             if n_bad else ""))
    return len(rows), n_bad


args = sys.argv[1:] or ["/isaac-sim/.cache/fire_bakes_dtc"]
files = []
for a in args:
    files.extend(sorted(glob.glob(os.path.join(a, "*.usd")))
                 if os.path.isdir(a) else [a])
print("[fake_interior_census] predicate: %s" % (fc.FAKE_INTERIOR_HINTS,))
tot = bad = 0
for f in files:
    a, b = census(f)
    tot += a
    bad += b
print("[fake_interior_census] %d file(s), %d break fragment(s), %d on a "
      "fake-interior material (0 wanted)" % (len(files), tot, bad))
