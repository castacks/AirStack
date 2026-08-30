#!/usr/bin/env python
"""_roof_seat_probe -- BEFORE/AFTER `fire_bake._judge_candidates` counts for
the `deck_z` roof-seating fix, per debris family, on a real `burn_gac` run.

WHY. Row-2 review, 2026-08-30: "Lots of floating roof props and also
floating debris in B7, B6, B5, B4" (SM_Building_09 F6, SM_Building_05 F5,
SM_Building_23 F4, SM_Building_12 F3). The fix is `gac_fire.mass_from_grid`
measuring the real roof plateau as `deck_z` and `urban_fire.dress_roof_urban`
/ `_deck_slab` / `_rafter_teeth` / `r_roof_scorch` seating everything on it
instead of on `m["top"]` (the parapet coping); `fire_bake.deactivate_
airborne`'s family cap was also narrowed so it no longer hides a genuinely
airborne `frag`/roof-prop family the way it hid 35/121 unsupported roof-lid
fragments on SM_Building_23 F4.

METHOD. Runs `gac_fire.burn_gac` (same shape as `gac_burn_probe.py`) twice
in FRESH SUBPROCESSES of this same interpreter -- never an in-process
`sys.path` swap, because `disaster`/`detail` are regular packages this
script would otherwise leave half-cached under their old `sys.modules`
entries:

  * BEFORE: `sys.path` points at `/tmp/sg_orig` inside the container, a copy
    of the whole live `scene_gen` tree (minus `assets/`, 11 GB of Nucleus
    references nothing here needs) with `gac_fire.py`, `urban_fire.py` and
    `fire_bake.py` swapped for the PRE-FIX snapshots at
    `/tmp/agent_gac_roof_orig/*.py.orig` (put there by `docker cp` from the
    agent's scratchpad before any edit was made -- see the skill/task note
    this file was written under). Built once, cached across runs.
  * AFTER: `sys.path` points straight at the live tree
    (`/isaac-sim/AirStack/scene_gen`).

Each subprocess places, plans, bakes and slices the building exactly like
`gac_burn_probe.py`, then calls `fire_bake._judge_candidates(stage, cell)`
on the resulting in-memory stage and prints one `RESULT {json}` line this
script parses back out -- `{prefix: [n_judged, n_unsupported]}`.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/_roof_seat_probe.py SM_Building_03 F1"

With no arguments, runs the four combinations the task named:
SM_Building_03/F1, SM_Building_12/F3, SM_Building_23/F4, SM_Building_09/F6.
"""
import json
import os
import shutil
import subprocess
import sys
import time

LIVE_ROOT = "/isaac-sim/AirStack/scene_gen"
ORIG_ROOT = "/tmp/sg_orig"
ORIG_SNAPSHOT_DIR = "/tmp/agent_gac_roof_orig"   # docker cp'd from the host
                                                 # scratchpad .orig files
ORIG_FILES = {
    "disaster/gac_fire.py": "gac_fire.py.orig",
    "disaster/urban_fire.py": "urban_fire.py.orig",
    "disaster/fire_bake.py": "fire_bake.py.orig",
}
DEFAULT_CASES = (("SM_Building_03", "F1"), ("SM_Building_12", "F3"),
                 ("SM_Building_23", "F4"), ("SM_Building_09", "F6"))
SEED = 7

_PHASE_SCRIPT = r'''
import json, random, sys
sys.path.insert(0, {root!r})
import numpy as np
from pxr import Usd, UsdGeom
from detail import gac_storey_slice as gss
from disaster import fracture, gac_fire as gf, urban_fire as uf, fire_bake as fb

name, level, seed = {name!r}, {level!r}, {seed}
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/W/bench")
rng = random.Random(seed)
nrng = np.random.default_rng(seed)
# ONE GAC SLICE AT A TIME, MACHINE-WIDE (`gss.slice_lock`) -- these buildings
# have no baked kit yet (`kit_bake.have_kit` is False for all of them), so
# `burn_gac` always takes the live VTK slice path here, exactly the
# multi-gigabyte-working-set case the lock exists for.
with gss.slice_lock(verbose=True):
    ctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, "g0",
                      flow_root=None, mat_cache={{}}, ssf=1.0, verbose=False)
info = fb._judge_candidates(st, cell)
by = {{}}
for j in info["judged"]:
    d = by.setdefault(j["prefix"], [0, 0])
    d[0] += 1
    d[1] += 1 if j["deactivate"] else 0
print("RESULT " + json.dumps({{"by": by, "n_meshes": info["n_meshes"],
                              "n_pieces": ctx["gac"]["n_pieces"]}}))
'''


def _ensure_orig_tree():
    """Build `/tmp/sg_orig` once: the whole live `scene_gen` tree (`disaster`/
    `detail` alone is not enough -- `quake_flow.materials` reaches for the
    top-level `scene_generator` module, and that kind of sideways import is
    not worth enumerating by hand) MINUS `assets/` (11 GB of Nucleus-side
    references this needs none of; everything else is ~50 MB), with the
    three edited files swapped for their pre-fix snapshots."""
    if os.path.isdir(os.path.join(ORIG_ROOT, "disaster")):
        return
    if os.path.isdir(ORIG_ROOT):
        shutil.rmtree(ORIG_ROOT)
    shutil.copytree(LIVE_ROOT, ORIG_ROOT,
                    ignore=shutil.ignore_patterns("assets", "__pycache__"))
    for rel, snap in ORIG_FILES.items():
        src = os.path.join(ORIG_SNAPSHOT_DIR, snap)
        if not os.path.isfile(src):
            raise RuntimeError(
                "missing pre-fix snapshot {0} -- docker cp the agent's "
                "scratchpad .orig files to {1} first".format(src, ORIG_SNAPSHOT_DIR))
        shutil.copyfile(src, os.path.join(ORIG_ROOT, rel))


def _run_phase(root, name, level, seed=SEED):
    snippet = _PHASE_SCRIPT.format(root=root, name=name, level=level, seed=seed)
    try:
        out = subprocess.check_output([sys.executable, "-c", snippet],
                                      stderr=subprocess.STDOUT, cwd=root,
                                      timeout=600)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError("phase subprocess failed:\n" +
                           exc.output.decode(errors="replace")[-4000:])
    text = out.decode(errors="replace")
    lines = [ln for ln in text.splitlines() if ln.startswith("RESULT ")]
    if not lines:
        raise RuntimeError("phase produced no RESULT line:\n" + text[-4000:])
    return json.loads(lines[-1][len("RESULT "):])


# families this fix targets landing on the roof / a burn-through hole --
# printed first and in this order regardless of what else showed up
ROOF_FAMILIES = ("frag", "rafter", "joist", "bulkhead", "bulkcap", "acpad",
                 "vent", "tank", "fireheap", "heap", "rdeb")


def run_case(name, level):
    print("\n=== {0} {1} (seed {2}) ===".format(name, level, SEED))
    before = _run_phase(ORIG_ROOT, name, level)
    after = _run_phase(LIVE_ROOT, name, level)
    keys = list(ROOF_FAMILIES) + sorted(
        (set(before["by"]) | set(after["by"])) - set(ROOF_FAMILIES))
    print("{0:<12} {1:>18} {2:>18}".format("family", "before (unsup/n)",
                                           "after (unsup/n)"))
    for k in keys:
        nb, gb = before["by"].get(k, [0, 0])
        na, ga = after["by"].get(k, [0, 0])
        if nb == 0 and na == 0:
            continue
        print("{0:<12} {1:>18} {2:>18}".format(
            k, "{0}/{1}".format(gb, nb), "{0}/{1}".format(ga, na)))
    print("n_pieces before={0} after={1}".format(before["n_pieces"],
                                                 after["n_pieces"]))


if __name__ == "__main__":
    _ensure_orig_tree()
    t0 = time.time()
    if len(sys.argv) >= 3:
        cases = [(sys.argv[1], sys.argv[2])]
    else:
        cases = DEFAULT_CASES
    for name, level in cases:
        try:
            run_case(name, level)
        except Exception as exc:
            print("{0} {1} FAILED: {2}".format(name, level, exc))
    print("\n[_roof_seat_probe] {0:.0f}s total".format(time.time() - t0))
