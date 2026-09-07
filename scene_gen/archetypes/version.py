"""version — what pipeline built an archetype, and whether it is still current.

An archetype is a COOKED artifact. It carries no trace of the code that cut
it, so a library baked before a fix and one baked after are the same shape on
disk and behave differently in a scene. `QUAKE_STATE.md` records two of these
already: everything baked before `c1f0e5b0` was cut hollow, and everything
baked before the debris pass stands on a clean lot. Both were found by
remembering, months later, that a re-bake was owed — which is not a mechanism.

So every record says WHEN it was baked and BY WHAT:

    "baked_at":            "2026-08-29T18:42:10-04:00"
    "pipeline_version":    "4"
    "pipeline_fingerprint": "a3f19c2b7e04"

TWO FIELDS, BECAUSE THEY FAIL DIFFERENTLY
-----------------------------------------
`pipeline_version` is DECLARED. A human bumps it when the pipeline changes in
a way that invalidates what is already baked, and writes down why in
`CHANGES`. It is the field worth reading: it says what changed and whether a
re-bake is owed.

`pipeline_fingerprint` is MEASURED — a hash over the sources that actually
decide an archetype's geometry. It exists because the declared version depends
on somebody remembering to bump it, and the whole reason this module exists is
that people do not. A stale library whose author forgot the bump still reads
as stale here, because the bytes moved.

Neither alone is enough. The fingerprint changes on a comment edit, so it
cannot be the thing a human reasons about; the version does not change at all
unless someone acts, so it cannot be the thing a script trusts.

WHAT IS FINGERPRINTED
---------------------
`SOURCES` — the modules whose behaviour is baked INTO the USD. A change to
`disaster/mesh_damage.py` moves every fragment; a change to
`archetypes/library.py` (naming, manifest layout) does not touch geometry and
is deliberately absent, so re-organising the read side does not invalidate a
30 GB library. `preview.py` and `census.py` are absent for the same reason —
they produce pictures and bookkeeping, not fragments. Verified in practice: an
edit that retuned the preview camera mid-bake left the fingerprint at
`4feecda11b23`, which is exactly what should happen.

FREEZE `SOURCES` FOR THE DURATION OF A BAKE
-------------------------------------------
A full library is hours, `write_manifest` stamps each record as its cell lands,
and the stamp is read at that moment — so editing any file in `SOURCES` while a
bake runs gives you one library with two fingerprints in it, and `audit` will
then call the first half stale. The running process does not pick the edit up
either (Python imported the module at startup), so the split is pure
bookkeeping damage with no upside. Edit freely OUTSIDE that list; wait for
anything inside it.
"""

from __future__ import annotations

import datetime
import hashlib
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)

#: Bump when a change makes already-baked archetypes WRONG rather than merely
#: different, and add a line to `CHANGES`.
PIPELINE_VERSION = "7"

#: Why each version exists — the re-bake log, newest first.
CHANGES = (
    ("7", "2026-08-30", "Interior structure is kept SEPARATE from the rubble. "
                        "`fill_interior` binds its floors and columns to a new "
                        "`InteriorStructure_<kind>` material instead of the "
                        "shared fracture core, and `bake._merge_fragments` "
                        "emits those faces as `interior_<kind>` rather than "
                        "folding them into `rubble_<core>`. GEOMETRY IS "
                        "UNCHANGED — same cut, same seeds; what changes is "
                        "which mesh a face ends up in, so a finished archetype "
                        "still has a separable interior. Before this, an "
                        "archetype whose interior was fully cut (measured: "
                        "BG_Building_C partial_collapse, slab_tower, "
                        "office_tower) had no interior prim left at all."),
    ("6", "2026-08-30", "VTK fracture backend (`mesh_damage.FRACTURE_BACKEND`, "
                        "default `auto`). PERFORMANCE ONLY — the cut is the "
                        "same seeds, same bisectors, and the partition is "
                        "verified identical (99.1% of sample points in exactly "
                        "one cell, 0% in none, same as numpy). Measured 5-7.5x "
                        "faster on larger meshes and ~2.5x fewer output points. "
                        "**THE v5 ARCHETYPES ARE DELIBERATELY KEPT AND MUST NOT "
                        "BE RE-BAKED** (user direction): they are correct, and "
                        "this library is EXPECTED to hold both backends. Read "
                        "`fracture_backend` on a record to tell which cut it; "
                        "a v5 record reading 'stale' here means 'cut by the "
                        "numpy clipper', not 'redo it'."),
    ("5", "2026-08-30", "Three fixes found by the first real bake. (a) "
                        "`disaster/bake._portable_asset` had no `from pxr "
                        "import Sdf`, so EVERY damaged rung failed to export "
                        "— a v4 library is pristine shells only. (b) A cell "
                        "is no longer rejected for a handful of fragments "
                        "that sank: they are deleted and the wreck is kept "
                        "(13% of structure cells were being thrown away for "
                        "one free-falling piece in 144-455). (c) Earthquake "
                        "no longer has a vegetation ladder — shaking does not "
                        "fell trees — so its 18 tree archetypes are gone. "
                        "STRUCTURE archetypes are affected by (a) and (b) and "
                        "must be re-baked; nothing carried over."),
    ("4", "2026-08-29", "Archetype library build-out: bake provenance "
                        "(this module), per-rung previews, and the scene "
                        "asset census. First library baked with debris AND "
                        "the c1f0e5b0 interior fix, so everything before it "
                        "is invalid — hollow wrecks on clean lots."),
    ("3", "2026-08-26", "Debris baked into the cell; archetypes instanced."),
    ("2", "2026-08-25", "c1f0e5b0: measure the interior once, before "
                        "solidify. Earlier libraries were cut hollow."),
    ("1", "", "Before any of this was recorded."),
)

#: Sources whose behaviour ends up in the exported geometry. Relative to
#: `scene_gen/`. Missing files are skipped rather than fatal — a pack that
#: does not use the modular kit should not fail to fingerprint because
#: `damage_flow.py` was moved.
SOURCES = (
    "archetypes/bake.py",
    "archetypes/plan.py",
    "disaster/authoring.py",
    "disaster/bake.py",
    "disaster/damage.py",
    "disaster/damage_flow.py",
    "disaster/debris.py",
    "disaster/field.py",
    "disaster/kinds.py",
    "disaster/levels.py",
    "disaster/mesh_damage.py",
    "disaster/quake.py",
    "disaster/rubble.py",
    "disaster/settle.py",
    "disaster/solids.py",
    "disaster/source.py",
    "disaster/survey.py",
    "disaster/tornado.py",
    "disaster/vegetation.py",
    "disaster/vtk_fracture.py",
)


def source_fingerprint(scene_gen_dir: str = "") -> str:
    """12 hex chars over `SOURCES`, in listed order.

    The PATH is hashed alongside the bytes so that moving a file, or dropping
    one out of the list, changes the fingerprint too — otherwise deleting a
    module that contributed nothing to the last hash would be invisible.
    """
    root = scene_gen_dir or _SCENE_GEN
    h = hashlib.sha256()
    for rel in SOURCES:
        path = os.path.join(root, rel)
        if not os.path.isfile(path):
            continue
        h.update(rel.encode())
        with open(path, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()[:12]


def stamp(scene_gen_dir: str = "") -> dict:
    """The provenance fields to merge into a manifest record."""
    return {
        "baked_at": datetime.datetime.now().astimezone().isoformat(
            timespec="seconds"),
        "pipeline_version": PIPELINE_VERSION,
        "pipeline_fingerprint": source_fingerprint(scene_gen_dir),
    }


def is_stale(record: dict, scene_gen_dir: str = "") -> bool:
    """True when *record* was baked by a pipeline that is not this one.

    A record with NO provenance is stale by definition: it predates this
    module, so it predates every fix listed in `CHANGES`.
    """
    rec = record or {}
    if not rec.get("pipeline_fingerprint"):
        return True
    return (str(rec.get("pipeline_version")) != PIPELINE_VERSION
            or str(rec.get("pipeline_fingerprint"))
            != source_fingerprint(scene_gen_dir))


def audit(manifest: dict, scene_gen_dir: str = "") -> dict:
    """Split a manifest's records into current and stale."""
    recs = (manifest or {}).get("archetypes") or []
    fp = source_fingerprint(scene_gen_dir)
    fresh = [r for r in recs
             if str(r.get("pipeline_fingerprint")) == fp
             and str(r.get("pipeline_version")) == PIPELINE_VERSION]
    stale = [r for r in recs if r not in fresh]
    by_fp = {}
    for r in stale:
        key = (str(r.get("pipeline_version") or "?"),
               str(r.get("pipeline_fingerprint") or "none"))
        by_fp[key] = by_fp.get(key, 0) + 1
    return {"current": fresh, "stale": stale, "fingerprint": fp,
            "stale_by_pipeline": by_fp}


def main():
    import argparse
    import json

    ap = argparse.ArgumentParser(
        description="Report which archetypes in a library are stale.")
    ap.add_argument("manifest", nargs="?",
                    help="path to a library manifest.json")
    ap.add_argument("--list", action="store_true",
                    help="name every stale archetype")
    args = ap.parse_args()

    print(f"pipeline version     : {PIPELINE_VERSION}")
    print(f"source fingerprint   : {source_fingerprint()}")
    if not args.manifest:
        print("\nchanges:")
        for v, when, why in CHANGES:
            print(f"  {v:<3} {when:<11} {why}")
        return

    with open(args.manifest) as fh:
        doc = json.load(fh)
    res = audit(doc)
    print(f"\n{args.manifest}")
    print(f"  {len(res['current'])} current, {len(res['stale'])} stale")
    for (ver, fp), n in sorted(res["stale_by_pipeline"].items()):
        print(f"    {n:4d} baked by version {ver} / {fp}")
    if args.list:
        for r in sorted(res["stale"], key=lambda r: (r.get("type", ""),
                                                     r.get("level", ""))):
            print(f"      {r.get('type')}_{r.get('level')}")


if __name__ == "__main__":
    main()
