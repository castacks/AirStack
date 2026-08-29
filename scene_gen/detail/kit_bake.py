"""kit_bake — load a GreatAmericanCity kit that was sliced ONCE and saved,
instead of re-slicing it every launch.

`gac_storey_slice.slice_to_kit` is deterministic for a given asset and slicer
version: measure the window grid, clip a VTK plane stack per storey per ring
cell, write the pieces. That is most of a bench's build time (user,
2026-08-29: "save those kitbashes locally... so we don't have to break them
every time and can just assemble"), so `tools/bake_gac_kits.py` does it once
per asset and writes the result to `scene_gen/assets/kits/<asset>.usd`, plus
one record per asset in `scene_gen/assets/kits/kits.json`. This module is the
other half: reference that file back in and hand a caller EXACTLY what
`slice_to_kit` would have — `load_kit` returns the same
`(placements, grid, measured)` shape, on purpose, so a launcher can swap

    pls, g, meas = gss.slice_to_kit(stage, src, cell, style, verbose=False)

for

    pls, g, meas = kb.load_kit(stage, cell, name, ssf)

on one line, falling back to the live path with `have_kit(name)`.

THE MATERIAL TRAP THIS DEPENDS ON THE BAKER HAVING ALREADY SOLVED
-------------------------------------------------------------------
A live `slice_to_kit` binds every sliced subset to a material prim living
under the SOURCE building's own subtree, and merely makes that subtree
invisible — it is still composed, which is why materials render fine in every
launcher today. A baked kit has no source subtree at all, so the bake had to
REHOME every material onto a prim of its own before it could drop the source
(`gac_slice.rehome_materials`, driven from `bake_gac_kits.py` — see that
file's docstring). This module trusts that the `.usd` it references is
already self-contained that way; it does no material surgery of its own.

MANIFEST SHAPE. `kits.json` is a JSON list, one record per asset, each with:
`asset` (the manifest key = the name `have_kit`/`load_kit` take), `usd` (the
baked file's absolute path), `src_usd`, `fingerprint`, `grid` (exactly what
`gac_storey_slice.grid_for` returns — `storey_h`, `storeys`, `bays`, `W`, `D`,
`H`, `z0`, `bbox`, `measured` — all of it plain JSON), and `pieces` (one dict
per placement, with a `name` field — the piece's bare prim name under the
baked file's `/Kit/pieces` scope — replacing the live path's `prim_path`,
which cannot be known until this loader decides where to reference the file).

STALENESS. `fingerprint()` hashes `gac_storey_slice.py` + `gac_slice.py` (byte
for byte) plus `CUT_OFFSET`, so ANY change to the slicing grammar — the
piece-budget rework in flight alongside this file included — changes it.
`have_kit` refuses a bake whose recorded fingerprint no longer matches; a
caller that reaches `load_kit` on a stale entry directly gets pieces that are
almost certainly still fine (the parapet/ring math does not change what a
building's WALLS look like) but a loud warning either way, because "almost
certainly fine" is not the same claim `have_kit` makes.
"""

import json
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
KIT_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "assets", "kits"))
MANIFEST_PATH = os.path.join(KIT_DIR, "kits.json")
_GSS_PATH = os.path.join(_THIS_DIR, "gac_storey_slice.py")
_GSL_PATH = os.path.join(_THIS_DIR, "gac_slice.py")

# `slice_to_kit`'s own default (`offset=0.5`, the fraction of a storey the cut
# lattice is shifted off the window centres). Shared between the baker and
# `fingerprint()` so the two never drift apart — a baker that ever wants a
# different offset should import this name and change it here, not pass a
# bare literal, or a stale bake at the old offset would still look "fresh".
CUT_OFFSET = 0.5


# ---------------------------------------------------------------------------
# Manifest I/O — no `pxr` import anywhere below, so this half works on the
# host with no Kit/USD environment at all (`have_kit`, a size report, ...).
# ---------------------------------------------------------------------------
def fingerprint():
    """A hash of everything that decides what a bake CONTAINS.

    Hashing the two slicer files byte-for-byte, rather than pattern-matching
    for a version string, means this needs no cooperation from whoever edits
    them next — the piece-budget rework happening in `gac_storey_slice.py`
    alongside this file changes the hash the moment it lands, with nothing
    to remember to bump.
    """
    import hashlib

    h = hashlib.sha1()
    for p in (_GSS_PATH, _GSL_PATH):
        try:
            with open(p, "rb") as fh:
                h.update(fh.read())
        except OSError:
            h.update(("MISSING:" + p).encode("utf-8"))
    h.update(repr(("offset", CUT_OFFSET)).encode("utf-8"))
    return h.hexdigest()[:16]


def read_manifest():
    if not os.path.exists(MANIFEST_PATH):
        return []
    with open(MANIFEST_PATH) as fh:
        return json.load(fh)


def write_manifest(records):
    os.makedirs(KIT_DIR, exist_ok=True)
    with open(MANIFEST_PATH, "w") as fh:
        json.dump(records, fh, indent=1, sort_keys=True)


def merge_manifest(records):
    """Read-merge-write `records` into `kits.json` under a file lock, keyed
    by `asset` — so re-baking one asset does not drop the others, and two
    bakes finishing together (mirroring `bake_quake_headless.sh`) cannot lose
    each other's rows either. Returns the merged list.
    """
    import fcntl

    os.makedirs(KIT_DIR, exist_ok=True)
    with open(MANIFEST_PATH + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            old = read_manifest()
            fresh = set(r.get("asset") for r in records)
            merged = [r for r in old if r.get("asset") not in fresh] + list(records)
            write_manifest(merged)
        finally:
            fcntl.flock(lk, fcntl.LOCK_UN)
    return merged


def _entry(name):
    for r in read_manifest():
        if r.get("asset") == name:
            return r
    return None


def have_kit(name):
    """Is there a bake for `name` that is both PRESENT and CURRENT?

    False covers three different reasons a caller should fall back to a live
    `slice_to_kit` — no manifest row, a row whose `.usd` is missing on disk,
    or a row baked against a slicer that has since changed — on purpose:
    telling them apart buys the caller nothing, since the fallback is the
    same either way.
    """
    e = _entry(name)
    if not e:
        return False
    if not e.get("usd") or not os.path.exists(e["usd"]):
        return False
    return e.get("fingerprint") == fingerprint()


# ---------------------------------------------------------------------------
# The loader
# ---------------------------------------------------------------------------
def load_kit(stage, cell, name, ssf=1.0):
    """Reference the baked kit `name` under `cell` and return kit placements.

    `cell` is assumed to already exist on `stage` (an Xform the caller
    positioned, exactly the precondition `slice_to_kit(stage, src, cell,
    ...)` has today — this never creates `cell` itself, only a child of it).
    The bake is referenced onto `cell + "/kit"` — a dedicated child, the same
    idiom `place_asset`/`place_source` use to isolate a reference's own
    transform from the cell's (see `fire_pack_rows_launch_script.place_asset`)
    — so a piece's final path is `cell/kit/pieces/<name>`.

    `ssf` scales that reference (only if it differs from 1.0, the identity
    every current caller passes): the bake was authored at
    `metersPerUnit=1.0` like every other slicer probe in this repo, and
    `slice_to_kit` itself takes no such factor because its callers always set
    their working stage to 1.0 too — this parameter exists so a FUTURE caller
    on a stage in different units is not stuck re-slicing.

    Returns `(placements, grid, measured)` — the same shape `slice_to_kit`
    returns. `grid["measured"]` is `measured` again, redundantly: `slice_to_kit`
    folds it into the grid dict it returns AND returns it separately, and this
    keeps that same double return rather than inventing a different contract.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: F401  (UsdShade: doc parity)
    from detail import gac_slice as gsl
    from detail import gac_storey_slice as gss

    e = _entry(name)
    if e is None:
        raise KeyError(
            "no baked kit named {0!r} in {1} — bake it first "
            "(scene_gen/tools/bake_gac_kits.py --assets {0}) or fall back to "
            "a live slice_to_kit".format(name, MANIFEST_PATH))
    usd_path = e.get("usd") or ""
    if not os.path.exists(usd_path):
        raise IOError(
            "kits.json names {0} for {1!r} but that file is not on disk — "
            "re-run bake_gac_kits.py".format(usd_path, name))
    fp = fingerprint()
    if e.get("fingerprint") != fp:
        print("[kit_bake] WARNING: baked kit {0!r} predates the current "
              "slicer (bake {1}, current {2}) — the geometry on disk is from "
              "an older grammar; re-bake when convenient, `have_kit` already "
              "told you not to trust this silently".format(
                  name, e.get("fingerprint"), fp))

    kid_path = cell.rstrip("/") + "/kit"
    kid = stage.DefinePrim(Sdf.Path(kid_path))
    if not kid.GetReferences().AddReference(usd_path):
        raise IOError("failed to reference baked kit {0} at {1}".format(
            usd_path, kid_path))
    stage.Load(Sdf.Path(kid_path))
    ssf = float(ssf)
    if abs(ssf - 1.0) > 1e-9:
        UsdGeom.Xformable(kid).AddScaleOp().Set(Gf.Vec3f(ssf, ssf, ssf))

    pieces_scope = kid_path + "/pieces"
    placements = []
    for p in e.get("pieces") or []:
        d = dict(p)
        nm = d.pop("name", None)
        if not nm:
            continue
        d["prim_path"] = "{0}/{1}".format(pieces_scope, nm)
        placements.append(d)

    grid = dict(e.get("grid") or {})
    measured = bool(grid.get("measured"))

    # EXACTLY WHAT `slice_to_kit` DOES AT THE END OF A LIVE SLICE: install the
    # synthetic `urban_building` style from the measured grid, then correct
    # the advertised parapet band to what the ACTUAL pieces contain (see
    # `gac_storey_slice._fix_advertised_bands` — it is a private-looking name
    # but a public, importable module function, and skipping it here would
    # leave `ub.STYLES[name]["bands"]` claiming a band the baked pieces may
    # not back, which is the exact defect that function exists to prevent).
    spec = gsl.register_style(grid, name, pieces_of=placements)
    gss._fix_advertised_bands(spec, placements, name, verbose=False)
    return placements, grid, measured
