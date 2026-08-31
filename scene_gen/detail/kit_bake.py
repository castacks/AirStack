"""kit_bake — load a GreatAmericanCity kit that was sliced ONCE and saved,
instead of re-slicing it every launch.

`gac_storey_slice.slice_to_kit` is deterministic for a given asset, a given
set of CUT PARAMETERS, and a given slicer version: measure the window grid,
clip a VTK plane stack per storey per ring cell, write the pieces. That is
most of a bench's build time (user, 2026-08-29: "save those kitbashes
locally... so we don't have to break them every time and can just assemble";
user, 2026-08-31: "every time we slice a new building it is stored so that we
don't have to do it again"), so a bake is cached to
`scene_gen/assets/kits/<key>.usd`, plus one record per bake in
`scene_gen/assets/kits/kits.json`. This module is BOTH halves now: `save_kit`
freezes a fresh slice the moment a caller makes one — `gac_fire.burn_gac`'s
cache block calls it automatically the instant `use_baked_kit` is true and
nothing was already cached — and `load_kit` references a frozen one back in
and hands a caller EXACTLY what `slice_to_kit` would have — the same
`(placements, grid, measured)` shape, on purpose, so a launcher can swap

    pls, g, meas = gss.slice_to_kit(stage, src, cell, style, verbose=False)

for

    pls, g, meas = kb.load_kit(stage, cell, name, ssf, sig)

on one line, falling back to the live path with `have_kit(name, sig)`.
`tools/bake_gac_kits.py` still exists as a standalone OFFLINE way to warm the
cache ahead of time for the unconditional, whole-building slice (no fire
plan -> `sig=None`, see KEYING below); nothing about this module requires it.

THE TWO THINGS THAT MAKE A CACHED KIT WRONG, NOT JUST SLOW
-------------------------------------------------------------
A kit keyed by asset name ALONE is unsafe two different ways, both found the
same night (2026-08-31):

  1. KEYING. `slice_to_kit`'s `region=` argument cuts a DIFFERENTLY SHAPED
     kit depending on the fire plan that produced it (`origin`/`top` decide
     how much of the bottom/top merges into one piece; `sides` decides which
     elevations ring in bays/thirds versus collapse to one piece per band —
     see that function's own docstring). Two fire plans on the SAME building
     can legitimately want two DIFFERENT cuts, so a bake for one is the WRONG
     bake for the other, not just a slower one. `slice_signature()` hashes
     exactly the call-site parameters that decide this, and every manifest
     record — and every `.usd` file, see NAMING below — is keyed on
     `(asset, signature)`, never `asset` alone.
  2. VINTAGE. See STALENESS below — unchanged in spirit, widened in scope.

MANIFEST SHAPE. `kits.json` is a JSON list, one record per BAKE (one asset
can have several records, one per distinct `signature`), each with: `asset`
(the manifest key's first half), `signature` (the second half —
`None`/absent for a bake made with no `slice_signature()` at all, i.e. every
bake `tools/bake_gac_kits.py` has ever produced), `usd` (the baked file's
absolute path), `src_usd`, `fingerprint`, `grid` (exactly what
`gac_storey_slice.grid_for` returns — `storey_h`, `storeys`, `bays`, `W`, `D`,
`H`, `z0`, `bbox`, `measured` — all of it plain JSON), and `pieces` (one dict
per placement, with a `name` field — the piece's bare prim name under the
baked file's `/Kit/pieces` scope — replacing the live path's `prim_path`,
which cannot be known until this loader decides where to reference the file).

NAMING. A `signature=None` bake's file is `<asset>.usd` — the same name
`tools/bake_gac_kits.py` has always written, so an already-baked whole-
building kit keeps its path and its manifest row unchanged. A
`signature="<hex>"` bake's file is `<asset>__<hex>.usd`, so two fire plans on
the same building never collide on disk. NOTHING is ever deleted or
overwritten to make room for a new key: a superseded bake (a different
signature, or the same signature at an old vintage) just stops being
referenced by `have_kit`/`load_kit` and sits there orphaned until someone
cleans `scene_gen/assets/kits/` by hand.

THE MATERIAL TRAP THIS DEPENDS ON THE BAKER HAVING ALREADY SOLVED
-------------------------------------------------------------------
A live `slice_to_kit` binds every sliced subset to a material prim living
under the SOURCE building's own subtree, and merely makes that subtree
invisible — it is still composed, which is why materials render fine in every
launcher today. A baked kit has no source subtree at all, so the bake had to
REHOME every material onto a prim of its own before it could drop the source
(`gac_slice.rehome_materials`, driven from `_save_kit_unsafe` below the same
way `bake_gac_kits.bake_one` drives it — see both docstrings). `load_kit`
trusts that the `.usd` it references is already self-contained that way; it
does no material surgery of its own.

STALENESS. `fingerprint()` hashes `gac_storey_slice.py` + `gac_slice.py` +
`disaster/gac_fire.py` (byte for byte, all three) plus `CUT_OFFSET`, so ANY
change to the slicing grammar — the piece-budget rework in flight alongside
this file, or a change to `gac_fire.window_rects`/`_islands` (the window-
island grouping a fire plan's `origin`/`sides` come from, upstream of the
`region=` this module now keys on) — changes it. `gac_fire.py` was not part
of this hash before 2026-08-31; adding it means every kit baked earlier reads
as stale on its first check afterward — self-invalidating exactly as
designed, nothing on disk is touched, the next slice just writes a fresh key.
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
# The fire-plan half of the slicing grammar (`region=`'s origin/top/sides)
# is COMPUTED here, not in either slicer file — see `fingerprint()`.
_GF_PATH = os.path.normpath(os.path.join(_THIS_DIR, "..", "disaster", "gac_fire.py"))

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
    """A hash of everything that decides what a bake CONTAINS — the SLICER's
    own vintage, shared by every asset and every call (contrast
    `slice_signature()`, which is per-call).

    Hashing the three files byte-for-byte, rather than pattern-matching for a
    version string, means this needs no cooperation from whoever edits them
    next — the piece-budget rework happening in `gac_storey_slice.py`
    alongside this file, or a change to `gac_fire.window_rects`/`_islands`,
    changes the hash the moment it lands, with nothing to remember to bump.
    `gac_fire.py` is included because the fire plan a `region=` cut is BUILT
    FROM (`origin`/`sides`) is computed there, not in either slicer file —
    see the module docstring's STALENESS section.
    """
    import hashlib

    h = hashlib.sha1()
    for p in (_GSS_PATH, _GSL_PATH, _GF_PATH):
        try:
            with open(p, "rb") as fh:
                h.update(fh.read())
        except OSError:
            h.update(("MISSING:" + p).encode("utf-8"))
    h.update(repr(("offset", CUT_OFFSET)).encode("utf-8"))
    return h.hexdigest()[:16]


def slice_signature(region=None, family=None, force_regular=False, style=None):
    """Hash the exact PER-CALL parameters that decide what ONE slice cuts —
    the KEYING half of the cache (contrast `fingerprint()`, the slicer's own
    shared vintage). Two calls that agree on every field below always
    produce the same pieces from the same asset at the same vintage; two
    calls that disagree on any of them may not, and must not share a cache
    entry.

    THE FIELD LIST, AND WHY EACH ONE IS HERE (read `gac_storey_slice.
    slice_to_kit`'s own docstring for the mechanism each of these drives):

      * `region["origin"]` — the physical storey at/above which per-storey
        ringing begins; everything below collapses into ONE pre-origin piece
        (`merged_lower`). Two fire plans with different origins physically
        cut a different set of lines.
      * `region["top"]` — the mirror image at the roof end: the physical
        storey at/below which ringing ends: above it merges into one piece
        and the roof/parapet split is skipped entirely (`merged_upper`), so
        a building sliced with one `top` can be missing a `role="roof"`
        piece a different `top` would have produced.
      * `region["sides"]` — which elevations are "hot" (`_region_hot_sides`):
        a hot side rings in bays/thirds, a cold one collapses to one piece
        per band. Sorted before hashing — `fire["sides"]` is built upstream
        with no documented order guarantee, and two calls that agree on the
        SET of hot sides must hash identically regardless of what order that
        set came out in.
      * `family` — not a `slice_to_kit` cut-geometry input on its own, but it
        picks the construction-type ladder `register_style` writes into the
        synthetic style spec that the cached RESULT carries alongside its
        pieces (`gac_fire.burn_gac`'s `_family`: urm/rc/rc_glass). Two calls
        that cut identically but disagree on `family` must not collide on
        one cache entry, or a rebaked-as-urm building could be served to a
        caller that asked for rc.
      * `force_regular` — switches `grid_for` from the measured window
        lattice to a fixed regular grid: a different physical cut-line
        spacing on the exact same mesh.
      * `style` — included DEFENSIVELY. As of this writing it is a pure
        function of `(kind, asset)` (`gac_fire.prepare`: `style =
        pack["style_prefix"] + asset`), so it is redundant with the `name`
        half of the `(asset, signature)` cache key today; keeping it in the
        hash means that invariant is allowed to stop holding later without
        this cache silently starting to collide.

    NOT INCLUDED: `offset`/`target`, `slice_to_kit`'s other two keyword
    parameters. Neither is ever overridden at the one call site this cache
    serves (`gac_fire.burn_gac` takes both at `slice_to_kit`'s own
    defaults — `offset` via the shared `CUT_OFFSET` constant, `target` via
    `gac_storey_slice.TARGET_STOREY_M`), and both constants live inside files
    `fingerprint()` already hashes byte-for-byte, so a future change to
    either already busts every cache entry through the VINTAGE stamp instead
    of needing a place here.

    `region=None` (no fire plan at all — `tools/bake_gac_kits.py`'s
    unconditional whole-building slice) still returns a real hash, NOT the
    bare Python `None` `have_kit`/`load_kit` treat as "no signature was
    computed at all" for backward compatibility with that offline tool's
    existing manifest rows. The two are deliberately different values in
    different namespaces — nothing in this module ever compares one to the
    other.
    """
    import hashlib

    r = region or {}
    payload = {
        "origin": (int(r["origin"]) if r.get("origin") is not None else None),
        "top": (int(r["top"]) if r.get("top") is not None else None),
        "sides": sorted(str(s) for s in (r.get("sides") or ())),
        "family": family,
        "force_regular": bool(force_regular),
        "style": style,
    }
    blob = json.dumps(payload, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha1(blob).hexdigest()[:16]


def read_manifest():
    if not os.path.exists(MANIFEST_PATH):
        return []
    with open(MANIFEST_PATH) as fh:
        return json.load(fh)


def write_manifest(records):
    os.makedirs(KIT_DIR, exist_ok=True)
    with open(MANIFEST_PATH, "w") as fh:
        json.dump(records, fh, indent=1, sort_keys=True)


def _key(r):
    """The composite cache key of a manifest record — `(asset, signature)`,
    with a missing `"signature"` field (every row `tools/bake_gac_kits.py`
    has ever written) read as `None`, exactly what `have_kit`/`load_kit`'s
    own `signature=None` default means: "the caller asked for the
    unconditional whole-building bake, not one keyed to a fire plan"."""
    return (r.get("asset"), r.get("signature"))


def merge_manifest(records):
    """Read-merge-write `records` into `kits.json` under a file lock, keyed
    by `(asset, signature)` — so re-baking one `(asset, signature)` pair
    does not drop any other bake of the SAME asset under a different
    signature (or of a different asset), and two bakes finishing together
    (mirroring `bake_quake_headless.sh`) cannot lose each other's rows
    either. Returns the merged list.
    """
    import fcntl

    os.makedirs(KIT_DIR, exist_ok=True)
    with open(MANIFEST_PATH + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            old = read_manifest()
            fresh = set(_key(r) for r in records)
            merged = [r for r in old if _key(r) not in fresh] + list(records)
            write_manifest(merged)
        finally:
            fcntl.flock(lk, fcntl.LOCK_UN)
    return merged


def _entry(name, signature=None):
    for r in read_manifest():
        if r.get("asset") == name and r.get("signature") == signature:
            return r
    return None


def have_kit(name, signature=None):
    """Is there a bake for `(name, signature)` that is both PRESENT and
    CURRENT?

    `signature` defaults to `None` — the unconditional whole-building bake
    `tools/bake_gac_kits.py` produces — so every existing caller that only
    ever passed `name` (`quake_gac_probe.py`, that offline tool itself) keeps
    matching exactly the rows it always matched. A fire-plan-aware caller
    (`gac_fire.burn_gac`) passes the real `slice_signature(...)` instead, so
    it can only ever hit a bake keyed to the SAME region/family/
    force_regular/style it is about to ask for — never one cut for a
    different fire plan on the same building.

    False covers four different reasons a caller should fall back to a live
    `slice_to_kit` — no manifest row for this exact `(name, signature)`, a
    row whose `.usd` is missing on disk, a row baked against a slicer that
    has since changed, or (implicitly, via the exact-signature match above) a
    row that exists but was cut for a DIFFERENT fire plan — on purpose:
    telling them apart buys the caller nothing, since the fallback is the
    same either way.
    """
    e = _entry(name, signature)
    if not e:
        return False
    if not e.get("usd") or not os.path.exists(e["usd"]):
        return False
    return e.get("fingerprint") == fingerprint()


# ---------------------------------------------------------------------------
# The loader
# ---------------------------------------------------------------------------
def load_kit(stage, cell, name, ssf=1.0, signature=None):
    """Reference the baked kit `(name, signature)` under `cell` and return
    kit placements.

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

    `signature`, like `have_kit`'s, defaults to `None` (the unconditional
    whole-building bake) and MUST be the exact `slice_signature(...)` a
    fire-plan-aware caller already confirmed with `have_kit` — this function
    does not re-derive it, it only looks up `(name, signature)` verbatim.

    Returns `(placements, grid, measured)` — the same shape `slice_to_kit`
    returns. `grid["measured"]` is `measured` again, redundantly: `slice_to_kit`
    folds it into the grid dict it returns AND returns it separately, and this
    keeps that same double return rather than inventing a different contract.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: F401  (UsdShade: doc parity)
    from detail import gac_slice as gsl
    from detail import gac_storey_slice as gss

    e = _entry(name, signature)
    if e is None:
        raise KeyError(
            "no baked kit named {0!r} (signature={1!r}) in {2} — bake it "
            "first (scene_gen/tools/bake_gac_kits.py --assets {0} for the "
            "unconditional bake, or `save_kit` for a fire-plan-specific one) "
            "or fall back to a live slice_to_kit".format(
                name, signature, MANIFEST_PATH))
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


# ---------------------------------------------------------------------------
# The saver — SAVE-ON-SLICE. `gac_fire.burn_gac`'s cache block calls this
# right after its OWN live `slice_to_kit` call whenever `have_kit` just came
# back False, so the next bake of the same `(asset, signature)` at the same
# vintage hits `load_kit` instead of slicing again.
# ---------------------------------------------------------------------------
def _place_source_for_bake(stage, holder, url, scale):
    """Reference `url` under `holder/asset`, scaled and re-seated (centred
    in plan, based at `holder`'s own parent's z=0) exactly the way
    `gac_fire.place_source` seats a LIVE one — and, like that function,
    return `holder` (the WRAPPER), never `holder + "/asset"` (the prim that
    actually carries the scale/translate ops). Duplicated rather than
    imported (`detail/` does not depend on `disaster/`; see `place_source`
    for the original this mirrors).

    RETURNING THE WRAPPER IS NOT A STYLE CHOICE, IT IS LOAD-BEARING.
    `gac_slice.window_centres` measures every window's position RELATIVE TO
    its `prim_path` argument's OWN local frame (`root_inv =
    GetLocalToWorldTransform(root).GetInverse()`, then every mesh's world
    transform is multiplied by it) — by design, so a caller gets bay
    coordinates "in the prim's own frame" regardless of where in a scene
    that prim sits. Hand it a prim that ITSELF carries the scale/translate
    ops (the "asset" child, not the wrapper) and `root_inv` divides that
    scale straight back out: every window comes back at its RAW, pre-scale,
    off-centre coordinate (for `SM_Building_02`, ~100x too large and nowhere
    near z=0), `grid_for`'s confidence check fails on numbers that no longer
    look like storeys, and the result is a REGULAR-grid fallback that cuts a
    COMPLETELY different set of pieces — 7 instead of 53, for the IDENTICAL
    `(region, family, force_regular)` a correctly-seated live slice
    (`gac_fire.place_source` -> `slice_to_kit`) produces. Found by running
    the live/save/load round trip against a real Nucleus asset (2026-08-31)
    and comparing piece counts — exactly the mismatch `save_kit` exists to
    never produce, since a wrong cached kit is silently worse than no cache
    at all. `tools/bake_gac_kits.py._place_source` has this SAME bug (it
    also returns the asset-carrying child): its `SM_Building_02`/
    `SM_Building_09` manifest rows show the identical symptom
    (`grid["measured"] is False`, `confidence: 0.0`) — a pre-existing defect
    in that offline tool, out of scope here (it is not part of this file's
    KEYING/VINTAGE work), left as a pointer for whoever next touches it.

    Returns `holder`, or `None` if nothing composed (a broken Nucleus path
    still lets `AddReference` succeed, so the real test is a non-empty world
    bound, not the return of that call).
    """
    from pxr import Gf, Sdf, Usd, UsdGeom

    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(url)
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    scale = float(scale)
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    c = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(holder).GetParent()).ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - c[0]),
                    -(0.5 * (mn[1] + mx[1]) - c[1]), -(mn[2] - c[2])))
    return holder


def _rehome_pieces_for_bake(stage, pieces_scope, dst_looks, verbose=True):
    """Rebind every `GeomSubset` under `pieces_scope` off the SOURCE's
    material prims and onto fresh ones under `dst_looks` — steps 2/3/4 of
    the material-trap fix `bake_gac_kits._rehome_piece_materials` already
    does for the offline baker; see the module docstring's MATERIAL TRAP
    section for why an exported kit cannot skip this. Discovers what to
    rehome by asking the SUBSETS what they are bound to right now
    (`ComputeBoundMaterial`) rather than threading a materials list out of
    `slice_to_kit`, so this stays decoupled from the slicer's internals.

    Returns `(new_mats, n_bound, n_subsets)`.
    """
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(pieces_scope)
    mat_prims, targets = {}, []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Subset):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        key = str(mat.GetPrim().GetPath())
        mat_prims.setdefault(key, mat)
        targets.append((prim, key))
    new_mats = gsl.rehome_materials(stage, mat_prims, dst_looks, verbose=verbose)
    n_bound = 0
    for prim, key in targets:
        nm = new_mats.get(key)
        if nm is None:
            continue
        UsdShade.MaterialBindingAPI(prim).Bind(nm)
        n_bound += 1
    if verbose:
        print("[kit_bake]   rehomed {0} material(s), rebound {1}/{2} "
              "subset(s)".format(len(new_mats), n_bound, len(targets)))
    return new_mats, n_bound, len(targets)


def _kit_filename(name, signature):
    """`<asset>.usd` for `signature=None` — the SAME name
    `tools/bake_gac_kits.py` has always written, so an already-baked
    whole-building kit's file (and its `(asset, None)` manifest row) is
    never orphaned by this cache growing a signature. `<asset>__<hex>.usd`
    otherwise, so distinct fire plans on the same building never collide."""
    return name + ".usd" if signature is None else "{0}__{1}.usd".format(
        name, signature)


def save_kit(name, signature, url, scale, style, region=None, family=None,
            force_regular=False, out_dir=None, verbose=True):
    """Slice `name` a SECOND time, in a disposable in-memory stage, purely to
    freeze the result under the cache key `(name, signature)` — mirrors
    `tools/bake_gac_kits.bake_one` exactly (same rehome-then-export material
    fix, same root-layer-only export; see that file's docstring for why
    neither step can be skipped), parameterised over the SAME
    `region`/`family`/`force_regular` a live caller just used, so the frozen
    pieces match what the live scene already has, piece for piece.

    RE-SLICES RATHER THAN COPYING THE LIVE PIECES ALREADY ON `stage`. The
    live pieces are bound to material prims living under the live
    building's OWN subtree (`slice_to_kit` only hides that subtree, never
    detaches it) — exporting them as-is would reproduce exactly the "turned
    every sliced building white" defect `bake_gac_kits.py`'s docstring
    already survived once. Rehoming the LIVE scene's own bindings to save a
    second slice was considered and rejected: it would mutate prims the
    caller's own scene still depends on, purely as a side effect of caching.
    A disposable isolated stage costs one extra slice — paid ONLY on the
    call that populates the cache, never on a hit, since `gac_fire.burn_gac`
    only reaches this function after a live slice it already needed anyway.

    Best-effort: returns the manifest record on success, or `None` (with a
    printed reason, never a raised exception) on ANY failure — a broken save
    must never take the live building's own bake down with it.
    """
    try:
        return _save_kit_unsafe(name, signature, url, scale, style,
                                region=region, family=family,
                                force_regular=force_regular,
                                out_dir=out_dir, verbose=verbose)
    except Exception as exc:
        print("[kit_bake] save_kit({0!r}, signature={1!r}) failed, "
              "continuing without a cache entry: {2}".format(
                  name, signature, exc))
        return None


def _save_kit_unsafe(name, signature, url, scale, style, region=None,
                     family=None, force_regular=False, out_dir=None,
                     verbose=True):
    import time

    from detail import gac_storey_slice as gss
    from pxr import Sdf, Usd, UsdGeom

    out_dir = out_dir or KIT_DIR
    t0 = time.time()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    kit = UsdGeom.Xform.Define(stage, Sdf.Path("/Kit"))
    stage.SetDefaultPrim(kit.GetPrim())

    src = _place_source_for_bake(stage, "/Src", url, scale)
    if not src:
        print("[kit_bake] save_kit({0!r}): nothing composed from {1} — "
              "not caching".format(name, url))
        return None

    t_slice0 = time.time()
    # SAME PARAMETERS THE LIVE CALL JUST USED, so this bake is the exact
    # geometry `slice_signature(region, family, force_regular, style)`
    # already committed to hashing — `offset` pinned to the shared
    # `CUT_OFFSET` (never overridden by any caller today, see that
    # function's own docstring) rather than left at `slice_to_kit`'s
    # internal default, so the two can never quietly drift apart.
    pls, g, _measured = gss.slice_to_kit(
        stage, src, "/Kit", style, offset=CUT_OFFSET, verbose=verbose,
        region=region, family=family, force_regular=force_regular)
    slice_s = time.time() - t_slice0
    if not pls:
        print("[kit_bake] save_kit({0!r}): sliced to nothing — not caching"
              .format(name))
        return None

    new_mats, n_bound, n_subsets = _rehome_pieces_for_bake(
        stage, "/Kit/pieces", "/Kit/Looks", verbose=verbose)
    if n_bound < n_subsets and verbose:
        print("[kit_bake] save_kit({0!r}): only {1}/{2} subset(s) rehomed — "
              "{3} piece(s) will render untextured once reloaded from this "
              "cache entry".format(name, n_bound, n_subsets,
                                    n_subsets - n_bound))

    # NOW SAFE: every subset that had a resolvable material points at
    # `/Kit/Looks/*` instead of into `/Src`.
    stage.RemovePrim(Sdf.Path("/Src"))

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, _kit_filename(name, signature))
    # ROOT LAYER ONLY — see the module docstring's MATERIAL TRAP section for
    # why this is not `stage.Export()` / `stage.Flatten()`.
    stage.GetRootLayer().Export(out_path)
    mb = round(os.path.getsize(out_path) / 1e6, 3)
    bake_s = time.time() - t0

    pieces = []
    for p in pls:
        d = {k: v for k, v in p.items() if k != "prim_path"}
        d["name"] = p["prim_path"].rsplit("/", 1)[-1]
        pieces.append(d)

    record = {
        "asset": name,
        "signature": signature,
        "usd": os.path.abspath(out_path),
        "src_usd": url,
        "fingerprint": fingerprint(),
        "grid": g,                       # already carries "measured"
        "pieces": pieces,
        "n_pieces": len(pieces),
        "materials": len(new_mats),
        "mb": mb,
        "slice_s": round(slice_s, 2),
        "bake_s": round(bake_s, 2),
    }
    merge_manifest([record])
    if verbose:
        print("[kit_bake] save_kit: {0} (signature={1}): {2} piece(s), {3} "
              "material(s), {4:.3f} MB, sliced {5:.1f}s, saved {6:.1f}s -> "
              "{7}".format(name, signature, len(pieces), len(new_mats), mb,
                           slice_s, bake_s, out_path))
    return record
