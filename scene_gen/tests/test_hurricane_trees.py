#!/usr/bin/env python3
"""
test_hurricane_trees.py — offline gate on the hurricane tree bake and the
saturated-soil windthrow boost. No Isaac Sim; `pxr` runs fine bare on this
host (`usd-core` is pip-installed) so the bake itself is exercised directly,
not mocked.

What this pins, and why each one has already been wrong once:

  1. RETENTION MATCHES THE STREAM T5 DIRECTIVE ("damage read by crown
     colour, not crown removal"). Every non-`snapped` level keeps a real,
     silhouette-forming MAJORITY of its crown (`defoliated` 70-85%, `limbed`
     55-70%, `leaning`/`fallen` 45-60%) instead of the pre-T5 10-30% that
     made every damaged tree a near-invisible skeleton at altitude
     (measured: `tree_American_Beech_defoliated.usd` had 122,862 visible
     mesh faces yet topped out at 5.9 m tall). `snapped`'s severed top is
     lower (35%) because it is the one level with an actual geometric break.
     THE COTTON-BALL MUST STILL NOT RECUR at the other end: a damaged level
     must never be 100% visible AND untinted -- a full, dense, GREEN canopy
     on a `defoliated`/`limbed`/etc. archetype is the class of bug that put
     a full-white (its texture failed to resolve on the render host) tree
     in a Cat-3 flood, and staying under the retention CEILING plus the tint
     checks below are what rules it out.
  2. EVERY VISIBLE MESH BINDS A RESOLVABLE MATERIAL, case-exact, on THIS
     machine — the fix for the cotton ball's underlying class of fault
     (SESSION_2026-08-31 §4 item 3's "References an asset that can not be
     found" pattern), proven both by auditing the real bake output and by
     forcing a fake unresolved texture through `_material_ok`.
  3. THE PER-CARD CULL STILL WORKS AT THE NEW (HIGHER) TARGETS, weighted
     toward the lower/inner crown — not "whichever whole branch group
     happened to round up", which is what per-PRIM culling gave a
     three-instancer species before this file's per-CARD rewrite (a defect
     just as reachable at a 80%-keep target as it was at a 12%-keep one).
  4. A SNAG IS A CLIPPED TRUNK, not a thinned crown: the stump stays
     standing, a severed top exists as separate geometry, and — the second
     bug this file's own history records — the top's pivot must be at
     GROUND level, not at the elevated break height, or it hangs in the air
     (measured before the fix: 6.8-11.9 m up) instead of resting near the
     lawn.
  5. FALLEN TREES GROUND-CONTACT via POINT bounds, keep foliage inside the
     `leaning`/`fallen` band, and the trunk stays planted (`lift == 0`).
  6. THE WINDTHROW BOOST MAKES L2 REACHABLE. Replayed against the real
     `V2_L2`/`FINAL3_L3_brown` GT (species, x, y, seed, field cfg) with the
     exact launcher draw order, `leaning`/`fallen`/`snapped` must go from
     UNREACHABLE at L2 to non-zero once local flood depth is applied.
  7. THE BAKE REPRODUCES BYTE-FOR-BYTE.
  8. A WHOLLY-KEPT FOLIAGE REL STILL GETS TINTED (Correction 1, STREAM T4).
     `_author_foliage_cull` used to skip tinting entirely for any rel whose
     OWN cards were 100% kept -- reachable even when the crown's GLOBAL
     keep fraction is far below 1.0, since selection is per-card pooled
     across every rel. Not present in the current seeded bakes (verified
     by audit), so exercised directly with a crafted `kept` dict.
  9. DRY-LAND WINDTHROW IS NONZERO (Correction 2, STREAM T4). Before this,
     100% of leaning/fallen/snapped trees at BOTH L2 and L3 stood in >0.2 m
     of water -- `dry_windthrow_chance` now gives a measured-DRY tree an
     independent, intensity-driven path to a structural outcome, replayed
     against the same real GT to hit ~2% (L2) / 5-8% (L3) of DRY trees
     while keeping L3's total near 30%.
  10. PRISTINE IS REACHABLE AT BOTH LEVELS (STREAM T5). Before the
      `_TREE_CUTS` re-cut, `pristine` was 0% of L3 and ~1% of L2 — every
      tree in the scene was drawn as damaged. Replayed against the same
      real GT, `pristine` must now land in 15-25% at L3 and 40-50% at L2,
      `defoliated` must be the outright plurality level at L3, and the
      structural (`leaning`+`fallen`+`snapped`) share must stay where it
      was (L3 ~28-32%, L2 ~6-8%) — the re-cut only moved the `pristine`/
      `defoliated`/`limbed` boundaries, never the `limbed`-to-`leaning`
      structural threshold or the windthrow mechanisms.

Run:  python3 -m pytest scene_gen/tests/test_hurricane_trees.py -v
  or: python3 scene_gen/tests/test_hurricane_trees.py   (prints every check)
"""
import filecmp
import json
import math
import os
import random
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN_DIR = os.path.dirname(TESTS_DIR)
REPO = os.path.dirname(SCENE_GEN_DIR)
sys.path.insert(0, os.path.join(SCENE_GEN_DIR, "tools"))
sys.path.insert(0, SCENE_GEN_DIR)

import numpy as np  # noqa: E402

import bake_hurricane_trees as bht  # noqa: E402
from disaster import hurricane as hu  # noqa: E402
from disaster import surge as sgw  # noqa: E402

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def strict(fn):
    """Make a recorded FAIL an actual pytest failure. Same idiom as
    `test_car_toss.py`'s own `strict` — see that file's docstring for why
    `check()` records rather than raises.
    """
    def run(*a, **kw):
        mine = len(FAILS)
        out = fn(*a, **kw)
        assert len(FAILS) == mine, FAILS[mine:]
        return out
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


# ---------------------------------------------------------------------------
# shared bake fixture — build once, every test below reads it
# ---------------------------------------------------------------------------

_BAKE_DIR = None


def _bake_dir():
    global _BAKE_DIR
    if _BAKE_DIR is not None:
        return _BAKE_DIR
    d = tempfile.mkdtemp(prefix="hur_tree_bake_")
    for species, rel in sorted(bht.TREE_SPECIES.items()):
        for level in bht.LEVELS:
            out, note = bht.bake_one(species, rel, level, d)
            assert out is not None, "{0} {1}: {2}".format(species, level, note)
    _BAKE_DIR = d
    return d


def _world_visible_meshes(stage):
    """`[(prim, points_world)]` for every VISIBLE Mesh in `stage`."""
    xcache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "Mesh":
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts or len(pts) == 0:
            continue
        m = np.array(xcache.GetLocalToWorldTransform(prim))
        arr = np.array(pts, dtype=np.float64)
        world = np.hstack([arr, np.ones((len(arr), 1))]) @ m
        out.append((prim, world[:, :3]))
    return out


# ---------------------------------------------------------------------------
# 1. every baked archetype's visible-canopy fraction matches the STREAM T5
#    art-directed retention band for its level -- neither a bare skeleton
#    (the pre-T5 defect: "1-2 trees in the whole scene") nor a full,
#    UNTINTED cotton ball (the original defect this file was written for).
# ---------------------------------------------------------------------------

# (lo, hi) on the MEASURED visible-card fraction, `_FOL_KEEP`'s own target
# band plus a few points of rounding slack -- see `bake_hurricane_trees.
# _FOL_KEEP`'s comment for the sourcing/art-direction split behind each band.
#
# RE-TUNED 2026-09-01 (STREAM T7 -- "the trees all look perfect, need some
# of them to be broken down"): `limbed`/`leaning`/`fallen` all dropped, and
# `limbed`'s is now the realised AVERAGE of a two-region sector split
# (`_select_kept_cards_sector`'s independent per-card Bernoulli draws), not
# a ranked top-N -- so it lands close to but not exactly on `_FOL_KEEP
# ["limbed"]`, with more slack on a small-N crown (measured: Shumard_Oak's
# 270-triangle crown landed at 32.6% against a 35% target). `leaning`/
# `fallen` still use the ranked selection and land within a point of their
# target on every species measured.
_RETENTION_BANDS = {
    "defoliated": (0.68, 0.87),
    "limbed": (0.25, 0.45),
    "leaning": (0.40, 0.50),
    "fallen": (0.30, 0.40),
}


@strict
def test_foliage_retention_matches_directive_bands():
    """For every foliage rel in the SOURCE inventory, the BAKED archetype's
    own visible-card count (instances not hidden by `invisibleIds`, or
    triangles in a "*_cards" replacement mesh) must fall inside that level's
    `_RETENTION_BANDS` entry.

    STREAM T5 FLIPPED THIS TEST'S DIRECTION. The ORIGINAL version of this
    test asserted every damaged level stayed UNDER 16% visible -- correct
    when `_FOL_KEEP` targeted the textbook 86-94% broadleaf leaf-loss
    number, and it caught the "cotton ball" defect (a full-crown mesh/
    instancer left 100% visible on a tree the ladder said should be bare).
    That target made every damaged tree a near-invisible skeleton at a
    400 m altitude (measured: `tree_American_Beech_defoliated.usd`, 122,862
    visible mesh FACES, topped out at 5.9 m) -- which is exactly what the
    user's own directive named ("I see only 1-2 trees in the whole scene").
    The retention bands are now art-directed to keep a real MAJORITY of the
    crown visible at every standing damaged level (see `bake_hurricane_
    trees._FOL_KEEP`'s own comment), so this test now checks BOTH sides: not
    so little that the tree disappears, not so much (nor, per the other
    tests in this file, untinted) that it silently reintroduces the cotton
    ball.

    Counted in the SAME units `_foliage_inventory`/`_select_kept_cards`
    themselves use (instances for a `PointInstancer`, triangles for a
    mesh) -- comparing raw mesh POINT counts across the two kinds compares
    "cards" to "geometry describing one card's shape", which are unrelated
    numbers (an early version of this test did exactly that and reported a
    10,000%+ "leak" that was really just unit confusion).
    """
    d = _bake_dir()
    for species, rel in sorted(bht.TREE_SPECIES.items()):
        if species in bht.CONIFERS:
            continue  # conifers have their own (higher) floor -- see _CONIFER_FOL_KEEP
        src_abs = os.path.join(REPO, rel)
        src_stage = Usd.Stage.Open(src_abs)
        foliage, _ = bht._classify(src_stage)
        default = src_stage.GetDefaultPrim()
        root_path = default.GetPath()
        xcache = UsdGeom.XformCache()
        inv = bht._foliage_inventory(src_stage, root_path, foliage, xcache)
        for level, (lo, hi) in sorted(_RETENTION_BANDS.items()):
            path = os.path.join(d, "tree_{0}_{1}.usd".format(species, level))
            if not os.path.exists(path):
                continue  # e.g. `fallen` skipped for a NO_UPROOT species
            stage = Usd.Stage.Open(path)
            total = sum(v["n"] for v in inv.values())
            n_visible = 0
            for r, v in inv.items():
                if v["kind"] == "instancer":
                    over = stage.GetPrimAtPath(Sdf.Path("/Root/src").AppendPath(r))
                    if not over or not over.IsValid():
                        n_visible += v["n"]  # untouched original: fully visible
                        continue
                    if (UsdGeom.Imageable(over).ComputeVisibility()
                            == UsdGeom.Tokens.invisible):
                        continue
                    inv_ids = UsdGeom.PointInstancer(over).GetInvisibleIdsAttr().Get()
                    n_visible += v["n"] - (len(inv_ids) if inv_ids else 0)
                else:
                    cards_path = (Sdf.Path("/Root/src").AppendPath(r).GetParentPath()
                                 .AppendChild(r.name + "_cards"))
                    cards_prim = stage.GetPrimAtPath(cards_path)
                    if cards_prim and cards_prim.IsValid():
                        fvc = UsdGeom.Mesh(cards_prim).GetFaceVertexCountsAttr().Get()
                        n_visible += len(fvc) if fvc else 0
                    else:
                        orig = stage.GetPrimAtPath(Sdf.Path("/Root/src").AppendPath(r))
                        if not orig or not orig.IsValid() or (
                                UsdGeom.Imageable(orig).ComputeVisibility()
                                != UsdGeom.Tokens.invisible):
                            n_visible += v["n"]  # no replacement AND not hidden
            frac = n_visible / total if total else 0.0
            check(lo <= frac <= hi,
                  "{0}/{1}: {2}/{3} = {4:.1%} of foliage cards visible "
                  "(want {5:.0%}-{6:.0%})"
                  .format(species, level, n_visible, total, frac, lo, hi))


# ---------------------------------------------------------------------------
# 2. every visible prim binds a resolvable material
# ---------------------------------------------------------------------------

@strict
def test_every_visible_mesh_has_a_resolvable_material():
    """Walks the REAL baked output (all 34 archetypes) and, for every
    visible mesh, resolves its bound material's MDL file and every texture
    it references, case-exact, on this machine. This is the actual audit
    behind the module docstring's "every leaf/bark material on every one of
    the six species' SOURCE assets resolves cleanly" claim -- checked here
    against the ARCHETYPES (which may carry safety-net or tinted-copy
    materials), not just the raw sources.
    """
    d = _bake_dir()
    n_checked = 0
    n_bad = 0
    for fn in sorted(os.listdir(d)):
        if not fn.endswith(".usd"):
            continue
        # `tree_<species>_<level>.usd`, and species names may themselves
        # contain underscores (`Black_Oak`) -- strip the fixed prefix/level
        # suffix instead of a naive split.
        stem = fn[len("tree_"):-len(".usd")]
        species = next((s for s in bht.TREE_SPECIES if stem.startswith(s + "_")), None)
        species_src_abs = (os.path.join(REPO, bht.TREE_SPECIES[species])
                           if species else None)
        arch_abs = os.path.join(d, fn)
        stage = Usd.Stage.Open(arch_abs)
        for prim, _ in _world_visible_meshes(stage):
            mat = bht._bound_material(prim)
            n_checked += 1
            # `mat is None` is NOT the full "no material" test: an unbound
            # prim's `ComputeBoundMaterial()` can come back as a technically
            # non-`None` but invalid `UsdShade.Material` (`bool(mat) is
            # False`, `mat.GetPath()` empty) -- measured on this exact
            # asset family (a prototype mesh under a bark `PointInstancer`
            # whose binding lives on the instancer, authored without
            # `MaterialBindingAPI` applied, so inheritance does not fire).
            if not mat:
                n_bad += 1
                check(False, "{0}: {1} has NO bound material".format(fn, prim.GetPath()))
                continue
            # A material under `/Root/src/...` or `/Root/top/src/...`
            # reaches the REFERENCED species asset's own MDL/texture files,
            # relative to THAT file's directory -- resolve against
            # `species_src_abs`. A `tint_mats`/`safe_mats` material is
            # authored FRESH directly in the archetype's own layer
            # (`_omnipbr_leaf_material`, STREAM T6, or `_ensure_safe_
            # material`), with its own texture paths already relative to
            # the ARCHETYPE, so resolve those against the archetype file
            # itself.
            mp = str(mat.GetPath())
            root_for_mat = (arch_abs if ("/tint_mats/" in mp or "/safe_mats/" in mp)
                            else species_src_abs)
            ok = root_for_mat is not None and bht._material_ok(root_for_mat, mat)
            if not ok:
                n_bad += 1
                check(False, "{0}: {1} bound to '{2}', which has an "
                      "unresolvable texture reference (resolved against "
                      "{3})".format(fn, prim.GetPath(), mp, root_for_mat))
    check(n_checked > 0, "sanity: checked at least one visible mesh")
    print("    ({0} visible meshes checked across {1}, {2} bad)".format(
        n_checked, d, n_bad))


@strict
def test_material_safety_net_engages_on_a_forced_miss():
    """Proves the safety net actually DOES something, offline: monkeypatch
    `_case_exact_exists` to report everything missing and confirm
    `_material_ok` flips to False for a material that is fine on disk.
    """
    d = _bake_dir()
    stage = Usd.Stage.Open(os.path.join(d, "tree_Largetooth_Aspen_pristine.usd"))
    leaves = None
    for prim, _ in _world_visible_meshes(stage):
        if bht._is_foliage(prim):
            leaves = prim
            break
    check(leaves is not None, "pristine Aspen has a visible foliage mesh to test with")
    if leaves is None:
        return
    mat = bht._bound_material(leaves)
    src_abs = os.path.join(REPO, bht.TREE_SPECIES["Largetooth_Aspen"])
    check(bht._material_ok(src_abs, mat),
          "sanity: the real Aspen leaf material resolves on this machine")
    real = bht._case_exact_exists
    bht._material_ok_cache.clear()
    try:
        bht._case_exact_exists = lambda p: False
        check(not bht._material_ok(src_abs, mat),
              "with every path forced missing, _material_ok must report False")
    finally:
        bht._case_exact_exists = real
        bht._material_ok_cache.clear()


@strict
def test_red_flag_diffuse_constant_detector():
    """Unit-tests `_is_red_flag_constant` directly -- the defence-in-depth
    check for "a stock material with a red diffuse constant and no
    texture" (build-hurricane-scenes' red-twig finding). A texture
    resolution failure has nothing to flag here; this is specifically for
    a colour that is simply wrong.
    """
    check(bht._is_red_flag_constant((0.9, 0.05, 0.05)), "saturated red flagged")
    check(bht._is_red_flag_constant((0.6, 0.1, 0.15)), "a duller red still flagged")
    check(not bht._is_red_flag_constant((0.24, 0.19, 0.13)),
          "the module's own SAFE_WOOD_RGB must NOT be flagged")
    check(not bht._is_red_flag_constant((0.24, 0.34, 0.12)),
          "the module's own SAFE_LEAF_RGB must NOT be flagged")
    check(not bht._is_red_flag_constant((0.6, 0.46, 0.24)),
          "a brown-olive dead-leaf colour (TARGET_BROADLEAF's own family) "
          "must NOT be flagged")
    check(not bht._is_red_flag_constant((0.5, 0.5, 0.5)),
          "a neutral grey MDL default must NOT be flagged")


@strict
def test_material_level_override_texture_is_used_and_resolves():
    """THE HOLLYPRIVET BUG, PINNED UNDER THE STREAM T6 MECHANISM.
    `Common_Apple`'s `Apple_leaf_Mat` MATERIAL prim (not its Shader child)
    authors its own `inputs:diffuse_texture = ./materials/textures/
    hollyprivet_basecolor.png` -- an interface override the Shader's own
    `Apple_leaf_Mat.mdl` never mentions (it names `apple_leaf_basecolor.png`
    instead). This is what the render ACTUALLY shows for Common_Apple's
    leaves, so it is what any replacement material's own diffuse texture
    must be too.

    RE-WRITTEN 2026-08-31 (STREAM T6). The mechanism this used to pin --
    `Sdf.CopySpec` the whole Material+Shader subtree, then re-anchor every
    relative Asset attribute it copied (`_copy_tinted_material`/
    `_reanchor_relative_assets`) -- no longer exists: `_omnipbr_leaf_
    material` never copies the source subtree at all, it READS the
    effective texture path itself (`_leaf_texture_paths`, override-aware)
    and authors a FRESH, already-correctly-anchored relative path. So this
    now pins the two things that actually matter under the new mechanism:
    (1) `_leaf_texture_paths` resolves the OVERRIDE, not the MDL's own
    compiled-in default, for `Common_Apple`; (2) the real baked, COMPOSED
    `tree_Common_Apple_defoliated.usd` archetype's OmniPBR replacement
    binds that exact texture, archetype-relative, and it resolves on disk
    -- the same "does the launcher's Hydra see a real file" question the
    original hollyprivet defect (`References an asset that can not be
    found: './materials/textures/hollyprivet_basecolor.png'`, 4 times) was
    about, still asked here against the CURRENT authoring path.
    """
    src_abs = os.path.join(REPO, bht.TREE_SPECIES["Common_Apple"])
    src_stage = Usd.Stage.Open(src_abs)
    mat_prim = UsdShade.Material(src_stage.GetPrimAtPath("/Root/Looks/Apple_leaf_Mat"))
    check(bool(mat_prim), "sanity: Common_Apple/Looks/Apple_leaf_Mat exists")
    found_material_level_asset = any(
        p.GetTypeName() == "Material"
        for p in Usd.PrimRange(mat_prim.GetPrim())
        for _, _ in bht._prim_asset_attrs(p))
    check(found_material_level_asset,
          "sanity: Apple_leaf_Mat's MATERIAL prim itself carries a relative "
          "Asset attribute (the hollyprivet override) -- if this ever stops "
          "being true the rest of this test is checking nothing")

    # (1) the read side -- `_leaf_texture_paths` must pick the OVERRIDE
    # (hollyprivet_basecolor.png), not the MDL's own compiled-in default
    # (apple_leaf_basecolor.png).
    tex = bht._leaf_texture_paths(src_abs, mat_prim)
    check(tex["diffuse"] is not None, "sanity: a diffuse texture was found at all")
    if tex["diffuse"] is not None:
        check(os.path.basename(tex["diffuse"]) == "hollyprivet_basecolor.png",
              "expected the MATERIAL-level override (hollyprivet_basecolor.png), "
              "got {0} -- the override is not winning over the MDL default"
              .format(os.path.basename(tex["diffuse"])))
        check(bht._case_exact_exists(tex["diffuse"]),
              "the resolved override path must exist, case-exact, on disk: {0}"
              .format(tex["diffuse"]))

    # (2) the write side -- the real baked archetype's OmniPBR replacement
    # binds that same texture, archetype-relative, and it resolves.
    d = _bake_dir()
    arch_path = os.path.join(d, "tree_Common_Apple_defoliated.usd")
    arch_stage = Usd.Stage.Open(arch_path)
    # the salt is the foliage rel's own name -- Common_Apple's single leaf
    # mesh is named "Apple_leaf_Mat" (see `_classify`'s docstring: these
    # assets name a mesh after its bound material), so the replacement is
    # "Apple_leaf_Mat_Apple_leaf_Mat"; look it up by prefix instead of
    # hard-coding that in case the source asset's own naming ever changes.
    tinted = None
    for prim in Usd.PrimRange(arch_stage.GetPrimAtPath("/Root/tint_mats")):
        if prim.GetTypeName() == "Material" and prim.GetName().startswith("Apple_leaf_Mat"):
            tinted = prim
            break
    check(tinted is not None,
          "the defoliated archetype has an OmniPBR replacement for Apple_leaf_Mat "
          "(if this is None the rest of the check has nothing to verify)")
    if tinted is None:
        return
    shd = None
    for prim in Usd.PrimRange(tinted):
        if prim.GetTypeName() == "Shader":
            shd = UsdShade.Shader(prim)
            break
    check(shd is not None, "the replacement material has a Shader child")
    if shd is None:
        return
    tex_inp = shd.GetInput("diffuse_texture")
    tex_val = tex_inp.Get() if tex_inp else None
    check(tex_val is not None, "the replacement Shader authors diffuse_texture")
    if tex_val is not None:
        check(os.path.basename(tex_val.path) == "hollyprivet_basecolor.png",
              "the baked replacement's diffuse_texture ('{0}') should be the "
              "same override the source Material carries".format(tex_val.path))
        check(bool(tex_val.resolvedPath),
              "unresolved diffuse_texture in the baked, COMPOSED archetype: "
              "'{0}' -- exactly the class of defect the original hollyprivet "
              "bug was (a relative path that only resolves against the WRONG "
              "directory)".format(tex_val.path))

    # every OTHER relative Asset attribute on the replacement must ALSO
    # resolve (`bht._is_relative_asset_path` -- STREAM T6 -- correctly
    # excludes the Shader's own `info:mdl:sourceAsset = "OmniPBR.mdl"`,
    # a Kit BUILT-IN module with no on-disk file next to this or any
    # asset; see that function's own docstring).
    unresolved = []
    for prim in Usd.PrimRange(tinted):
        for attr in prim.GetAttributes():
            if not attr.HasAuthoredValue():
                continue
            v = attr.Get()
            if isinstance(v, Sdf.AssetPath) and bht._is_relative_asset_path(v.path):
                if not v.resolvedPath:
                    unresolved.append((str(prim.GetPath()), attr.GetName(), v.path))
    check(not unresolved,
          "unresolved relative Asset path(s) in the baked, COMPOSED "
          "archetype: {0}".format(unresolved))


@strict
def test_no_unresolved_relative_asset_paths_in_any_archetype():
    """Deliverable #3's own audit assertion, run against the REAL bake:
    every relative `Sdf.AssetPath` anywhere in every one of the 34 baked
    archetypes must resolve on disk once composed. Reuses `hurricane_tree_
    audit`'s whole-stage walk rather than re-deriving a second one here.
    """
    import hurricane_tree_audit as hta
    d = _bake_dir()
    n_checked = 0
    n_bad = 0
    for fn in sorted(os.listdir(d)):
        if not fn.endswith(".usd"):
            continue
        stage = Usd.Stage.Open(os.path.join(d, fn))
        rows = hta._all_relative_asset_paths(stage)
        n_checked += len(rows)
        for prim_path, attr_name, rel_path, resolved in rows:
            if not resolved:
                n_bad += 1
                check(False, "{0}: {1}.{2} = '{3}' does not resolve".format(
                    fn, prim_path, attr_name, rel_path))
    check(n_checked > 0, "sanity: checked at least one relative Asset path")
    print("    ({0} relative Asset paths checked across {1}, {2} unresolved)"
          .format(n_checked, d, n_bad))


# ---------------------------------------------------------------------------
# 3. partial defoliation, weighted low/inner, at every crown shape
# ---------------------------------------------------------------------------

@strict
def test_defoliated_keeps_70_to_85_percent_of_cards():
    """Every species, including the three that ship their whole crown as
    ONE mesh and the one whose leaf instancers only have 3 groups -- both
    shapes that a whole-PRIM cull cannot hit this target with (see
    `_select_kept_cards`'s own per-card rewrite for why).

    RE-TARGETED 2026-08-31 (STREAM T5) from 8-15% to 70-85% -- see
    `bake_hurricane_trees._FOL_KEEP`'s own comment for why `defoliated` now
    keeps the MOST of any damaged level rather than the least: the crown
    stays visible and reads its damage through colour (a strong brown tint,
    checked elsewhere in this file), only thinned enough to break the
    perfectly round `pristine` silhouette.
    """
    for species, rel in sorted(bht.TREE_SPECIES.items()):
        if species in bht.CONIFERS:
            continue
        src_abs = os.path.join(REPO, rel)
        src_stage = Usd.Stage.Open(src_abs)
        foliage, _ = bht._classify(src_stage)
        default = src_stage.GetDefaultPrim()
        xcache = UsdGeom.XformCache()
        inv = bht._foliage_inventory(src_stage, default.GetPath(), foliage, xcache)
        rng = random.Random("{0}/defoliated".format(species))
        kept = bht._select_kept_cards(inv, bht._FOL_KEEP["defoliated"], "low_inner", rng)
        total = sum(v["n"] for v in inv.values())
        n_kept = sum(len(k) for k in kept.values())
        frac = n_kept / total if total else 0.0
        check(0.68 <= frac <= 0.87,
              "{0}: defoliated kept {1}/{2} = {3:.1%} (want 70-85%, "
              "+-2pt for rounding)".format(species, n_kept, total, frac))


@strict
def test_defoliated_bias_favours_lower_inner_crown():
    """The KEPT subset's mean (z, radius) must be lower than the HIDDEN
    subset's, for a species with a wide enough crown to tell the
    difference (Black_Oak: 8 foliage groups spanning the whole canopy).
    """
    species, rel = "Black_Oak", bht.TREE_SPECIES["Black_Oak"]
    src_abs = os.path.join(REPO, rel)
    src_stage = Usd.Stage.Open(src_abs)
    foliage, _ = bht._classify(src_stage)
    default = src_stage.GetDefaultPrim()
    xcache = UsdGeom.XformCache()
    inv = bht._foliage_inventory(src_stage, default.GetPath(), foliage, xcache)
    rng = random.Random("Black_Oak/defoliated")
    kept = bht._select_kept_cards(inv, bht._FOL_KEEP["defoliated"], "low_inner", rng)

    def _mean_zr(idx_map):
        zs, rs = [], []
        for rel_, idx in idx_map.items():
            if len(idx) == 0:
                continue
            zs.extend(inv[rel_]["z"][idx])
            rs.extend(inv[rel_]["r"][idx])
        return (float(np.mean(zs)) if zs else None, float(np.mean(rs)) if rs else None)

    hidden = {rel_: np.array([i for i in range(v["n"]) if i not in set(kept.get(rel_, []))])
              for rel_, v in inv.items()}
    kz, kr = _mean_zr(kept)
    hz, hr = _mean_zr(hidden)
    check(kz is not None and hz is not None and kz < hz,
          "kept mean z ({0}) should be lower than hidden mean z ({1})".format(kz, hz))
    check(kr is not None and hr is not None and kr < hr,
          "kept mean radius ({0}) should be smaller than hidden mean radius "
          "({1})".format(kr, hr))


@strict
def test_kept_instancer_foliage_is_tinted_not_green():
    """THE DISCARDED-TINT BUG, PINNED -- AND RE-VERIFIED UNDER THE STREAM T6
    MECHANISM. `_author_foliage_cull`'s instancer branch used to compute
    `tinted_rels` (which prototypes have surviving instances) and then
    never act on it -- every PointInstancer-crowned species (`Black_Oak`,
    `Douglas_Fir`, `Shumard_Oak`: most of the pool) kept its residual
    10-30% of foliage cards fully GREEN and untinted at every damaged
    level, because only the invisibleIds got authored and nothing ever
    rebound the shared PROTOTYPE mesh to a tinted copy.

    RE-TARGETED 2026-08-31 (STREAM T6). This used to check the raw
    `diffuse_tint` VALUE's own ordering (`r >= g >= b`) -- correct for the
    STREAM T5 design, where `diffuse_tint` WAS the absolute brown colour
    (a single, texture-independent constant, `TINT_BROADLEAF`), but wrong
    for the STREAM T6 replacement, where `diffuse_tint` is a per-TEXTURE
    COMPENSATING factor (`texel_mean * diffuse_tint ~= TARGET_BROADLEAF`)
    that need not itself be brown-ordered: measured, `Black_Oak`'s own
    `Shumard_Oak_leaf_Mat2` variant computes `diffuse_tint = (2.947, 0.998,
    3.000)` -- B's tint EXCEEDS R's -- yet the PRODUCT with that texture's
    own dark, low-blue texel mean is (0.300, 0.170, 0.037): still
    unambiguously brown. So this test now checks the quantity that
    actually reaches the screen -- `diffuse_texture`'s own resolved
    texel mean, TIMES `diffuse_tint` -- not the tint tuple alone. Also
    swaps the old bare "/tint_mats/ in path" check for `hurricane_tree_
    audit._is_omnipbr_with_tint` (see that function's own docstring for
    why a path-only check would not have caught the STREAM T5 regression
    it is meant to guard against).

    Checked on `Black_Oak` (the widest instancer species, 4 leaf groups)
    at `defoliated`: every surviving instance's PROTOTYPE must be bound to
    a genuine OmniPBR-with-tint replacement, and TEXEL * TINT must read
    brown (R > G > B), never green.
    """
    import hurricane_tree_audit as hta

    d = _bake_dir()
    stage = Usd.Stage.Open(os.path.join(d, "tree_Black_Oak_defoliated.usd"))
    n_checked = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetTypeName() != "PointInstancer":
            continue
        pi = UsdGeom.PointInstancer(prim)
        inv_ids = pi.GetInvisibleIdsAttr().Get()
        pos = pi.GetPositionsAttr().Get() or []
        n_kept = len(pos) - (len(inv_ids) if inv_ids else 0)
        if n_kept <= 0:
            continue  # this instancer group has nothing surviving to check
        for proto_path in pi.GetPrototypesRel().GetTargets():
            proto_prim = stage.GetPrimAtPath(proto_path)
            if not proto_prim or not proto_prim.IsValid():
                continue
            if not bht._is_foliage(proto_prim):
                continue  # a woody instancer (e.g. `branch1`'s bark) -- not this test's concern
            n_checked += 1
            mat = bht._bound_material(proto_prim)
            check(bool(mat), "{0}: kept prototype {1} has a bound material"
                  .format(prim.GetPath(), proto_path))
            if not mat:
                continue
            mp = str(mat.GetPath())
            check("/tint_mats/" in mp,
                  "{0}: kept prototype {1} bound to '{2}' -- expected the "
                  "STREAM T6 OmniPBR replacement under /Root/tint_mats/, "
                  "not the original (untinted, green) material".format(
                      prim.GetPath(), proto_path, mp))
            check(hta._is_omnipbr_with_tint(mat),
                  "{0}: material {1} is not a genuine OmniPBR shader with "
                  "an authored diffuse_tint (the exact regression class "
                  "this stream fixes: a copy of the CUSTOM species MDL "
                  "with a diffuse_tint override that never reached the "
                  "render)".format(prim.GetPath(), mp))
            shd = None
            for shp in Usd.PrimRange(mat.GetPrim()):
                if shp.GetTypeName() == "Shader":
                    shd = UsdShade.Shader(shp)
                    break
            check(shd is not None, "{0}: tinted material {1} has a Shader child"
                  .format(prim.GetPath(), mp))
            if shd is None:
                continue
            tint_inp = shd.GetInput("diffuse_tint")
            tint_val = tint_inp.Get() if tint_inp else None
            check(tint_val is not None,
                  "{0}: tinted material {1} authors diffuse_tint".format(
                      prim.GetPath(), mp))
            tex_inp = shd.GetInput("diffuse_texture")
            tex_val = tex_inp.Get() if tex_inp else None
            check(tex_val is not None and tex_val.resolvedPath,
                  "{0}: tinted material {1} authors a resolvable "
                  "diffuse_texture".format(prim.GetPath(), mp))
            if tint_val is None or tex_val is None or not tex_val.resolvedPath:
                continue
            texel_info = bht._alpha_weighted_linear_mean(tex_val.resolvedPath)
            check(texel_info is not None,
                  "{0}: {1}'s diffuse_texture is readable by PIL".format(
                      prim.GetPath(), tex_val.resolvedPath))
            if texel_info is None:
                continue
            texel = texel_info[0]
            tint = (float(tint_val[0]), float(tint_val[1]), float(tint_val[2]))
            pred = tuple(texel[c] * tint[c] for c in range(3))
            check(pred[0] > pred[1] > pred[2],
                  "{0}: texel {1} * diffuse_tint {2} = predicted colour {3} "
                  "should read brown (R>G>B), not green".format(
                      prim.GetPath(), texel, tint, pred))
    check(n_checked > 0, "sanity: at least one kept foliage prototype was checked")


@strict
def test_hue_flip_tint_ratio_exceeds_texel_ratio_for_every_species():
    """NEW (STREAM T6): pins the actual HUE-FLIP invariant directly, for
    EVERY species' foliage material, not just `Black_Oak`'s.
    `tint_R / tint_G > texel_G / texel_R` is algebraically identical to
    `texel_R * tint_R > texel_G * tint_G` (cross-multiply both sides by the
    positive quantity `texel_R * texel_G`) -- i.e. "the predicted RED
    channel ends up strictly brighter than the predicted GREEN channel",
    which is the one inequality that actually flips a green texture's hue
    to brown. Checked on the REAL bake (`bake_hurricane_trees._species_
    material_report`, populated as a side effect of `_bake_dir()`'s full
    bake), not a synthetic texel, so this also catches a future species
    whose texture is dark/green enough that `_hue_flip_tint`'s per-channel
    cap needs its own targeted correction (see that function's docstring)
    to still land on a brown result.
    """
    _bake_dir()  # ensure `bht._species_material_report` is populated
    check(len(bht._species_material_report) > 0,
          "sanity: at least one OmniPBR foliage material was reported")
    for (species, mat_name, tag), row in sorted(bht._species_material_report.items()):
        texel = row["texel_mean_linear"]
        if texel is None:
            continue  # flat_fallback -- no texture, nothing to ratio-check
        tint = row["tint"]
        texel_r, texel_g = texel[0], texel[1]
        tint_r, tint_g = tint[0], tint[1]
        lhs = tint_r / max(tint_g, 1e-9)
        rhs = texel_g / max(texel_r, 1e-9)
        check(lhs > rhs,
              "{0}/{1} [{2}]: tint_R/tint_G = {3:.4f} should exceed "
              "texel_G/texel_R = {4:.4f} -- the hue must actually flip "
              "(equivalent to predicted_R > predicted_G, i.e. brown, not "
              "green)".format(species, mat_name, tag, lhs, rhs))
        pred = row["predicted_linear"]
        check(pred[0] > pred[1] > pred[2],
              "{0}/{1} [{2}]: predicted colour {3} is not strictly brown "
              "(R>G>B)".format(species, mat_name, tag, pred))


@strict
def test_wholly_kept_foliage_rel_still_gets_tinted():
    """THE LATENT FULLY-KEPT-REL BUG, PINNED (found reviewing Correction 1's
    severed-top material audit). `_author_foliage_cull` used to `continue`
    unconditionally whenever a rel's OWN cards were 100% kept (`len(keep_
    idx) == v["n"]`) -- reachable even when the GLOBAL keep fraction across
    the whole crown is far below 1.0, because `_select_kept_cards` selects
    per-CARD pooled across every rel: a small rel can land entirely inside
    the kept set purely by chance while the crown as a whole is mostly
    culled. Skipping the tint step in that case left the rel bound to the
    RAW, GREEN, UNTOUCHED original -- the cotton-ball class of bug, reached
    through a different door than the ones `test_kept_instancer_foliage_
    is_tinted_not_green` above already covers (that test only exercises a
    PARTIALLY kept rel).

    Not reproduced by the current seeded bakes (`hurricane_tree_audit.py`
    reports zero untinted-leaf faults across all 34 shipped archetypes), so
    this exercises `_author_foliage_cull` DIRECTLY with a crafted `kept`
    dict rather than hoping a future re-seed happens to hit it. Uses
    `Black_Oak`, which has BOTH a plain-mesh foliage rel (`branch3`) and an
    instancer foliage rel (`branch2_instancer`) so both of `_author_
    foliage_cull`'s branches are exercised in one pass.
    """
    species = "Black_Oak"
    src_abs = os.path.join(REPO, bht.TREE_SPECIES[species])
    src_stage = Usd.Stage.Open(src_abs)
    foliage, _ = bht._classify(src_stage)
    root_path = src_stage.GetDefaultPrim().GetPath()
    xcache = UsdGeom.XformCache()
    inv = bht._foliage_inventory(src_stage, root_path, foliage, xcache)

    mesh_small = next((r for r, v in inv.items() if v["kind"] == "mesh"), None)
    inst_small = next((r for r, v in inv.items() if v["kind"] == "instancer"), None)
    check(mesh_small is not None, "sanity: {0} has a plain-mesh foliage rel"
          .format(species))
    check(inst_small is not None, "sanity: {0} has an instancer foliage rel"
          .format(species))
    if mesh_small is None or inst_small is None:
        return

    # Craft `kept`: `mesh_small` and `inst_small` are FORCED wholly kept
    # (the exact scenario the old code skipped); everything else gets a
    # small partial subset (or nothing), so the overall crown keep fraction
    # stays low -- this is not "pristine" in disguise.
    kept = {}
    for rel, v in inv.items():
        if rel in (mesh_small, inst_small):
            kept[rel] = np.arange(v["n"])
        elif v["n"] > 0:
            kept[rel] = np.arange(max(1, int(v["n"] * 0.10)))
        else:
            kept[rel] = np.zeros(0, dtype=int)

    out_dir = tempfile.mkdtemp(prefix="hur_tree_fullykept_")
    try:
        out_path = os.path.join(out_dir, "probe.usd")
        stage = Usd.Stage.CreateNew(out_path)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, Sdf.Path("/Root"))
        stage.SetDefaultPrim(root.GetPrim())
        src = stage.DefinePrim(Sdf.Path("/Root/src"), "Xform")
        src.GetReferences().AddReference(bht._relpath(src_abs, out_path))
        bht._author_foliage_cull(stage, src_stage, src_abs, out_path, root_path,
                                  inv, kept, bht.TARGET_BROADLEAF, xcache)
        stage.GetRootLayer().Save()

        check2 = Usd.Stage.Open(out_path)

        # -- the wholly-kept INSTANCER rel's prototype must be tinted -----
        src_inst = src_stage.GetPrimAtPath(root_path.AppendPath(inst_small))
        for proto_path in UsdGeom.PointInstancer(src_inst).GetPrototypesRel().GetTargets():
            proto_rel = proto_path.MakeRelativePath(root_path)
            out_proto = check2.GetPrimAtPath(Sdf.Path("/Root/src").AppendPath(proto_rel))
            check(out_proto.IsValid(),
                  "wholly-kept instancer rel {0}: prototype exists on output stage"
                  .format(inst_small))
            mat = bht._bound_material(out_proto) if out_proto.IsValid() else None
            mp = str(mat.GetPath()) if mat else ""
            check("/tint_mats/" in mp,
                  "wholly-kept instancer rel {0}: prototype bound to '{1}' -- "
                  "expected a tinted copy under /Root/tint_mats/, not the raw "
                  "original".format(inst_small, mp))

        # -- the wholly-kept plain-MESH rel must be tinted, in place -------
        v_mesh = inv[mesh_small]
        mesh_prim = src_stage.GetPrimAtPath(v_mesh["mesh_path"])
        mesh_rel = mesh_prim.GetPath().MakeRelativePath(root_path)
        out_mesh = check2.GetPrimAtPath(Sdf.Path("/Root/src").AppendPath(mesh_rel))
        check(out_mesh.IsValid(),
              "wholly-kept mesh rel {0}: mesh exists on output stage".format(mesh_small))
        check(UsdGeom.Imageable(out_mesh).ComputeVisibility() != UsdGeom.Tokens.invisible,
              "wholly-kept mesh rel {0}: stays VISIBLE (nothing of its own "
              "cards was hidden)".format(mesh_small))
        mat = bht._bound_material(out_mesh) if out_mesh.IsValid() else None
        mp = str(mat.GetPath()) if mat else ""
        check("/tint_mats/" in mp,
              "wholly-kept mesh rel {0} bound to '{1}' -- expected a tinted "
              "copy under /Root/tint_mats/, not the raw original".format(
                  mesh_small, mp))
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# 4. the snag: geometric clip, ground-level pivot, stump stays standing
# ---------------------------------------------------------------------------

@strict
def test_snag_is_a_real_clip_not_a_thinned_crown():
    d = _bake_dir()
    for species in sorted(bht.TREE_SPECIES):
        path = os.path.join(d, "tree_{0}_snapped.usd".format(species))
        stage = Usd.Stage.Open(path)
        stump = stage.GetPrimAtPath("/Root/src/trunk_cut_below")
        top = stage.GetPrimAtPath("/Root/top/trunk_cut_above")
        check(stump.IsValid() and UsdGeom.Mesh(stump).GetPointsAttr().Get(),
              "{0}: stump has a real clipped mesh with points".format(species))
        check(top.IsValid(), "{0}: a severed-top clip prim exists (may be "
              "empty if the break landed above every triangle)".format(species))


@strict
def test_snag_stump_stands_and_top_touches_ground():
    """The stump's LOWEST point is planted at/near 0 (it never moved); the
    top's lowest point must be near ground -- NOT still up at the break
    height, which is what pivoting about the elevated break point (the
    first version of this bake) produced (measured 6.8-11.9 m up).
    """
    d = _bake_dir()
    for species in sorted(bht.TREE_SPECIES):
        path = os.path.join(d, "tree_{0}_snapped.usd".format(species))
        stage = Usd.Stage.Open(path)
        stump_z = [z for prim, pts in _world_visible_meshes(stage)
                   if str(prim.GetPath()).startswith("/Root/src") for z in pts[:, 2]]
        top_z = [z for prim, pts in _world_visible_meshes(stage)
                 if str(prim.GetPath()).startswith("/Root/top") for z in pts[:, 2]]
        check(bool(stump_z), "{0}: stump has visible geometry".format(species))
        if stump_z:
            # A FEW SPECIES' SOURCE MESH HAS A ROOT FLARE THAT DIPS SLIGHTLY
            # BELOW ITS OWN z=0 (measured: Shumard_Oak to -34 cm, Douglas_
            # Fir to -21 cm -- present identically on `pristine`, so it is
            # an asset property, not something this clip introduced). The
            # bound here is "never moved beyond that", not "exactly zero".
            check(min(stump_z) > -0.5,
                  "{0}: stump's lowest point ({1:.3f} m) should be at/near "
                  "grade -- it was never moved".format(species, min(stump_z)))
        check(bool(top_z), "{0}: severed top has visible geometry".format(species))
        if top_z:
            # "near ground" per the directive -- generous band, since a
            # crown resting on its own branch tips does not lie flat (the
            # wildfire/tornado precedent this file's own comments cite).
            check(min(top_z) < 1.0,
                  "{0}: severed top's lowest point ({1:.3f} m) should be "
                  "close to the ground, not still floating at the break "
                  "height".format(species, min(top_z)))
            check(max(top_z) < 6.0,
                  "{0}: severed top's highest point ({1:.3f} m) is "
                  "implausibly tall for something lying down".format(
                      species, max(top_z)))


@strict
def test_snapped_top_keeps_nonzero_foliage_on_every_crown_shape():
    """THE ROUNDING-TO-ZERO BUG, PINNED. The severed top's residual
    foliage used to be picked with the whole-PRIM `_cull(above_fol,
    SNAP_TOP_FOL_KEEP=0.20, rng)` -- fine for a species with several
    separate foliage groups above the break, but the three single-mesh-
    crown species (`Largetooth_Aspen`, `American_Beech`, `Common_Apple`)
    hand it a ONE-ITEM pool, and `round(1 * (1-0.20)) == 1` hides that one
    item outright: measured before this fix, every one of those three
    species' `snapped` archetype baked to 0% top foliage, not the intended
    ~20% residue "went over only just, at the very end" the module
    docstring describes. Fixed by reusing the per-CARD machinery
    (`_foliage_inventory`/`_select_kept_cards`/`_author_foliage_cull`)
    `bake_one` already uses for the standing levels.
    """
    d = _bake_dir()
    for species in ("Largetooth_Aspen", "American_Beech", "Common_Apple"):
        stats_path = os.path.join(d, "tree_{0}_snapped.usd".format(species))
        check(os.path.exists(stats_path), "{0}: snapped archetype exists".format(species))
        # Re-derive the actual kept/total the bake computed, the same way
        # `bake_snapped` itself does, rather than re-parsing prose stats.
        src_abs = os.path.join(REPO, bht.TREE_SPECIES[species])
        src_stage = Usd.Stage.Open(src_abs)
        foliage, _ = bht._classify(src_stage)
        check(len(foliage) == 1,
              "sanity: {0}'s whole crown is one foliage prim (if this is "
              "no longer true, this test's premise -- 'a 1-item pool "
              "rounds to zero' -- no longer applies to it)".format(species))
        stage = Usd.Stage.Open(stats_path)
        n_top_visible = 0
        n_top_total = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/Root/top")):
            if prim.GetTypeName() != "Mesh":
                continue
            name = prim.GetName()
            if not (name.endswith("_cards") or bht._is_foliage(prim)):
                continue
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            n = len(pts) if pts else 0
            if UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible:
                n_top_visible += n
        check(n_top_visible > 0,
              "{0}: severed top keeps SOME visible foliage geometry (was "
              "measured at exactly 0 before this fix)".format(species))


# ---------------------------------------------------------------------------
# 5. fallen trees: ground contact via POINTS, trunk planted, foliage ~30%
# ---------------------------------------------------------------------------

@strict
def test_fallen_trees_ground_contact_and_planted_base():
    d = _bake_dir()
    for species in sorted(bht.TREE_SPECIES):
        path = os.path.join(d, "tree_{0}_fallen.usd".format(species))
        stage = Usd.Stage.Open(path)
        meshes = _world_visible_meshes(stage)
        check(bool(meshes), "{0} fallen: has visible geometry".format(species))
        if not meshes:
            continue
        all_z = np.concatenate([pts[:, 2] for _, pts in meshes])
        # the crown lies ON the ground (some point near/below grade)
        check(all_z.min() < 0.30,
              "{0} fallen: lowest point {1:.3f} m -- expected the crown to "
              "reach near the ground".format(species, all_z.min()))
        # the base of the trunk stays at the origin (lift == 0)
        keep_wood, lean, lift = bht._WOOD_PLAN["fallen"]
        check(lift == 0.0, "{0}: `fallen` lift must be 0 (trunk planted)".format(species))


@strict
def test_retention_table_matches_stream_t5_directive():
    """RE-WRITTEN 2026-08-31 (STREAM T5), RE-WRITTEN AGAIN 2026-09-01
    (STREAM T7). The pre-T5 invariant here was "fallen keeps MORE than
    defoliated" (0.30 vs 0.12) -- true under the old "textbook leaf-loss"
    model. STREAM T5 flipped the table so damage reads through crown
    colour, not crown removal: `defoliated` became the level with the LEAST
    geometry removed, and the ladder was monotonically non-increasing from
    there (`pristine >= defoliated >= limbed >= max(leaning, fallen)`).

    STREAM T7 ("the trees all look perfect, need some of them to be broken
    down") breaks THAT invariant on purpose, and this test now checks the
    replacement rather than merely relaxing the old one. `leaning` (0.45)
    now keeps MORE than `limbed`/`fallen` (0.35 each) -- `limbed >= max
    (leaning, fallen)` is false by construction (0.35 < 0.45). Retention
    percentage stopped being the only channel damage travels through:
    `limbed` now carries its read through an ASYMMETRIC sector strip plus
    constructed break-debris geometry (`_author_limb_break`), and `leaning`/
    `fallen` both carry theirs partly through a constructed root-plate disc
    (`_author_root_plate_disc`) that a bare retention number cannot see at
    all -- see `test_limbed_gets_an_asymmetric_sector_strip_and_break_
    debris`/`test_leaning_and_fallen_get_a_root_plate_disc` below for the
    geometry-level checks that cover what this retention-only test cannot.
    So the invariants that remain are the ones still true regardless of
    which channel a level uses: `defoliated` is still the fullest of every
    damaged level (the mildest outcome, and the plurality of the plate), and
    the severed `snapped` top is still the emptiest of everything (it is the
    one level with an actual geometric break, not just a pose/colour/debris
    change).
    """
    bands = {
        "defoliated": (0.70, 0.85),
        "limbed": (0.30, 0.40),
        "leaning": (0.40, 0.50),
        "fallen": (0.30, 0.40),
    }
    check(bht._FOL_KEEP["pristine"] == 1.0, "pristine keeps 100% of cards")
    for level, (lo, hi) in sorted(bands.items()):
        v = bht._FOL_KEEP[level]
        check(lo <= v <= hi,
              "_FOL_KEEP['{0}'] = {1} outside its directive band {2}-{3}"
              .format(level, v, lo, hi))
    damaged = ("defoliated", "limbed", "leaning", "fallen")
    check(bht._FOL_KEEP["defoliated"] == max(bht._FOL_KEEP[l] for l in damaged),
          "defoliated ({0}) must still keep the MOST of any standing "
          "damaged level -- it is the mildest outcome and the plate's "
          "plurality level, even though the ladder below it is no longer "
          "monotonic (leaning {1} > limbed {2} / fallen {3})".format(
              bht._FOL_KEEP["defoliated"], bht._FOL_KEEP["leaning"],
              bht._FOL_KEEP["limbed"], bht._FOL_KEEP["fallen"]))
    check(0.20 <= bht.SNAP_TOP_FOL_KEEP <= 0.30,
          "SNAP_TOP_FOL_KEEP ({0}) outside its directive band 20-30%"
          .format(bht.SNAP_TOP_FOL_KEEP))
    check(bht.SNAP_TOP_FOL_KEEP == min([bht.SNAP_TOP_FOL_KEEP]
                                        + [bht._FOL_KEEP[l] for l in damaged]),
          "the severed snapped top ({0}) should keep the LEAST of every "
          "level, standing or severed -- it is the one level with a real "
          "geometric break".format(bht.SNAP_TOP_FOL_KEEP))


# ---------------------------------------------------------------------------
# 5b. STREAM T7 (2026-09-01) — visible breakage on the structural levels.
# "the trees all look perfect, need some of them to be broken down."
# ---------------------------------------------------------------------------

@strict
def test_limbed_gets_an_asymmetric_sector_strip():
    """`limbed`'s crown thinning is no longer a single global fraction: a
    seeded azimuth SECTOR is stripped far harder than the rest of the
    crown (`bake_hurricane_trees._select_kept_cards_sector`), so the tree
    reads as broken on ONE SIDE, not uniformly thinner all over.

    Measured directly on the baked archetype by bucketing every foliage
    INSTANCE by azimuth about the trunk axis and comparing the emptiest
    bin's visible fraction against the fullest -- not by re-deriving the
    exact sector centre/half-width from `bake_one`'s own rng draw order,
    which would couple this test to an internal detail (draw order) that
    is free to change independently of the actual visible result.

    Restricted to INSTANCER-backed foliage (`invisibleIds` gives an exact
    per-item visibility list); a mesh-crown species' "*_cards" replacement
    does not preserve a cheap index-to-original-triangle map, so the
    signal is read from whichever of Black_Oak/Shumard_Oak/Douglas_Fir
    have it (real, per this asset family's own construction — see the
    build-wildfire-scenes skill's "NOT EVERY TREE IS INSTANCED" section).
    """
    d = _bake_dir()
    checked = 0
    for species, rel in sorted(bht.TREE_SPECIES.items()):
        src_abs = os.path.join(REPO, rel)
        src_stage = Usd.Stage.Open(src_abs)
        foliage, _ = bht._classify(src_stage)
        root_path = src_stage.GetDefaultPrim().GetPath()
        xcache = UsdGeom.XformCache()
        inv = bht._foliage_inventory(src_stage, root_path, foliage, xcache)
        inst_rels = [r for r, v in inv.items()
                     if v["kind"] == "instancer" and v["n"] >= 40]
        if not inst_rels:
            continue
        path = os.path.join(d, "tree_{0}_limbed.usd".format(species))
        if not os.path.exists(path):
            continue
        stage = Usd.Stage.Open(path)
        n_bins = 12
        vis = np.zeros(n_bins, dtype=np.int64)
        tot = np.zeros(n_bins, dtype=np.int64)
        for rel_ in inst_rels:
            v = inv[rel_]
            n = v["n"]
            bins = (np.floor((v["theta"] + math.pi) / (2.0 * math.pi) * n_bins)
                    .astype(int) % n_bins)
            over = stage.GetPrimAtPath(Sdf.Path("/Root/src").AppendPath(rel_))
            vis_mask = np.ones(n, dtype=bool)
            if over and over.IsValid():
                if (UsdGeom.Imageable(over).ComputeVisibility()
                        == UsdGeom.Tokens.invisible):
                    vis_mask[:] = False
                else:
                    inv_ids = UsdGeom.PointInstancer(over).GetInvisibleIdsAttr().Get() or []
                    for iid in inv_ids:
                        if 0 <= int(iid) < n:
                            vis_mask[int(iid)] = False
            np.add.at(tot, bins, 1)
            if vis_mask.any():
                np.add.at(vis, bins[vis_mask], 1)
        populated = tot > 5
        if populated.sum() < 6:
            continue
        frac = vis[populated] / tot[populated]
        lo_frac, hi_frac = float(frac.min()), float(frac.max())
        check(lo_frac < hi_frac - 0.15,
              "{0} limbed: emptiest populated azimuth bin ({1:.0%} visible) "
              "is not meaningfully stripped relative to the fullest "
              "({2:.0%}) -- expected an asymmetric, one-sided strip"
              .format(species, lo_frac, hi_frac))
        checked += 1
    check(checked >= 2,
          "at least 2 instancer-crowned species were actually exercised "
          "by this check (found {0})".format(checked))


@strict
def test_limbed_gets_broken_limb_stubs_and_fallen_debris():
    """2-4 bare limb stubs still attached to the tree (children of
    `/Root/src`) plus 1-2 of their fallen counterparts on the ground
    (children of `/Root` directly, at z=0) per `limbed` archetype -- see
    `bake_hurricane_trees._author_limb_break`. World-z verified via ACTUAL
    TRANSFORMED POINTS (`_world_visible_meshes`), never `BBoxCache` -- see
    the build-tornado-scenes skill's "`UsdGeom.BBoxCache` CANNOT SEAT
    DEBRIS" section for why that cache alone is not proof of anything here.
    """
    d = _bake_dir()
    for species in sorted(bht.TREE_SPECIES):
        path = os.path.join(d, "tree_{0}_limbed.usd".format(species))
        stage = Usd.Stage.Open(path)
        meshes = _world_visible_meshes(stage)
        stubs = [(p, pts) for p, pts in meshes
                 if str(p.GetPath()).startswith("/Root/src/limb_stub_")]
        fallen = [(p, pts) for p, pts in meshes
                  if str(p.GetPath()).startswith("/Root/fallen_limb_")]
        check(2 <= len(stubs) <= 4,
              "{0} limbed: 2-4 broken limb stub(s) authored (found {1})"
              .format(species, len(stubs)))
        check(1 <= len(fallen) <= 2,
              "{0} limbed: 1-2 fallen limb(s) authored (found {1})"
              .format(species, len(fallen)))
        all_z = (np.concatenate([pts[:, 2] for _, pts in meshes])
                 if meshes else np.zeros(0))
        tree_zmax = float(all_z.max()) if len(all_z) else 0.0
        for p, pts in stubs:
            mat = bht._bound_material(p)
            check(bool(mat), "{0}: stub {1} binds a material"
                  .format(species, p.GetPath()))
            zmin, zmax = float(pts[:, 2].min()), float(pts[:, 2].max())
            check(zmin > -0.05, "{0}: stub {1} min z {2:.3f} m is below "
                  "grade".format(species, p.GetPath(), zmin))
            check(zmax <= tree_zmax + 0.5, "{0}: stub {1} max z {2:.3f} m "
                  "exceeds the tree's own height ({3:.3f} m)".format(
                      species, p.GetPath(), zmax, tree_zmax))
        for p, pts in fallen:
            mat = bht._bound_material(p)
            check(bool(mat), "{0}: fallen limb {1} binds a material"
                  .format(species, p.GetPath()))
            zmin = float(pts[:, 2].min())
            # "seated at z=0 in archetype space": bedded a hair below 0
            # (see `_author_limb_break`'s own bed-in comment), never
            # floating above it -- the tornado skill's own shadow-
            # displacement argument for why a few cm matters.
            check(-0.03 <= zmin <= 0.03,
                  "{0}: fallen limb {1} lowest point {2:.4f} m is not "
                  "seated at z=0 (archetype space)".format(
                      species, p.GetPath(), zmin))


@strict
def test_leaning_and_fallen_get_a_root_plate_disc():
    """`leaning`/`fallen` gain a torn root-plate disc (`/Root/rootplate`)
    at the tree's base -- see `bake_hurricane_trees._author_root_plate_
    disc`. Partly BELOW grade by construction (a root plate is a hole in
    the ground with the plug beside it, not a coin on the lawn -- see that
    function's own docstring, carried over from `vegetation.root_plate`'s
    identical note), so the check is "reaches at/below grade and does not
    float implausibly high", not "sits exactly at z=0".
    """
    d = _bake_dir()
    xcache = UsdGeom.XformCache()
    for level in ("leaning", "fallen"):
        r_lo, r_hi = bht._ROOTPLATE_R_M[level]
        for species in sorted(bht.TREE_SPECIES):
            path = os.path.join(d, "tree_{0}_{1}.usd".format(species, level))
            if not os.path.exists(path):
                continue  # NO_UPROOT species skip `fallen`
            stage = Usd.Stage.Open(path)
            prim = stage.GetPrimAtPath("/Root/rootplate")
            check(prim.IsValid(),
                  "{0} {1}: root-plate disc exists".format(species, level))
            if not prim.IsValid():
                continue
            mat = bht._bound_material(prim)
            check(bool(mat), "{0} {1}: root-plate disc binds a material"
                  .format(species, level))
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            check(bool(pts), "{0} {1}: root-plate disc has points"
                  .format(species, level))
            if not pts:
                continue
            m = np.array(xcache.GetLocalToWorldTransform(prim))
            arr = np.array(pts, dtype=np.float64)
            world = np.hstack([arr, np.ones((len(arr), 1))]) @ m
            zmin, zmax = float(world[:, 2].min()), float(world[:, 2].max())
            xy_r = float(np.hypot(world[:, 0], world[:, 1]).max())
            check(zmin < 0.05,
                  "{0} {1}: root-plate disc lowest point {2:.3f} m should "
                  "reach at/below grade".format(species, level, zmin))
            check(zmax < 2.5 * r_hi,
                  "{0} {1}: root-plate disc highest point {2:.3f} m is "
                  "implausibly tall for a {3:.1f}-{4:.1f} m-radius disc"
                  .format(species, level, zmax, r_lo, r_hi))
            check(xy_r <= r_hi * 1.4,
                  "{0} {1}: root-plate disc reaches {2:.2f} m from the "
                  "trunk axis, beyond its own {3:.1f} m radius band + "
                  "jitter".format(species, level, xy_r, r_hi))


@strict
def test_snapped_gets_a_jagged_break_collar():
    """The stump's break plane gains 3-5 small bark spikes
    (`/Root/src/snag_collar_NN`) instead of the bare, almost sawn-looking
    disc `_clip_mesh_flat`'s plane cut otherwise leaves -- see
    `bake_hurricane_trees._snap_collar`. Material resolvability is already
    covered by `test_every_visible_mesh_has_a_resolvable_material` (it
    walks the whole stage); this test checks count, presence and that each
    spike sits near the stump's own break-height range rather than
    somewhere arbitrary.
    """
    d = _bake_dir()
    xcache = UsdGeom.XformCache()
    for species in sorted(bht.TREE_SPECIES):
        path = os.path.join(d, "tree_{0}_snapped.usd".format(species))
        stage = Usd.Stage.Open(path)
        spikes = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/Root/src"))
                  if p.GetName().startswith("snag_collar_")]
        check(3 <= len(spikes) <= 5,
              "{0} snapped: 3-5 collar spike(s) authored (found {1})"
              .format(species, len(spikes)))
        meshes = _world_visible_meshes(stage)
        stump_pts = [pts for p, pts in meshes
                     if str(p.GetPath()).startswith("/Root/src")
                     and not p.GetName().startswith("snag_collar_")]
        if not stump_pts:
            continue
        stump_all = np.concatenate([p[:, 2] for p in stump_pts])
        stump_zmin, stump_zmax = float(stump_all.min()), float(stump_all.max())
        for p in spikes:
            pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
            if not pts:
                continue
            m = np.array(xcache.GetLocalToWorldTransform(p))
            arr = np.array(pts, dtype=np.float64)
            world = np.hstack([arr, np.ones((len(arr), 1))]) @ m
            zc = float(world[:, 2].mean())
            check(stump_zmin - 0.1 <= zc <= stump_zmax + 0.1,
                  "{0}: collar spike {1} centre z {2:.3f} m is not near "
                  "the stump's own height range [{3:.3f}, {4:.3f}]".format(
                      species, p.GetPath(), zc, stump_zmin, stump_zmax))


# ---------------------------------------------------------------------------
# 6. the windthrow boost — L2 reachability, replayed against real GT
# ---------------------------------------------------------------------------

_GT_DIR = os.path.expanduser("~/hurricane_previews")


def _load_gt(name):
    path = os.path.join(_GT_DIR, name, "GT_hurricane.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


@strict
def test_windthrow_boost_anchors():
    """Anchors re-tuned 2026-08-31 (STREAM T3) alongside `_TREE_CUTS`'s
    re-cut -- 0.10/0.25 -> 0.22/0.55 -- then re-tuned AGAIN 2026-08-31
    (STREAM T4, Correction 2) to 0.21/0.51: a small, deliberate reduction of
    the WET contribution to make room for the new `dry_windthrow_chance`
    term, which adds a DISJOINT (dry-population) structural share on top of
    this one. See `hurricane.py`'s own comment on `_WINDTHROW_BOOST_HI` for
    why the un-compensated 0.22/0.55 anchors overshot "L3 total near 30%"
    once the dry term existed.
    """
    check(hu.windthrow_depth_boost(0.0) == 0.0, "dry: boost is exactly 0")
    check(abs(hu.windthrow_depth_boost(0.3) - 0.21) < 1e-9, "0.3 m -> 0.21")
    check(abs(hu.windthrow_depth_boost(1.0) - 0.51) < 1e-9, "1.0 m -> 0.51")
    check(abs(hu.windthrow_depth_boost(5.0) - 0.51) < 1e-9, "saturates at 0.51 beyond 1 m")
    lo = hu.windthrow_depth_boost(0.5)
    hi = hu.windthrow_depth_boost(0.8)
    check(0.21 < lo < hi < 0.51, "monotone increasing between the anchors")


@strict
def test_dry_windthrow_chance_shape():
    """`dry_windthrow_chance` (Correction 2, STREAM T4): 0 at/below the
    onset, rising linearly above it, capped. Onset is 0.52, not the "~0.6"
    first guessed, BECAUSE 0.6 would leave the term nearly inert at L2 --
    see `hurricane.py`'s own comment on `_DRY_WINDTHROW_ONSET` for the
    measured L2 dry-population range (0.516-0.600) this is checked against.
    """
    check(hu.dry_windthrow_chance(0.0) == 0.0, "far below onset: 0")
    check(hu.dry_windthrow_chance(hu._DRY_WINDTHROW_ONSET) == 0.0,
          "exactly at onset: 0")
    check(hu.dry_windthrow_chance(0.60) > 0.0,
          "L2's dry-population ceiling (i=0.600) must have SOME nonzero "
          "chance, or the '~2% of L2 dry trees' target is unreachable")
    lo = hu.dry_windthrow_chance(0.60)
    hi = hu.dry_windthrow_chance(0.73)
    check(lo < hi, "monotone increasing above onset (checked at L2's dry "
          "ceiling vs L3's dry mean)")
    check(hu.dry_windthrow_chance(999.0) == hu._DRY_WINDTHROW_CAP,
          "capped at _DRY_WINDTHROW_CAP for extreme intensity")


@strict
def test_dry_windthrow_only_promotes_never_demotes():
    """The dry mechanism in `tree_level_for_intensity` may only PROMOTE a
    non-structural draw to `leaning` for a DRY tree -- it must never fire
    for a tree that is already structural (would be a silent no-op either
    way, since it only ever sets `leaning`, but pinning the GATE, not just
    the outcome, catches a future edit that makes it able to DEMOTE by
    accident) and must never fire for a WET tree (`depth_m` at/above
    `_DRY_WINDTHROW_DEPTH_M`), which the depth boost already covers.
    """
    order = list(hu.TREE_LEVELS)
    # A high intensity that would already draw `snapped` via the ladder
    # alone (no jitter noise -- use a fixed rng and a very high i).
    high_i = 2.5  # comfortably above every `_TREE_CUTS` cut
    for trial in range(20):
        rng = random.Random("promote-never-demote-{0}".format(trial))
        lv = hu.tree_level_for_intensity(high_i, rng, species=None, depth_m=0.0)
        check(lv == "snapped",
              "sanity: at i={0} the ladder alone should already read "
              "'snapped' regardless of any dry roll, got '{1}'".format(high_i, lv))
    # A WET tree (depth_m >= _DRY_WINDTHROW_DEPTH_M) must never be touched
    # by the dry mechanism. Chosen so the ladder+depth-boost path, even at
    # the TOP of the jitter band, cannot reach `leaning` on its own
    # (verified: i=0.60, depth=0.25 -> eff_max = 0.60 + boost(0.25) + 0.30
    # = 1.075, still under the 1.09 `limbed` cut -- STREAM T5 widened the
    # jitter from 0.26 to 0.30 alongside `_TREE_CUTS`'s re-cut, tightening
    # this margin from 0.055 to 0.015, still comfortably positive) -- so if
    # `leaning` (or worse) EVER appears across many trials, the dry
    # mechanism's depth gate must have leaked onto a wet tree, since nothing
    # else here can produce it at this configuration.
    wet_depth = hu._DRY_WINDTHROW_DEPTH_M + 0.05
    i_probe = 0.60
    for trial in range(200):
        rng = random.Random("wet-not-dry-promoted-{0}".format(trial))
        lv = hu.tree_level_for_intensity(i_probe, rng, species=None, depth_m=wet_depth)
        check(order.index(lv) <= order.index("limbed"),
              "a WET tree (depth={0}) at i={1} should never reach 'leaning' "
              "or worse (ladder+depth-boost alone tops out at 'limbed' "
              "here) -- got '{2}', which means the dry mechanism's depth "
              "gate leaked onto a wet tree".format(wet_depth, i_probe, lv))


@strict
def test_l2_field_alone_cannot_reach_structural_tree_levels():
    """Pins the BROKEN baseline this fix addresses: at L2's own measured
    field ceiling, with `depth_m=0`, the ladder must NOT be able to reach
    `leaning`/`fallen`/`snapped` even at the top of the jitter band.
    """
    i_ceiling = 0.598 + 0.30  # measured L2 field max + full jitter (0.30 as of STREAM T5)
    rng = random.Random(0)
    lv = hu._ladder(hu._TREE_CUTS, i_ceiling)
    check(lv in ("pristine", "defoliated", "limbed"),
          "at i={0:.3f}, depth=0, the ladder reads '{1}' -- should still be "
          "unable to reach leaning/fallen/snapped".format(i_ceiling, lv))


@strict
def test_windthrow_boost_makes_l2_reachable():
    i_ceiling = 0.598
    boosted = i_ceiling + hu.windthrow_depth_boost(1.0) + 0.30
    lv = hu._ladder(hu._TREE_CUTS, boosted)
    check(lv not in ("pristine", "defoliated", "limbed"),
          "with a 1 m depth boost at the same field ceiling, the ladder "
          "reads '{0}' -- should now reach leaning/fallen/snapped".format(lv))


@strict
def test_l2_l3_replay_against_real_gt():
    """Replays the launcher's EXACT per-tree draw (same seeds, same field
    config, same tree positions) from a real recorded scene, before and
    after the depth boost, and prints the tallies -- this is the offline
    equivalent of the render the directive asks for.

    L3'S SOURCE GT CHANGED 2026-08-31 (STREAM T5): from `V2_L3` to
    `FINAL3_L3_brown`, the more recent recorded L3 plate the T5 directive
    itself named (`~/hurricane_previews/FINAL3_L3_brown/GT_hurricane.json`)
    and the one whose tally (0 pristine trees) is the diagnosis this stream
    fixes. `V2_L2` is unchanged for L2 -- no more recent L2 recording was
    named.

    EXPECTED TALLIES, UPDATED 2026-08-31 (STREAM T5: the `_TREE_CUTS`
    `pristine`/`defoliated` re-cut and the jitter widening to 0.30 -- see
    that constant's own comment for the full reasoning). The windthrow
    boost and dry-windthrow mechanisms (`_WINDTHROW_BOOST_LO`/`_HI`,
    `_DRY_WINDTHROW_ONSET`/`_SLOPE`) are UNCHANGED by this stream, so the
    structural-share numbers below are nearly identical to the pre-T5
    (STREAM T4) measurement; what changed is `pristine` going from
    unreachable to a real minority/plurality, and `defoliated` overtaking
    `limbed` as L3's plurality level:

      L2 (`V2_L2`): wind-alone (depth forced 0) structural 1.3% (20/1504,
          all `leaning` -- the dry mechanism's only output); WITH real
          depth, PRISTINE 46.3% (697/1504, inside the 40-50% target),
          dry-subpopulation structural 2.35% (26/1107 dry trees, "~2%"
          target), total structural 7.91% (119/1504, inside 6-8%).
      L3 (`FINAL3_L3_brown`): wind-alone structural 5.2% (87/1684); WITH
          real depth, PRISTINE 15.7% (264/1684, inside the 15-25% target),
          DEFOLIATED 42.9% (722/1684, the outright PLURALITY level -- more
          than 4x `limbed`'s 10.4%/175, reversing the pre-T5 `limbed`-
          dominant tally of 41.4%/28.3%), dry-subpopulation structural 6.83%
          (52/761 dry trees, inside the 5-8% target), total structural
          31.06% (523/1684, inside 28-32%, essentially unchanged from the
          pre-T5 30.23% -- confirming the `pristine`/`defoliated` boundary
          moves do not disturb the structural threshold at 1.09); L3
          broadleaf-pooled snapped share 23.1% (101/437, inside the ≤1/3
          ceiling).
    """
    for name in ("V2_L2", "FINAL3_L3_brown"):
        gt = _load_gt(name)
        if gt is None:
            print("    SKIP  {0}: no {1}/GT_hurricane.json on this "
                  "machine".format(name, name))
            continue
        region = gt["region"]
        hcfg = hu.resolve_cfg({"disaster": {"hurricane": gt["hurricane"]}})
        scfg = sgw.resolve_cfg({k: v for k, v in gt["surge"].items()
                                if k in sgw.DEFAULTS})
        inten = hu.intensity_field(hcfg, region, np.random.default_rng(23))
        depth = sgw.depth_at(scfg, region, np.random.default_rng(41))
        trees = gt["trees"]
        seed = gt["seed"]

        def _tally(use_depth):
            trng = random.Random(seed + 9)
            t = {}
            dry_total = wet_total = dry_struct = wet_struct = 0
            for tr in trees:
                it = float(inten(tr["x"], tr["y"]))
                dep_real = float(depth(tr["x"], tr["y"]))
                dep = dep_real if use_depth else 0.0
                sp = tr.get("species") or None
                lv = hu.tree_level_for_intensity(it, trng, species=sp, depth_m=dep)
                t[lv] = t.get(lv, 0) + 1
                struct = lv in ("leaning", "fallen", "snapped")
                if dep_real < 0.2:
                    dry_total += 1
                    dry_struct += int(struct)
                else:
                    wet_total += 1
                    wet_struct += int(struct)
            return t, dry_total, wet_total, dry_struct, wet_struct

        before, _, _, _, _ = _tally(False)
        after, dry_total, wet_total, dry_struct, wet_struct = _tally(True)
        n_total = sum(after.values())
        down_before = sum(before.get(k, 0) for k in ("leaning", "fallen", "snapped"))
        down_after = sum(after.get(k, 0) for k in ("leaning", "fallen", "snapped"))
        struct_frac = down_after / n_total if n_total else 0.0
        before_frac = down_before / n_total if n_total else 0.0
        dry_frac = dry_struct / dry_total if dry_total else 0.0
        wet_frac = wet_struct / wet_total if wet_total else 0.0
        print("    {0}: BEFORE (depth forced 0) {1}  (structural {2:.1%})".format(
            name, before, before_frac))
        print("    {0}: AFTER  (real depth)    {1}  (structural {2:.1%}; "
              "dry {3}/{4}={5:.1%}, wet {6}/{7}={8:.1%})".format(
                  name, after, struct_frac, dry_struct, dry_total, dry_frac,
                  wet_struct, wet_total, wet_frac))
        pristine_frac = after.get("pristine", 0) / n_total if n_total else 0.0
        defol_frac = after.get("defoliated", 0) / n_total if n_total else 0.0
        limbed_frac = after.get("limbed", 0) / n_total if n_total else 0.0
        if name == "V2_L2":
            # "Before" NO LONGER means "impossible" -- see docstring. It is
            # now a small, wind-only, dry-mechanism-only residual (the
            # depth boost is what turns it into the larger "after" share).
            check(down_before > 0,
                  "L2 wind-alone (depth forced to 0): expected a SMALL "
                  "NONZERO dry-windthrow share now that Correction 2 "
                  "exists -- 'before' is no longer 'impossible', got 0")
            check(before_frac <= 0.05,
                  "L2 wind-alone structural fraction {0:.1%} -- expected a "
                  "SMALL share (dry mechanism only, no depth boost), well "
                  "under the with-depth total".format(before_frac))
            check(down_after > down_before,
                  "L2 with real depth: expected MORE leaning/fallen/snapped "
                  "than wind alone (depth boost adds to the dry mechanism's "
                  "own contribution), got after={0} before={1}".format(
                      down_after, down_before))
            # The proven-unreachable 10% floor (see `_TREE_CUTS`'s comment):
            # pin the ACHIEVED value to a tight band around the measured
            # ~7% so a future cut/anchor change that silently drifts this
            # is caught either way.
            check(0.05 <= struct_frac <= 0.10,
                  "L2 structural fraction {0:.1%} -- expected ~5-10% (the "
                  "provably-closest-achievable band to the 10-15% target, "
                  "see `_TREE_CUTS`'s comment)".format(struct_frac))
            # Correction 2's own literal target: ~2% of L2's DRY trees.
            check(0.010 <= dry_frac <= 0.035,
                  "L2 dry-subpopulation structural fraction {0:.1%} "
                  "({1}/{2}) -- expected ~2% (the dry-windthrow target)"
                  .format(dry_frac, dry_struct, dry_total))
            # STREAM T5's own target: L2 pristine 40-50% (was ~0.9% before
            # the `_TREE_CUTS` re-cut -- effectively unreachable).
            check(0.40 <= pristine_frac <= 0.50,
                  "L2 pristine fraction {0:.1%} -- expected 40-50% (STREAM "
                  "T5's target; was ~0.9% before the pristine cut moved)"
                  .format(pristine_frac))
        else:
            # L3 already reaches every level without the boost (its field is
            # higher); the boost should not REDUCE the down-tree count.
            check(down_after >= down_before,
                  "L3: the boost should not reduce the down-tree count "
                  "(before {0}, after {1})".format(down_before, down_after))
            check(0.28 <= struct_frac <= 0.32,
                  "L3 structural fraction {0:.1%} -- expected ~28-32% "
                  "(near 30%, Correction 2's explicit total target, "
                  "UNCHANGED by STREAM T5's pristine/defoliated re-cut)"
                  .format(struct_frac))
            # Correction 2's own literal target: 5-8% of L3's DRY trees.
            check(0.05 <= dry_frac <= 0.08,
                  "L3 dry-subpopulation structural fraction {0:.1%} "
                  "({1}/{2}) -- expected 5-8% (the dry-windthrow target)"
                  .format(dry_frac, dry_struct, dry_total))
            # STREAM T5's own targets: a genuine pristine minority (was 0.0%
            # before the re-cut -- the diagnosis this stream fixes) and
            # `defoliated` as the outright PLURALITY level (was `limbed` at
            # the pre-T5 cut: 41.4% vs `defoliated`'s 28.3%).
            check(0.15 <= pristine_frac <= 0.25,
                  "L3 pristine fraction {0:.1%} -- expected 15-25% (STREAM "
                  "T5's target; was 0.0% before the pristine cut moved)"
                  .format(pristine_frac))
            check(defol_frac > limbed_frac and defol_frac == max(
                      after.get(k, 0) / n_total for k in hu.TREE_LEVELS),
                  "L3 defoliated ({0:.1%}) should be the outright PLURALITY "
                  "level (limbed {1:.1%}) -- was backwards pre-T5 (limbed "
                  "41.4% vs defoliated 28.3%)".format(defol_frac, limbed_frac))
            # Species-level breakdown, off the SAME RNG STREAM in the SAME
            # tree order `_tally`/the launcher use -- a fresh `Random(seed+9)`
            # per tree (rather than one stream consumed sequentially across
            # all trees) would hand every tree the identical FIRST jitter
            # draw instead of the i-th one, silently desyncing this
            # classification from what `after` (and the real launcher)
            # actually assigned.
            trng2 = random.Random(seed + 9)
            struct_bl = snapped_bl = 0
            for tr in trees:
                sp = tr.get("species") or None
                it = float(inten(tr["x"], tr["y"]))
                dep = float(depth(tr["x"], tr["y"]))
                lv = hu.tree_level_for_intensity(it, trng2, species=sp, depth_m=dep)
                if sp == "Douglas_Fir":
                    continue  # the one conifer -- exempt, it snaps regardless
                if lv in ("leaning", "fallen", "snapped"):
                    struct_bl += 1
                    if lv == "snapped":
                        snapped_bl += 1
            share_bl = snapped_bl / struct_bl if struct_bl else 0.0
            check(share_bl <= 0.3401,
                  "L3 broadleaf-pooled snapped share {0:.1%} ({1}/{2}) -- "
                  "expected <=1/3".format(share_bl, snapped_bl, struct_bl))


# ---------------------------------------------------------------------------
# 7. reproducibility
# ---------------------------------------------------------------------------

@strict
def test_bake_is_byte_reproducible():
    d1 = _bake_dir()
    d2 = tempfile.mkdtemp(prefix="hur_tree_bake_repro_")
    try:
        for species, rel in sorted(bht.TREE_SPECIES.items()):
            for level in bht.LEVELS:
                bht.bake_one(species, rel, level, d2)
        files1 = sorted(f for f in os.listdir(d1) if f.endswith(".usd"))
        files2 = sorted(f for f in os.listdir(d2) if f.endswith(".usd"))
        check(files1 == files2, "same file set on both runs")
        _, mismatch, errors = filecmp.cmpfiles(
            d1, d2, files1, shallow=False)
        check(not mismatch and not errors,
              "byte-identical re-run: mismatches={0} errors={1}".format(
                  mismatch, errors))
    finally:
        shutil.rmtree(d2, ignore_errors=True)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    n_fail = 0
    for t in tests:
        print("--", t.__name__)
        try:
            t()
        except AssertionError as e:
            n_fail += 1
            print("  ASSERTION FAILED:", e)
    print()
    print("{0}/{1} tests passed".format(len(tests) - n_fail, len(tests)))
    sys.exit(1 if n_fail else 0)


def test_launcher_style_reference_lands_at_the_placement_point():
    """THE STACKED-TREES REGRESSION (2026-09-01). Every archetype's cm->m
    correction must live on a CHILD, never on /Root: the launcher's
    `_ref` appends translate/rotate AFTER the referenced prim's op order,
    and a root-level op therefore wraps the placement -- a 0.01 root scale
    put every tree of every render to date in a full-size stack within
    +-2.5 m of the plate origin. This replicates `_ref` and asserts the
    composed tree actually lands where it is placed, at tree scale."""
    from pxr import Usd, UsdGeom, Gf, Sdf
    import glob, os
    ARCH_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "archetypes_hurricane")
    files = sorted(glob.glob(os.path.join(ARCH_DIR, "tree_*.usd")))
    assert files
    checked = 0
    for fn in files:
        st = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        prim = st.DefinePrim(Sdf.Path("/World/t"), "Xform")
        prim.GetReferences().AddReference(os.path.abspath(fn))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(100.0, 50.0, 0.0))
        xf.AddRotateZOp().Set(30.0)
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        assert not r.IsEmpty(), fn
        mid, size = r.GetMidpoint(), r.GetSize()
        assert abs(mid[0] - 100.0) < 20.0 and abs(mid[1] - 50.0) < 20.0, (
            fn, tuple(mid), "composed centre is not at the placement point -- "
            "a root-level xformOp is wrapping the launcher's translate")
        assert 1.0 < max(size) < 45.0, (fn, tuple(size),
            "composed size is not tree-scale metres")
        checked += 1
    assert checked >= 30
