#!/usr/bin/env python
"""repatch_gac_x_soot.py -- bare-USD, no-reslice, no-Kit, no-VTK in-place
repatch for TWO related soot-application bugs in `disaster/urban_fire.py`,
both fixed 2026-08-31 and both gated to the sliced/GAC path (kit path
frozen, verified with `tools/kit_burn_probe.py` before/after):

BUG A -- `_r_soot_overlay`'s `_soot_cover = 0.5` hardcode for merged/
interior ("_side == 'x'") pieces (the below-origin block, the block above
the band, every per-storey "core" element) unconditionally forced
`_bind_soot`'s `tone_ok` floor True regardless of whether the plume ever
reached them, defeating the hard "clean band below the origin" cut
`_severity`'s own docstring requires.

BUG B -- the SAME floor (`SOOT_PALE_MAX`) also fired on ordinary RING
pieces well inside the burning band: `tone_ok` legitimately becomes true
there (real measured coverage), a genuine per-texel gradient bake IS
computed, but when its mean exceeds 0.45 the floor discarded that gradient
for the flat, unnumbered, shared "soot"/"soot_mid"/"soot_light" tone --
losing all pattern on the subset. This is what `wall_W_0_10_0034` (the
piece the user named directly, "haven't gotten the scorch pattern well")
actually shows. Fixed by keeping the bake whenever its own covered-texel
luminance spread shows real structure (`SOOT_PALE_SPREAD_MIN` /
`SOOT_PALE_STD_MIN`); the floor still fires on a genuinely near-uniform,
pale result (the checkerboard case it was built for).

This tool does NOT re-slice, re-settle, or touch Kit/physics: it opens an
ALREADY-BAKED bake .usd, re-runs ONLY the fixed per-piece soot bind, and
exports a NEW file -- the input is never modified.

Usage (bare USD, container):
    usd_python.sh repatch_gac_x_soot.py <bake.usd> <bake.json> <out.usd>

BUG A pieces (`_side=='x'`, identified by the naming convention
`<role>_x_<band>_<index>`): every subset is handed to the fixed
`_bind_soot`, with the crop-refresh classification below deciding what
"prebaked" set to feed it.

BUG B pieces (any RING piece with >=1 subset bound to the literal, digest-
less shared soot/soot_mid/soot_light material -- `_bind_soot`'s own tell for
"the floor fired here"): the floor-affected subset's TRUE original texture
was never persisted (a straight rebind, no reference, unlike the per-piece
`FireLooks/soot_N` copies BUG A's subsets get), so it is approximated by
BORROWING the SAME-NAMED GeomSubset's material from a DONOR piece of the
SAME role that was NOT flattened -- verified on `SM_Building_23`: the S/W
(burning) wall family's `mat_6`/`mat_7` are FLAT on all 19 storeys, while
the N/E (non-burning) family's `mat_6`/`mat_7` are genuinely textured
(`M_Glass_In_Standard_Inst`/`M_Wall_in_Inst`) on all 19 -- the SAME
material names at the SAME subset slot on every storey and every side,
confirming subset enumeration is stable across a GAC building's repeating
modules. The donor's material is temporarily bound to the affected subset
(so `_bind_soot`'s own `find_basecolor` finds a genuine texture to bake
over, on the SUBJECT piece's own UVs/skin sample) and handed to the fixed
`_bind_soot`, which then makes its OWN numbered `FireLooks/soot_N` copy --
same shape as the normal path, an approximation (borrowed base texture) not
a byte-exact undo. A subset with no same-role, same-name donor anywhere in
the building is left untouched and counted separately.

Per-subset classification, using ONLY information recoverable from the
exported file (no re-run of `gac_fire.prepare`):
  * a diffuse texture named `gacsoot_*.png` -> the PRE-SLICE atlas bake
    (`gac_fire.bake_atlases`), a DIFFERENT code path neither fix touches --
    added to a synthetic `soot_prebaked` set so `_bind_soot` skips it
    exactly as the real pipeline does.
  * a diffuse texture named `sootbake_*.png` (BUG A's own per-piece naming)
    whose material is an INTERNAL REFERENCE (`piece_material_like`) -- the
    reference target is the TRUE pre-bug material, still resolvable in the
    stage; the subset is rebound to it (exact undo) and handed to the fixed
    `_bind_soot` to redecide.
  * a `sootbake_*.png` material with NO reference (the `piece_material`
    flat-diffuse fallback, BUG A path) -- approximated from the composite's
    own brightest ("least sooted") texel.
  * a bind to the literal shared soot/soot_mid/soot_light tone (no digest
    suffix) -- BUG B's tell. Repaired via the donor mechanism above where a
    donor exists; left untouched and counted otherwise.
  * anything else (still the clean original) -- left alone; handed to the
    fixed `_bind_soot` anyway so a genuinely-touched texel still gets baked.

STATUS (2026-08-31, verified against `city_138`'s 21 gac bakes, read-only
census, no writes to the live cache): Bug A/B combined affect 504 ring
subset-instances across 219 pieces in 18/21 buildings; 443 of those (88%)
have a same-role/same-slot donor and are fully repairable by THIS tool with
zero GPU time; 61 (mostly on `SM_Building_02`'s three bakes) have no donor
anywhere in the building and need a real re-bake. Acceptance piece
`gac_SM_Building_23_F5_o7_SW_s1254`, `wall_W_0_10_0034` (the piece the user
named): `mat_6`/`mat_7` went from the literal, unnumbered flat `soot` tone
to numbered gradient copies (`soot_68`/`soot_69`, luminance spread ~0.16 and
~0.23 over their own covered texels) -- PASS.
"""
import json
import re
import sys
from collections import defaultdict

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade, Sdf          # noqa: E402
from disaster import urban_fire as uf                 # noqa: E402

X_RE = re.compile(r"^(wall|corner|core|parapet_corner|parapet|balcony)_x_")
# role(one of the same families) _ side(1-2 letters, or corner code) _
# band(digits) _ index(digits) -- RING pieces only ("_x_" excluded, X_RE
# above owns those)
RING_RE = re.compile(
    r"^(?P<role>wall|corner|parapet_corner|parapet|balcony)_"
    r"(?P<side>[A-Za-z]{1,2})_(?P<ring>\d+)_(?P<storey>\d+)_(?P<idx>\d+)$")


def diffuse_tex_name(mat_prim):
    if mat_prim is None:
        return None
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        idv = sh.GetIdAttr().Get()
        if idv == "UsdPreviewSurface":
            i = sh.GetInput("diffuseColor")
            if i and i.HasConnectedSource():
                ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                f = ts.GetInput("file")
                v = f.Get() if f else None
                if v:
                    return str(v).rsplit("/", 1)[-1]
        if idv == "OmniPBR":
            ti = sh.GetInput("diffuse_texture")
            v = ti.Get() if ti else None
            if v:
                return str(v).rsplit("/", 1)[-1]
    return None


def internal_ref_target(mat_prim):
    """The primPath of an internal (`AddInternalReference`) reference on
    `mat_prim`, or None. `piece_material_like` authors exactly one."""
    refs = mat_prim.GetMetadata("references")
    if not refs:
        return None
    items = list(refs.prependedItems) + list(refs.addedItems) + \
        list(refs.explicitItems)
    for r in items:
        if not r.assetPath and r.primPath:
            return r.primPath
    return None


def diffuse_tex_asset(mat_prim):
    """Full asset path of the bound diffuse texture, or None -- companion
    to `diffuse_tex_name` (which only returns the basename)."""
    if mat_prim is None:
        return None
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        idv = sh.GetIdAttr().Get()
        if idv == "UsdPreviewSurface":
            i = sh.GetInput("diffuseColor")
            if i and i.HasConnectedSource():
                ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                f = ts.GetInput("file")
                v = f.Get() if f else None
                if v:
                    return v
        if idv == "OmniPBR":
            ti = sh.GetInput("diffuse_texture")
            v = ti.Get() if ti else None
            if v:
                return v
    return None


def recover_flat_rgb(tex_asset):
    """`piece_material`'s flat-diffuse-fallback bake (fix #3) composites the
    skin OVER the subset's own constant colour, so a texel the plume never
    reached (alpha ~= 0) is still ~= the true original colour. The BRIGHTEST
    pixel of the resulting map is the best surviving estimate of it -- soot
    only ever darkens, so whichever texel got the least of it is closest to
    the pre-bug value. An approximation, not a recovery: good enough to put
    the subset back roughly where it started so the fixed per-texel bake can
    redecide it, not a byte-exact undo."""
    import numpy as np
    from PIL import Image

    p = str(tex_asset).replace("@", "")
    try:
        Image.MAX_IMAGE_PIXELS = None
        im = np.asarray(Image.open(p).convert("RGB"), dtype=np.float32) / 255.0
    except Exception:
        return None
    lum = im.mean(axis=2)
    iy, ix = np.unravel_index(np.argmax(lum), lum.shape)
    return tuple(float(v) for v in im[iy, ix])


def piece_role(name):
    """The role token (wall/corner/core/parapet/parapet_corner/balcony) of a
    piece name, ring or merged -- shared by `X_RE` and `RING_RE`'s two
    different group shapes."""
    m = RING_RE.match(name)
    if m:
        return m.group("role")
    m = X_RE.match(name)
    return m.group(1) if m else None


def build_flat_material(stage, path, rgb, roughness=0.7):
    """A plain OmniPBR constant-colour material -- the shape `_flat_diffuse`
    reads and `_bind_soot`'s fix-#3 flat-diffuse-fallback bakes over."""
    from pxr import Gf

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*rgb))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def main():
    src, sidecar_path, out = sys.argv[1], sys.argv[2], sys.argv[3]

    sidecar = json.load(open(sidecar_path))
    stage = Usd.Stage.Open(src)

    pieces_scope = None
    for pr in stage.Traverse():
        if pr.GetName() == "pieces" and pr.GetTypeName() == "Scope":
            pieces_scope = pr
            break
    if pieces_scope is None:
        print("[repatch] no /pieces scope -- not a sliced GAC bake, abort")
        sys.exit(1)
    cell_root = pieces_scope.GetPath().GetParentPath()

    fire = dict(sidecar["fire"])
    fire["events"] = sidecar.get("events") or []
    city = sidecar.get("city") or {}
    mass_tag = fire.get("mass", "main")
    ctx = {
        "stage": stage,
        "tag": sidecar.get("tag", cell_root.name),
        "parent": str(cell_root),
        "fire": fire,
        "info": {"masses": sidecar["masses"], "type": sidecar.get("kind", "gac"),
                 "x": float(city.get("x", 0.0)), "y": float(city.get("y", 0.0))},
        "cache": {},
        "soot_mats": {},
        "notes": [],
    }
    # THE SHARED FLAT-TONE MATERIALS LIVE AT THE BAKE ROOT, NOT THE CELL.
    # `_bind_soot`'s own per-piece composites are created under
    # `ctx["parent"]` (the CELL, e.g. `/World/bake/g24` -- confirmed via
    # `tools/_mat_ref_probe_tmp.py` finding `soot_77` there), but
    # `urban_fire.materials()` is called ONCE per bake PROCESS at the
    # process root (`sidecar["root"]`, `/World/bake` on every bake this
    # tool has seen). Calling it again at the cell path -- this tool's
    # first cut -- silently CREATES a second, unused soot/soot_mid/
    # soot_light set at the wrong path (`materials()` creates whatever it
    # cannot find), so every `flat_tone_paths` membership test missed the
    # real ones and Bug B detection found nothing (measured: 0 flat-tone
    # subsets on a building `donor_probe` had already shown 38 of, before
    # this fix). `_bind_soot` only ever reads `ctx["mats"]["soot"/
    # "soot_mid"/"soot_light"]` -- a plain LOOKUP of those three at the
    # real root is enough, and skips the full `materials()` call's
    # megascan/brick-texture load (most of this tool's wall-clock time).
    looks_root = sidecar.get("root") or str(cell_root.GetParentPath())
    flat_tone_paths = set()
    ctx["mats"] = {}
    for key in ("soot", "soot_mid", "soot_light"):
        fm = UsdShade.Material.Get(stage, "{0}/FireLooks/{1}".format(
            looks_root, key))
        if fm:
            ctx["mats"][key] = fm
            flat_tone_paths.add(str(fm.GetPath()))

    all_pieces = [c for c in pieces_scope.GetChildren() if c.GetTypeName() == "Mesh"]
    x_targets = [c for c in all_pieces if X_RE.match(c.GetName())]

    sk = uf._soot_skin(ctx, 1.0)
    if sk is None:
        print("[repatch] no soot skin (no fire events) -- nothing to do")
        stage.GetRootLayer().Export(out)
        return

    # ------------------------------------------------------------------
    # BUG B: index every subset's bound material by (role, subset name),
    # excluding the flat-tone ones, so a flattened subset can borrow a
    # same-role, same-slot donor from ANYWHERE in the building (a burning
    # elevation's whole family can be flattened storey to storey -- the
    # donor is usually a non-burning side of the SAME piece role).
    # ------------------------------------------------------------------
    donor_index = defaultdict(list)   # (role, subname) -> [(piece, mat_prim)]
    ring_flat = []                    # (piece_prim, role, side, storey, [subs])
    for pr in all_pieces:
        m = RING_RE.match(pr.GetName())
        if not m:
            continue
        role = m.group("role")
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
        flat_subs = []
        for sub in subs:
            mat = UsdShade.MaterialBindingAPI(sub).ComputeBoundMaterial()[0]
            mp = mat.GetPrim() if mat else None
            subname = sub.GetPrim().GetName()
            if mp is None or not mp.IsValid():
                continue
            if str(mp.GetPath()) in flat_tone_paths:
                flat_subs.append(sub)
            else:
                donor_index[(role, subname)].append((pr.GetName(), mp))
        if flat_subs:
            ring_flat.append((pr, role, m.group("side"), int(m.group("storey")),
                              flat_subs))

    print("[repatch] %s: %d '_side==x' piece(s), %d RING piece(s) with a "
          "flat-tone subset (of %d total pieces)"
          % (sidecar.get("name"), len(x_targets), len(ring_flat),
             len(all_pieces)))

    n_undone = 0
    n_approx_recovered = 0
    n_unrecoverable = 0
    n_pieces_touched = 0
    n_recovered_ctr = 0
    n_donor_repaired = 0
    n_no_donor = 0

    def process_piece(pr, e_side, e_storey, extra_prebaked_check):
        """Classify every subset of `pr`, mutate bindings where recoverable,
        build the `soot_prebaked` set, then hand the piece to the fixed
        `_bind_soot`. Returns True if any binding changed."""
        nonlocal n_undone, n_approx_recovered, n_unrecoverable, \
            n_recovered_ctr, n_donor_repaired, n_no_donor
        path = pr.GetPath().pathString
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
        prebaked = set()
        role = piece_role(pr.GetName())
        for sub in subs:
            mat = UsdShade.MaterialBindingAPI(sub).ComputeBoundMaterial()[0]
            mprim = mat.GetPrim() if mat else None
            if mprim is None or not mprim.IsValid():
                continue
            mpath = str(mprim.GetPath())
            if mpath in flat_tone_paths:
                # BUG B: borrow a same-role, same-subset-name donor
                subname = sub.GetPrim().GetName()
                donors = donor_index.get((role, subname)) if role else None
                if donors:
                    _, donor_mat = donors[0]
                    UsdShade.MaterialBindingAPI(sub).Bind(
                        UsdShade.Material(donor_mat))
                    n_donor_repaired += 1
                else:
                    prebaked.add(mpath)
                    n_no_donor += 1
                continue
            tex = diffuse_tex_name(mprim)
            if tex and tex.startswith("gacsoot_"):
                prebaked.add(mpath)
                continue
            if tex and tex.startswith("sootbake_"):
                ref = internal_ref_target(mprim)
                if ref is not None:
                    orig = stage.GetPrimAtPath(ref)
                    if orig and orig.IsValid():
                        UsdShade.MaterialBindingAPI(sub).Bind(
                            UsdShade.Material(orig))
                        n_undone += 1
                        continue
                tex_asset = diffuse_tex_asset(mprim)
                rgb = recover_flat_rgb(tex_asset) if tex_asset else None
                if rgb is not None:
                    rec_path = "{0}/RecoveredLooks/rec_{1}".format(
                        ctx["parent"], n_recovered_ctr)
                    n_recovered_ctr += 1
                    recovered = build_flat_material(stage, rec_path, rgb)
                    UsdShade.MaterialBindingAPI(sub).Bind(recovered)
                    n_approx_recovered += 1
                    continue
                prebaked.add(mpath)
                n_unrecoverable += 1
                continue
            # else: already the clean/true original -- left alone, handed
            # to _bind_soot fresh below

        ctx["soot_prebaked"] = prebaked
        e = {"side": e_side, "name": pr.GetName(), "mass": mass_tag,
             "storey": e_storey,
             "p": {"prim_path": path,
                   "_side": "x" if e_side == "x" else "",
                   "usd": "slice://repatch"}}
        return bool(uf._bind_soot(ctx, e, sk))

    for pr in x_targets:
        if process_piece(pr, "x", None, None):
            n_pieces_touched += 1

    for pr, role, side, band, flat_subs in ring_flat:
        if process_piece(pr, side, band, None):
            n_pieces_touched += 1

    print("[repatch] BUG A -- undone+redone(exact ref)=%d "
          "approx-recovered+redone(brightest-texel)=%d "
          "unrecoverable(left alone)=%d"
          % (n_undone, n_approx_recovered, n_unrecoverable))
    print("[repatch] BUG B -- donor-repaired+redone=%d  no-donor(left "
          "alone)=%d  (%d ring piece(s) had >=1 flat-tone subset)"
          % (n_donor_repaired, n_no_donor, len(ring_flat)))
    print("[repatch] pieces re-bound=%d/%d  soot_stats: %s"
          % (n_pieces_touched, len(x_targets) + len(ring_flat),
             ctx["soot_stats"]))
    stage.GetRootLayer().Export(out)
    print("[repatch] wrote", out)


if __name__ == "__main__":
    main()
