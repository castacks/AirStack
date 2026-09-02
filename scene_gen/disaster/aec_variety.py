#!/usr/bin/env python
"""aec_variety.py — per-UNIT variance on an AEC brownstone row.

THE PROBLEM. `Reference_Brownstone12Row.usd` is not twelve houses. It is ONE
6.67 m facade merged twelve times: every row asset in the `rowhouse` pool
(2/5/6/8/10/11/12 units) works out to the same 6.67 m unit, so mixing row
LENGTHS gives no facade variance at all. Every brownstone in a district is
literally the same house, and no amount of asset-pool work can change that —
measured, the seven row assets are 21.1 m deep and 13.4 / 33.4 / 40.0 / 53.3 /
66.6 / 73.4 / 80.1 m long, i.e. 6.67 m per unit in all seven.

Real brownstone blocks are not uniform. They were built speculatively in short
runs by different builders, so a Brooklyn or Bath terrace varies in brick tone,
cornice paint, door colour and stoop stone from house to house while keeping
one silhouette. That per-unit colour variance is what makes a row read as
twelve houses rather than one repeated twelve times, and it is the cheapest
possible fix here because `aec_burn` has ALREADY proved the addressing: units
are separately named (`_unit_of` / `_unit_index`), each has exactly one
`Walls_Exterior/Walls_ExteriorFacade` mesh, and a conformal layer over the MDL
brick renders correctly (verified in an F3 render, 2026-09-02).

WHY A TINT AND NOT A REPLACEMENT MATERIAL. The facade's brick `st` tiles every
~1 m; the MDL brick map carries the mortar, the courses and the weathering.
Replacing it with a flat colour throws all of that away and gives twelve flat
boxes, which is worse than twelve identical brick houses. So the variance is a
`UsdPreviewSurface` TINT bound over the unit's facade with the brick still
underneath, exactly the layering `aec_burn` uses for soot — and it composes
with soot rather than fighting it, because soot is authored as its own later
layer on top.

WHAT IS AND IS NOT VARIED
-------------------------
Varied per unit: brick tone (the big one — warm red through brown through
painted grey), trim/cornice, door. All three are drawn together from one
per-unit roll so a unit is internally coherent rather than randomly speckled.

NOT varied: silhouette, storey height, window rhythm, depth. Those are
geometry, and changing them needs the row sliced into per-unit meshes (see
`slice-buildings-into-kits`). That is the follow-up if colour proves not to be
enough; it would also buy per-unit collapse under fire, which this cannot.

NEIGHBOURS NEVER MATCH. A real terrace does repeat a colour here and there,
but two IDENTICAL neighbours is the exact artefact this exists to remove, so
consecutive units are forced apart in the palette — see `assign_palette`.

Everything above `author_variety` is pure arithmetic and is unit-tested
offline; only `author_variety` needs a stage.
"""
import os
import zlib

#: Brick tones, as (r, g, b) multipliers over the MDL brick — NOT absolute
#: colours. 1.0 is the asset as authored; the spread is deliberately narrow
#: (0.72-1.18) because a terrace varies in tone, not in hue: push it wider and
#: the row stops reading as one street.
BRICK_TONES = [
    ("as_built",     (1.00, 1.00, 1.00)),
    ("warm_red",     (1.14, 0.92, 0.84)),
    ("deep_red",     (1.05, 0.80, 0.72)),
    ("brown",        (0.90, 0.80, 0.72)),
    ("pale_buff",    (1.12, 1.06, 0.94)),
    ("grey_painted", (0.88, 0.89, 0.92)),
    ("cream_painted", (1.18, 1.14, 1.02)),
    ("soot_darkened", (0.74, 0.72, 0.70)),
]

#: Cornice / window trim. Terraces are overwhelmingly off-white or stone with
#: the occasional dark, so the palette is weighted that way rather than even.
TRIM_COLOURS = [
    ("stone",      (0.82, 0.79, 0.73)),
    ("off_white",  (0.90, 0.89, 0.85)),
    ("cream",      (0.87, 0.83, 0.72)),
    ("white",      (0.94, 0.94, 0.93)),
    ("slate",      (0.42, 0.44, 0.47)),
]

#: Front doors — the one place a terrace is allowed to be loud.
DOOR_COLOURS = [
    ("black",      (0.10, 0.10, 0.11)),
    ("oxblood",    (0.33, 0.09, 0.10)),
    ("bottle",     (0.09, 0.22, 0.16)),
    ("navy",       (0.11, 0.16, 0.30)),
    ("olive",      (0.25, 0.26, 0.14)),
    ("teal",       (0.09, 0.28, 0.30)),
    ("plum",       (0.26, 0.13, 0.24)),
    ("mustard",    (0.62, 0.47, 0.13)),
]

#: Mesh-name fragments that identify what a mesh IS. `aec_burn`'s own
#: measurement of the row: one `Walls_ExteriorFacade` per unit, plus named
#: Doors / Windows / Railings categories.
FACADE_HINTS = ("Walls_Exterior", "ExteriorFacade")
TRIM_HINTS = ("Cornice", "Casework", "Railings", "Window", "Trim")
DOOR_HINTS = ("Door",)


def _unit_roll(row_key, unit_index, salt):
    """A stable 0..1 for one unit of one row.

    Keyed on the ROW's identity as well as the unit index, so two different
    brownstone rows in the same city do not come out with the same colour
    sequence — otherwise the variance just moves the repetition up a level
    from unit to row. `zlib.crc32`, not `hash()`: `hash()` is salted per
    interpreter (`PYTHONHASHSEED`) and this pipeline has already been bitten
    by that.
    """
    key = "%s|%d|%s" % (row_key, int(unit_index), salt)
    return (zlib.crc32(key.encode("utf-8")) % 100003) / 100003.0


def assign_palette(row_key, n_units):
    """`[{unit, brick, trim, door}, ...]`, one entry per unit.

    NEIGHBOURS NEVER SHARE A BRICK TONE. A terrace does repeat tones down its
    length, and forbidding that entirely would look as synthetic as the
    uniform row does — but two identical ADJACENT facades is precisely the
    artefact being removed, so only the immediate neighbour is excluded. With
    eight tones that leaves seven choices at every step, so the sequence still
    looks drawn rather than cycled.
    """
    out = []
    prev_brick = None
    for u in range(int(n_units)):
        r = _unit_roll(row_key, u, "brick")
        pool = [t for t in BRICK_TONES if t[0] != prev_brick] or BRICK_TONES
        brick = pool[int(r * len(pool)) % len(pool)]
        prev_brick = brick[0]
        rt = _unit_roll(row_key, u, "trim")
        rd = _unit_roll(row_key, u, "door")
        out.append({
            "unit": u,
            "brick": brick[0], "brick_rgb": brick[1],
            "trim": TRIM_COLOURS[int(rt * len(TRIM_COLOURS)) % len(TRIM_COLOURS)][0],
            "trim_rgb": TRIM_COLOURS[int(rt * len(TRIM_COLOURS)) % len(TRIM_COLOURS)][1],
            "door": DOOR_COLOURS[int(rd * len(DOOR_COLOURS)) % len(DOOR_COLOURS)][0],
            "door_rgb": DOOR_COLOURS[int(rd * len(DOOR_COLOURS)) % len(DOOR_COLOURS)][1],
        })
    return out


def classify(mesh_name):
    """`"facade" | "trim" | "door" | None` for one mesh name."""
    n = str(mesh_name)
    for h in DOOR_HINTS:
        if h in n:
            return "door"
    for h in FACADE_HINTS:
        if h in n:
            return "facade"
    for h in TRIM_HINTS:
        if h in n:
            return "trim"
    return None


def _tint_material(stage, path, rgb, rough=0.72):
    """A tint layer. Mirrors `aec_burn._flat_material`'s shape so the two
    compose predictably; the difference is intent, not construction — this is
    meant to sit UNDER the soot layer and OVER the brick."""
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage,
                                Sdf.Path(path).AppendChild("PreviewSurface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*[float(v) for v in rgb]))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def author_variety(stage, root_path, row_key=None, verbose=True):
    """Bind per-unit tints across one brownstone row. Returns a stats dict.

    Idempotent: re-running rebinds the same materials from the same rolls.
    Safe to run before OR after `aec_burn.burn_row` — soot is authored as its
    own later layer and binds the burning units' facades itself, so a burnt
    unit simply ends up sooty rather than tinted, which is correct.
    """
    from pxr import Sdf, Usd, UsdShade
    sys_path_note = None                       # keeps linters quiet
    del sys_path_note

    # `_unit_of` is aec_burn's, and deliberately reused rather than copied:
    # the unit naming is a property of the ASSET, and two implementations of
    # it would drift.
    from disaster.aec_burn import _unit_of, _unit_index

    root = stage.GetPrimAtPath(root_path)
    if not root:
        raise ValueError("no prim at %s" % root_path)
    row_key = row_key or str(root_path)

    units = {}
    for prim in Usd.PrimRange(root):
        if not prim.IsA(Usd.Prim) and not prim.GetTypeName():
            continue
        u = _unit_of(prim.GetPath(), root_path)
        if u:
            units.setdefault(u, []).append(prim)
    if not units:
        return {"units": 0, "bound": 0, "note": "no addressable units"}

    order = sorted(units, key=_unit_index)
    palette = assign_palette(row_key, len(order))
    scope = Sdf.Path(str(root_path)).AppendChild("Looks_variety")
    stage.DefinePrim(scope, "Scope")

    made, bound = {}, 0
    for slot, uname in enumerate(order):
        p = palette[slot]
        for kind, rgb in (("facade", p["brick_rgb"]),
                          ("trim", p["trim_rgb"]),
                          ("door", p["door_rgb"])):
            mpath = scope.AppendChild("u%02d_%s" % (slot, kind))
            made[(slot, kind)] = _tint_material(stage, mpath, rgb)
        for prim in units[uname]:
            kind = classify(prim.GetPath().name)
            if kind is None:
                continue
            mat = made.get((slot, kind))
            if mat is None:
                continue
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
            bound += 1

    stats = {"units": len(order), "bound": bound,
             "palette": [(p["brick"], p["trim"], p["door"]) for p in palette]}
    if verbose:
        print("[aec_variety] %s: %d units, %d meshes tinted"
              % (root_path, len(order), bound))
        for p in palette:
            print("    u%02d  brick=%-14s trim=%-10s door=%s"
                  % (p["unit"], p["brick"], p["trim"], p["door"]))
    return stats


#: Prim/asset names that identify an AEC brownstone ROW. The pool holds seven
#: row lengths, all `Reference_Brownstone<N>Row`.
ROW_ASSET_HINT = "Reference_Brownstone"


def apply_to_stage(stage, root="/World/stage/generated", verbose=True):
    """Tint every AEC brownstone row on the stage. Returns a stats dict.

    THE HOOK. Rows are placed as ordinary `house_*` cells, so this walks the
    generated scope and picks out the ones whose reference is a brownstone
    row. Doing it stage-side rather than at placement time means it works
    the same for the intact city, for a re-run, and for any disaster that
    reuses this asset — the variance is a property of the ASSET, not of the
    fire.

    Off with `AEC_VARIETY=0`, because a scene that wants the asset exactly as
    authored (an A/B, or a regression check against an older cell) has to be
    able to get it.
    """
    from pxr import Usd
    if str(os.environ.get("AEC_VARIETY", "1")).strip() in ("0", "false", ""):
        if verbose:
            print("[aec_variety] disabled by AEC_VARIETY=0")
        return {"rows": 0, "bound": 0, "disabled": True}

    scope = stage.GetPrimAtPath(root)
    if not scope:
        return {"rows": 0, "bound": 0, "note": "no %s" % root}

    rows, bound, failed = 0, 0, []
    for cell in scope.GetChildren():
        hit = None
        for prim in Usd.PrimRange(cell):
            if ROW_ASSET_HINT in prim.GetPath().name:
                hit = cell
                break
        if hit is None:
            continue
        try:
            st = author_variety(stage, hit.GetPath(),
                                row_key=str(hit.GetPath()), verbose=False)
        except Exception as exc:                       # noqa: BLE001
            # One malformed row must not take the city down with it -- this
            # runs late, after the expensive stages.
            failed.append((str(hit.GetPath()), str(exc)[:120]))
            continue
        rows += 1
        bound += st.get("bound", 0)

    if verbose:
        print("[aec_variety] %d brownstone row(s), %d meshes tinted%s"
              % (rows, bound,
                 (", %d failed" % len(failed)) if failed else ""))
        for path, err in failed[:5]:
            print("    FAILED %s: %s" % (path, err))
    return {"rows": rows, "bound": bound, "failed": failed}
