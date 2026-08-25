"""ground stage — the burn scar, as a translucent overlay on the grass.

WHAT THIS IS
------------
A burnt-floor material laid over the lawn in BANDS of constant opacity that
follow the fire's own coverage field, so the scar is the ellipse the front
actually swept, feathered and fingered at its edge, with unburned islands the
fire skipped. It is the fourth of the approaches in the
`build-wildfire-scenes` skill — the one that "drew nothing" until two things
were understood, both now load-bearing here:

  * OmniPBR's `opacity_constant` is a FRACTIONAL CUTOUT opacity
    (`OmniPBRBase.mdl`), and RTX Real-Time discards fractional cutout unless
    `/rtx/raytracing/fractionalCutoutOpacity` is on. It has to be a
    COMMAND-LINE flag (`KIT_ARGS`, via `SimulationApp(extra_args=...)`); a
    `carb.settings.set_bool` after startup is never copied onto the USD
    render property the renderer reads, and the overlay silently vanishes.
  * The burnt texture tiled at ~8 m reads as a grid of squares from the
    air. One tile is projected across the whole overlay instead — soft up
    close, unrepeated from altitude — and the opacity is kept low.

THE EDGE IS WHERE THE WORK IS
-----------------------------
A coverage field that steps from 0 to something at the arrival line ends the
scar on a conic with a hard cut against green grass. `feathered_coverage`
ramps over `edge_m` metres PERPENDICULAR to the front (the front is the level
set t = elapsed, so distance is (elapsed - t) / |grad t|; measuring along the
ray from the origin was ~10 m on a flank because the ray meets it obliquely),
wobbles the line by +-`finger_m` of band-limited seamless noise, and removes
compact islands. The noise only ever MOVES the boundary or REMOVES coverage,
so ground the fire never reached stays clean — the level-set lesson.

`burn_ground_preview_launch_script.py` is the bench for all of this;
`suburb_mini_wildfire_launch_script.py` is the scene that uses it.
"""

import math
import os

import numpy as np

BURNT_TEXTURE = ("airstack://scene_gen/assets/materials/megascans/"
                 "Burnt_Forest_Floor/T_uhwpehcdy_2K_B.png")

# Pass through `SimulationApp(launch_config={"extra_args": KIT_ARGS})`.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]


def knobs_from_env(span_m):
    """The tuning knobs, from GROUND_* env vars, defaults derived from the plate.

    Shared by the bench and the scene so the same variables mean the same
    thing in both: what was tuned on the floor carries straight over.
    """
    def _f(name, default):
        return float(os.environ.get(name, default))

    edge = _f("GROUND_EDGE_M", "0") or max(10.0, 0.10 * float(span_m))
    return dict(
        cell_m=_f("GROUND_CELL", "3.0"),
        bands=int(os.environ.get("GROUND_BANDS", "12")),
        tile_m=_f("GROUND_TILE_M", "0") or None,        # None = whole overlay
        op_range=(_f("GROUND_OPACITY_MIN", "0.14"),
                  _f("GROUND_OPACITY_MAX", "0.85")),
        edge_m=edge,
        finger_m=_f("GROUND_FINGER_M", "0") or 0.8 * edge,
        # UNBURNED ISLANDS OFF BY DEFAULT. They are a real mixed-severity
        # feature, but inside a scar that is supposed to read as continuously
        # burnt they look like patches the pass missed. `GROUND_ISLANDS=0.06`
        # brings them back.
        islands=_f("GROUND_ISLANDS", "0.0"),
    )


# ---------------------------------------------------------------------------
# material
# ---------------------------------------------------------------------------

def overlay_material(stage, path, opacity, tile_m, center=(0.0, 0.0),
                     texture=BURNT_TEXTURE):
    """A translucent burnt-ground OmniPBR with a CONSTANT opacity.

    ONE TILE PER `tile_m`, CENTRED ON `center`. OmniPBR's world projection
    maps u = x * texture_scale + texture_translate (scale, then offset), so a
    plate-sized tile needs the half-tile offset or its seam runs through the
    middle of the scar.

    Constant rather than a mask because OmniPBR carries ONE texture_scale for
    every map it samples and so cannot tile a diffuse while stretching a mask
    once across the plate; the gradient comes from the geometry being split
    into bands instead.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(texture, "")))
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / float(tile_m)
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
    sh.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(0.5 - center[0] * k, 0.5 - center[1] * k))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.94)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("opacity_constant",
                   Sdf.ValueTypeNames.Float).Set(float(opacity))
    # 0 blends; anything above turns a soft edge into a stippled cutout.
    sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


# ---------------------------------------------------------------------------
# the field
# ---------------------------------------------------------------------------

def edge_fields(region, rng, n=256, islands=0.06,
                finger_band_m=(25.0, 80.0), island_band_m=(20.0, 60.0)):
    """Two seamless noise fields over `region`, returned as samplers f(x, y).

    `finger(x, y)` in -1..1 wobbles the front line; `island(x, y)` in 0..1 is
    1 inside an unburned island. Both are `scorch._noise` — spectral, so
    tileable and with no lattice — band-limited so the outline fingers at
    house-lot scale and islands are compact, with nothing so large it reads
    as one shape and nothing so fine it comes out as one-cell specks (an
    8-25 m island band thresholded this hard was confetti). The island
    threshold is a QUANTILE of the field, so `islands` is the share of area
    by construction rather than by luck.
    """
    from . import scorch

    x0, y0, x1, y1 = region
    w, h = float(x1 - x0), float(y1 - y0)
    px = max(w, h) / float(n)                        # metres per pixel

    def band(lo_m, hi_m):                            # wavelengths -> cycles/px
        return (px / float(hi_m), px / float(lo_m))

    lo, hi = band(*finger_band_m)
    fng = scorch._noise(rng, n, n, beta=2.0, lo=lo, hi=hi) * 2.0 - 1.0
    lo, hi = band(*island_band_m)
    raw = scorch._noise(rng, n, n, beta=2.0, lo=lo, hi=hi)
    if islands > 0.0:
        thr = float(np.quantile(raw, 1.0 - min(0.9, float(islands))))
        e = np.clip((raw - thr) / 0.05, 0.0, 1.0)
        isl = e * e * (3.0 - 2.0 * e)
    else:
        isl = np.zeros_like(raw)

    def _ij(x, y):
        i = min(n - 1, max(0, int((y - y0) / h * n)))
        j = min(n - 1, max(0, int((x - x0) / w * n)))
        return i, j

    def finger(x, y):
        i, j = _ij(x, y)
        return float(fng[i, j])

    def island(x, y):
        i, j = _ij(x, y)
        return float(isl[i, j])

    return finger, island


def elapsed_for_fraction(arrival, region, frac, n=50):
    """The time at which `frac` of `region` has been reached by the front.

    A quantile of arrival times sampled on an n x n grid. Running the front
    "just past the far corner" burns EVERY cell of a plate and leaves no edge
    on screen to judge; this puts the whole outline on the plate.
    """
    x0, y0, x1, y1 = region
    xs = [x0 + (k + 0.5) * (x1 - x0) / float(n) for k in range(n)]
    ys = [y0 + (k + 0.5) * (y1 - y0) / float(n) for k in range(n)]
    ts = sorted(t for t in (arrival(x, y) for x in xs for y in ys)
                if math.isfinite(t))
    if not ts:
        return 200.0
    return ts[int(min(0.999, max(0.0, float(frac))) * (len(ts) - 1))]


def feathered_coverage(arrival, elapsed, origin, region, rng, edge_m=25.0,
                       finger_m=20.0, islands=0.06):
    """`(x, y) -> 0..1`: 0 outside the burn, ramping in over `edge_m`,
    heavier with burn age, fingered and with islands.

    `arrival(x, y)` is the front's arrival time (seconds, inf if never),
    `origin` the ignition point, `region` the extent the noise fields cover.

    NOT A STEP AT THE FRONT. `behind` is metres behind the front,
    perpendicular to it, from the arrival gradient by central differences;
    then the fingering noise moves the line in and out. Far outside the burn
    `behind` is hugely negative and +-finger_m cannot cross zero, which is
    what keeps the noise from speckling ground the fire never reached.
    """
    ox, oy = origin
    finger, island = edge_fields(region, rng, islands=islands)
    edge_m = max(1e-3, float(edge_m))
    elapsed = float(elapsed)

    def coverage_at(x, y):
        t = arrival(x, y)
        if not math.isfinite(t):
            return 0.0
        if t <= 1e-6:
            return 1.0
        h = 1.0
        tx0, tx1 = arrival(x - h, y), arrival(x + h, y)
        ty0, ty1 = arrival(x, y - h), arrival(x, y + h)
        if all(math.isfinite(v) for v in (tx0, tx1, ty0, ty1)):
            g = math.hypot((tx1 - tx0) / (2.0 * h), (ty1 - ty0) / (2.0 * h))
        else:
            g = 0.0
        if g <= 1e-9:
            g = t / max(1e-6, math.hypot(x - ox, y - oy))   # along-ray fallback
        behind = (elapsed - t) / g + finger_m * finger(x, y)
        if behind <= 0.0:
            return 0.0
        e = min(1.0, behind / edge_m)
        ramp = e * e * (3.0 - 2.0 * e)
        age = max(0.0, elapsed - t)
        sev = 0.55 + 0.45 * min(1.0, age / max(1e-6, elapsed))
        return ramp * sev * (1.0 - island(x, y))

    return coverage_at


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def _inside_convex(px, py, ring):
    """Point in a convex ring (either winding)."""
    sign = 0
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % n]
        c = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if abs(c) < 1e-9:
            continue
        s = 1 if c > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


def skip_rects(rects, pad=0.0):
    """A `skip(x0, y0, x1, y1) -> bool` for cells whose CENTRE is over a ring.

    Pools: a pool is a hole in the lawn with water below grade, and a burnt
    overlay cell drawn over it is a film floating on water — you cannot scorch
    water (`damage.INCOMBUSTIBLE`), so it is left out.

    CENTRE, not "any corner touches". Skipping every cell that merely clips a
    pool dropped a whole 3 m border of cells around each pool, and that border
    is grass — so a pool inside the burn wore a bright green halo the fire
    would never have left. Testing the centre alone keeps the scar right up to
    the coping and skips only cells genuinely over the water. `pad` grows the
    ring first, for a deliberate margin; 0 is flush.
    """
    rings = [list(r) for r in (rects or ()) if r and len(r) >= 3]

    def _grown(ring):
        if pad <= 0.0:
            return ring
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        out = []
        for px, py in ring:
            dx, dy = px - cx, py - cy
            d = math.hypot(dx, dy) or 1.0
            out.append((px + dx / d * pad, py + dy / d * pad))
        return out

    grown = [_grown(r) for r in rings]

    def skip(x0, y0, x1, y1):
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        return any(_inside_convex(cx, cy, ring) for ring in grown)

    return skip


def build_overlay(stage, coverage_at, region, ssf, z_m, *, material_parent,
                  root="/World/burnGround", cell_m=3.0, bands=12, tile_m=None,
                  op_range=(0.08, 0.50), texture=BURNT_TEXTURE, skip=None,
                  verbose=True):
    """The scar as bands of translucent overlay. Returns the band prim paths.

    ONE MESH PER BAND, not one prim per cell: `region` is diced into
    `cell_m` quads, each bucketed by its coverage, and every quad in a bucket
    becomes another face of that bucket's single mesh. A 250 m block costs
    `bands` prims and `bands` materials rather than thousands of each.
    `skip(x0, y0, x1, y1)` drops a cell (pools).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    x0, y0, x1, y1 = region
    tile = float(tile_m or max(x1 - x0, y1 - y0))
    center = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
    op_lo, op_hi = (float(op_range[0]), float(op_range[1]))
    nx = max(1, int(round((x1 - x0) / float(cell_m))))
    ny = max(1, int(round((y1 - y0) / float(cell_m))))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny

    buckets, n_skip = {}, 0
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cov = float(coverage_at(ax + dx * 0.5, ay + dy * 0.5))
            if cov <= 0.06:
                continue
            if skip is not None and skip(ax, ay, ax + dx, ay + dy):
                n_skip += 1
                continue
            buckets.setdefault(
                min(int(bands) - 1, int(cov * int(bands))), []).append((ax, ay))
    if not buckets:
        if verbose:
            print("[ground] coverage is zero everywhere — nothing to draw")
        return []

    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    if verbose:
        print("[ground] overlay tile {0:.0f} m centred on ({1:.0f}, {2:.0f}); "
              "band opacity {3:.2f}-{4:.2f}; {5} cell(s) skipped".format(
                  tile, center[0], center[1], op_lo, op_hi, n_skip))
    made = []
    for b, cells in sorted(buckets.items()):
        op = op_lo + (op_hi - op_lo) * (b + 0.5) / float(bands)
        mat = overlay_material(
            stage, "{0}/BurnLooks/band_{1}".format(material_parent, b), op,
            tile_m=tile, center=center, texture=texture)
        pts, counts, idx = [], [], []
        e = 0.02          # hairline overlap, so cells in a band show no seam
        for (ax, ay) in cells:
            k = len(pts)
            for (px, py) in ((ax - e, ay - e), (ax + dx + e, ay - e),
                             (ax + dx + e, ay + dy + e), (ax - e, ay + dy + e)):
                pts.append(Gf.Vec3f(px * ssf, py * ssf, z_m * ssf))
            counts.append(4)
            idx += [k, k + 1, k + 2, k + 3]
        path = "{0}/band_{1}".format(root, b)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * len(pts))
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.15, 0.13)])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), z_m * ssf),
                            Gf.Vec3f(max(xs), max(ys), z_m * ssf)])
        UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)
        made.append(path)
        if verbose:
            print("[ground] band {0:2d}: {1:5d} cells  opacity {2:.2f}".format(
                b, len(cells), op))
    return made
