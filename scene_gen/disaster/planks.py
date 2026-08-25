"""planks stage — the sawn-timber debris field a wind event leaves behind.

WHY THIS IS AUTHORED GEOMETRY AND NOT FRACTURE
----------------------------------------------
`disaster.fracture` cuts real modules into real fragments and `disaster.settle`
arranges them with PhysX. That is the right tool for a house coming apart, and
it is the wrong one for the debris FIELD, for three reasons that all point the
same way:

  * IT DOES NOT SCALE. Fracture is CPU trimesh and settle is GPU PhysX, and
    together they are the entire ~27-minute cost of the 250 m mini block. A
    tornado track across a plate is tens of thousands of loose boards. The
    whole point of the bake-and-assemble path is that the expensive work is
    done once per ARCHETYPE, and a debris field is per-PLACE by definition:
    it is where the wind went, not what the house was made of.
  * WHAT IT PRODUCES IS THE WRONG SHAPE ANYWAY. A Voronoi cell is a chunk. A
    building blown apart sheds BOARDS — studs, joists, sheathing, siding,
    decking — because that is the shape the mill cut them and the shape the
    fasteners let go at. `tornado.jpeg` is a field of pale rectangles.
  * A BOX NEEDS NO SOLVER. A board lying flat on open ground is fully
    described by a position, a yaw and a couple of degrees of tilt. There is
    nothing for physics to discover, and authoring it directly means the
    field can be RETUNED WITHOUT REBAKING ANYTHING — which is most of what
    iterating on this scene consists of.

So: `scatter_*` decides where every piece goes, `build` authors them, and the
two are pure geometry with no stage-side cost beyond the meshes themselves.

ONE MESH PER PATCH, NOT ONE PER BOARD
-------------------------------------
Same argument `ground.build_overlay` makes for its bands. A wrecked house
sheds a few hundred pieces and a plate holds hundreds of wrecked houses; at
one prim each that is a six-figure prim count for geometry that never moves,
never animates and shares one material. `build` merges every piece in a patch
into a single `UsdGeom.Mesh` — 8 points and 6 faces a board — so the whole
field costs a few hundred prims. It is also why the pieces are BOXES rather
than referenced assets: an instanced reference cannot be merged, and merging
is worth more here than instancing is.

NO CHAR, ANYWHERE
-----------------
This is bare structural timber that was inside a wall an hour ago. It takes
one plain wood material (`wood_material`) and nothing composites soot onto it.
`Ash_Planks` is the pack, chosen for its VALUE: fresh framing lumber and the
back of siding are pale, which is why a tornado debris field photographs
light against green grass while a burn scar photographs dark. Getting that
backwards is the single most visible way to make this read as the wrong
disaster.
"""

import math

# The lumber. Ash rather than oak because a debris field is mostly the INSIDE
# of the building — framing, sheathing, the unpainted back of everything — and
# that is pale. `damage._pbr` only carries a diffuse map, so this module
# authors its own OmniPBR to keep the normal and ORM that make a board read as
# a board rather than as a coloured rectangle.
WOOD_BASE = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
             "Wood/Ash_Planks/Ash_Planks_BaseColor.png")
WOOD_NORMAL = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
               "Wood/Ash_Planks/Ash_Planks_Normal.png")
WOOD_ORM = ("airstack://scene_gen/assets/aec/brownstone/Materials/Base/"
            "Wood/Ash_Planks/Ash_Planks_ORM.png")


# THE STOCK LIST, and it is a real cutting list rather than a size range.
#
# A uniform draw over "length 0.5-4 m, width 0.05-0.6 m" produces a cloud of
# arbitrary rectangles, and arbitrary is the one thing building material is
# not: a house is made of a handful of standard sections, and the debris from
# one is those sections in broken lengths. Naming them also makes the mix
# tunable by what it IS — more sheathing, fewer studs — instead of by moving
# an abstract number.
#
# (weight, length_range_m, width_range_m, thickness_range_m)
STOCK = {
    # 2x4 / 2x6 framing. The most numerous thing in a stick-built house and
    # the most numerous thing in its debris.
    "stud":      (0.34, (0.7, 3.4), (0.09, 0.15), (0.038, 0.048)),
    # Rafters, joists, headers — longer and deeper, and what makes a pile
    # read as structure rather than as kindling.
    "joist":     (0.14, (1.8, 5.0), (0.18, 0.30), (0.038, 0.058)),
    # OSB / plywood sheathing, in broken sheets. THE BIG PALE RECTANGLES.
    # These are what you actually see from altitude — a stud is a stick at
    # 60 m, a half-sheet of sheathing is an object.
    "sheathing": (0.26, (0.9, 2.4), (0.55, 1.22), (0.011, 0.019)),
    # Siding, fence boards, trim, decking. Long, thin, and they scatter
    # furthest because they have the most area per unit mass.
    "board":     (0.20, (1.0, 4.2), (0.14, 0.24), (0.016, 0.026)),
    # Roof decking with its shingles still on, in slabs. Darker, and the one
    # class that is genuinely a slab rather than a board.
    "deck":      (0.06, (0.8, 2.0), (0.6, 1.4), (0.05, 0.09)),
}

# Per-class colour tint on the shared map, so a field is not one uniform
# lumber-yard colour. Values, not hues: fresh framing is pale, sheathing is
# a shade browner, decking with shingles on it is dark.
_TINT = {
    "stud":      (1.00, 0.98, 0.94),
    "joist":     (0.93, 0.90, 0.86),
    "sheathing": (0.86, 0.79, 0.68),
    "board":     (1.00, 1.00, 0.98),
    "deck":      (0.44, 0.42, 0.40),
}


def _weighted(rng):
    """Draw a stock class by weight."""
    r = rng.random() * sum(v[0] for v in STOCK.values())
    for name, spec in STOCK.items():
        r -= spec[0]
        if r <= 0.0:
            return name
    return "stud"


# ---------------------------------------------------------------------------
# material
# ---------------------------------------------------------------------------

def wood_material(stage, path, tile_m=1.1, tint=(1.0, 1.0, 1.0),
                  roughness=0.78):
    """A plain sawn-timber OmniPBR, projected from WORLD coordinates.

    TRIPLANAR IS NOT OPTIONAL HERE, for the reason `damage._pbr` records: the
    boards are authored meshes with no UVs at all, so a UV-space material
    falls back to a per-face default and the map repeats inside every single
    triangle. World projection makes `texture_scale` metric — repeats per
    metre, so SMALLER means BIGGER features — and `tile_m` is its reciprocal.

    ~1.1 m a tile against a map that is itself a run of ~15 cm boards puts
    about seven board-widths across a tile, so a 2.4 m sheet of sheathing
    carries a plausible grain and a 0.10 m stud gets one board's worth of it
    rather than a compressed atlas.
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg

    existing = UsdShade.Material.Get(stage, path)
    if existing:
        return existing

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_BASE, "")))
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_NORMAL, "")))
    sh.CreateInput("enable_ORM_texture", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("ORM_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(sg._join_asset_root(WOOD_ORM, "")))
    # `diffuse_color_constant` MULTIPLIES the map in OmniPBR, so this is the
    # tint and not a replacement colour — the grain survives it.
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / max(1e-6, float(tile_m))
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def materials(stage, parent_path, tile_m=1.1):
    """One material per stock class, under `<parent_path>/PlankLooks`."""
    return {name: wood_material(stage,
                                "{0}/PlankLooks/{1}".format(parent_path, name),
                                tile_m=tile_m, tint=_TINT.get(name,
                                                              (1.0, 1.0, 1.0)))
            for name in STOCK}


# ---------------------------------------------------------------------------
# where the pieces go
# ---------------------------------------------------------------------------

def _piece(rng, klass=None, scale=1.0):
    """One board's dimensions and its class."""
    k = klass or _weighted(rng)
    _w, ln, wd, th = STOCK[k]
    return (k,
            rng.uniform(*ln) * scale,
            rng.uniform(*wd) * scale,
            rng.uniform(*th))


def _lay(x, y, k, ln, wd, th, heading, rng, ground_z=0.0, tilt_p=0.22):
    """Pose one board on the ground. Returns the spec dict `build` consumes.

    MOSTLY FLAT, AND THAT IS THE OBSERVATION. Debris photographs as a mat, not
    as a bramble: a board that lands on open ground lies down, and only the
    ones that came to rest ON something else are at an angle. `tilt_p` is the
    share that did — a fifth is enough to give the field depth without turning
    it into a jackstraw pile, which is what a uniform tilt looks like.

    ALIGNED, WEAKLY, TO THE FLOW. A long thin object in a moving fluid ends up
    lying across the flow more often than along it, and survey photographs of
    debris mats do show a grain. `heading + 90` with a wide spread reproduces
    it without asserting more than the picture supports.
    """
    if rng.random() < tilt_p:
        pitch = rng.uniform(-34.0, 34.0)
        roll = rng.uniform(-30.0, 30.0)
    else:
        pitch = rng.uniform(-4.0, 4.0)
        roll = rng.uniform(-6.0, 6.0)
    yaw = heading + 90.0 + rng.gauss(0.0, 46.0)
    # Seat it on the ground: half the vertical extent of the rotated box,
    # so a tilted board rests on its low corner instead of sinking into the
    # lawn or hovering over it.
    p, r = math.radians(pitch), math.radians(roll)
    half_h = 0.5 * (abs(th * math.cos(p) * math.cos(r))
                    + abs(wd * math.sin(r))
                    + abs(ln * math.sin(p)))
    return {"x": x, "y": y, "z": ground_z + half_h + 0.004,
            "l": ln, "w": wd, "t": th,
            "yaw": yaw, "pitch": pitch, "roll": roll, "class": k}


def scatter_from_wreck(cx, cy, footprint_m, intensity, heading_deg, reach_m,
                       rng, n_pieces=140, ground_z=0.0, scale=1.0):
    """The debris trail off ONE wrecked building. Returns a list of specs.

    THE SHAPE IS A COMET, and it is the single most recognisable feature of a
    tornado track from the air: a dense mat on and around the slab, thinning
    into a tail that runs downtrack and FANS as it goes. Three draws make it:

      * `s` — distance downtrack, `reach * u ** 1.9`, so most pieces stay
        near the building and a minority carry a long way. A linear draw
        gives an even smear, which reads as a painted stripe rather than as
        material that was thrown.
      * `t` — lateral offset, gaussian with a width that GROWS with `s`. A
        constant width is a corridor, and a corridor is what a bulldozer
        leaves. The spreading is what says the debris was airborne.
      * a base cloud inside the footprint itself, because the slab is never
        actually clean — even at EF5 there is a rim of sill plate, plumbing
        and the bottom course of everything.

    `intensity` scales the count and the reach together: a house at the edge
    of the path loses its shingles into the next garden, one on the centreline
    is found in the next field.
    """
    i = max(0.0, min(1.0, float(intensity)))
    th = math.radians(float(heading_deg))
    ux, uy = math.cos(th), math.sin(th)
    vx, vy = -math.sin(th), math.cos(th)
    fp = max(2.0, float(footprint_m))
    reach = float(reach_m) * (0.35 + 0.65 * i)
    n = max(0, int(round(n_pieces * (0.25 + 0.75 * i))))

    out = []
    for _ in range(n):
        if rng.random() < 0.30:
            # The mat on and immediately around the slab.
            s = rng.gauss(0.0, fp * 0.34)
            t = rng.gauss(0.0, fp * 0.40)
        else:
            s = reach * (rng.random() ** 1.9)
            t = rng.gauss(0.0, fp * 0.30 + s * 0.26)
        k, ln, wd, tk = _piece(rng, scale=scale)
        out.append(_lay(cx + ux * s + vx * t, cy + uy * s + vy * t,
                        k, ln, wd, tk, float(heading_deg), rng,
                        ground_z=ground_z))
    return out


def scatter_over_region(region, intensity_at, heading_deg, rng,
                        per_100m2=0.55, cell_m=12.0, ground_z=0.0,
                        min_intensity=0.12, scale=1.0):
    """Loose timber strewn across the CORRIDOR, not just on the lots.

    THE TRACK ITSELF IS COVERED, and this is what a per-building scatter
    cannot produce. In `tornado.jpeg` the debris does not stop at the property
    lines: it runs continuously across roads, verges, the field beyond and
    everything in between, because what is lying there came from somewhere
    else. A field assembled only from per-house trails leaves clean green
    gaps between the lots and the corridor stops reading as one event.

    Sampled on a `cell_m` lattice with the density scaled by the local
    intensity, so the corridor's own gradient carries into the debris: dense
    on the centreline, thinning to nothing at the path edge, with no separate
    edge to tune.
    """
    x0, y0, x1, y1 = region
    nx = max(1, int(round((x1 - x0) / float(cell_m))))
    ny = max(1, int(round((y1 - y0) / float(cell_m))))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area = dx * dy / 100.0
    out = []
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            i = float(intensity_at(ax + dx * 0.5, ay + dy * 0.5))
            if i < min_intensity:
                continue
            # Expected count, then a Poisson-ish draw so the field is not a
            # visible lattice of equal clumps.
            lam = per_100m2 * area * (i ** 1.4)
            n = int(lam) + (1 if rng.random() < (lam - int(lam)) else 0)
            for _ in range(n):
                k, ln, wd, tk = _piece(rng, scale=scale)
                out.append(_lay(ax + rng.random() * dx, ay + rng.random() * dy,
                                k, ln, wd, tk, float(heading_deg), rng,
                                ground_z=ground_z))
    return out


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

# Corner order is (sx, sy, sz) over (-1,-1,-1), (1,-1,-1), (1,1,-1),
# (-1,1,-1), (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1) — see `_box`.
#
# EVERY FACE IS WOUND COUNTER-CLOCKWISE SEEN FROM OUTSIDE, so the right-hand
# rule on the winding agrees with the normal beside it. A box whose windings
# and normals disagree is not a cosmetic problem: USD's default `orientation`
# is `rightHanded`, so the renderer culls on the winding and shades on the
# normal, and the result is boards that are lit from inside and disappear from
# half the angles you look at them from. Verified by cross product, face by
# face, rather than by eye — the six are easy to get individually right and
# collectively inconsistent.
_FACES = ((0, 3, 2, 1),        # -Z, the underside
          (4, 5, 6, 7),        # +Z
          (0, 1, 5, 4),        # -Y
          (1, 2, 6, 5),        # +X
          (2, 3, 7, 6),        # +Y
          (3, 0, 4, 7))        # -X
_FACE_N = ((0, 0, -1), (0, 0, 1), (0, -1, 0),
           (1, 0, 0), (0, 1, 0), (-1, 0, 0))


def _box(spec):
    """One board's 8 world points and its 6 face normals, from its spec."""
    hl, hw, ht = 0.5 * spec["l"], 0.5 * spec["w"], 0.5 * spec["t"]
    cy_, sy_ = (math.cos(math.radians(spec["yaw"])),
                math.sin(math.radians(spec["yaw"])))
    cp, sp = (math.cos(math.radians(spec["pitch"])),
              math.sin(math.radians(spec["pitch"])))
    cr, sr = (math.cos(math.radians(spec["roll"])),
              math.sin(math.radians(spec["roll"])))

    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll), written out rather than assembled
    # with numpy: this runs once per board over tens of thousands of boards
    # and the matrix product dominates if it allocates.
    m00 = cy_ * cp
    m01 = cy_ * sp * sr - sy_ * cr
    m02 = cy_ * sp * cr + sy_ * sr
    m10 = sy_ * cp
    m11 = sy_ * sp * sr + cy_ * cr
    m12 = sy_ * sp * cr - cy_ * sr
    m20 = -sp
    m21 = cp * sr
    m22 = cp * cr
    x, y, z = spec["x"], spec["y"], spec["z"]

    pts = []
    for (sx, sy2, sz) in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                          (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)):
        a, b, c = sx * hl, sy2 * hw, sz * ht
        pts.append((x + m00 * a + m01 * b + m02 * c,
                    y + m10 * a + m11 * b + m12 * c,
                    z + m20 * a + m21 * b + m22 * c))
    nrm = [(m00 * n[0] + m01 * n[1] + m02 * n[2],
            m10 * n[0] + m11 * n[1] + m12 * n[2],
            m20 * n[0] + m21 * n[1] + m22 * n[2]) for n in _FACE_N]
    return pts, nrm


def build(stage, root, specs, mats, ssf, verbose=True):
    """Author the field. ONE MESH PER STOCK CLASS. Returns the prim paths.

    Grouping by CLASS rather than by patch, because the material is per-class
    and a mesh can carry exactly one binding without GeomSubsets — and a
    GeomSubset per class inside one giant mesh buys nothing over five meshes.
    Five prims for a whole plate's worth of debris is as cheap as this gets.

    Normals are authored `faceVarying`. A board is a hard-edged box and the
    renderer's fallback is to average normals at shared vertices, which
    rounds every corner and makes a stack of lumber look like a heap of
    pillows — visible immediately at close range and, oddly, still visible
    from altitude as a loss of the crisp rectangular read that is the whole
    point of authoring boxes.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if not specs:
        return []
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    by_class = {}
    for s in specs:
        by_class.setdefault(s["class"], []).append(s)

    made = []
    for k, group in sorted(by_class.items()):
        pts, counts, idx, nrm = [], [], [], []
        for s in group:
            p, n = _box(s)
            base = len(pts)
            pts.extend(Gf.Vec3f(float(q[0]) * ssf, float(q[1]) * ssf,
                                float(q[2]) * ssf) for q in p)
            for fi, face in enumerate(_FACES):
                counts.append(4)
                idx.extend(base + v for v in face)
                nrm.extend([Gf.Vec3f(*(float(c) for c in n[fi]))] * 4)
        path = "{0}/{1}".format(root, k)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr([Gf.Vec3f(0.66, 0.58, 0.48)])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])
        mat = (mats or {}).get(k)
        if mat is not None:
            # APPLY the schema before binding. `MaterialBindingAPI(prim).Bind()`
            # authors the relationship either way and Kit honours it, but core
            # USD then warns "Found material bindings on prim ... but
            # MaterialBindingAPI is not applied" on every read — which is noise
            # in exactly the place (an offline validation pass) where you are
            # reading the file to find real problems.
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        made.append(path)
        if verbose:
            print("[planks] {0:<10s} {1:6d} piece(s) -> 1 mesh, {2} point(s)"
                  .format(k, len(group), len(pts)))
    return made
