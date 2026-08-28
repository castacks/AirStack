"""scour_relief stage — the THREE-DIMENSIONAL half of a tornado's ground scour.

WHAT THIS ADDS, AND WHAT IT DOES NOT REPLACE
--------------------------------------------
`ground.build_overlay` driven by `tornado.scour_coverage` paints the track
brown: a translucent mud surface in bands of constant opacity, darkest on the
centreline and fading to nothing at the path edge. That is right, it is what
the reference photograph shows, and it stays exactly as it is. This module is
ADDITIVE and the overlay is its backdrop — remove the overlay and what is
authored here is a scatter of lumps on a green lawn.

What the overlay cannot do is cast a shadow. It is a flat film on a flat lawn,
so from 40 m up the track reads as a STAIN: the colour is right, the surface is
still a billiard table, and at the hour a survey flight actually photographs —
low sun, long shadows — that is the whole difference between "somebody painted
the grass" and "something took the ground away".

THE EARTHQUAKE PIPELINE ALREADY SOLVED THIS, FOR A BUILDING
-----------------------------------------------------------
`quake_flow._c_ground_response` and the passes under it (`_c_heave`,
`_c_clods`, `_c_soil_patch`, `_ejecta`, `_berm`) author real earth with real
silhouettes round a building that leaned or sank, and the round-2 bench notes
in that file are the record of what it took to make them read. Three of those
lessons transfer verbatim and are why this module has the shape it does:

  * A FLAT POLYGON IS A PAPER CUT-OUT, however it is textured. `_c_soil_patch`
    raises the centre of every spilt fan for exactly this reason. NOTHING here
    is flat — the lowest feature in the table below still stands 4 cm proud.
  * THE MATERIAL HAS TO BE WORLD-PROJECTED. These meshes carry no UVs at all,
    so a referenced UV-space `.usda` — the AEC `Dirt`, or a megascans pack
    bound directly — renders as one flat cream mat ("spilled paper on the
    asphalt", round-2 bench). It has to go through `damage._pbr(texture=...)`,
    which sets `project_uvw` + `world_or_object` and scales in repeats per
    METRE; and the tint has to go on `diffuse_tint` (NOT
    `diffuse_color_constant`, which the map replaces) with
    `albedo_desaturation` beside it, or no multiplier will take the orange out
    of an orange mud map. See `_TEX` below.
  * A SMOOTH EXTRUSION READS AS A MOULDING. `_c_clods` exists because the
    round-1 berm was a clean swept surface and looked machined. Loose material
    ON a crest, at more than one size, overlapping, is what makes earth read as
    earth rather than as trim.

WHY THE QUAKE CODE COULD NOT SIMPLY BE CALLED
---------------------------------------------
Two reasons, and both are structural rather than cosmetic.

FRAMING. Every one of those passes is framed on a BUILDING. They take `m` — a
mass dict with `W`, `D`, `yaw`, `cx`, `cy`, `z0` — and author around its
footprint: `_c_perim` walks a rectangle, `_c_heave` wedges soil against one of
its four sides, `_berm` rings it, `_ejecta` fans out from it. A tornado track
has no footprint. It is a FIELD over the whole plate and the thing to author is
where the field is strong, not where a wall is. There is no `m` to pass.

COST. They author ONE PRIM PER PIECE. That is correct for a building — a few
dozen clods round one lean, and the recipes make almost no loose bodies — and
it does not survive a 500 m plate: at the densities below it is tens of
thousands of prims for geometry that never moves, never animates and shares
three materials. So this module follows `planks` instead. `scatter` decides
where everything goes in pure Python with no stage anywhere near it, `build`
merges the lot into ONE MESH PER MATERIAL CLASS, and because the two are
separable the field can be retuned, measured and unit-tested offline —
`tests/test_scour_relief.py` never imports `pxr`.

THE FEATURES, AND WHAT EACH ONE IS IN THE PHOTOGRAPH
----------------------------------------------------
| kind    | what it is                             | where it goes            |
|---------|----------------------------------------|--------------------------|
| `arc`   | a cycloidal ground mark: the groove a   | the core, at high         |
|         | suction vortex cut, authored as its     | intensity only            |
|         | flanking spoil                          |                           |
| `ridge` | a windrow — soil and grass swept into a | the core and shoulders    |
|         | low transverse bank                     |                           |
| `mound` | a heap of turned-up subsoil             | anywhere well scoured,    |
|         |                                         | denser on the LEFT flank  |
| `sod`   | turf peeled off in a mat and ROLLED UP  | the coverage EDGE, where  |
|         | like carpet                             | mud meets surviving grass |
| `clod`  | a lump of earth thrown clear            | everywhere, densest in    |
|         |                                         | the core                  |
| `fan`   | mud washed across pavement              | roads only                |

`sod` is the one that explains the other five. The overlay says the ground is
brown; a rolled mat of turf lying at the edge of the brown says WHY it is
brown, and nothing else in this scene does. It is placed on the coverage
GRADIENT rather than at the coverage peak (a hump centred near `sod_at`), which
is where turf actually tears: in the middle of the core there is no turf left to
peel.

WE CANNOT CUT A GROOVE, SO WE AUTHOR ITS SPOIL
----------------------------------------------
A real cycloidal mark is a trench — Jarrell 1997 scoured to nearly half a metre
in places. The ground here is an opaque sheet at `_Z_GRASS` laid by
`suburb_scene.apply_ground`, and anything authored below it is simply hidden by
it: a trench floor at -0.10 m is under the plate and the plate is what you see.
Dropping a hole in the plate is not available either — it is one merged sheet
per region and the overlay bands are laid on top of it.

So an `arc` is authored as WHAT CAME OUT OF THE TRENCH: two levees flanking a
strip of bare ground at grade, asymmetric, with the bigger levee on the outside
of the vortex's turn. With the mud overlay running between them that reads as a
gouge from any altitude worth flying, and the read fails only from ground level
at a grazing angle. The honest fix is a displaced ground mesh, which means
`apply_ground` taking a height field; noted in the skill's known gaps.

THE ARCS ARE NOT DECORATION — THEY ARE THE VORTEX
--------------------------------------------------
Cycloidal ground marks are the canonical aerial signature of a violent tornado
(Van Tassel on Scottsbluff 1955; Fujita's suction-vortex analyses), and they
are the one ground feature that says which way the thing was TURNING rather
than only which way it was going. They are generated here from the mechanism
rather than drawn as ornament: a suction vortex orbiting at radius `R` and rate
`w` about a centre translating at `V` traces a trochoid,

    along(t) = V t + R cos(w t + phase)
    cross(t) =       R sin(w t + phase)

and with `k = R w / V > 1` that curve LOOPS, which is what the marks look like
from the air. Two consequences fall out of the same maths for free, and both
agree with what `tornado.py` already says about this scene:

  * the vortex's speed over the ground is `V * sqrt(1 - 2k sin(theta) + k^2)`,
    maximised at `theta = -pi/2` — the RIGHT flank of the track, where the
    rotational and translational components add. So a mark is only cut where
    that factor is high, which puts the arcs on the right of the centreline;
  * and material lofted on the strong side is carried across and dropped on
    the weak one, which is precisely the reason `tornado.DEFAULTS` throws
    debris `curl_deg` to the LEFT. So the deposition features (`mound`,
    `ridge`) are biased left by `left_bias` while the erosional one is biased
    right, out of one model rather than out of two guesses.
"""

import math
import os

# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------
#
# Same table, same reasoning and very nearly the same numbers as
# `quake_flow._C_TEX` — deliberately, so a tint tuned on one disaster's soil
# means something on the other's. Kept as a copy rather than an import because
# `_c_look` is reached only through a quake `ctx` dict and the 9,700-line
# module it lives in imports the whole earthquake pipeline to get there.
#
# (relative texture, TINT, roughness, repeats per metre, desaturation)
_TEX = {
    # Wet dark subsoil: what is under a lawn and what the overlay is a picture
    # of. The bulk of everything authored here.
    "soil": ("megascans/Soil_Mud/T_pjuph20_1K_B.jpg",
             (0.34, 0.33, 0.32), 0.98, (0.70, 0.70), 0.55),
    # Drier, paler, coarser — a minority class purely so a field of mounds is
    # not one colour. A scour exposes more than one horizon.
    "silt": ("megascans/Dirt_Rough/T_yd0lfcqcc_1k_B.png",
             (0.42, 0.42, 0.43), 0.96, (0.55, 0.55), 0.65),
    # THE UNDERSIDE OF PEELED TURF, which is what a sod roll shows: root mat
    # and the few centimetres of soil that came with it. Darker and finer than
    # the subsoil (more repeats per metre) — a rolled mat photographed from
    # above is a dark cylinder, not a green one, and painting it green is the
    # single fastest way to make a sod roll read as a hedge.
    "sod": ("megascans/Soil_Mud/T_pjuph20_1K_B.jpg",
            (0.24, 0.235, 0.22), 1.0, (1.15, 1.15), 0.45),
}
# Kit materials to fall back on when a texture will not resolve, by the name
# `quake_flow.materials()` / a launcher's material dict uses.
_FALLBACK = {"soil": "soil", "silt": "soil", "sod": "soil"}

CLASSES = tuple(sorted(_TEX))


def materials(stage, parent_path, kit_mats=None):
    """The three soil looks under `<parent_path>/ScourLooks`. Returns a dict
    keyed by the class names the specs carry.

    `kit_mats`, if given, is any dict of already-built materials to fall back
    on per `_FALLBACK` — pass `quake_flow.materials(...)`'s output or the
    suburb's own, or nothing at all and get `None` (unbound, which draws in the
    viewport's default grey and is a loud enough failure to notice).
    """
    from pxr import Gf, Sdf, UsdShade

    import scene_generator as sg
    from . import damage

    out = {}
    for key in CLASSES:
        rel, rgb, rough, scale, desat = _TEX[key]
        path = "{0}/ScourLooks/{1}".format(parent_path, key)
        try:
            mat = damage._pbr(
                stage, path, rgb, rough, tint=rgb, scale_uv=scale,
                texture=sg._join_asset_root(
                    "airstack://scene_gen/assets/materials/" + rel, ""))
            sh = UsdShade.Shader.Get(stage, path + "/Shader")
            if sh:
                # THE TINT GOES HERE, not on `diffuse_color_constant`.
                # `damage._pbr` puts it on the latter, which OmniPBR.mdl
                # documents as "the albedo base color" — the thing the map
                # REPLACES. `diffuse_tint` is what multiplies the final
                # albedo, and `albedo_desaturation` is the only control that
                # can take the orange out of an orange mud map.
                sh.CreateInput("diffuse_tint",
                               Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
                sh.CreateInput("albedo_desaturation",
                               Sdf.ValueTypeNames.Float).Set(float(desat))
        except Exception as exc:
            print("[scour_relief] look {0} unavailable ({1})".format(key, exc))
            mat = (kit_mats or {}).get(_FALLBACK[key])
        out[key] = mat
    return out


# ---------------------------------------------------------------------------
# the knobs
# ---------------------------------------------------------------------------
#
# Densities are per 100 m2 of AFFECTED ground, weighted by the coverage field —
# the same contract `planks.scatter_over_region` and the `path_*_per_100m2`
# debris rules use, so "0.5 per 100 m2" means the same thing whatever the track
# is doing. Defaults were chosen against the shipped 500 m preset (a 150 m
# track, ~30% of the plate in the path): they put roughly one mound every 9 m
# and one clod every 3.5 m in the core, thinning to nothing at the path edge.
DEFAULT_KNOBS = {
    # per 100 m2 of affected ground
    # 1.6 rather than the 0.55 the first sweep used. Measured on the shipped
    # preset that is ~250 heaps over a 500 m plate, or four or five in the
    # 120 m square a drone frames at 60 m — at 0.55 it was one, and one heap
    # in a frame reads as a molehill somebody left rather than as ground that
    # has been turned over.
    "mounds_per_100m2": 1.6,
    "clods_per_100m2": 5.0,
    "ridges_per_100m2": 0.26,
    "sod_per_100m2": 0.32,
    # ...and the share of the mound/clod rate that survives onto pavement, as
    # low flat `fan`s. NOT zero: the overlay deliberately passes under the
    # asphalt so the streets stay legible through the track, which means the
    # roads currently come through a tornado spotlessly clean. They do not in
    # any photograph of one — a track lays mud right across a carriageway —
    # and a 3D fan is the only way to say so without touching the overlay's z
    # ladder.
    "pave_wash": 0.30,

    # Coverage below which nothing at all is authored. The overlay reaches
    # further than the relief does, and that is correct: peeling turf takes
    # less wind than gouging subsoil out from under it.
    "min_coverage": 0.24,
    # Where in the coverage ramp sod rolls peak, and how wide that hump is.
    # The middle of the band, not the core — see the module docstring.
    "sod_at": 0.46,
    "sod_width": 0.17,
    # Coverage a cycloidal mark needs. Marks are an EF3+ feature; on a
    # `peak = 0.92` track this keeps them in the core.
    "arc_min_coverage": 0.66,

    # HOW MANY SUCTION VORTICES, and the trochoid each one traces. `k` is
    # R * omega / V: below 1 the mark is a wavy line, at 1 it cusps, above 1 it
    # LOOPS, which is what the aerial photographs of cycloidal marks show.
    "arc_vortices": 3,
    "arc_k": (1.25, 2.40),
    # Orbit radius as a fraction of the track HALF-width. Suction vortices
    # ride the core, not the shoulders.
    "arc_radius_frac": (0.16, 0.52),
    # Cut a mark only where the vortex's speed over the ground is in the top
    # of its range — 0 would draw the whole trochoid, which is a doodle.
    "arc_speed_min": 0.55,
    # Sampling step along a mark, and the shortest run worth keeping.
    "arc_step_m": 1.4,
    "arc_min_len_m": 4.5,

    # Share of the deposition features kept on the RIGHT (weak) flank. The
    # left is the deposition side — see the module docstring — so this is
    # below 1 and above 0: a bias, not a rule.
    "left_bias": 0.55,

    # Everything vertical is multiplied by this. The one knob to reach for
    # first: the geometry is right or it is not, but "how much" is a judgement
    # made at altitude and it is the only thing a bench flight can settle.
    #
    # A clod and a sod roll scale BODILY rather than only in z, because their
    # height IS their size — a 0.3 m lump squashed to 0.15 m in z alone is a
    # flake, not a smaller lump. So this knob moves a handful of plan extents
    # too, which means it can change what `clip_to_region` drops at the plate
    # edge by a feature or two. That is the whole of its effect on counts.
    "height": 1.0,
    # Sampling lattice for the field scatter. 8 m is fine enough that the
    # coverage gradient across a 150 m track is resolved in ~19 steps and
    # coarse enough that a plate is a few thousand cells.
    "cell_m": 8.0,
}


def knobs_from_env():
    """`DEFAULT_KNOBS` with `SCOUR_*` environment overrides applied.

    Deliberately parallel to `tornado.knobs_from_env` and `ground.knobs_from_env`
    so what was tuned on a bench means the same in a scene. Every scalar knob is
    reachable; the two tuple-valued arc knobs are not, because nothing has yet
    needed to sweep them from a shell.
    """
    kn = dict(DEFAULT_KNOBS)
    for name, val in sorted(DEFAULT_KNOBS.items()):
        if isinstance(val, tuple):
            continue
        raw = os.environ.get("SCOUR_" + name.upper())
        if raw is None or raw.strip() == "":
            continue
        kn[name] = int(raw) if isinstance(val, int) else float(raw)
    return kn


def enabled():
    """`SCOUR_RELIEF=0` turns the whole pass off."""
    return os.environ.get("SCOUR_RELIEF", "1").strip().lower() not in (
        "0", "false", "no")


# ---------------------------------------------------------------------------
# where the pavement is
# ---------------------------------------------------------------------------

def pavement_mask(corridors, region, cell_m=2.0, pad_m=0.5):
    """`at(x, y) -> bool` for "this point is on a carriageway".

    `corridors` is an iterable of `(points, half_width_m)` — the shape a
    `suburb_net.Edge` already has (`e.pts`, `e.half_w`), passed as plain data so
    this module never imports the layout package and the mask can be built for
    any road source.

    RASTERISED ONCE, not tested per sample. The field scatter draws several
    thousand candidates and a plat carries a couple of hundred edges; a
    point-to-polyline test per candidate is a million distance evaluations for
    an answer that is constant over a 2 m cell. Marking a disc per half-step
    along each segment over-covers slightly at the corners, which is the safe
    direction — the cost of a false positive is a mound that becomes a mud fan.
    """
    x0, y0, x1, y1 = (float(q) for q in region)
    cw = max(0.25, float(cell_m))
    nx = max(1, int(math.ceil((x1 - x0) / cw)))
    ny = max(1, int(math.ceil((y1 - y0) / cw)))
    hit = bytearray(nx * ny)

    def _mark(px, py, r):
        i0 = max(0, int((px - r - x0) / cw))
        i1 = min(nx - 1, int((px + r - x0) / cw))
        j0 = max(0, int((py - r - y0) / cw))
        j1 = min(ny - 1, int((py + r - y0) / cw))
        rr = r * r
        for j in range(j0, j1 + 1):
            cy = y0 + (j + 0.5) * cw
            for i in range(i0, i1 + 1):
                cx = x0 + (i + 0.5) * cw
                if (cx - px) ** 2 + (cy - py) ** 2 <= rr:
                    hit[j * nx + i] = 1

    for pts, half_w in corridors or ():
        r = float(half_w) + float(pad_m)
        pts = [(float(p[0]), float(p[1])) for p in (pts or ())]
        if r <= 0.0 or len(pts) < 2:
            continue
        step = max(0.5, cw * 0.5)
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            n = max(1, int(seg / step))
            for k in range(n + 1):
                t = k / float(n)
                _mark(ax + (bx - ax) * t, ay + (by - ay) * t, r)

    def at(x, y):
        i = int((float(x) - x0) / cw)
        j = int((float(y) - y0) / cw)
        if i < 0 or j < 0 or i >= nx or j >= ny:
            return False
        return bool(hit[j * nx + i])

    return at


# ---------------------------------------------------------------------------
# where everything goes
# ---------------------------------------------------------------------------
#
# EVERY SPEC IS A COMPLETE DESCRIPTION AND `build` DRAWS NO RANDOM NUMBERS.
# The lumpiness of a mound is three sine harmonics whose phases and amplitudes
# are IN the spec (`wob`), not an rng call at author time. That is what makes
# the whole pass measurable offline: a spec list can be counted, summed,
# histogrammed and diffed between seeds by a test with no `pxr` on the path,
# and re-authoring the same specs twice gives the same stage.
#
# The extruded kinds (`ridge`, `arc`, `sod`) all carry `stations`, a polyline
# of 7-tuples
#
#     (x, y, crest_left, crest_right, width_left, width_right, groove_half)
#
# in one arity for all three so the section builder is the only thing that
# differs. `sod` puts its half-height in both crest slots and its half-width in
# both width slots; `ridge` leaves `groove_half` at zero, which is what selects
# the single-crest section.


def _wob(rng):
    """Three harmonics of plan-shape wobble: `(a1, p1, a2, p2, a3, p3)`.

    Two would be enough to stop a mound being an ellipse and three is what
    `vegetation.scar_patch` and `_c_soil_patch` both settled on, for the same
    reason: at two the eye still finds the shape, at three it stops trying.
    """
    return (rng.uniform(0.14, 0.26), rng.uniform(0.0, 6.2832),
            rng.uniform(0.07, 0.16), rng.uniform(0.0, 6.2832),
            rng.uniform(0.04, 0.10), rng.uniform(0.0, 6.2832))


def _draw(lam, rng):
    """A count from an expected value. Integer part plus a coin on the rest —
    `planks.scatter_over_region`'s draw, and for its reason: a lattice of equal
    clumps is visible as a lattice."""
    n = int(lam)
    return n + (1 if rng.random() < (lam - n) else 0)


def _clod(x, y, base_z, size, cls, rng):
    """One lump of earth, seated on its own low corner.

    The seating is `planks._lay`'s, which is the version that MEASURES: half
    the vertical extent of the rotated box, so a tipped clod rests on a corner
    instead of sinking in or hovering. A guessed lift is wrong for every size
    at once and the sizes here span 4x.
    """
    ln = size
    wd = ln * rng.uniform(0.60, 1.10)
    th = ln * rng.uniform(0.45, 0.90)
    pitch = rng.uniform(-26.0, 26.0)
    roll = rng.uniform(-26.0, 26.0)
    p, r = math.radians(pitch), math.radians(roll)
    # The width term carries the PITCH COSINE with it — the true vertical
    # half-extent of a box under R = Rz . Ry . Rx. `planks._lay` had the same
    # slip and it was corrected there on 2026-08-27 for the same reason: the
    # error is small (millimetres at this size) and it is always in the FLOAT
    # direction, and a clod is 0.10-0.34 m so a millimetre of it shows.
    half_h = 0.5 * (abs(th * math.cos(p) * math.cos(r))
                    + abs(wd * math.cos(p) * math.sin(r))
                    + abs(ln * math.sin(p)))
    return {"kind": "clod", "cls": cls, "x": x, "y": y,
            # `z` IS THE BOX CENTRE for a clod and the BASE PLANE for every
            # other kind — `planks._box` wants a centre and a dome wants a
            # grade. `base` carries the grade for all of them so a reader (and
            # the seating test) has one key that means one thing.
            "z": base_z + half_h - 0.015, "base": base_z,
            "l": ln, "w": wd, "t": th,
            "yaw": rng.uniform(0.0, 360.0), "pitch": pitch, "roll": roll}


def _polyline(x, y, heading_deg, length_m, curve, step_m, rng):
    """A gently curving polyline of `length_m` CENTRED on `(x, y)`.

    `curve` is radians of turn per metre. A windrow is not a ruled line and a
    straight one at 8 m long is immediately legible as authored geometry; a
    couple of degrees a metre is enough to break it without making it a snake.

    Centred rather than started at the sample point, because the sample point
    is where the density field said to put a feature: an 11 m windrow that
    RUNS from there puts most of itself somewhere the field was never asked
    about, which shows up as a corridor whose windrows all lean one way off it.
    """
    n = max(3, int(round(length_m / max(0.4, step_m))))
    th = math.radians(heading_deg)
    px = float(x) - math.cos(th) * length_m * 0.5
    py = float(y) - math.sin(th) * length_m * 0.5
    step = length_m / n
    pts = [(px, py)]
    for _k in range(n):
        th += curve * step
        px += math.cos(th) * step
        py += math.sin(th) * step
        pts.append((px, py))
    return pts


def scatter_field(region, coverage_at, rng, *, flow_deg, ground_z=0.0,
                  pave_z=None, pavement_at=None, skip=None, to_track=None,
                  knobs=None):
    """Mounds, windrows, sod rolls, clods and road wash over the whole plate.

    Sampled on a `cell_m` lattice with every density scaled by the LOCAL
    coverage, exactly as `planks.scatter_over_region` does it, so the
    corridor's own gradient carries into the relief and there is no separate
    edge to tune. What differs per feature is only the exponent on that ramp,
    and each one says something:

      * clods (1.1) reach nearly as far as the mud does — a thrown lump does
        not need the turf to have gone;
      * mounds (1.6) need subsoil turned over, so they hold to the core;
      * windrows (1.8) need enough material moving to pile up, and are the
        first thing to disappear off a weak track;
      * sod rolls are the exception and do NOT use the ramp at all. They peak
        on a HUMP at `sod_at`, because a peeled mat is a thing that happens at
        the edge of the scour. In the middle of the core there is no turf left
        to peel, and a sod roll lying on the centreline is a mat that came from
        somewhere it cannot have come from.

    `to_track`, if given (`tornado.frame`'s first return), biases the
    deposition features toward the LEFT flank by `left_bias`. `pavement_at`
    turns a cell over to low mud `fan`s based at `pave_z`. `skip(x, y)` drops a
    point outright — pools, and the footprint of a house still standing.
    """
    kn = dict(DEFAULT_KNOBS)
    kn.update(knobs or {})
    x0, y0, x1, y1 = (float(q) for q in region)
    cell = max(1.0, float(kn["cell_m"]))
    hs = float(kn["height"])
    min_cov = float(kn["min_coverage"])
    left_bias = float(kn["left_bias"])
    pz = ground_z if pave_z is None else float(pave_z)
    nx = max(1, int(round((x1 - x0) / cell)))
    ny = max(1, int(round((y1 - y0) / cell)))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    area_100 = dx * dy / 100.0

    out = []

    def _clear(spec):
        """Is every station of an extruded feature off the carriageway?"""
        if pavement_at is None:
            return True
        return not any(pavement_at(q[0], q[1]) for q in spec["stations"])

    def _pt(ax, ay, want_paved=None):
        """`(x, y, is_paved)` in the cell, or None if nothing suitable is free.

        THE PAVEMENT TEST IS ON THE POINT, not on the cell. Which passes run at
        all is decided once per 8 m cell, which is cheap and right — but a cell
        that straddles a kerb is half lawn and half carriageway, and taking the
        cell's answer for the POINT put mounds on the asphalt and, worse, put
        clods at lawn grade INSIDE a road surface that sits 4 cm above it.
        `want_paved` is what the caller needs: False for anything that only
        happens in soil, True for the road wash, None for a clod, which can
        land on either and takes the grade of wherever it landed.
        """
        for _try in range(4):
            px, py = ax + rng.random() * dx, ay + rng.random() * dy
            if skip is not None and skip(px, py):
                continue
            # AND RE-TEST THE FIELD AT THE POINT. The density was decided on
            # the cell CENTRE, which is right for a rate and wrong for a
            # placement: `tornado.scour_coverage` cuts ISLANDS of surviving
            # turf out of the band, they are smaller than a cell, and a cell
            # whose centre is in the mud can still draw a point inside one. A
            # heap of subsoil standing on the green patch the vortex missed is
            # the one artefact in this pass that says "scattered", not
            # "scoured".
            if float(coverage_at(px, py)) < min_cov:
                continue
            pv = bool(pavement_at(px, py)) if pavement_at is not None else False
            if want_paved is not None and pv != want_paved:
                continue
            return px, py, pv
        return None

    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cx, cy = ax + dx * 0.5, ay + dy * 0.5
            cov = float(coverage_at(cx, cy))
            if cov < min_cov:
                continue
            ramp = min(1.0, (cov - min_cov) / max(1e-6, 1.0 - min_cov))
            paved = bool(pavement_at(cx, cy)) if pavement_at is not None else False
            # DEPOSITION IS LEFT OF TRACK — see the module docstring. `cross`
            # positive is left (`tornado.frame`), and a right-flank CELL keeps
            # its deposition features with probability `left_bias`.
            #
            # ONE COIN PER CELL, not one per candidate point. The first cut put
            # the draw inside the four-try point loop, where a rejected point
            # is immediately re-rolled in the same cell: a bias of 0.55 came
            # out as 1 - 0.45**4 = 0.96, which measured as 53% of mounds on the
            # left against the 65% the knob asks for. A rejection that can be
            # retried is not a rejection.
            keep_dep = True
            if to_track is not None and left_bias < 1.0:
                if to_track(cx, cy)[1] < 0.0 and rng.random() > left_bias:
                    keep_dep = False

            if paved:
                # THE ROAD. No heaps and no windrows — a carriageway does not
                # turn over, and a 0.4 m mound on the asphalt is a boulder. It
                # gets what a road actually gets: mud washed across it and the
                # odd clod dragged out of the verge.
                wash = float(kn["pave_wash"])
                for _k in range(_draw(kn["mounds_per_100m2"] * 1.7 * wash
                                      * area_100 * (ramp ** 1.4), rng)):
                    p = _pt(ax, ay, True)
                    if p is None:
                        continue
                    rx = rng.uniform(1.4, 4.6)
                    out.append({
                        "kind": "fan",
                        "cls": "soil" if rng.random() < 0.8 else "silt",
                        "x": p[0], "y": p[1], "z": pz, "base": pz,
                        "rx": rx, "ry": rx * rng.uniform(0.40, 0.85),
                        "h": rng.uniform(0.035, 0.13) * hs,
                        "yaw": flow_deg + rng.gauss(0.0, 20.0),
                        "wob": _wob(rng)})
                for _k in range(_draw(kn["clods_per_100m2"] * 0.45 * wash
                                      * area_100 * (ramp ** 1.1), rng)):
                    p = _pt(ax, ay)
                    if p is None:
                        continue
                    out.append(_clod(p[0], p[1], pz if p[2] else ground_z,
                                     rng.uniform(0.10, 0.30) * hs,
                                     "soil" if rng.random() < 0.7 else "silt",
                                     rng))
                continue

            # --- mounds -------------------------------------------------
            for _k in range(_draw(kn["mounds_per_100m2"] * area_100
                                  * (ramp ** 1.6), rng) if keep_dep else 0):
                p = _pt(ax, ay, False)
                if p is None:
                    continue
                rx = rng.uniform(0.9, 3.2) * (0.70 + 0.50 * ramp)
                out.append({
                    "kind": "mound",
                    "cls": "soil" if rng.random() < 0.72 else "silt",
                    "x": p[0], "y": p[1], "z": ground_z, "base": ground_z,
                    "rx": rx, "ry": rx * rng.uniform(0.55, 1.00),
                    "h": rng.uniform(0.12, 0.48) * (0.55 + 0.55 * ramp) * hs,
                    # ELONGATED ALONG THE FLOW, weakly. A drift of soil is
                    # drawn out the way the wind was going; a field of circles
                    # is a field of circles from any altitude.
                    "yaw": flow_deg + rng.gauss(0.0, 26.0),
                    "wob": _wob(rng)})

            # --- clods --------------------------------------------------
            for _k in range(_draw(kn["clods_per_100m2"] * area_100
                                  * (ramp ** 1.1), rng)):
                p = _pt(ax, ay)
                if p is None:
                    continue
                out.append(_clod(p[0], p[1], pz if p[2] else ground_z,
                                 rng.uniform(0.12, 0.46) * (0.75 + 0.45 * ramp) * hs,
                                 "soil" if rng.random() < 0.70 else "silt", rng))

            # --- windrows -----------------------------------------------
            for _k in range(_draw(kn["ridges_per_100m2"] * area_100
                                  * (ramp ** 1.8), rng) if keep_dep else 0):
                p = _pt(ax, ay, False)
                if p is None:
                    continue
                # THE WHOLE RUN, not just where it started. `_pt` clears the
                # sample point, and a windrow is up to 11 m long CENTRED on it,
                # so one that starts on a verge can lie straight across the
                # carriageway — at lawn grade, half inside a road surface 4 cm
                # above it. Dropped rather than retried: it is one in thirty and
                # a retry would only shift the same run a metre.
                out.append(_ridge_spec(p[0], p[1], ground_z, flow_deg, ramp,
                                       hs, rng))
                if not _clear(out[-1]):
                    out.pop()

            # --- sod rolls ----------------------------------------------
            hump = math.exp(-(((cov - float(kn["sod_at"]))
                               / max(1e-6, float(kn["sod_width"]))) ** 2))
            for _k in range(_draw(kn["sod_per_100m2"] * area_100 * hump, rng)):
                p = _pt(ax, ay, False)
                if p is None:
                    continue
                out.append(_sod_spec(p[0], p[1], ground_z, flow_deg, hs, rng))
                if not _clear(out[-1]):
                    out.pop()

    return out


def _ridge_spec(x, y, base_z, flow_deg, ramp, hs, rng):
    """A windrow: soil and torn grass swept into a low transverse bank.

    ACROSS THE FLOW, not along it. Material being dragged over the ground piles
    against whatever slows it and the pile grows perpendicular to the drag —
    the same observation `planks._lay` makes about boards ending up lying
    across the flow, and the same wide spread on it, because neither is a
    strong enough effect to assert as a rule.
    """
    length = rng.uniform(3.5, 11.0) * (0.7 + 0.5 * ramp)
    heading = flow_deg + 90.0 + rng.gauss(0.0, 30.0)
    pts = _polyline(x, y, heading, length,
                    rng.gauss(0.0, 0.024), 1.2, rng)
    h_peak = rng.uniform(0.10, 0.35) * (0.50 + 0.60 * ramp) * hs
    w = rng.uniform(0.55, 1.50)
    # The crest leans downwind: the toe on the lee side reaches further and the
    # windward face is the steeper one. `asym` is which way, once per windrow.
    asym = rng.uniform(0.15, 0.40) * (1.0 if rng.random() < 0.5 else -1.0)
    ph = rng.uniform(0.0, 6.2832)
    n = len(pts) - 1
    stations = []
    for k, (px, py) in enumerate(pts):
        t = k / float(n)
        # DIES AT BOTH ENDS. A bank that stops at full height has two vertical
        # walls, which no pile of earth has.
        taper = math.sin(math.pi * t) ** 0.7
        h = h_peak * taper * (0.78 + 0.30 * math.sin(t * 9.0 + ph))
        stations.append((px, py, h, h,
                         w * (1.0 + asym), w * (1.0 - asym), 0.0))
    # `x`/`y` are the SEED POINT, kept because it is the only place the
    # density field was ever consulted: a windrow is 11 m long and its own mid
    # station can sit a metre away, which on `scour_coverage`'s ~2 m island
    # lattice is enough to read a different value. A reader (or a test) asking
    # "was this placed where the field allowed it?" has to ask here.
    return {"kind": "ridge", "cls": "soil" if rng.random() < 0.8 else "silt",
            "z": base_z, "base": base_z, "x": x, "y": y,
            "stations": stations}


def _sod_spec(x, y, base_z, flow_deg, hs, rng):
    """Turf peeled off in a mat and ROLLED UP like carpet.

    A half-cylinder of rolled sod with the unrolled mat still lying beyond one
    end — the taper from roll to mat is the whole read, because a bare
    half-cylinder is a log and this scene already has plenty of those.

    THE AXIS IS ACROSS THE FLOW because that is how a carpet rolls when it is
    pushed: the roll grows perpendicular to the push. And the material is the
    UNDERSIDE (`sod`, dark root mat), not grass — a mat that has rolled has by
    definition turned its green side inward, and a green cylinder in a mud
    corridor reads as a hedge.
    """
    roll_len = rng.uniform(1.4, 4.6)
    r = rng.uniform(0.13, 0.34) * hs
    tail = roll_len * rng.uniform(0.35, 0.95)
    heading = flow_deg + 90.0 + rng.gauss(0.0, 24.0)
    pts = _polyline(x, y, heading, roll_len + tail,
                    rng.gauss(0.0, 0.03), 0.55, rng)
    n = len(pts) - 1
    split = roll_len / max(1e-6, roll_len + tail)
    ph = rng.uniform(0.0, 6.2832)
    stations = []
    for k, (px, py) in enumerate(pts):
        t = k / float(n)
        if t <= split:
            # the roll: full radius, easing off at the outer end
            u = t / max(1e-6, split)
            f = min(1.0, 0.55 + 1.1 * math.sin(math.pi * min(1.0, u * 0.92)))
            h = r * f * (0.88 + 0.16 * math.sin(u * 7.0 + ph))
            hw = h * rng.uniform(0.95, 1.15)
        else:
            # the mat: flattens and WIDENS as it unrolls
            u = (t - split) / max(1e-6, 1.0 - split)
            h = max(0.025, r * (1.0 - u) ** 1.6)
            hw = r * (1.0 + 1.5 * u)
        stations.append((px, py, h, h, hw, hw, 0.0))
    return {"kind": "sod", "cls": "sod", "z": base_z, "base": base_z,
            "x": x, "y": y, "stations": stations}


def scatter_arcs(cfg, region, coverage_at, rng, *, ground_z=0.0,
                 pavement_at=None, skip=None, knobs=None):
    """The CYCLOIDAL MARKS — the grooves a tornado's suction vortices cut.

    Generated from the mechanism, in the track's own frame, and then placed
    through `tornado.from_track`. See the module docstring for the trochoid and
    for why the same three lines of algebra also decide which flank the marks
    land on and which side of each mark the spoil banks to.

    A mark is emitted only where ALL of these hold, and every one of them is
    the reason a real track does not have marks everywhere:

      * the vortex is moving fast over the ground (`arc_speed_min`) — the mark
        is cut on the flank where rotation and translation ADD, so the arcs sit
        right of the centreline and read as a rotating thing rather than as
        hatching;
      * the coverage is at `arc_min_coverage` — cycloidal marks are an EF3+
        feature and on a `peak = 0.92` track that holds them to the core;
      * the ground is not pavement, is inside the plate, and nothing else owns
        it. A run BREAKS at a road rather than skipping it, which is right:
        the mark stops at the kerb and picks up on the far verge, and the gap
        is one of the more convincing things in the pass.

    Runs shorter than `arc_min_len_m` are dropped — a two-metre stub of levee
    is a mound with an odd shape, and there is already a mound pass.
    """
    from . import tornado as tn

    kn = dict(DEFAULT_KNOBS)
    kn.update(knobs or {})
    hs = float(kn["height"])
    half = max(1e-6, 0.5 * float(cfg.get("width_m", 150.0)))
    to_world = tn.from_track(cfg)
    to_track, _u, _v = tn.frame(cfg)
    x0, y0, x1, y1 = (float(q) for q in region)

    # The along-track window the plate actually occupies. Its four corners
    # bound it, plus a pad for the orbit radius so a vortex that is off the
    # plate at the top of its loop still draws the part of the mark that is on.
    aa = [to_track(px, py)[0] for px in (x0, x1) for py in (y0, y1)]
    a_lo, a_hi = min(aa) - half, max(aa) + half

    def _on_plate(px, py):
        return x0 <= px <= x1 and y0 <= py <= y1

    def _ok(px, py):
        """The floor a mark's loose material has to clear: on the plate, in the
        band, and nothing else owns the ground."""
        return (_on_plate(px, py)
                and float(coverage_at(px, py)) >= float(kn["min_coverage"])
                and not (skip is not None and skip(px, py)))

    out = []
    _n_runs = 0
    for _v_i in range(max(0, int(kn["arc_vortices"]))):
        R = half * rng.uniform(*kn["arc_radius_frac"])
        k = rng.uniform(*kn["arc_k"])
        phase = rng.uniform(0.0, 6.2832)
        # Height and section are drawn ONCE PER VORTEX, not per station: one
        # vortex cuts one groove and its depth varies smoothly along it. Drawn
        # per station the mark comes out as a row of independent lumps.
        h0 = rng.uniform(0.08, 0.22) * hs
        wl0, wr0 = rng.uniform(0.45, 1.05), rng.uniform(0.45, 1.05)
        g0 = rng.uniform(0.55, 1.50)
        ph = rng.uniform(0.0, 6.2832)
        # AND THE MARK FADES IN AND OUT. `g_norm` alone breaks a trochoid
        # between loops and leaves each loop a single unbroken 100 m stroke,
        # which reads as something that was ploughed. A real mark is
        # intermittent along its own length — the vortex lifts, touches, lifts
        # — so a slow wobble is subtracted from the speed factor before the
        # threshold, which breaks a long run into a family of segments without
        # moving any of them.
        ph2 = rng.uniform(0.0, 6.2832)
        v_lo, v_hi = abs(1.0 - k), 1.0 + k

        run = []
        s = a_lo
        guard = 0
        while s <= a_hi and guard < 20000:
            guard += 1
            th = phase + k * s / R
            sin_t, cos_t = math.sin(th), math.cos(th)
            speed = math.sqrt(max(1e-9, 1.0 - 2.0 * k * sin_t + k * k))
            s += float(kn["arc_step_m"]) / max(0.25, speed)
            g_norm = ((speed - v_lo) / max(1e-6, v_hi - v_lo)
                      + 0.11 * math.sin(s * 0.14 + ph2)
                      + 0.06 * math.sin(s * 0.41 + ph2 * 1.7))
            a = s + R * cos_t
            c = R * sin_t
            px, py = to_world(a, c)
            keep = (g_norm >= float(kn["arc_speed_min"])
                    and _on_plate(px, py)
                    and float(coverage_at(px, py)) >= float(kn["arc_min_coverage"])
                    and not (pavement_at is not None and pavement_at(px, py))
                    and not (skip is not None and skip(px, py)))
            if not keep:
                if run:
                    _n_runs += _emit_arc(out, run, ground_z, rng,
                                        min_len_m=float(kn["arc_min_len_m"]),
                                        pavement_at=pavement_at, ok=_ok)
                    run = []
                continue
            # BANKED TO THE OUTSIDE OF THE TURN. `bias` is the outward radial
            # of the orbit projected onto the mark's left normal, which works
            # out to `(sin(theta) - k) / speed` — negative for every looping
            # mark, so the spoil is consistently on one side and the two levees
            # are never the same size. Symmetric levees read as a moulding;
            # this is the asymmetry a river bend has, from the same cause.
            bias = (sin_t - k) / speed
            grow = 0.55 + 0.65 * g_norm
            h = h0 * grow * (0.82 + 0.26 * math.sin(s * 0.35 + ph))
            run.append((px, py,
                        max(0.02, h * (1.0 + 0.45 * bias)),
                        max(0.02, h * (1.0 - 0.45 * bias)),
                        wl0 * grow, wr0 * grow,
                        g0 * (0.85 + 0.30 * g_norm)))
        if run:
            _n_runs += _emit_arc(out, run, ground_z, rng,
                                min_len_m=float(kn["arc_min_len_m"]),
                                pavement_at=pavement_at, ok=_ok)

    return out


def _emit_arc(out, run, ground_z, rng, min_len_m=4.5, pavement_at=None,
              ok=None):
    """Turn one continuous run of stations into an `arc` plus its loose
    material. Returns 1 if it was kept, 0 if it was too short.

    THE CLODS ARE NOT OPTIONAL. `_c_clods` is in the earthquake pipeline
    because the round-1 berm was a clean swept surface and read as machined
    trim; a levee is the same shape with the same failure mode. Lumps ON the
    crest, at more than one size, are what stops an extrusion looking extruded.
    """
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(run, run[1:]))
    if length < min_len_m or len(run) < 3:
        return 0
    out.append({"kind": "arc", "cls": "soil", "z": ground_z,
                "base": ground_z, "stations": run})
    for _k in range(int(length * 0.55)):
        # DRAW THE INDEX, not the station: two stations of a slow-moving mark
        # can be equal to the last bit, and looking the tuple back up with
        # `.index` then puts every clod of that pair at the first one's normal.
        i = rng.randrange(len(run))
        px, py, hl, hr, wl, wr, g = run[i]
        # On or just outside whichever levee is the bigger one, most of the
        # time — that is where the material actually came to rest.
        side = 1.0 if (rng.random() < 0.72) == (hr >= hl) else -1.0
        d = (g + (wr if side > 0 else wl)) * rng.uniform(0.25, 1.25)
        j = i if i > 0 else 1
        tx, ty = run[j][0] - run[j - 1][0], run[j][1] - run[j - 1][1]
        tl = math.hypot(tx, ty) or 1.0
        nx_, ny_ = -ty / tl, tx / tl
        wx, wy = px + nx_ * d * side, py + ny_ * d * side
        # A run BREAKS at a kerb, so its last station is right against one and
        # a clod thrown a metre off it lands in the road — at lawn grade, which
        # is under the asphalt. `ok` is the same coverage floor the field pass
        # uses: a lump really could be thrown clear onto green grass, but ONE
        # rule for where this module authors is worth more than that detail —
        # it is four clods in two thousand, and it is the difference between an
        # invariant a test can hold and a footnote.
        if pavement_at is not None and pavement_at(wx, wy):
            continue
        if ok is not None and not ok(wx, wy):
            continue
        out.append(_clod(wx, wy, ground_z, rng.uniform(0.10, 0.34),
                         "soil" if rng.random() < 0.75 else "silt", rng))
    return 1


def clip_to_region(specs, region):
    """Drop features whose GEOMETRY leaves the plate. Returns `(kept, n_dropped)`.

    `suburb_scene.apply_ground` lays its base sheet over exactly `region` and
    nothing beyond it, so a mound whose centre is legally inside the plate but
    whose 4 m skirt is not hangs over the void — from the air, a lump of earth
    floating half a metre off the end of the world. Rejecting the CENTRE
    against an inset instead would need the inset to be the largest feature's
    reach, which pushes every big mound away from the edge and prints a clean
    border round the whole scene.

    Testing the geometry costs one `geometry()` call per spec, which is the
    same call `build` is about to make anyway and is measured in milliseconds
    for a plate.
    """
    x0, y0, x1, y1 = (float(q) for q in region)
    kept, dropped = [], 0
    for s in specs:
        pts, _f = geometry(s)
        if (min(q[0] for q in pts) < x0 or max(q[0] for q in pts) > x1
                or min(q[1] for q in pts) < y0 or max(q[1] for q in pts) > y1):
            dropped += 1
            continue
        kept.append(s)
    return kept, dropped


def scatter(cfg, region, coverage_at, rng, *, flow_deg=None, ground_z=0.0,
            pave_z=None, pavement_at=None, skip=None, knobs=None,
            to_track=None, clip=True):
    """Everything: the cycloidal marks, then the field.

    ARCS FIRST, deliberately. They are the only pass with a hard coverage
    floor and the only one whose placement is decided by a mechanism rather
    than by a density, so they get first refusal on the core; the field pass
    then fills round them. Reversing the order changes nothing about what is
    authored — nothing here reserves ground — but it does change which rng
    draws land where, and a stable order is what makes a seed mean something.
    """
    from . import tornado as tn

    kn = dict(DEFAULT_KNOBS)
    kn.update(knobs or {})
    if flow_deg is None:
        flow_deg = float(cfg.get("heading_deg", 0.0)) + float(
            cfg.get("curl_deg", 0.0))
    if to_track is None:
        to_track = tn.frame(cfg)[0]
    out = scatter_arcs(cfg, region, coverage_at, rng, ground_z=ground_z,
                       pavement_at=pavement_at, skip=skip, knobs=kn)
    out += scatter_field(region, coverage_at, rng, flow_deg=flow_deg,
                         ground_z=ground_z, pave_z=pave_z,
                         pavement_at=pavement_at, skip=skip,
                         to_track=to_track, knobs=kn)
    if clip:
        out, n_out = clip_to_region(out, region)
        if n_out:
            print("[scour_relief] {0} feature(s) overhung the plate edge and "
                  "were dropped".format(n_out))
    return out


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
#
# `geometry(spec) -> (points, faces)` in METRES, world frame, with no `pxr`
# anywhere: `build` is the only thing here that touches USD and all it does is
# copy these buffers onto a stage. That split is what lets the geometry tests
# assert on winding, seating and extent without a container.
#
# WINDING AND NORMALS MUST AGREE. USD's default `orientation` is `rightHanded`,
# so the renderer culls on the winding and shades on the normal; a mesh whose
# two disagree is lit from inside and vanishes from half the angles you look at
# it from. `planks._FACES` records the same trap for the box. Every face below
# was checked by cross product against a worked example rather than by eye —
# the strip winding in particular is the one that is easy to get backwards,
# because the obvious order (across, then along) is the wrong one.

# Segments round a mound, and its ring profile as (radius, height) fractions.
# Three rings rather than one because a single skirt from apex to ground is a
# CONE, and the difference between a cone and a heap of earth is entirely in
# the shoulder.
_DOME_N = 12
_DOME_RINGS = ((0.40, 0.74), (0.74, 0.38), (1.00, 0.0))
# Every feature's outer rim is set this far BELOW its base plane. A rim landing
# exactly on the ground leaves a hairline of background showing under the
# silhouette from a low angle, and on a translucent overlay it also
# co-planar-fights with the band beneath it.
_SINK_M = 0.02
# Points across the arch of a rolled sod mat.
_ROLL_N = 7


def _wobble_at(wob, ang):
    """The three-harmonic plan multiplier of `_wob` at one angle."""
    a1, p1, a2, p2, a3, p3 = wob
    return (1.0 + a1 * math.sin(2.0 * ang + p1)
            + a2 * math.sin(5.0 * ang + p2)
            + a3 * math.sin(3.0 * ang + p3))


def _dome(spec):
    """A mound or a road fan: apex, three rings, a lumpy plan and a wandering
    crest. `fan` is the same construction with a height a tenth of the radius —
    it is not a separate shape, it is a squashed one, and keeping it that way
    means a knob can walk continuously between them."""
    x, y, z = float(spec["x"]), float(spec["y"]), float(spec["z"])
    rx, ry, h = float(spec["rx"]), float(spec["ry"]), float(spec["h"])
    wob = spec["wob"]
    ca, sa = (math.cos(math.radians(spec["yaw"])),
              math.sin(math.radians(spec["yaw"])))

    def _place(lx, ly, lz):
        return (x + ca * lx - sa * ly, y + sa * lx + ca * ly, lz)

    # THE APEX IS OFF CENTRE. A heap of spoil has its high point downwind of
    # its middle; a centred apex is a bell and reads as one.
    #
    # MEASURED AGAINST THE FIRST RING, not as a fixed fraction of `rx`. The
    # first cut offset it by a flat 0.16 of the plan radii, which is over half
    # the first ring on a mound whose wobble happens to pull that ring in — and
    # an apex that lands OUTSIDE the ring reverses the plan winding of the two
    # triangles either side of it, so a lit mound comes back with one dark
    # facet. 0.35 of the ring's own radius in the chosen direction is inside it
    # for every wobble, chord shortening included (cos(pi/12) = 0.97).
    _a1, p1, _a2, p2, _a3, _p3 = wob
    off = 0.35 * _DOME_RINGS[0][0] * _wobble_at(wob, p1)
    pts = [_place(off * rx * math.cos(p1), off * ry * math.sin(p1), z + h)]
    for (rr, hh) in _DOME_RINGS:
        for i in range(_DOME_N):
            ang = 6.2831853 * i / _DOME_N
            w = _wobble_at(wob, ang)
            if rr >= 1.0:
                lz = z - _SINK_M
            else:
                lz = z + max(0.0, h * hh * (1.0 + 0.22 * math.sin(
                    3.0 * ang + p2)))
            pts.append(_place(rx * rr * w * math.cos(ang),
                              ry * rr * w * math.sin(ang), lz))
    faces = [(0, 1 + i, 1 + (i + 1) % _DOME_N) for i in range(_DOME_N)]
    for r in range(len(_DOME_RINGS) - 1):
        a0, b0 = 1 + r * _DOME_N, 1 + (r + 1) * _DOME_N
        for i in range(_DOME_N):
            j = (i + 1) % _DOME_N
            faces.append((a0 + i, b0 + i, b0 + j, a0 + j))
    return pts, faces


def _section(kind, hl, hr, wl, wr, g):
    """The cross-section of an extruded feature as `(lateral, height)` pairs,
    ordered RIGHT to LEFT (increasing lateral offset, `lateral` positive being
    left of the direction of travel).

    Three shapes, one extruder:

      * `ridge` — toe, single crest, toe. A windrow.
      * `arc`   — toe, crest, groove lip, groove lip, crest, toe. The spoil of
                  a trench we cannot cut, with the floor drawn between the two
                  levees AT GRADE so they read as one groove instead of as two
                  independent banks. See the module docstring.
      * `sod`   — a half arch. A rolled mat of turf.
    """
    if kind == "arc":
        return ((-(g + wr), -_SINK_M),
                (-(g + 0.42 * wr), hr),
                (-g, 0.012),
                (g, 0.012),
                (g + 0.42 * wl, hl),
                (g + wl, -_SINK_M))
    if kind == "sod":
        # phi 0 -> pi walks the arch from the RIGHT toe to the LEFT one, so
        # `lateral` increases with j like the other two sections do. Getting
        # that backwards flips the winding for this kind alone, which shows as
        # sod rolls that are invisible from above and lit from inside below.
        out = []
        for j in range(_ROLL_N):
            phi = math.pi * j / float(_ROLL_N - 1)
            out.append((-wl * math.cos(phi), hl * math.sin(phi) - _SINK_M))
        return tuple(out)
    return ((-wr, -_SINK_M), (0.0, hl), (wl, -_SINK_M))


def _extrude(spec):
    """A `ridge`, `arc` or `sod` swept along its own polyline.

    The lateral axis at each station is the LEFT NORMAL of the local tangent,
    taken from the neighbouring stations rather than from a stored heading, so
    a curving mark's section stays perpendicular to it round the bend instead
    of shearing.
    """
    kind = spec["kind"]
    st = spec["stations"]
    z0 = float(spec["z"])
    n = len(st)
    pts, faces = [], []
    for k in range(n):
        px, py = st[k][0], st[k][1]
        ax, ay = st[max(0, k - 1)][0], st[max(0, k - 1)][1]
        bx, by = st[min(n - 1, k + 1)][0], st[min(n - 1, k + 1)][1]
        tx, ty = bx - ax, by - ay
        tl = math.hypot(tx, ty) or 1.0
        mx, my = -ty / tl, tx / tl            # left normal
        base = len(pts)
        for (d, dz) in _section(kind, st[k][2], st[k][3], st[k][4], st[k][5],
                                st[k][6]):
            pts.append((px + mx * d, py + my * d, z0 + dz))
        m = len(pts) - base
        if k:
            prev = base - m               # every section of a kind is one size
            for j in range(m - 1):
                faces.append((prev + j, base + j, base + j + 1, prev + j + 1))
    return pts, faces


def _clod_mesh(spec):
    """One lump of earth. Straight through `planks._box`.

    REUSED, NOT REIMPLEMENTED. That routine's winding and its six face normals
    were verified against each other face by face (see `planks._FACES`), and
    the failure mode of a second copy is a box lit from the inside — which is
    exactly the bug that is easy to author and hard to see, because it only
    shows from some angles.
    """
    from . import planks

    pts, _nrm = planks._box(spec)
    return pts, list(planks._FACES)


def geometry(spec):
    """`(points, faces)` in metres for any spec. Points are 3-tuples, faces
    are tuples of indices into them (3 or 4 long)."""
    kind = spec["kind"]
    if kind in ("mound", "fan"):
        return _dome(spec)
    if kind == "clod":
        return _clod_mesh(spec)
    return _extrude(spec)


def _normal(pts, face):
    """Newell's method: the face normal of an arbitrary planar-ish polygon.

    Newell rather than one cross product because a wobbled quad on the flank of
    a mound is NOT planar, and a cross product taken at one corner of a warped
    quad points somewhere the face as a whole does not.
    """
    nx = ny = nz = 0.0
    m = len(face)
    for i in range(m):
        ax, ay, az = pts[face[i]]
        bx, by, bz = pts[face[(i + 1) % m]]
        nx += (ay - by) * (az + bz)
        ny += (az - bz) * (ax + bx)
        nz += (ax - bx) * (ay + by)
    d = math.sqrt(nx * nx + ny * ny + nz * nz)
    if d < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / d, ny / d, nz / d)


# Fallback vertex colour per class, for the moment before a material resolves
# and for any viewport that will not load MDL. Dark: this is wet subsoil, and
# the failure that matters here is the one `planks` records in reverse — a
# tornado's TIMBER photographs pale and its GROUND photographs dark, and a
# scour that comes out light reads as sand.
_DISPLAY = {"soil": (0.20, 0.16, 0.12),
            "silt": (0.30, 0.26, 0.21),
            "sod": (0.13, 0.11, 0.085)}


def build(stage, root, specs, mats, ssf, verbose=True):
    """Author the relief. ONE MESH PER MATERIAL CLASS. Returns the prim paths.

    Same argument `planks.build` makes and the same shape of answer: this is
    static, shares three materials and never moves, so a plate's worth of it is
    three prims. Grouping by class rather than by feature kind because the
    material is per class and a mesh carries exactly one binding without
    GeomSubsets — and a subset per kind inside one mesh buys nothing over three
    meshes.

    Normals are authored `faceVarying` and per FACE. Left to the renderer they
    are averaged at shared vertices, which rounds every corner: the clods come
    out as pillows and, worse, the levee crests come out smooth, which is
    precisely the machined look the whole pass exists to avoid.

    `doubleSided`, because none of these is a closed solid — a dome is a cap
    with no underside and an extrusion is an open ribbon. Seen from below or
    edge-on at grade, a single-sided one is a hole.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if not specs:
        return []
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    by_cls = {}
    for s in specs:
        by_cls.setdefault(s.get("cls", "soil"), []).append(s)

    made = []
    for cls, group in sorted(by_cls.items()):
        pts, counts, idx, nrm = [], [], [], []
        for s in group:
            p, faces = geometry(s)
            base = len(pts)
            pts.extend(Gf.Vec3f(float(q[0]) * ssf, float(q[1]) * ssf,
                                float(q[2]) * ssf) for q in p)
            for f in faces:
                n = _normal(p, f)
                counts.append(len(f))
                idx.extend(base + v for v in f)
                nrm.extend([Gf.Vec3f(float(n[0]), float(n[1]),
                                     float(n[2]))] * len(f))
        path = "{0}/{1}".format(root, cls)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        m.CreateDoubleSidedAttr(True)
        m.CreateDisplayColorAttr([Gf.Vec3f(*_DISPLAY.get(cls, (0.2, 0.16, 0.12)))])
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        zs = [q[2] for q in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])
        mat = (mats or {}).get(cls)
        if mat is not None:
            # APPLY the schema before binding, or core USD warns "Found
            # material bindings on prim ... but MaterialBindingAPI is not
            # applied" on every read of the file — noise in exactly the place
            # an offline validation pass is trying to find real problems.
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        made.append(path)
        if verbose:
            print("[scour_relief] {0:<5s} {1:6d} feature(s) -> 1 mesh, "
                  "{2} point(s), {3} face(s)".format(
                      cls, len(group), len(pts), len(counts)))
    return made


# ---------------------------------------------------------------------------
# host-side reporting
# ---------------------------------------------------------------------------

def summarise(specs):
    """Everything worth knowing about a scatter, with no stage.

    The tornado analogue of `tornado.summarise`, and used the same way: the
    numbers that decide whether a pass is worth a container launch are the
    counts, the height range and the geometry budget, and all three are cheap.
    A run with no arcs, or one whose mound count is in the tens of thousands,
    is visible here in a second rather than in twenty minutes.
    """
    kinds, cls, n_pts, n_faces = {}, {}, 0, 0
    hi = 0.0
    xs = []
    ys = []
    for s in specs:
        kinds[s["kind"]] = kinds.get(s["kind"], 0) + 1
        cls[s.get("cls", "soil")] = cls.get(s.get("cls", "soil"), 0) + 1
        p, f = geometry(s)
        n_pts += len(p)
        n_faces += len(f)
        # THE FEATURE'S OWN VERTICAL EXTENT, not its height above `z`: on a
        # clod `z` is the box CENTRE (it is seated by `planks._lay`'s maths)
        # and on a dome it is the base plane, so subtracting `z` would report
        # two different quantities under one column heading.
        zs = [q[2] for q in p]
        hi = max(hi, max(zs) - min(zs))
        xs.append(min(q[0] for q in p))
        xs.append(max(q[0] for q in p))
        ys.append(min(q[1] for q in p))
        ys.append(max(q[1] for q in p))
    return {
        "features": len(specs),
        "by_kind": kinds,
        "by_class": cls,
        "points": n_pts,
        "faces": n_faces,
        "max_height_m": round(hi, 3),
        "extent": (round(min(xs), 1), round(min(ys), 1),
                   round(max(xs), 1), round(max(ys), 1)) if xs else None,
    }
