"""ground_class — what is actually under a piece of broken ground.

ROUND 5, WORK PACKAGE E. The user, on the second earthquake scene: "the
ground 'dirt' coming out looks weird now ... For like bent/broken
sidewalk/asphalt use the material of what's near where the broken ground is
placed, not the grassy sidewalk one." `quake_flow`'s ground-response family
(`_c_heave`, `_c_kerb`, `_c_pave_break`, `_c_lip_slabs`, `_c_subsidence`,
`_c_soil_patch`, `_ejecta`) already authors real 3-D earth and broken
pavement round a building that leaned or sank — see that module's own
`_C_TEX` table and `_c_look` — but every one of those pieces picks its
material from a COIN (`"pave" if rng.random() < 0.5 else "asph"`) or a fixed
default, with no idea whether the ground under it is actually a carriageway,
a sidewalk, a paved block interior or a lawn. A kerb segment authored two
metres from a park block gets the same 50/50 draw as one authored in the
middle of a six-lane arterial.

THIS MODULE IS THE SAMPLER, NOT THE FIX. It answers one question —
`at(x, y)` — from the SAME layout record `scene_generator.build_city`
already returns (`city_layout`, stashed onto `config["_city_layout"]` by
`generate_scene_on_stage`). `quake_flow` and `quake.py` are what actually USE
the answer (`ctx["ground_at"]`, wired through `_c_ground_response`) — see the
docstring on `quake_flow._c_ground_look` for how a piece's LOOK is chosen
once its class is known, and `_C_TEX` / `look_for` below for the mapping.

WHY A RASTER, LIKE `scour_relief.pavement_mask` AND NOT A PER-QUERY SCAN
-------------------------------------------------------------------------
`city_layout["road_corridors"]`, `["paved_blocks"]` and `["sidewalk_rects"]`
are all axis-aligned rectangles — a BSP block grid never needs anything
richer — so a point-in-rect test is cheap even done a few hundred times per
building. But `scour_relief.pavement_mask` already established the discipline
for exactly this kind of "where is the pavement" query (see its own
docstring), and `at()` here is called from inside dense per-piece scatter
loops (a clod pass alone samples its lattice thousands of times over a city),
so paying the rasterisation cost ONCE at construction and turning every
subsequent `at()` into an array lookup is worth it for the same reason it was
there: the field scatter draws far more candidates than there are rects.

PAINTED IN PRIORITY ORDER: road, then sidewalk, then paved block, and
everything left over is grass. A block's own rect and the road that cut it
out of the plate NECESSARILY overlap at their shared edge (the corridor is
what got REMOVED from the raw BSP leaf to make the block, not a hole punched
out of it afterwards — see `scene_generator._subdivide_region_metric`), so
filling classes in this order means the raster never depends on which order
the source lists happen to be iterated in: paint the permissive class first
and let the more specific ones overwrite it.

EXACT WHERE THE GENERATOR ALREADY SAYS SO, DERIVED WHERE IT DOES NOT
-------------------------------------------------------------------------
`build_city` stores exactly which block interiors it paved
(`city_layout["paved_blocks"]`, a list of the same `inset` rects it built the
downtown paving on) and exactly where it laid the frontage sidewalk strip
(`city_layout["sidewalk_rects"]`). Both are the GROUND TRUTH of an RNG-driven
decision (`park_set`, `pave_blocks`) this module has no way to re-roll and
match — so when they are present (every layout `build_city` returns carries
them, even as empty lists), this module uses them verbatim rather than
re-deriving anything. Only for a layout that does not carry them (a synthetic
one in a test, or a future caller with a different generator) does `at()`
fall back to reconstructing a sidewalk ring from `sidewalk_width_m` and
treating every block interior as grass — the same "block interiors are grass
unless something says otherwise" default `build_city` itself uses for a
suburb (`pave_blocks=False`) or a park block.
"""

import math

# `look_for`'s answer, in `quake_flow._C_TEX` terms. Sidewalk and a paved
# block interior share ONE look (`quake_flow._C_TEX["pave"]`, Damaged_Asphalt
# tinted light rather than the mossy `Worn_Pavement` the recipes' raw
# `mats["concrete"]` resolves to) because nothing here needs to tell a
# concrete sidewalk slab from a paved plaza apart at the resolution a broken
# kerb or a heave-riding pavement plate is authored at; the carriageway alone
# gets the darker, more worn `"asph"`.
_LOOK = {"road": "asph", "sidewalk": "pave", "paved": "pave", "grass": "soil"}

# Used only when a layout carries no `sidewalk_rects` of its own (see the
# module docstring). 2.0 m sits between `scene_generator`'s measured sidewalk
# footprints across the shipped asset sets (1.2-2.6 m); "some plausible
# sidewalk exists here" beats "no sidewalk class exists at all" for a caller
# that never measured one.
DEFAULT_SIDEWALK_WIDTH_M = 2.0

_CODE_NAME = ("grass", "paved", "sidewalk", "road")   # fill priority, low..high


def look_for(ground_cls):
    """The `quake_flow._C_TEX` key for one of `at()`'s answers. Free function
    (not just a method) because `quake_flow` never needs to hold a
    `GroundClass` instance to use it — a `ctx["ground_at"]` callable's return
    value goes straight through this."""
    return _LOOK.get(ground_cls, "soil")


def _rect(item):
    """One axis-aligned `(x0, y0, x1, y1)` from a `city_layout` record —
    either an `{"x0","y0","x1","y1", ...}` dict (`road_corridors`, as
    `scene_generator._subdivide_region_metric` emits them) or a plain
    4-tuple/list (`paved_blocks`, `sidewalk_rects`, `blocks`, all authored as
    bare tuples). Returns `None` for anything that is neither — a layout
    handed in from an untrusted or partial source should degrade to "no rect
    here", not raise mid-scatter."""
    try:
        if isinstance(item, dict):
            x0, y0, x1, y1 = (float(item[k]) for k in ("x0", "y0", "x1", "y1"))
        else:
            x0, y0, x1, y1 = (float(q) for q in item[:4])
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return (x0, y0, x1, y1)


def _rects(items):
    out = []
    for it in items or ():
        r = _rect(it)
        if r is not None:
            out.append(r)
    return out


def _sidewalk_rings(blocks, sw_w):
    """A ring `sw_w` metres deep just inside each block's own edge, as up to
    four axis-aligned strips per block — the same shape
    `scene_generator.build_city`'s own frontage-ring construction produces
    (`_inset_of` / the per-block `sidewalk_rects.extend([...])` in the main
    loop), minus its `verge_m` offset: this is a FALLBACK for a layout that
    did not already hand over the exact strips, so reproducing the ring's
    shape (four strips, sidewalk-width deep, running the block's own edges)
    matters far more than reproducing a few centimetres of verge that only a
    handful of preset configs set to anything but zero.

    A block too small to hold both rings and an interior is treated as ALL
    sidewalk — the same thing happens to a real corner lot too narrow for a
    planting strip."""
    out = []
    if sw_w <= 1e-6:
        return out
    for blk in blocks or ():
        r = _rect(blk)
        if r is None:
            continue
        x0, y0, x1, y1 = r
        if (x1 - x0) <= 2.0 * sw_w or (y1 - y0) <= 2.0 * sw_w:
            out.append(r)
            continue
        out.append((x0, y0, x1, y0 + sw_w))
        out.append((x0, y1 - sw_w, x1, y1))
        out.append((x0, y0 + sw_w, x0 + sw_w, y1 - sw_w))
        out.append((x1 - sw_w, y0 + sw_w, x1, y1 - sw_w))
    return out


class GroundClass:
    """`at(x, y) -> "road" | "sidewalk" | "paved" | "grass"` for one
    `city_layout` (`scene_generator.build_city`'s second return value, or
    `config["_city_layout"]`).

    Rasterised once at construction, cell `cell_m` (default 1 m — fine enough
    that a kerb segment or a sand-boil fan, both metre-scale, never straddles
    a class it should not).
    """

    def __init__(self, layout, roads_cfg=None, sidewalk_width_m=None,
                 cell_m=1.0):
        layout = layout or {}
        region = layout.get("region")
        self._cw = max(0.25, float(cell_m))
        if not region:
            # No layout at all (no city, or a caller that never ran
            # `build_city`) — `at()` answers "grass" everywhere rather than
            # raising, so a caller can wire `ground_at` unconditionally and
            # only see a behaviour change where there IS a city.
            self._nx = self._ny = 0
            self._x0 = self._y0 = 0.0
            self._grid = bytearray()
            return
        self._x0, self._y0, x1, y1 = (float(q) for q in region)
        self._nx = max(1, int(math.ceil((x1 - self._x0) / self._cw)))
        self._ny = max(1, int(math.ceil((y1 - self._y0) / self._cw)))
        grid = bytearray(self._nx * self._ny)   # 0 = grass, everywhere by default

        blocks = layout.get("blocks") or []
        roads = _rects(layout.get("road_corridors"))
        paved = _rects(layout.get("paved_blocks"))

        sw_rects = layout.get("sidewalk_rects")
        if sw_rects:
            sidewalks = _rects(sw_rects)
        else:
            sw_w = (float(sidewalk_width_m) if sidewalk_width_m is not None
                    else float((roads_cfg or {}).get(
                        "sidewalk_width_m", DEFAULT_SIDEWALK_WIDTH_M)))
            sidewalks = _sidewalk_rings(blocks, sw_w)

        # PRIORITY ORDER — see the module docstring: paved first (the most
        # permissive of the three non-default classes), sidewalk next, road
        # last, so each later fill overwrites any overlap with the ones
        # before it rather than depending on source-list order.
        self._fill(grid, paved, 1)
        self._fill(grid, sidewalks, 2)
        self._fill(grid, roads, 3)
        self._grid = grid

    def _fill(self, grid, rects, code):
        nx, ny = self._nx, self._ny
        cw = self._cw
        x0, y0 = self._x0, self._y0
        for (rx0, ry0, rx1, ry1) in rects:
            i0 = max(0, int(math.floor((rx0 - x0) / cw)))
            i1 = min(nx - 1, int(math.ceil((rx1 - x0) / cw)) - 1)
            j0 = max(0, int(math.floor((ry0 - y0) / cw)))
            j1 = min(ny - 1, int(math.ceil((ry1 - y0) / cw)) - 1)
            if i1 < i0 or j1 < j0:
                continue
            for j in range(j0, j1 + 1):
                base = j * nx
                for i in range(i0, i1 + 1):
                    grid[base + i] = code

    def at(self, x, y):
        """The ground class at one WORLD point, in the same metre frame the
        layout was built in (pre-`ssf` scale — callers at city-assembly time,
        where everything is already multiplied by `ssf`, must divide it back
        out before calling, exactly as `quake._c_mass` does for the building
        frame)."""
        if not self._grid:
            return "grass"
        i = int(math.floor((float(x) - self._x0) / self._cw))
        j = int(math.floor((float(y) - self._y0) / self._cw))
        if i < 0 or j < 0 or i >= self._nx or j >= self._ny:
            return "grass"
        return _CODE_NAME[self._grid[j * self._nx + i]]

    @staticmethod
    def look_for(ground_cls):
        return look_for(ground_cls)

    @classmethod
    def from_config(cls, config, roads_cfg=None, sidewalk_width_m=None,
                    cell_m=1.0):
        """A `GroundClass` for `config["_city_layout"]`, or `None` when the
        config carries no layout (no city was built, or the caller ran
        before `generate_scene_on_stage` stashed one — see that function's
        own note on why the key is set on the in-memory dict only, never
        written to a serialised config)."""
        config = config or {}
        layout = config.get("_city_layout")
        if not layout:
            return None
        rc = roads_cfg if roads_cfg is not None else config.get("roads", {})
        return cls(layout, roads_cfg=rc, sidewalk_width_m=sidewalk_width_m,
                   cell_m=cell_m)
