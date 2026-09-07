"""What a damaged building sheds: the right material, in the right amount.

Three properties, one per thing that was wrong with the pass this replaced —
see `disaster/debris.py`. They run on a fake pool and a fake resolver, because
none of the three is a fact about any particular asset set.
"""

import os
import random
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import debris                                     # noqa: E402


class _Resolver:
    """Every asset is a metre cube unless its name says otherwise."""

    def __init__(self, sizes=None):
        self.sizes = sizes or {}

    def get(self, usd, cat, scale=1.0, axis_up="Z"):
        s = self.sizes.get(usd, 1.0)
        return {"sx": s, "sy": s, "sz": s, "base": 0.0,
                "cx": 0.0, "cy": 0.0, "cz": 0.0}


def _config(pieces, piles=(), **debris_rules):
    """A config carrying nothing but a debris pool and its rules."""
    rules = {"shed_m3_per_m": 0.4, "spread_m": 6.0, "pile_share": 0.5}
    rules.update(debris_rules)
    return {
        "asset_scale": 1.0,
        "disaster": {"debris": rules},
        "usds": {"debris": {
            "pieces": [{"usd": u, "material": m} for u, m in pieces],
            "piles": [{"usd": u, "material": m} for u, m in piles],
        }},
    }


def _plan(cfg, kind, rung="pancaked", resolver=None, seed=3, **kw):
    return debris.plan(cfg, resolver or _Resolver(), kind=kind, rung=rung,
                       centre=(0.0, 0.0), footprint_m=(20.0, 10.0),
                       height_m=12.0,
                       rules=cfg["disaster"]["debris"],
                       rng=random.Random(seed), **kw)


_MIXED = (("brick_a.usd", "brick"), ("brick_b.usd", "brick"),
          ("slab_a.usd", "concrete"), ("slab_b.usd", "concrete"),
          ("plank_a.usd", "wood"), ("plank_b.usd", "wood"))


# ---------------------------------------------------------------------------
# 1. the material
# ---------------------------------------------------------------------------

def test_every_structure_may_shed_every_asset():
    """MATERIAL MATCHING IS OFF, and this is what replaced it.

    `SHEDS` used to route a structure to the debris its own fabric would
    produce, so a concrete frame shed only slabs and a masonry wall never shed
    lumber. The packs cannot support that: `urban_nucleus` has six steel and
    three glass buildings and not one piece of steel or glazing debris, so the
    whole-pool fallback was already doing most of the work, and whether a
    building shed anything recognisable came down to how well the pack had
    been TAGGED rather than to the collapse. Every asset is a candidate now.
    """
    cfg = _config(_MIXED)
    seen = {k: {p["usd"] for p in _plan(cfg, k)}
            for k in ("masonry", "concrete", "timber", "steel")}
    for kind, got in seen.items():
        assert got, kind
    # No structure is confined to its own material any more.
    assert seen["concrete"] - {"slab_a.usd", "slab_b.usd"}
    assert seen["masonry"] & {"plank_a.usd", "plank_b.usd"}


def test_the_draw_is_still_weighted_not_flat():
    """`PRIMACY` still orders the groups, so one material dominates a given
    building's rubble — the pool is no longer FILTERED by the structure, but
    the draw is not uniform over it either, or every wreck would look the same.
    """
    import collections

    cfg = _config(_MIXED)
    got = collections.Counter(p["usd"][:5] for p in _plan(cfg, "masonry"))
    top = got.most_common(1)[0][1]
    assert top > 0.4 * sum(got.values()), got


def test_ground_spoil_is_shed_by_anything():
    """`earth` is not a construction: nothing shed it, the collapse scraped it
    up. It is also the only thing a timber suburb has to pile."""
    cfg = _config((("plank.usd", "wood"),), (("dirt.usd", "earth"),))
    piles = [p for p in _plan(cfg, "timber") if p["category"] == "debris_pile"]
    assert piles and all(p["usd"] == "dirt.usd" for p in piles)


def test_a_structure_with_no_debris_of_its_own_still_sheds_something():
    """`urban_nucleus` has six steel buildings and no steel debris. Without the
    fallback every factory shed in the scene stands on a spotless lot."""
    cfg = _config((("plank.usd", "wood"),))
    assert _plan(cfg, "steel")


# ---------------------------------------------------------------------------
# 2. the amount
# ---------------------------------------------------------------------------

def test_more_damage_sheds_more():
    """The rung is the dial now, so the ladder has to read as a ladder."""
    cfg = _config(_MIXED)
    counts = [len(_plan(cfg, "masonry", rung=r))
              for r in ("pristine", "cracked", "partial_collapse", "pancaked")]
    assert counts[0] == 0
    assert counts == sorted(counts)
    assert counts[-1] > counts[1]


def test_the_budget_is_a_volume_so_big_pieces_come_in_smaller_numbers():
    """The bug the count knobs could not express: `pieces_per_building: 20`
    places twenty pieces whether they are 0.3 m bricks or 3 m slabs."""
    cfg = _config((("chunk.usd", "concrete"),))
    small = _plan(cfg, "concrete", resolver=_Resolver({"chunk.usd": 0.5}))
    large = _plan(cfg, "concrete", resolver=_Resolver({"chunk.usd": 2.0}))
    assert len(small) > 4 * len(large)


def test_throwing_debris_further_does_not_make_more_of_it():
    """A windstorm carries the same material further, not more of it. Folding
    the reach into the budget made a tornado shed six times an earthquake's
    rubble for no reason except that it spread it wider."""
    near = _config(_MIXED, spread_m=4.0)
    far = _config(_MIXED, spread_m=18.0)
    n_near, n_far = len(_plan(near, "masonry")), len(_plan(far, "masonry"))
    assert abs(n_near - n_far) <= max(2, n_near // 10)

    reach = max(abs(p["x_m"]) for p in _plan(far, "masonry"))
    assert reach > max(abs(p["x_m"]) for p in _plan(near, "masonry"))


def test_fallen_is_the_pile_and_not_the_dial():
    """Slabs never fell — nothing threw them. Consumed fragments did: they are
    the material the collapse pulverised, which is exactly what ends up around
    the base as fines."""
    assert debris.fallen({"cells": 0, "loose": ["a"]}) == 0.0
    assert debris.fallen({"cells": 100, "loose": ["a"] * 40}) == 0.4
    assert debris.fallen({"cells": 100, "loose": ["a"] * 40,
                          "slabs": ["a"] * 10}) == 0.3
    assert debris.fallen({"cells": 100, "loose": ["a"] * 40,
                          "consumed": 20}) == 0.6
    assert debris.fallen({"cells": 10, "loose": ["a"] * 40}) == 1.0


# ---------------------------------------------------------------------------
# 3. where it lands
# ---------------------------------------------------------------------------

def test_debris_is_densest_where_the_building_stood():
    """A NORMAL DISTRIBUTION ABOUT THE CENTRE, not a ring around the footprint.

    The annulus this replaced put every piece beyond the footprint edge, which
    leaves a bare patch exactly where the building came down. So the field is
    now centred: the mean sits on the building, the density falls off with
    distance, and the reach is what `spread_m` and the footprint set.
    """
    import statistics

    cfg = _config(_MIXED, spread_m=6.0)
    pls = _plan(cfg, "masonry")
    assert len(pls) >= 30, "too few to say anything about a distribution"
    xs = [p["x_m"] for p in pls]
    ys = [p["y_m"] for p in pls]
    # Centred: the mean is near the building, not pushed out onto a ring.
    assert abs(statistics.fmean(xs)) < 4.0
    assert abs(statistics.fmean(ys)) < 3.0
    # Dense in the middle: a ring would put almost nothing over the footprint.
    inside = sum(1 for p in pls
                 if abs(p["x_m"]) <= 10.0 and abs(p["y_m"]) <= 5.0)
    assert inside >= 0.15 * len(pls), "nothing landed on the footprint"
    # And still bounded — two sigma is (half-extent + spread) per axis.
    assert statistics.pstdev(xs) < 10.0 + 6.0
    assert statistics.pstdev(ys) < 5.0 + 6.0


def test_a_veto_removes_debris_without_moving_the_rest():
    """A lot beside an exclusion zone must not spill everything it has just
    inside the zone's edge, so a vetoed piece is still charged to the budget."""
    cfg = _config(_MIXED)
    kept = _plan(cfg, "masonry", keep=lambda x, y: x < 0.0)
    assert kept and all(p["x_m"] < 0.0 for p in kept)
    assert len(kept) < len(_plan(cfg, "masonry"))


def test_the_reach_is_recorded_so_a_baked_wreck_can_carry_it():
    """Once the debris is inside an archetype USD it is no longer a placement,
    and `targets` samples casualties against the placement list."""
    cfg = _config(_MIXED)
    pls = _plan(cfg, "masonry")
    assert debris.radius_m(pls, (0.0, 0.0)) > 0.0
    assert debris.radius_m([], (0.0, 0.0)) == 0.0


# ---------------------------------------------------------------------------
# 4. the seam — the cutter authors what it shed
# ---------------------------------------------------------------------------

def _cube_usd(tmp_path, name="chunk.usda", size=1.0):
    """A real one-metre box on disk, so the reference resolves and the
    resolver measures something rather than falling back."""
    from pxr import Usd, UsdGeom

    path = str(tmp_path / name)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    c = UsdGeom.Cube.Define(st, "/chunk")
    c.CreateSizeAttr(size)
    st.GetRootLayer().Save()
    return path


def _box_building(stage, path, at=(0.0, 0.0), w=20.0, h=24.0):
    """A closed box with real MESH faces — `mesh_prims` sees nothing else, and
    a building the cutter cannot read sheds nothing, correctly."""
    from pxr import UsdGeom

    x0, y0 = at[0] - w / 2.0, at[1] - w / 2.0
    x1, y1 = at[0] + w / 2.0, at[1] + w / 2.0
    v = [(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0),
         (x0, y0, h), (x1, y0, h), (x1, y1, h), (x0, y1, h)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
    mesh.GetPointsAttr().Set(v)
    mesh.GetFaceVertexCountsAttr().Set([4] * len(f))
    mesh.GetFaceVertexIndicesAttr().Set([i for q in f for i in q])
    return stage.GetPrimAtPath(path)


def test_the_live_cutter_authors_the_debris_it_sheds(tmp_path):
    """The seam `disaster_stage` gave up: a building cut on the stage sheds its
    rubble THERE, from its own report, and hands it back as placements so the
    scene's settle and `targets` both see it."""
    import compile_disaster as cd
    from pxr import Usd, UsdGeom

    import disaster.mesh_damage as M

    chunk = _cube_usd(tmp_path)
    block = cd.DISASTERS["earthquake"](0.9, {}, (400.0, 400.0))
    block["type"] = "earthquake"
    cfg = {"seed": 42, "asset_scale": 1.0, "measure_usds": True,
           "disaster": block,
           "usds": {"debris": {"pieces": [{"usd": chunk, "material": "brick"}],
                               "piles": [{"usd": chunk, "material": "brick"}]}}}

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.Xform.Define(st, "/World")
    placements = []
    for i in range(2):
        p = f"/World/b{i}"
        _box_building(st, p, at=(i * 80.0, 0.0))
        placements.append({"prim_path": p, "category": "house", "usd": "",
                           "x_m": i * 80.0, "y_m": 0.0,
                           "_mesh_damage": 0.9, "_footprint_m": (20.0, 20.0)})

    out = M.apply_to_stage(st, cfg, placements,
                           {"region": (-200.0, -200.0, 200.0, 200.0)})

    shed = out["debris"]
    assert shed, "a cut building shed nothing"
    assert all(q["category"] in ("debris", "debris_pile") for q in shed)
    # Authored, and under the BUILDING's own scope — `apply_placements` names
    # prims by their index in the list it is handed, so two buildings sharing a
    # parent would silently overwrite each other's rubble.
    for q in shed:
        assert st.GetPrimAtPath(q["prim_path"]).IsValid()
    assert len({q["prim_path"].rsplit("/", 1)[0] for q in shed}) == 2
    # Pieces are dropped, so the scene's one settle pass has to be told.
    assert set(q["prim_path"] for q in shed if q.get("settle")) <= set(
        out["loose"])
    # And the building remembers how far its rubble reached.
    assert any(p.get("_debris_r_m") for p in placements)


def test_the_first_material_dominates_rather_than_merely_appearing():
    """`SHEDS` is a preference, not a menu. A flat draw over "brick, then
    concrete" gave the urban set's 52 masonry buildings 4,771 concrete pieces
    and 172 brick — a concrete city — because there are twice as many concrete
    assets in the pack."""
    cfg = _config(_MIXED)
    got = [p["usd"] for p in _plan(cfg, "masonry", seed=11)
           if p["category"] == "debris"]
    brick = sum(1 for u in got if u.startswith("brick"))
    assert brick > 0.6 * len(got), (brick, len(got))


def test_stage_a_bakes_the_debris_into_the_cell(tmp_path):
    """The other half of the seam: `Baker` authors the shed rubble INSIDE the
    cell, so `_unload` takes it away with everything else and `_export_cell`
    merges it into the archetype. Runs the two methods directly — a whole bake
    needs Isaac, these do not."""
    from pxr import Usd, UsdGeom

    from archetypes.bake import Baker

    chunk = _cube_usd(tmp_path, "pile.usda")
    cfg = {"seed": 7, "asset_scale": 1.0, "measure_usds": True,
           "disaster": {"type": "earthquake",
                        "debris": {"shed_m3_per_m": 0.5, "spread_m": 6.0,
                                   "pile_share": 0.5}},
           "usds": {"debris": {"pieces": [{"usd": chunk, "material": "brick"}],
                               "piles": [{"usd": chunk, "material": "brick"}]}}}

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    cell = "/World/a_lib_thing_pancaked"
    UsdGeom.Scope.Define(st, cell)
    _box_building(st, cell + "/asset", at=(40.0, 0.0))

    baker = Baker(st, cfg, "earthquake", str(tmp_path), seed=7,
                  parent="/World")
    (sx, sy), sz = baker._envelope(cell + "/asset")
    assert (round(sx), round(sy), round(sz)) == (20, 20, 24)

    item = type("It", (), {"type": "thing", "kind": "structure"})()
    out = baker._shed(item, "pancaked", ((sx, sy), sz), 40.0, 0.0, cell,
                      "masonry", {"cells": 100, "loose": ["x"] * 90})
    assert out["placements"]
    assert all(p["prim_path"].startswith(cell + "/debris/")
               for p in out["placements"])
    # PIECES FALL, MOUNDS DO NOT. A piece is placed by a Gaussian that knows
    # nothing about what is under it, so only the settle can seat it; a mound
    # is authored on the ground already and is a collider during that settle.
    assert out["statics"] and out["paths"]
    # AND THE REACH COMES BACK FROM `_shed`, for the caller to record.
    # `_shed` used to write `cell_record["debris_r_m"]` itself, which stopped
    # working when the shed moved into `_settle_batch`: by then the record has
    # already been snapshotted into the pending entry and is about to be
    # overwritten by that copy. `_flush` lands it on `pd["record"]` instead.
    assert out["radius_m"] > 0.0


# ---------------------------------------------------------------------------
# the amount is what was pulverised
# ---------------------------------------------------------------------------


def test_the_amount_is_size_times_rung():
    """SIZE x RUNG, and nothing from the cut.

    The budget was tied to the cut for a while — consumed fragment counts,
    then measured fragment volumes — and both made the rubble a function of
    how finely the cutter happened to work. Making steel's grain isotropic
    took `BG_Building_C` from 771 cells to 3687 without changing the building
    at all, and a count-based budget would have quintupled its debris for it.
    The shell area and a hand-set per-rung multiplier answer the question the
    scene is actually asking.
    """
    rules = {"pile_share": 0.55}
    # Monotonic in the rung.
    got = [sum(debris.budget_m3((17.0, 23.0), 19.0, r, rules))
           for r in ("cracked", "soft_storey", "partial_collapse", "pancaked")]
    assert got == sorted(got) and got[0] > 0.0, got
    # And in the building: a tower sheds more than a block because it is
    # bigger, which is the whole point of the size term.
    block = sum(debris.budget_m3((17.0, 23.0), 19.0, "pancaked", rules))
    tower = sum(debris.budget_m3((50.0, 40.0), 66.0, "pancaked", rules))
    assert tower > 5.0 * block, (block, tower)


def test_a_pristine_building_sheds_nothing():
    rules = {"pile_share": 0.55}
    assert sum(debris.budget_m3((17.0, 23.0), 19.0, "pristine", rules)) == 0.0


def test_an_unknown_rung_sheds_like_a_collapse():
    """Better to over-shed than to silently leave a wreck on a clean lot."""
    assert debris.rung_share("no_such_rung") == 1.0


def test_scatter_lines_the_footprint():
    """Loose pieces come off the SKIN and land against the wall they fell from.

    A centred Gaussian put as much scatter in the middle of the plan as around
    its edge, which reads as a building that exploded rather than one that fell
    down. The radius is the distance to the outline at that angle plus a band
    across it, so a piece lands just inside or just outside the wall line.
    """
    import math
    import statistics

    sizes = {"tiny.usd": 0.4, "mid.usd": 1.0, "huge.usd": 2.4,
             "mound.usd": 3.0}
    cfg = _config([(u, "brick") for u in sizes if u != "mound.usd"],
                  piles=[("mound.usd", "concrete")],
                  spread_m=6.0, m3_per_m2=1.0, max_per_building=3000)
    pls = _plan(cfg, "masonry", resolver=_Resolver(sizes))
    hw, hh = 10.0, 5.0                     # the _plan footprint, halved

    def outline(x, y):
        a = math.atan2(y, x)
        ca, sa = math.cos(a), math.sin(a)
        return min(hw / max(abs(ca), 1e-6), hh / max(abs(sa), 1e-6))

    off = [math.hypot(p["x_m"], p["y_m"]) - outline(p["x_m"], p["y_m"])
           for p in pls if p["category"] == "debris"]
    assert len(off) >= 40
    # Centred ON the line, not inside it and not beyond it.
    assert abs(statistics.fmean(off)) < 2.0, statistics.fmean(off)
    # And genuinely hugging it: most of the scatter is within a few metres.
    near = sum(1 for d in off if abs(d) < 3.0) / len(off)
    assert near > 0.5, near
    # It straddles the wall — some pieces inside the outline, some outside.
    assert any(d < 0 for d in off) and any(d > 0 for d in off)
