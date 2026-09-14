"""Offline checks for `layout/site_plan.py` — the traced-from-a-photo layout.

No `pxr`, no OpenCV, no image: everything here reads the committed spec, which
is exactly the contract the module is built around (`layout/site_plan.py`: the
YAML is the source of truth, not the aerial). The tracer that WROTE the spec is
not exercised — it is a one-shot authoring tool and needs cv2/skimage.

What is actually being defended:

* the network is PLANAR and CLOSED. The first working trace produced a graph
  whose streets never joined — every junction arrived as several nodes a metre
  or two apart — and the symptom was not an error, it was `faces()` returning
  one polygon the size of the whole region with all the streets dangling inside
  it. `test_blocks_partition_the_region` is that bug's regression: it fails
  loudly the moment the biggest block is the region.
* MEASURED WIDTHS SURVIVE. `_connect_route` takes a road class, not a width,
  and a class would round every 5.4 m service road up to a 10.7 m street.
* ROLES REACH THE BLOCKS THEY WERE WRITTEN FOR, which they only do if the block
  floor used here is the one the roles were classified under.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from layout import site_plan as spl                                   # noqa: E402
from layout import suburb_net as sn                                   # noqa: E402

PLAN = "disaster_city"


@pytest.fixture(scope="module")
def spec():
    return spl.load_spec(PLAN)


@pytest.fixture(scope="module")
def built(spec):
    return spl.generate(*spl.region_of(spec), None, {"_spec": spec})


# ---------------------------------------------------------------------------
# the spec itself
# ---------------------------------------------------------------------------

def test_spec_is_well_formed(spec):
    w, h = spl.region_of(spec)
    assert w > 0 and h > 0
    assert spec["roads"], "a site plan with no roads is not a layout"
    for i, r in enumerate(spec["roads"]):
        assert len(r["pts"]) >= 2, f"road {i} has fewer than two points"
        assert r["width_m"] > 0.0, f"road {i} has no width"
        assert r["class"] in sn.CLASSES and r["class"] != "boundary"


def test_every_road_point_is_inside_the_region(spec):
    """Metres, region-centred: the tracer's pixel flip is the only conversion,
    so a point outside the box means it got the origin or the sign wrong."""
    w, h = spl.region_of(spec)
    for r in spec["roads"]:
        for x, y in r["pts"]:
            assert -w / 2 - 1 <= x <= w / 2 + 1
            assert -h / 2 - 1 <= y <= h / 2 + 1


def test_class_follows_width():
    assert spl.class_for_width(14.0) == "arterial"
    assert spl.class_for_width(10.7) == "local"
    assert spl.class_for_width(3.0) == "cul_de_sac"
    # A width between two classes takes the nearer, never `boundary` — that one
    # is 0 m and would otherwise win every narrow road and make it undrawable.
    assert spl.class_for_width(0.4) != "boundary"


# ---------------------------------------------------------------------------
# the built network
# ---------------------------------------------------------------------------

def test_network_is_connected_and_planar(built):
    net, _blocks, _info = built
    roads = [e for e in net.edges.values() if e.road_class != "boundary"]
    assert len(roads) >= 50
    # Euler on the planar graph: every independent cycle is a face, and a
    # traced grid has many. One or two would mean the streets are a tree again.
    cycles = len(net.edges) - len(net.nodes) + 1
    assert cycles >= 10, f"only {cycles} cycles — the streets are not joining"
    # No orphans: every node carries an edge, and no edge is a zero-length stub.
    for n in net.nodes.values():
        assert n.edges
    for e in roads:
        assert e.length > 0.5


def test_blocks_partition_the_region(built):
    """Blocks tile the region, and none of them IS the region.

    The failure this catches is the one that took the longest to see: when the
    graph does not close, `faces()` still returns something — a single polygon
    with the area of the whole crop. It looks like a successful build.
    """
    _net, blocks, info = built
    x0, y0, x1, y1 = info["region"]
    area = (x1 - x0) * (y1 - y0)
    assert blocks, "no blocks recovered"
    total = sum(b["area"] for b in blocks)
    assert 0.3 * area < total < area, f"blocks cover {total / area:.0%} of the crop"
    assert max(b["area"] for b in blocks) < 0.6 * area


def test_measured_widths_survive_the_build(spec, built):
    """A 5.4 m service road stays 5.4 m and is not rounded to its class."""
    net, _b, _i = built
    widths = sorted({round(e.width_m, 1) for e in net.edges.values()
                     if e.road_class != "boundary"})
    assert len(widths) > 3, "every street took its class default"
    class_widths = {c["width_m"] for c in sn.CLASSES.values()}
    assert not class_widths.issuperset(widths)
    # NOT `issubset` of the spec's own widths any more: `merge_chains` joins
    # two halves of one street and gives the result their LENGTH-WEIGHTED
    # width, so 5.4 m and 5.8 m become 5.6 m, which appears in no spec entry.
    # What must still hold is that every width is a measured one — inside the
    # range the trace actually recorded, not a class constant.
    spec_widths = [float(r["width_m"]) for r in spec["roads"]]
    assert min(spec_widths) - 0.05 <= min(widths)
    assert max(widths) <= max(spec_widths) + 0.05


def test_every_block_has_a_role_from_the_spec(built):
    _net, blocks, info = built
    assert info["roles_tagged"] == len(blocks)
    assert all(b["role"] and b["role"] != "_untagged" for b in blocks)
    roles = {b["role"] for b in blocks}
    # Roles the classifier cannot find and a person set by hand. If they are
    # gone, the spec was re-traced and the hand edits were not re-applied.
    #
    # `collapsed` IS DELIBERATELY ABSENT. The one block that carried it — the
    # north-middle face at (-9.7, 68.7) — was roled off the two collapsed
    # buildings standing on it rather than off its ground, and the ground there
    # is lawn: sampled over the block in the aerial it reads (154, 152, 138),
    # green-dominant by +6.5, against a bare-ground control of (208, 201, 194)
    # on the pads. The debris sits on grass, which is what the photograph
    # shows. Do not re-add it without re-measuring.
    assert "rubble" in roles
    assert "collapsed" not in roles
    assert "parking" in roles


def test_open_roles_are_undeveloped(built):
    """`undeveloped` is what stops the parcelling pass building houses on a
    rubble field. Every role on this site is an open one, so every block."""
    _net, blocks, info = built
    for b in blocks:
        assert b["undeveloped"] is (b["role"] in spl.OPEN_ROLES)
    assert info["undeveloped"] == len(blocks)


def test_build_is_deterministic(spec):
    """No rng reaches the layout, so two builds are identical geometry."""
    a = spl.generate(*spl.region_of(spec), None, {"_spec": spec})
    b = spl.generate(*spl.region_of(spec), None, {"_spec": spec})
    pa = sorted(tuple(round(v, 4) for p in bl["poly"] for v in p) for bl in a[1])
    pb = sorted(tuple(round(v, 4) for p in bl["poly"] for v in p) for bl in b[1])
    assert pa == pb


def test_region_comes_from_the_spec_not_the_caller(spec, capsys):
    """A config asking for a different box does not resize a real place."""
    w, h = spl.region_of(spec)
    _net, _blocks, info = spl.generate(50.0, 50.0, None, {"_spec": spec})
    x0, y0, x1, y1 = info["region"]
    assert math.isclose(x1 - x0, w) and math.isclose(y1 - y0, h)
    assert "region_m from the spec" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# clipping
# ---------------------------------------------------------------------------

def test_clip_keeps_the_inside_runs():
    rect = (-10.0, -10.0, 10.0, 10.0)
    runs = spl._clip_polyline([(-20.0, 0.0), (0.0, 0.0), (20.0, 0.0)], rect)
    assert len(runs) == 1
    assert runs[0][0] == pytest.approx((-10.0, 0.0))
    assert runs[0][-1] == pytest.approx((10.0, 0.0))


def test_clip_splits_a_road_that_leaves_and_returns():
    """Two runs, not one: joining them would pave land outside the scene."""
    rect = (-10.0, -10.0, 10.0, 10.0)
    pts = [(-5.0, 0.0), (-5.0, 20.0), (5.0, 20.0), (5.0, 0.0)]
    runs = spl._clip_polyline(pts, rect)
    assert len(runs) == 2
    for r in runs:
        for x, y in r:
            assert -10.001 <= x <= 10.001 and -10.001 <= y <= 10.001


def test_clip_drops_a_road_entirely_outside():
    assert spl._clip_polyline([(50.0, 50.0), (60.0, 60.0)],
                              (-10.0, -10.0, 10.0, 10.0)) == []


# ---------------------------------------------------------------------------
# the preset that drives it
# ---------------------------------------------------------------------------

def test_preset_names_the_plan():
    import yaml
    path = os.path.join(os.path.dirname(__file__), "..", "config", "presets",
                        "disaster_city.yaml")
    with open(path) as fh:
        preset = yaml.safe_load(fh)
    assert preset["overrides"]["site_plan"] == PLAN
    spec = spl.load_spec(PLAN)
    # The preset repeats the region for readability; it must not disagree.
    assert preset["region_m"] == pytest.approx(list(spl.region_of(spec)))


# ---------------------------------------------------------------------------
# features — what stands on the blocks
# ---------------------------------------------------------------------------

def test_every_feature_is_well_formed(spec):
    feats = spec.get("features") or []
    assert len(feats) >= 20, "the site plan lost its hand-authored features"
    w, h = spl.region_of(spec)
    for i, f in enumerate(feats):
        assert f.get("kind") in spl.FEATURE_KINDS, f"feature {i}: {f.get('kind')}"
        assert len(f["at"]) == 2
        assert f.get("size_m") or f.get("poly"), f"feature {i} has no geometry"
        for x, y in spl.feature_poly(f):
            assert -w / 2 - 1 <= x <= w / 2 + 1 and -h / 2 - 1 <= y <= h / 2 + 1


def test_feature_poly_is_an_oriented_rectangle():
    """`yaw_deg` turns the LONG axis, CCW from east — the convention the spec
    was authored against. A sign error here silently rotates every building."""
    p = spl.feature_poly({"at": [0.0, 0.0], "size_m": [10.0, 4.0], "yaw_deg": 0.0})
    assert len(p) == 4
    assert max(x for x, _ in p) == pytest.approx(5.0)
    assert max(y for _, y in p) == pytest.approx(2.0)
    q = spl.feature_poly({"at": [0.0, 0.0], "size_m": [10.0, 4.0], "yaw_deg": 90.0})
    assert max(x for x, _ in q) == pytest.approx(2.0)
    assert max(y for _, y in q) == pytest.approx(5.0)


def test_explicit_poly_wins_over_size():
    """A rubble field is not a rectangle; saying it is puts debris on clean
    ground, so an outline in the spec must not be replaced by a box."""
    poly = [[0.0, 0.0], [10.0, 0.0], [4.0, 8.0]]
    got = spl.feature_poly({"at": [4.0, 3.0], "size_m": [99.0, 99.0], "poly": poly})
    assert got == [(0.0, 0.0), (10.0, 0.0), (4.0, 8.0)]


def test_features_land_on_their_blocks(built):
    _net, blocks, info = built
    feats = info["features"]
    assert len(feats) == sum(len(b["features"]) for b in blocks) \
        + info["features_homeless"]
    # A feature attached to no block usually means a footprint drifted off the
    # plan — EXCEPT for vehicles, which are parked at the kerb by intent and
    # whose footprint therefore straddles the carriageway margin. The kind
    # check is the real guard; the count only stops the exemption being used
    # to hide a general drift.
    homeless = [f for f in feats if f["block"] is None]
    assert all(f["kind"] == "vehicle" for f in homeless), \
        [f["kind"] for f in homeless if f["kind"] != "vehicle"]
    assert len(homeless) <= 4


def test_the_rubble_fields_survive(built):
    """The two the user confirmed, and the reason the feature layer exists.
    They carry traced outlines, not boxes."""
    _net, _blocks, info = built
    rub = [f for f in info["features"] if f["kind"] == "rubble"]
    assert len(rub) == 2
    for f in rub:
        assert len(f["poly"]) > 6, "a traced field was replaced by a rectangle"
        assert abs(sn.polygon_area(f["poly"])) > 500.0


def test_buildings_sit_on_blocks_that_can_hold_them(built):
    _net, blocks, _info = built
    for b in blocks:
        built_area = sum(abs(sn.polygon_area(f["poly"])) for f in b["features"]
                         if f["kind"] == "building")
        assert built_area < b["area"], f"{b['role']} block is over-built"


# ---------------------------------------------------------------------------
# ground triangulation — the bug that made the zone fills unusable
# ---------------------------------------------------------------------------

def test_earclip_covers_a_concave_polygon_exactly():
    """An L: a fan from ANY single point spills outside it, ear clipping does
    not. `polygon_area` of the ring and the sum of the triangles must agree."""
    ring = [(0, 0), (6, 0), (6, 2), (2, 2), (2, 6), (0, 6)]
    tris = sn.earclip(ring)
    assert len(tris) == len(ring) - 2
    got = sum(abs((ring[b][0] - ring[a][0]) * (ring[c][1] - ring[a][1])
                  - (ring[b][1] - ring[a][1]) * (ring[c][0] - ring[a][0])) / 2.0
              for a, b, c in tris)
    assert got == pytest.approx(abs(sn.polygon_area(ring)))


def test_simple_rings_excises_a_self_crossing_fold():
    """A bow tie is two loops of opposite winding. Only the positive one is
    real surface; keeping the other lays a second sheet at the same height,
    which is what the z-fighting was."""
    bow = [(0, 0), (4, 4), (4, 0), (0, 4)]
    # Its own signed area is ZERO — the two lobes cancel — which is exactly why
    # a ring like this cannot be triangulated as one piece.
    assert sn.polygon_area(bow) == pytest.approx(0.0)
    rings = sn.simple_rings(bow)
    assert rings, "the whole ring was discarded"
    assert all(sn.polygon_area(r) > 0 for r in rings), "a fold survived"
    # One lobe, four square metres of it; the reversed twin is gone.
    assert sum(sn.polygon_area(r) for r in rings) == pytest.approx(4.0)


def test_every_committed_block_triangulates_to_its_own_area(built):
    """THE REGRESSION. Nine of these sixteen blocks self-cross, and the
    largest has its centroid OUTSIDE it — a centroid fan swept triangles across
    the whole site and every block was drawn over every other one at the same
    z. Covered area must equal polygon area, block by block.
    """
    _net, blocks, _info = built
    for b in blocks:
        got = 0.0
        for ring in sn.simple_rings(b["poly"]):
            for a, c, d in sn.earclip(ring):
                p, q, r = ring[a], ring[c], ring[d]
                got += abs((q[0] - p[0]) * (r[1] - p[1])
                           - (q[1] - p[1]) * (r[0] - p[0])) / 2.0
        assert got == pytest.approx(b["area"], rel=0.03), \
            f"{b['role']} block: {got:.0f} m² drawn for {b['area']:.0f} m² of block"


def test_the_biggest_block_would_defeat_a_centroid_fan(built):
    """Guards the REASON for the above, not just the behaviour: if this block
    ever becomes star-shaped, the ear clipper is no longer load-bearing and
    someone will be tempted to put the fan back."""
    _net, blocks, _info = built
    big = max(blocks, key=lambda b: b["area"])
    assert not sn.point_in_polygon(big["poly"], sn.polygon_centroid(big["poly"]))


# ---------------------------------------------------------------------------
# zone dressing — materials and planting per role
# ---------------------------------------------------------------------------

def test_every_role_has_ground_and_a_planting_rule():
    """A role that reaches the scene with no entry in either table is dressed
    by a default that was written for a suburban lawn — which is how a car park
    got planted with trees and a rubble field got mown grass."""
    import suburb_scene as ss
    spec = spl.load_spec(PLAN)
    roles = {str(b.get("role")) for b in (spec.get("blocks") or [])}
    for r in roles:
        assert r in ss.ZONE_GROUND, f"{r} has no ground material"
        assert r in ss.ZONE_PLANTING, f"{r} has no planting rule"


def test_paved_roles_are_never_planted():
    import suburb_scene as ss
    for r in ("parking", "pad", "staging", "rubble", "collapsed"):
        assert ss.ZONE_PLANTING[r] == 0.0, f"{r} would grow trees"
    # Woodland is the one role the aerial actually shows under canopy.
    assert ss.ZONE_PLANTING["wooded"] > ss.ZONE_PLANTING["grass"] > 0.0


def test_zone_materials_are_declared_and_local():
    """Every material a role asks for must exist in the asset set, and resolve
    to a file that is HERE: this box authenticates to Nucleus as `guest`, so an
    `omniverse://` ground material binds nothing at all and the zone renders as
    flat colour with no warning."""
    import os
    import suburb_scene as ss
    from compile_disaster import load_scene_config
    cfg = load_scene_config(os.path.join(os.path.dirname(__file__), "..",
                                         "config", "presets",
                                         "disaster_city.yaml"))
    declared = (cfg.get("usds", {}) or {}).get("materials", {}) or {}
    root = os.path.join(os.path.dirname(__file__), "..", "..")
    for role, (_col, key, tile) in ss.ZONE_GROUND.items():
        if not key:
            continue
        assert key in declared, f"{role} wants material {key!r}, undeclared"
        url = declared[key]
        assert url.startswith("airstack://"), f"{key} is not local: {url}"
        path = os.path.join(root, url[len("airstack://"):])
        assert os.path.isfile(path), f"{key} -> missing file {path}"
        assert tile > 0.0, f"{role} has no tiling size"


def test_the_car_park_outline_is_surveyed_not_traced(built):
    """A block with an `outline` override gets the surveyed shape, not the face.

    The car park's traced face runs down the centreline of the road that bounds
    it, so it inherits every wobble in that spline. The photograph shows a
    TRAPEZOID: a plumb west kerb, an east one slanting in going south, straight
    runs joined by arcs. The difference is not cosmetic — the bay markings are
    struck between the lot's own edges, so a wobbly boundary put painted stalls
    out on the grass.
    """
    _net, blocks, _info = built
    lot = next(b for b in blocks if b.get("role") == "parking")
    poly = lot["poly"]

    def span(yq):
        xs = []
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            if (y0 > yq) != (y1 > yq):
                xs.append(x0 + (yq - y0) * (x1 - x0) / (y1 - y0))
        return (min(xs), max(xs)) if len(xs) >= 2 else None

    rows = [(y, span(y)) for y in (-20, -35, -50, -65, -78)]
    assert all(sp for _y, sp in rows), "the outline does not span the aisles"
    # West kerb plumb: every sample within a few centimetres of the same x.
    wests = [sp[0] for _y, sp in rows]
    assert max(wests) - min(wests) < 0.05, f"west edge is not straight: {wests}"
    # East kerb slanting IN going south, monotonically and by metres.
    easts = [sp[1] for _y, sp in rows]
    assert all(a > b for a, b in zip(easts, easts[1:])), (
        f"east edge does not slant in going south: {easts}")
    assert easts[0] - easts[-1] > 3.0, f"east edge barely slants: {easts}"
    # Rounded, not mitred: no vertex reaches a corner of the bounding box.
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    for cx in (min(xs), max(xs)):
        for cy in (min(ys), max(ys)):
            assert min(math.dist((x, y), (cx, cy)) for x, y in poly) > 1.0


def test_every_bay_line_stays_inside_the_car_park(built):
    """The stall rectangles the marking pass would draw, checked offline.

    This mirrors `suburb_scene`'s bay loop rather than importing it — the pass
    needs a stage — and asserts the property the pass exists to hold: a stall is
    drawn only when BOTH ends land inside the lot. Struck across the bounding
    box instead, which is what shipped first, the lower rows ran out over the
    kerb; on a rounded rectangle the box and the lot differ by the corner
    radius, and that difference IS the bays painted on the grass.
    """
    import suburb_scene as ss
    from compile_disaster import load_scene_config
    cfg = load_scene_config(os.path.join(os.path.dirname(__file__), "..",
                                         "config", "presets",
                                         "disaster_city.yaml"))
    bays = cfg.get("site_parking_bays") or {}
    assert bays.get("spines"), "the plan has no car-park aisles"
    _net, blocks, _info = built
    lot = next(b for b in blocks if b.get("role") == "parking")
    poly = [(float(x), float(y)) for x, y in lot["poly"]]
    if sn.polygon_area(poly) < 0.0:
        poly = list(reversed(poly))
    inset = sn.offset_polygon(poly, [float(bays.get("edge_m", 2.5))]) or poly
    depth = float(bays.get("stall_depth_m", 5.5))
    pitch = float(bays.get("stall_pitch_m", 2.5))
    drawn = 0
    for spec_row in bays["spines"]:
        if isinstance(spec_row, (list, tuple)):
            sy, want = float(spec_row[0]), (float(spec_row[1]), float(spec_row[2]))
        else:
            sy, want = float(spec_row), None
        span = ss._span_at_y(inset, sy)
        assert span is not None, f"aisle y={sy} misses the lot entirely"
        lo, hi = span
        if want:
            lo, hi = max(lo, want[0]), min(hi, want[1])
        x = lo
        while x <= hi:
            for sgn in (-1.0, 1.0):
                tip = sy + sgn * depth
                if not ss._point_in_poly_xy((x, tip), inset):
                    continue
                assert ss._point_in_poly_xy((x, tip), poly), (
                    f"stall at x={x:.1f} y={tip:.1f} is outside the lot")
                drawn += 1
            x += pitch
    # 83 at the committed measurements. The floor is here to catch a clip that
    # eats the lot, not to pin the number: the southern row sits on the bottom
    # edge, so its outward stalls are correctly dropped and it comes out
    # single-sided — which is how the photograph shows it.
    assert drawn > 70, f"only {drawn} stalls survive the clip"


def test_every_ground_prim_name_is_a_legal_identifier(built):
    """The names the marking pass builds, checked as USD identifiers.

    `bay_{i}_{x}_{int(sgn)}` spelt the south side of every aisle
    `bay_0_25_-1`. A hyphen is not legal in a USD identifier, so `DefinePrim`
    dropped it and each aisle came out single-loaded — while the pass counted
    both sides and reported 140 lines against 50 prims actually on the stage.
    The photograph shows every aisle double-loaded.
    """
    from pxr import Tf
    import suburb_scene as ss
    from compile_disaster import load_scene_config
    cfg = load_scene_config(os.path.join(os.path.dirname(__file__), "..",
                                         "config", "presets",
                                         "disaster_city.yaml"))
    bays = cfg.get("site_parking_bays") or {}
    _net, blocks, _info = built
    lot = next(b for b in blocks if b.get("role") == "parking")
    poly = [(float(x), float(y)) for x, y in lot["poly"]]
    if sn.polygon_area(poly) < 0.0:
        poly = list(reversed(poly))
    inset = sn.offset_polygon(poly, [float(bays.get("edge_m", 2.5))]) or poly
    pitch = float(bays.get("stall_pitch_m", 2.5))
    depth = float(bays.get("stall_depth_m", 5.5))
    names, sides = [], {"n": 0, "s": 0}
    for i, row in enumerate(bays["spines"]):
        names.append(f"bay_spine_{i}")
        if isinstance(row, (list, tuple)):
            sy, want = float(row[0]), (float(row[1]), float(row[2]))
        else:
            sy, want = float(row), None
        lo, hi = ss._span_at_y(inset, sy)
        if want:
            lo, hi = max(lo, want[0]), min(hi, want[1])
        x = lo
        while x <= hi:
            for sgn, side in ((-1.0, "s"), (1.0, "n")):
                if not ss._point_in_poly_xy((x, sy + sgn * depth), inset):
                    continue
                names.append(f"bay_{i}_{int(round((x - lo) * 10))}_{side}")
                sides[side] += 1
            x += pitch
    for n in names:
        assert Tf.IsValidIdentifier(n), f"{n} is not a legal prim name"
    assert len(set(names)) == len(names), "two stalls would collide on one path"
    assert sides["s"] > 20 and sides["n"] > 20, (
        f"aisles are not double-loaded: {sides}")


def test_the_drawn_block_ring_is_exactly_the_face_edge(built):
    """Whatever the kerb and the zone fill draw, it is the road edge.

    `blocks_from_faces` insets each face by half of the carriageway on THAT
    side, so the polygon already tracks a road that changes width along its
    length. Two passes draw from it — the kerb ring and the ground fill — and
    both have to get the same points:

      * a kerb off the carriageway edge is not a kerb;
      * the road ribbon is swept from the centreline at its true width, so a
        ring pulled inward leaves a strip of bare ground between road and
        block.

    A simplify-and-fillet tidy-up was tried here and moved the edge by up to
    8.68 m, which rendered as grass showing around the corners of the sand
    pads. This pins it at zero.
    """
    import suburb_scene as ss
    _net, blocks, _info = built
    cfg = {"site_block_simplify_m": 2.0, "site_block_corner_r": 6.0}
    worst = 0.0
    for b in blocks:
        b.pop("_ring", None)
        raw = [(float(x), float(y)) for x, y in b["poly"]]
        ring = ss.block_ring(b, cfg)
        assert len(ring) == len(raw), "the drawn ring changed vertex count"
        for p, q in zip(raw, ring):
            worst = max(worst, math.dist(p, q))
    assert worst < 1e-9, f"drawn ring departs from the face by {worst:.3f} m"


def test_the_block_edge_tracks_a_road_that_changes_width(built):
    """The inset is per-edge, and this site has roads of very different widths.

    The guard above only says the ring is the face. This says the face is worth
    protecting: the carriageways here run from about 5.7 m to 11 m, so a block
    bounded by both is inset by different amounts on different sides, and that
    step is the kerb correctly following the road — not a defect to smooth out.
    """
    _net, _blocks, info = built
    widths = sorted({round(float(r.get("width_m", 0.0)), 1)
                     for r in (info.get("spec_roads") or [])})
    if not widths:
        import yaml
        import os as _os
        spec = yaml.safe_load(open(_os.path.join(
            _os.path.dirname(__file__), "..", "config", "site_plans",
            "disaster_city.yaml")))
        widths = sorted({round(float(r.get("width_m", 0.0)), 1)
                         for r in spec.get("roads") or []})
    assert max(widths) - min(widths) > 3.0, (
        f"roads are all one width ({widths}); the per-edge inset is moot")
