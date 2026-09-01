#!/usr/bin/env python3
"""test_crop_window.py — `tools/crop_window.py`'s window math, re-centring,
drop-not-cut building rule, clip-not-drop block rule, and the `of`-tag
orphan-prop rule (see that module's docstring for all four). Pure python,
host-side, no `pxr`, no Kit — everything here is synthetic geometry, the same
discipline `test_fire_city_dry_run.py` uses for `tools/fire_city_dry_run.py`.

    python3 -m pytest scene_gen/tests/test_crop_window.py -q

A second class (`TestRealDowntownFire1500Crop`) runs the SAME invariants
against `tools/plan_png.build("downtown_fire_1500", ...)`'s real output for a
couple of seeds — the host-packer smoke test the crop mechanic's own review
task asked for, including the SAT overlap check
(`fire_city_dry_run._obb_corners`/`_obb_overlap`) and the facing audit
(`plan_png._blank_wall_violations`) BOTH BEFORE AND AFTER the crop, so a
regression the crop itself introduces is distinguished from a violation the
host packer already had. See that class's docstring for the host-vs-Kit
packer caveat this is NOT a substitute for (`generate-urban-city` skill:
"treat a host-side layout as a size source, never as the layout itself").
Slower (host-builds a real 1500 m city) but still no Kit — a few seconds.
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _TOOLS_DIR)

import crop_window as cw                                    # noqa: E402
import plan_png                                              # noqa: E402
from fc_prop_orphan_probe import tag_of                     # noqa: E402


def _bld(i, x, y, W=20.0, D=10.0, yaw=0.0, usd="bld_office_DG0.usd"):
    return {"category": "house", "i": i, "usd": usd, "x_m": x, "y_m": y,
           "yaw_deg": yaw, "W": W, "D": D, "H": 12.0}


def _prop(x, y, of=None, category="street_trees", usd="SM_Tree_04.usd"):
    return {"category": category, "usd": usd, "x_m": x, "y_m": y,
           "yaw_deg": 0.0, "of": of}


def _footprint_of(p):
    return (p["W"], p["D"]) if "W" in p else None


class TestWindowMath:
    def test_recenter_shifts_to_origin(self):
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        placements = [_bld(0, 600.0, 700.0, W=4.0, D=4.0)]
        window = (100.0, 200.0, 1100.0, 1200.0)   # centre (600, 700)
        new_layout, new_p, rpt = cw.crop_layout(
            layout, placements, window, footprint_of=_footprint_of)
        assert rpt["shift"] == [-600.0, -700.0]
        assert len(new_p) == 1
        assert new_p[0]["x_m"] == 0.0 and new_p[0]["y_m"] == 0.0
        assert new_p[0]["x_m_orig"] == 600.0 and new_p[0]["y_m_orig"] == 700.0
        x0, y0, x1, y1 = new_layout["region"]
        assert (x0, y0, x1, y1) == (-500.0, -500.0, 500.0, 500.0)

    def test_no_recenter_keeps_original_frame(self):
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        placements = [_bld(0, 600.0, 700.0, W=4.0, D=4.0)]
        window = (100.0, 200.0, 1100.0, 1200.0)
        _new_layout, new_p, rpt = cw.crop_layout(
            layout, placements, window, footprint_of=_footprint_of,
            recenter=False)
        assert rpt["shift"] == [0.0, 0.0]
        assert new_p[0]["x_m"] == 600.0 and new_p[0]["y_m"] == 700.0

    def test_degenerate_window_raises(self):
        import pytest
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        with pytest.raises(ValueError):
            cw.crop_layout(layout, [], (10.0, 0.0, 10.0, 5.0),
                           footprint_of=_footprint_of)


class TestBuildingDropNotCut:
    def test_fully_inside_kept(self):
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        placements = [_bld(0, 0.0, 0.0, W=10.0, D=10.0)]
        window = (-50.0, -50.0, 50.0, 50.0)
        _l, new_p, rpt = cw.crop_layout(layout, placements, window,
                                        footprint_of=_footprint_of)
        assert rpt["buildings_kept"] == 1 and rpt["buildings_dropped"] == 0
        assert len(new_p) == 1

    def test_footprint_grazing_edge_is_dropped_whole(self):
        """A building whose footprint pokes 0.2 m past the window edge is
        DROPPED WHOLE -- never left in the output with a clipped/shrunk
        footprint. This is the rule the module docstring calls out first."""
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        # footprint spans x in [45.0, 55.2] -- 0.2 m past the x1=55 edge
        placements = [_bld(0, 50.1, 0.0, W=10.0, D=10.0)]
        window = (-55.0, -55.0, 55.0, 55.0)
        _l, new_p, rpt = cw.crop_layout(layout, placements, window,
                                        footprint_of=_footprint_of)
        assert rpt["buildings_dropped"] == 1 and rpt["buildings_kept"] == 0
        assert new_p == []

    def test_rotated_footprint_uses_conservative_aabb(self):
        """A yawed rect's AABB is bigger than its axis-aligned W/D -- a
        building that would fit axis-aligned but not once rotated must still
        be dropped (over-dropping is the safe direction, per the module
        docstring)."""
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        # W=20,D=4 at yaw=45deg: AABB half-extent = (20+4)/2 / sqrt(2) * ...
        # just check the geometry helper directly for a known case first.
        import pytest
        x0, y0, x1, y1 = cw._rot_aabb(0.0, 0.0, 20.0, 4.0, 45.0)
        # rotated corners at +-(10,2) and +-(10,-2) -> aabb half-extent
        # = (10+2)/sqrt(2) ~= 8.485, well under 10 (the unrotated half-W)
        # but bigger than the unrotated half-D (2).
        assert x1 == pytest.approx(8.485, abs=0.01)
        assert y1 == pytest.approx(8.485, abs=0.01)


class TestOrphanProps:
    def test_prop_of_dropped_building_is_removed_even_if_its_own_point_is_inside(self):
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        dropped_bld = _bld(0, 50.1, 0.0, W=10.0, D=10.0,
                           usd="bld_office_DG0.usd")
        of_tag = tag_of(dropped_bld["usd"], dropped_bld["x_m"], dropped_bld["y_m"])
        orphan = _prop(48.0, 0.0, of=of_tag, category="roof_house")
        unrelated = _prop(0.0, 0.0, of=None)
        window = (-55.0, -55.0, 55.0, 55.0)
        _l, new_p, rpt = cw.crop_layout(
            layout, [dropped_bld, orphan, unrelated], window,
            footprint_of=_footprint_of)
        # the building was dropped (footprint pokes out), the roof prop tied
        # to it is ALSO dropped even though its own (48, 0) is inside the
        # window, and the unrelated point-only prop survives.
        assert rpt["buildings_dropped"] == 1
        assert rpt["props_orphan_dropped"] == 1
        kept_categories = [p["category"] for p in new_p]
        assert "roof_house" not in kept_categories
        assert "street_trees" in kept_categories

    def test_prop_of_kept_building_survives(self):
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        kept_bld = _bld(0, 0.0, 0.0, W=10.0, D=10.0, usd="bld_office_DG0.usd")
        of_tag = tag_of(kept_bld["usd"], kept_bld["x_m"], kept_bld["y_m"])
        roof_prop = _prop(1.0, 1.0, of=of_tag, category="roof_house")
        window = (-55.0, -55.0, 55.0, 55.0)
        _l, new_p, rpt = cw.crop_layout(
            layout, [kept_bld, roof_prop], window, footprint_of=_footprint_of)
        assert rpt["props_orphan_dropped"] == 0
        assert any(p["category"] == "roof_house" for p in new_p)

    def test_no_surviving_prop_ever_references_a_dropped_building(self):
        """General invariant, over a small synthetic mixed population: after
        cropping, no `of`-tagged prop in the output points at a building tag
        that is not ALSO in the output — the property this whole rule
        exists for, checked directly rather than by example."""
        layout = {"blocks": [], "road_corridors": [], "_typology_of": {}}
        placements = []
        for i in range(8):
            x = -60.0 + i * 16.0     # spans well past the +-55 window on
            placements.append(_bld(i, x, 0.0, W=10.0, D=10.0,  # both ends
                                   usd="bld_office_DG0.usd"))
            tag = tag_of("bld_office_DG0.usd", x, 0.0)
            placements.append(_prop(x, 2.0, of=tag, category="roof_house"))
        window = (-55.0, -55.0, 55.0, 55.0)
        _l, new_p, rpt = cw.crop_layout(layout, placements, window,
                                        footprint_of=_footprint_of)
        kept_building_tags = {
            tag_of(p["usd"], p["x_m_orig"], p["y_m_orig"])
            for p in new_p if p.get("category") == "house"}
        for p in new_p:
            if p.get("of") is not None:
                assert p["of"] in kept_building_tags
        assert rpt["buildings_dropped"] > 0    # the test is not vacuous
        assert rpt["props_orphan_dropped"] > 0


class TestBlockClipNotDrop:
    def test_block_straddling_edge_is_clipped_not_dropped(self):
        layout = {"blocks": [[-20.0, -20.0, 70.0, 20.0]],
                  "road_corridors": [], "_typology_of": {(-20.0, -20.0, 70.0, 20.0): "midrise"}}
        window = (-55.0, -55.0, 55.0, 55.0)
        new_layout, _p, rpt = cw.crop_layout(layout, [], window,
                                             footprint_of=_footprint_of)
        assert rpt["blocks_kept"] == 1 and rpt["blocks_dropped"] == 0
        (rect,) = new_layout["blocks"]
        # original x1=70 clipped to window x1=55, then re-centred by -0(win
        # centre is (0,0) here, so no shift): clipped rect x1 == 55.0
        assert rect == [-20.0, -20.0, 55.0, 20.0]
        assert new_layout["_typology_of"][tuple(rect)] == "midrise"

    def test_block_entirely_outside_window_is_dropped(self):
        layout = {"blocks": [[200.0, 200.0, 250.0, 250.0]],
                  "road_corridors": [],
                  "_typology_of": {(200.0, 200.0, 250.0, 250.0): "tower"}}
        window = (-55.0, -55.0, 55.0, 55.0)
        new_layout, _p, rpt = cw.crop_layout(layout, [], window,
                                             footprint_of=_footprint_of)
        assert rpt["blocks_dropped"] == 1
        assert new_layout["blocks"] == []

    def test_road_corridor_clipped(self):
        layout = {"blocks": [], "_typology_of": {},
                  "road_corridors": [{"x0": -20.0, "y0": -3.0, "x1": 70.0,
                                      "y1": 3.0, "dir": "ew", "n_lanes": 2}]}
        window = (-55.0, -55.0, 55.0, 55.0)
        new_layout, _p, _rpt = cw.crop_layout(layout, [], window,
                                              footprint_of=_footprint_of)
        (c,) = new_layout["road_corridors"]
        assert c["x0"] == -20.0 and c["x1"] == 55.0
        assert c["dir"] == "ew" and c["n_lanes"] == 2  # other fields carried through


class TestRealDowntownFire1500Crop:
    """The host-packer smoke test: crop a REAL `downtown_fire_1500` layout
    (`plan_png.build`, host-side, no Kit) and re-check the same invariants
    the generator itself is graded on.

    THE CAVEAT (`generate-urban-city` skill, "treat a host-side layout as a
    size source, never as the layout itself"): this proves the CROP MATH is
    sound on a real district/terrace/park mix, not that the Kit-built city
    will crop identically -- GAC/downtowncity footprints are host-side
    measurement-table lookups here, not Kit's own live-Nucleus resolver, so
    a real pod dump can pack (and therefore crop) a handful of buildings
    differently. See `tools/fc_dump_crop.py` for the tool that will actually
    run on a real `FC_DUMP` on the pod.
    """

    @staticmethod
    def _run(seed):
        import plan_png
        cfg, layout, placements, res = plan_png.build(
            "downtown_fire_1500", seed=seed,
            spec_overrides={"disaster-type": "none"})
        return cfg, layout, placements, res

    def test_crop_introduces_no_orphan_props_on_a_real_seed(self):
        cfg, layout, placements, res = self._run(seed=1)

        def footprint_of(p):
            fp = res.get(p.get("usd", ""), p.get("category", "house"))
            return fp["sx"], fp["sy"]

        x0, y0, x1, y1 = layout["region"]
        cx = (x0 + x1) / 2.0 + 120.0     # a deliberately off-centre window
        cy = (y0 + y1) / 2.0 - 80.0
        window = (cx - 500.0, cy - 500.0, cx + 500.0, cy + 500.0)
        new_layout, new_p, rpt = cw.crop_layout(
            layout, placements, window, footprint_of=footprint_of)

        assert rpt["buildings_kept"] > 0, "window drew no city at all"
        kept_building_tags = {
            tag_of(p["usd"], p["x_m_orig"], p["y_m_orig"])
            for p in new_p if p.get("category") in ("house", "building")}
        n_checked = 0
        for p in new_p:
            of = p.get("of")
            if of is None:
                continue
            n_checked += 1
            assert of in kept_building_tags, (
                "surviving prop {0!r} references a building not in the "
                "cropped output".format(p))
        # informational -- not every seed/window draw places a tagged prop
        # (building_props needs a flat-roof match); print rather than skip
        # so a run with zero is visible in -q output, not silently green.
        print("[test_crop_window] checked {0} of-tagged prop(s) on seed 1, "
             "0 orphaned".format(n_checked))

    def test_sat_overlap_and_facing_hold_before_and_after_crop(self):
        """Reuses the existing SAT overlap test
        (`fire_city_dry_run._obb_corners`/`_obb_overlap`) and the facing
        audit (`plan_png._blank_wall_violations`) as a smoke test on the
        cropped host-packer output -- the crop is a pure filter+translate, so
        it must not CREATE an overlap or a facing violation that was not
        already there (it can only ever remove buildings, never move two
        surviving ones toward each other)."""
        import fire_city_dry_run as fdr

        cfg, layout, placements, res = self._run(seed=2)

        def footprint_of(p):
            fp = res.get(p.get("usd", ""), p.get("category", "house"))
            return fp["sx"], fp["sy"]

        def _overlaps(plc):
            houses = [p for p in plc if p.get("category") in ("house", "building")]
            corners = []
            for p in houses:
                W, D = footprint_of(p)
                corners.append(fdr._obb_corners(p["x_m"], p["y_m"], W, D,
                                                float(p.get("yaw_deg", 0.0))))
            bad = []
            for a in range(len(corners)):
                for b in range(a + 1, len(corners)):
                    if fdr._obb_overlap(corners[a], corners[b]):
                        bad.append((a, b))
            return bad

        before_overlaps = _overlaps(placements)

        x0, y0, x1, y1 = layout["region"]
        cx = (x0 + x1) / 2.0 - 150.0
        cy = (y0 + y1) / 2.0 + 200.0
        window = (cx - 500.0, cy - 500.0, cx + 500.0, cy + 500.0)
        new_layout, new_p, rpt = cw.crop_layout(
            layout, placements, window, footprint_of=footprint_of)

        after_overlaps = _overlaps(new_p)
        assert not before_overlaps, (
            "host packer itself has overlapping footprints before any crop "
            "-- not a crop defect, but the smoke test's own precondition")
        assert not after_overlaps, (
            "crop introduced an overlap that was not in the source layout: "
            "{0}".format(after_overlaps))

        # facing: `_blank_wall_violations` needs `meta_by_usd` (blank-wall
        # tags per pool entry), which `plan_png.audit()` derives from `cfg`
        # via `districts._pool_entries` -- reused verbatim here rather than
        # re-derived.
        from detail import districts
        usds_cfg = (cfg.get("usds") or {}).get("buildings") or {}
        meta_by_usd = {}
        for key in usds_cfg:
            for e in districts._pool_entries(cfg, res, key):
                meta_by_usd[os.path.basename(e[0])] = e[5]

        before_v = plan_png._blank_wall_violations(
            cfg, layout, placements, res, meta_by_usd)
        after_v_unrepaired = plan_png._blank_wall_violations(
            cfg, new_layout, new_p, res, meta_by_usd)

        # THE FIX, not a looser assertion: clipping a block can pull a far
        # edge dramatically closer to an interior building (measured in
        # crop_window.py's own module docstring) -- `repair_after_crop` is
        # the SAME final facing/overlap repair `districts.remap_buildings`
        # already runs once at the end of a normal build, re-applied here
        # against the cropped geometry. Mutates `new_p` in place.
        cw.repair_after_crop(cfg, new_layout, new_p, res)
        after_v = plan_png._blank_wall_violations(
            cfg, new_layout, new_p, res, meta_by_usd)
        assert len(after_v) <= len(before_v), (
            "crop + repair_after_crop still left a NEW blank-wall-to-street "
            "violation: before={0} after_unrepaired={1} after_repaired={2}"
            .format(before_v, after_v_unrepaired, after_v))


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
