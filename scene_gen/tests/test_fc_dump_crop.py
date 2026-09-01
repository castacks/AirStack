#!/usr/bin/env python3
"""test_fc_dump_crop.py — `tools/fc_dump_crop.py`, the `fire_city_placements_
dump.v1`-schema adapter over `tools/crop_window.crop_layout`. Pure python,
host-side, no `pxr`, no Kit for the synthetic tests; the last class builds a
REAL `fire_city_placements_dump.v1`-shaped doc from a host-built
`downtown_fire_1500` layout (`plan_png.build`, still no Kit) and proves the
cropped output round-trips through `fire_city_dry_run.load_placements_dump`
UNCHANGED — the literal claim `downtown_fire_1500.yaml`'s header and
`crop_window.py`'s docstring both make: "downstream sees a normal dump."

    python3 -m pytest scene_gen/tests/test_fc_dump_crop.py -q
"""
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _TOOLS_DIR)

import fc_dump_crop as fdc                                  # noqa: E402
import fire_city_dry_run as fdr                              # noqa: E402


def _synthetic_dump():
    """A tiny hand-built `fire_city_placements_dump.v1` -- five houses
    spanning a 1500 m plate (three inside a 1000 m centred-ish window, two
    outside it), one typology block per house so the block-clip path is
    exercised too, `n_placements_total` padded above the house count the
    same way a real dump interleaves houses among trees/cars/etc."""
    houses = [
        {"i": 3, "cell": "/World/stage/generated/house_0_3",
         "usd": "bld_office_DG0.usd", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0,
         "yaw_deg": 0.0, "scale": 1.0, "category": "house", "axis_up": "Z",
         "W": 20.0, "D": 15.0, "H": 12.0},
        {"i": 9, "cell": "/World/stage/generated/house_1_9",
         "usd": "bld_apartment_DG0.usd", "x_m": 300.0, "y_m": -200.0,
         "z_m": 0.0, "yaw_deg": 90.0, "scale": 1.0, "category": "house",
         "axis_up": "Z", "W": 22.0, "D": 17.0, "H": 18.0},
        {"i": 14, "cell": "/World/stage/generated/house_2_14",
         "usd": "bld_tower_DG0.usd", "x_m": -400.0, "y_m": 350.0, "z_m": 0.0,
         "yaw_deg": 0.0, "scale": 1.0, "category": "house", "axis_up": "Z",
         "W": 25.0, "D": 20.0, "H": 43.0},
        # outside a [-500,500]^2 window centred on (0,0):
        {"i": 20, "cell": "/World/stage/generated/house_3_20",
         "usd": "bld_office_DG0.usd", "x_m": 700.0, "y_m": 700.0, "z_m": 0.0,
         "yaw_deg": 0.0, "scale": 1.0, "category": "house", "axis_up": "Z",
         "W": 20.0, "D": 15.0, "H": 12.0},
        {"i": 25, "cell": "/World/stage/generated/house_4_25",
         "usd": "bld_office_DG0.usd", "x_m": -650.0, "y_m": -100.0,
         "z_m": 0.0, "yaw_deg": 0.0, "scale": 1.0, "category": "house",
         "axis_up": "Z", "W": 20.0, "D": 15.0, "H": 12.0},
    ]
    blocks = [
        {"rect": [-30.0, -30.0, 30.0, 30.0], "name": "lowrise"},
        {"rect": [280.0, -230.0, 330.0, -170.0], "name": "midrise"},
        {"rect": [-450.0, 300.0, -350.0, 400.0], "name": "tower"},
        # straddles the +x1000/y1000 crop's edge at x=500 -- should CLIP
        {"rect": [400.0, -50.0, 600.0, 50.0], "name": "lowrise"},
        # entirely outside -- should be DROPPED
        {"rect": [650.0, 650.0, 750.0, 750.0], "name": "lowrise"},
    ]
    return {
        "schema": "fire_city_placements_dump.v1",
        "preset": "downtown_fire_1500", "seed": 1,
        "region_m": [1500.0, 1500.0],
        "n_placements_total": 4000,
        "placements": houses,
        "typology": {"blocks": blocks},
    }


class TestSyntheticCrop:
    def test_schema_and_top_level_fields_survive(self):
        doc = _synthetic_dump()
        new_doc, rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        assert new_doc["schema"] == "fire_city_placements_dump.v1"
        assert new_doc["preset"] == "downtown_fire_1500"
        assert new_doc["seed"] == 1
        assert new_doc["n_placements_total"] == 4000   # unchanged, see docstring
        assert new_doc["region_m"] == [1000.0, 1000.0]

    def test_houses_inside_kept_outside_dropped(self):
        doc = _synthetic_dump()
        new_doc, rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        kept_i = sorted(p["i"] for p in new_doc["placements"])
        assert kept_i == [3, 9, 14]
        assert rpt["buildings_kept"] == 3 and rpt["buildings_dropped"] == 2

    def test_i_and_cell_unchanged(self):
        """`i`/`cell` must survive VERBATIM -- see crop_window.py's "what
        this does not solve" section: a future assembly-time consumer needs
        them unshifted to ever have a chance at matching a Kit rebuild."""
        doc = _synthetic_dump()
        new_doc, _rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        by_i = {p["i"]: p for p in new_doc["placements"]}
        assert by_i[3]["cell"] == "/World/stage/generated/house_0_3"
        assert by_i[9]["cell"] == "/World/stage/generated/house_1_9"
        assert by_i[14]["cell"] == "/World/stage/generated/house_2_14"

    def test_recentred_coordinates_and_orig_preserved(self):
        doc = _synthetic_dump()
        # window centred on (0, 0) already -- recenter should be a no-op
        new_doc, _rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        by_i = {p["i"]: p for p in new_doc["placements"]}
        assert by_i[3]["x_m"] == 0.0 and by_i[3]["y_m"] == 0.0
        assert by_i[3]["x_m_orig"] == 0.0 and by_i[3]["y_m_orig"] == 0.0

        # an OFF-CENTRE window: centre (100, -50), so house 3 (originally at
        # the plate origin) should land at (-100, 50) post-recentre.
        new_doc2, _rpt2 = fdc.crop_fc_dump(doc, (-400.0, -550.0, 600.0, 450.0))
        by_i2 = {p["i"]: p for p in new_doc2["placements"]}
        assert by_i2[3]["x_m"] == -100.0 and by_i2[3]["y_m"] == 50.0
        assert by_i2[3]["x_m_orig"] == 0.0 and by_i2[3]["y_m_orig"] == 0.0

    def test_blocks_clipped_not_dropped_at_edge_dropped_when_fully_outside(self):
        doc = _synthetic_dump()
        new_doc, rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        rects = {tuple(b["rect"]): b["name"] for b in new_doc["typology"]["blocks"]}
        # the straddling block [400,-50,600,50] clips to [400,-50,500,50]
        assert (400.0, -50.0, 500.0, 50.0) in rects
        assert rects[(400.0, -50.0, 500.0, 50.0)] == "lowrise"
        # the fully-outside block [650,650,750,750] is gone entirely
        assert not any(r[0] >= 600.0 for r in rects)
        assert rpt["blocks_kept"] == 4   # the three fully-inside + the clipped one
        assert rpt["blocks_dropped"] == 1

    def test_no_overlap_among_kept_house_footprints(self):
        """Reuses `fire_city_dry_run._obb_corners`/`_obb_overlap` (the exact
        SAT check `check_footprint` runs on a fire manifest) directly on the
        cropped dump's own house records -- the review task's own ask:
        'reuse the existing SAT ... verification ... as a smoke test'."""
        doc = _synthetic_dump()
        new_doc, _rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        recs = new_doc["placements"]
        corners = [fdr._obb_corners(p["x_m"], p["y_m"], p["W"], p["D"],
                                    p["yaw_deg"]) for p in recs]
        for a in range(len(corners)):
            for b in range(a + 1, len(corners)):
                assert not fdr._obb_overlap(corners[a], corners[b]), (
                    "cropped dump has overlapping house footprints "
                    "{0} vs {1}".format(recs[a]["cell"], recs[b]["cell"]))

    def test_crop_output_loads_through_the_real_loader(self, tmp_path):
        """The literal "downstream sees a normal dump" claim: write the
        cropped doc to disk and load it back through
        `fire_city_dry_run.load_placements_dump` -- the SAME function
        `fire_city_dry_run.run_dry_from_dump`/`fire_people_dry_run` use on a
        real pod-written `FC_DUMP` -- with zero special-casing."""
        doc = _synthetic_dump()
        new_doc, _rpt = fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))
        path = str(tmp_path / "cropped.json")
        with open(path, "w") as fh:
            json.dump(new_doc, fh)

        config, layout, placements, seed, preset, sha256 = \
            fdr.load_placements_dump(path)
        assert seed == 1 and preset == "downtown_fire_1500"
        houses = [p for p in placements if p.get("category") == "house"]
        assert len(houses) == 3
        # the padded reconstruction must still be n_placements_total long,
        # so index 3/9/14 land at the SAME positions they always would.
        assert len(placements) == 4000
        assert placements[3]["usd"] == "bld_office_DG0.usd"
        assert placements[3]["prim_path"] == "/World/stage/generated/house_0_3"
        assert placements[9]["x_m"] == 300.0 and placements[9]["y_m"] == -200.0
        typ_of = layout["_typology_of"]
        assert any(name == "lowrise" for name in typ_of.values())

    def test_degenerate_window_raises(self):
        import pytest
        doc = _synthetic_dump()
        with pytest.raises(ValueError):
            fdc.crop_fc_dump(doc, (10.0, 0.0, 10.0, 5.0))

    def test_wrong_schema_raises(self):
        import pytest
        doc = _synthetic_dump()
        doc["schema"] = "something_else.v1"
        with pytest.raises(ValueError):
            fdc.crop_fc_dump(doc, (-500.0, -500.0, 500.0, 500.0))


class TestRealDowntownFire1500FcDump:
    """Builds a REAL `fire_city_placements_dump.v1`-shaped doc from a host
    `downtown_fire_1500` layout (no Kit) the same way
    `urban_fire_city_launch_script.dump_city_placements` would (house
    placements only, `i` = index into the FULL placement list, typology
    blocks verbatim), then crops it and checks the same invariants."""

    @staticmethod
    def _real_dump(seed):
        import plan_png
        cfg, layout, placements, res = plan_png.build(
            "downtown_fire_1500", seed=seed,
            spec_overrides={"disaster-type": "none"})
        houses = []
        for i, p in enumerate(placements):
            if p.get("category") != "house":
                continue
            fp = res.get(p.get("usd", ""), "house",
                        scale=p.get("scale", 1.0),
                        axis_up=p.get("axis_up", "Z"))
            houses.append({
                "i": i, "cell": p.get("prim_path") or "/synthetic/{0}".format(i),
                "usd": p["usd"], "x_m": float(p["x_m"]), "y_m": float(p["y_m"]),
                "z_m": float(p.get("z_m", 0.0)),
                "yaw_deg": float(p.get("yaw_deg", 0.0)),
                "scale": float(p.get("scale", 1.0)), "category": "house",
                "axis_up": p.get("axis_up", "Z"),
                "W": float(fp["sx"]), "D": float(fp["sy"]), "H": float(fp["sz"]),
            })
        blocks = [{"rect": [float(v) for v in rect], "name": name}
                 for rect, name in (layout.get("_typology_of") or {}).items()]
        doc = {
            "schema": "fire_city_placements_dump.v1",
            "preset": "downtown_fire_1500", "seed": int(cfg["seed"]),
            "region_m": [float(v) for v in cfg["layout"]["region_m"]],
            "n_placements_total": len(placements),
            "placements": houses, "typology": {"blocks": blocks},
        }
        return doc

    def test_real_crop_loads_and_has_no_overlaps(self, tmp_path):
        doc = self._real_dump(seed=3)
        window = (-450.0, -480.0, 550.0, 520.0)   # deliberately off-centre
        new_doc, rpt = fdc.crop_fc_dump(doc, window)
        assert rpt["buildings_kept"] > 0

        path = str(tmp_path / "real_cropped.json")
        with open(path, "w") as fh:
            json.dump(new_doc, fh)
        _config, _layout, placements, seed, preset, _sha = \
            fdr.load_placements_dump(path)
        assert preset == "downtown_fire_1500"
        houses = [p for p in placements if p.get("category") == "house"]
        assert len(houses) == rpt["buildings_kept"]

        corners = [fdr._obb_corners(p["x_m"], p["y_m"], p["W"], p["D"],
                                    p["yaw_deg"]) for p in houses]
        bad = [(a, b) for a in range(len(corners)) for b in range(a + 1, len(corners))
              if fdr._obb_overlap(corners[a], corners[b])]
        assert not bad, "cropped real dump has overlapping footprints: {0}".format(bad)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
