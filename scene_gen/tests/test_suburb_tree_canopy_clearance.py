"""Offline regression tests for measured tree-crown clearance."""

import random
import textwrap
from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "suburb_scene.py").read_text()
START = SOURCE.index("class _CanopyPool:")
END = SOURCE.index("\ndef _size_t", START)
NS = {}
exec(textwrap.dedent(SOURCE[START:END]), NS)
CanopyPool = NS["_CanopyPool"]


class Resolver:
    widths = {"small": 4.0, "medium": 10.0, "large": 25.0}

    def get(self, path, *_args, **_kwargs):
        return {"sx": self.widths[path], "sy": self.widths[path] * 0.8}


class Pools:
    @staticmethod
    def scale_of(_path):
        return 1.0

    @staticmethod
    def axis_of(_path):
        return "Z"


def test_ranked_draw_falls_back_to_a_crown_that_fits_reserved_disc():
    pool = CanopyPool(Resolver(), Pools(), ["small", "medium", "large"])
    # At t=1 the ranked window contains only medium/large trees. The old code
    # selected either despite a parcel having reserved an 8 m-wide canopy.
    assert pool.draw(random.Random(4), 1.0, max_width=8.0) == "small"


def test_ranked_draw_refuses_station_when_no_measured_crown_fits():
    pool = CanopyPool(Resolver(), Pools(), ["small", "medium", "large"])
    assert pool.draw(random.Random(4), 0.0, max_width=3.0) is None


def test_ranked_draw_preserves_size_gradient_without_a_clearance_limit():
    pool = CanopyPool(Resolver(), Pools(), ["small", "medium", "large"])
    assert pool.draw(random.Random(4), 1.0) in {"medium", "large"}
