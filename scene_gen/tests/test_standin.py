"""The tilt-and-sink stand-in: who gets it, and that it never floats."""

import math
import os
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import disaster_stage as ds                       # noqa: E402


class _Resolver:
    """A footprint resolver that answers 80 x 30 m for everything."""

    def get(self, usd, cat, scale=1.0, axis_up="Z"):
        return {"sx": 80.0, "sy": 30.0, "sz": 60.0, "base": 0.0,
                "cx": 0.0, "cy": 0.0, "cz": 0.0}


def _house(i, damage, standin=(0.0, 6.0, 0.2)):
    return {"category": "house", "usd": f"b{i}.usd", "x_m": 10.0 * i,
            "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0, "scale": 1.0,
            "axis_up": "Z", "_mesh_damage": damage, "_standin": standin}


def test_only_buildings_beyond_the_budget_get_the_standin():
    ps = [_house(0, 0.9), _house(1, 0.5), _house(2, 0.7)]
    ds._apply_standins(ps, {"mesh_damage": {"max_buildings": 2}})
    assert all("_standin" not in p for p in ps)
    # 0.9 and 0.7 are cut and keep their true pose; 0.5 is the stand-in.
    assert ps[0]["z_m"] == 0.0 and ps[2]["z_m"] == 0.0
    assert ps[1]["z_m"] < 0.0 and ps[1]["pitch_deg"] == 6.0


def test_a_tilted_standin_sinks_far_enough_that_no_corner_floats():
    """A 6 degree pitch about the base centre of an 80 m building lifts one
    end 4.2 m; the sink has to cover it or the ruin levitates."""
    ps = [_house(0, 0.5)]
    # `max_buildings: 0` reads as unset (the config default is 40); mesh
    # damage switched off is how a scene has no cut budget at all.
    ds._apply_standins(ps, {"mesh_damage": {"enabled": False}},
                       resolver=_Resolver())
    lift = 0.5 * 80.0 * math.sin(math.radians(6.0))
    assert ps[0]["z_m"] <= -(0.2 + lift) + 1e-9
    assert ps[0]["z_m"] > -(0.2 + lift) - 1.0     # and not absurdly deeper
