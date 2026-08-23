"""The graph suburb, built onto a real USD stage, on the host.

WHY THIS EXISTS
---------------
`tools/fence_check.py` drives `suburb_net` / `suburb_parcel` / `build_placements`
with USD stubbed out, and says so: it skips "the park, the yard planting, the
frontage props and the ground". Everything it skips is exactly the half that
needs a stage — and that half was reached for the first time in Isaac Sim, four
minutes into a boot, as a `TypeError`.

Two of those landed in one run:

    _Budget.__init__() takes 4 positional arguments but 5 were given
    apply_placements() got an unexpected keyword argument 'instance_categories'

Both are the same shape: a module ported from another branch calling a shared
helper whose signature differs here. Neither is findable by importing, linting
or type-checking the files — only by *running* the call. So this runs it, in
0.8 s, against `Usd.Stage.CreateInMemory()`.

WHAT IT DOES NOT CHECK
----------------------
Nothing about how the scene LOOKS. Every `omniverse://` reference fails to
compose here because usd-core has no Omniverse resolver and Nucleus is not
reachable from the host — ~700 of them, which is expected and not a failure.
What is asserted is that the code path completes, authors prims, and resolves
every asset it claims to own LOCALLY. A local asset that goes missing — the
Megascans `.usda` wrappers did — is a real defect this catches.
"""

import os
import sys

import pytest

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TESTS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

pytest.importorskip("pxr", reason="needs usd-core (system python3, not .venv)")


def _build(preset: str):
    """Build *preset* onto an in-memory stage. Returns (placements, stage, log)."""
    import io
    import contextlib
    from pxr import Usd, UsdGeom

    import suburb_scene as ss
    from compile_disaster import load_scene_config

    cfg = load_scene_config(preset)
    # No Nucleus round-trips for asset measurement; the sizes fall back to the
    # config's own `fallback_sizes`, which is what the stub resolver uses too.
    cfg["measure_usds"] = False

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        placements = ss.generate_suburb_on_stage(
            stage, cfg, "/World/generated", 1.0, False)
    return placements, stage, buf.getvalue()


# The 250 x 250 m corner. Deliberately the small one: it exercises every pass
# the full plat does — net, parcels, yards, park, ground, markings, placement —
# in under a second, so it can sit in the default suite.
PRESET = "suburb_mini_wildfire"


@pytest.fixture(scope="module")
def built():
    return _build(PRESET)


def test_generate_suburb_on_stage_runs(built):
    """The whole stage path completes. This is the TypeError tripwire."""
    placements, _stage, _log = built
    assert placements, "no placements returned"


def test_authors_prims(built):
    _placements, stage, _log = built
    prims = list(stage.Traverse())
    assert len(prims) > 1000, f"only {len(prims)} prims authored"


def test_every_local_asset_resolves(built):
    """No local asset may be missing.

    `omniverse://` references cannot compose without Nucleus and are ignored.
    Anything else that fails to open is a file the repo is supposed to carry —
    which is how the generated Megascans `.usda` wrappers were found missing
    while their textures were committed.
    """
    placements, _stage, _log = built
    local = {p["usd"] for p in placements
             if p.get("usd") and not str(p["usd"]).startswith("omniverse:")}
    missing = sorted(u for u in local if not os.path.exists(u))
    assert not missing, "local assets referenced but not on disk:\n  " + \
                        "\n  ".join(missing[:10])


def test_instancing_is_applied(built):
    """`instance_categories` reaches `apply_placements` and does something.

    The preset lists the categories that do not burn. If the keyword were
    dropped again the scene would still build — just at full point cost — so
    assert the count rather than trusting the call.
    """
    _placements, _stage, log = built
    assert "instanced" in log, \
        "apply_placements reported no instancing; instance_categories lost?"
