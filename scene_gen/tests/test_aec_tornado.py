#!/usr/bin/env python3
"""test_aec_tornado.py — does `disaster/aec_tornado.py` tornado-damage a
real AEC brownstone row correctly, offline?

    uv run --with pytest --with vtk --with usd-core --with numpy \\
        --with pyyaml --with trimesh -- \\
        python -m pytest -q scene_gen/tests/test_aec_tornado.py

RUNS WITHOUT ISAAC, against the REAL LOCAL ASSETS (`scene_gen/assets/aec/
brownstone/.../Reference_Brownstone{2,5}Row.usd` — fully local, relative
refs, no Nucleus). VTK is needed now (the guarded top-corner brick bite
clips the facade mesh via `aec_burn._clip_box`), no Kit. `trimesh` is used
to WELD coincident vertices and find true connected pieces in a merged
flown-debris mesh — every triangle this module authors is DE-INDEXED (no
two triangles share a vertex INDEX even when they share a vertex POSITION),
so `trimesh.Trimesh(..., process=True)` is required before `.split()` finds
the real pieces; `process=False` (or omitting it) would report one
"component" per triangle. Each stage build costs one `aec_burn.measure_row`
(~2-3 s on the 5Row).

WHAT THIS FILE CANNOT SEE: whether the MDL brick renders, whether the voided
windows read as a tornado scrape, whether the debris tones match. That is
the bench render's job.
"""
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import numpy as np                                     # noqa: E402
import pytest                                          # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdShade            # noqa: E402

from disaster import aec_burn as ab                    # noqa: E402
from disaster import aec_tornado as at                 # noqa: E402
from disaster import gac_fire as gcf                   # noqa: E402

CELL = "/World/cell"
#: the bench's E-row cell frame: seat yaw 90 on world bearing 57.56
BEARING_CELL = 57.56 - 90.0

_ASSET_2 = gcf.asset_url("Reference_Brownstone2Row", "aec")
_ASSET_5 = gcf.asset_url("Reference_Brownstone5Row", "aec")

needs_assets = pytest.mark.skipif(
    not (os.path.exists(_ASSET_2) and os.path.exists(_ASSET_5)),
    reason="local AEC brownstone assets not present")

#: categories NEVER touched by the tornado (the asset's own fit-out)
_FORBIDDEN_CATS = ("Floors", "Ceilings", "Casework", "Stairs", "Runs", "Supports")


def _stage_with_row(asset_url):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    scale = gcf.asset_scale(asset_url, gcf.PACKS["aec"]["scale"],
                            verbose=False)
    src = gcf.place_source(stage, CELL, asset_url, scale)
    assert src, "place_source composed nothing for {0}".format(asset_url)
    return stage, src + "/asset"


def _wreck(asset_url, level, seed=7, bearing=BEARING_CELL):
    stage, root = _stage_with_row(asset_url)
    wind = {"bearing_deg": bearing, "speed_frac": 0.8}
    stats = at.wreck_row(stage, CELL, root, level, wind,
                         random.Random(seed), verbose=False)
    return stage, root, stats


def _piece_zranges(mesh_prim):
    """Every CONNECTED piece's own `(min_z, max_z)` in one merged,
    de-indexed flown-debris mesh — see the module docstring's note on why
    `process=True` is required before `trimesh` can find them."""
    import trimesh
    me = UsdGeom.Mesh(mesh_prim)
    pts = np.asarray(me.GetPointsAttr().Get(), dtype=np.float64)
    fvc = np.asarray(me.GetFaceVertexCountsAttr().Get())
    fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get())
    assert len(fvc) and (fvc == 3).all(), "expected an all-triangle mesh"
    faces = fvi.reshape(-1, 3)
    tm = trimesh.Trimesh(vertices=pts, faces=faces, process=True)
    out = []
    for comp in tm.split(only_watertight=False):
        z = comp.vertices[:, 2]
        out.append((float(z.min()), float(z.max())))
    return out


#: every stat key the module may legitimately emit. Anything else — a
#: `floor`, `ceiling`, `casework`, `wall`, any `*_char` — would mean the
#: tornado touched the fit-out or the masonry beyond the guarded corner
#: bite, which it must never do.
ALLOWED_KEYS = {"glass", "glass_cracked", "sash", "door_glass", "roof_plant",
                "railing", "rail_flown", "deck_lumber", "cornice",
                "lumber_flown", "corner_bite_tris", "deinstanced", "n_units",
                "windward", "n_debris"}


@needs_assets
class TestT3FiveRow:
    @classmethod
    def setup_class(cls):
        cls.stage, cls.root, cls.stats = _wreck(_ASSET_5, "T3")
        cls.intact_stage, cls.intact_root = _stage_with_row(_ASSET_5)
        cls.intact = ab.measure_row(cls.intact_stage, cls.intact_root,
                                    verbose=False)

    def test_windward_is_the_street_front(self):
        # the asset's street front is side "W"; the bench's yaw-90 seat
        # rotates the world-S windward onto it
        assert self.stats["windward"] == "W"

    def test_glass_mostly_cracks_not_blows_out(self):
        # 2026-09-02 round 2: "can't really tell the windows are blown out
        # ... we'd rather have cracks there" -- a HIT pane defaults to the
        # cracked-glass look now; only the rare top-storey extreme case
        # (`LADDER["T3"]["blowout_top"]`) still fully blows out.
        assert self.stats.get("glass_cracked", 0) >= 40
        assert self.stats.get("glass", 0) < self.stats.get("glass_cracked", 0)
        assert self.stats.get("deinstanced", 0) >= 1
        extra = set(self.stats) - ALLOWED_KEYS
        assert not extra, "tornado touched forbidden parts: {0}".format(extra)

    def test_killed_prims_are_inactive_and_windward_heavy(self):
        # a FRESH measure of the same stage sees only what survived;
        # compare against a fresh intact measure for the kill census.
        # Blowouts (and the sash that comes off with one) are windward-only
        # by construction now, so REAR must be exactly 0.
        front = rear = 0
        killed = 0
        for unit in self.intact["units"]:
            for mrec in unit["meshes"]:
                if mrec["cat"] != "Windows":
                    continue
                prim = self.stage.GetPrimAtPath(mrec["path"])
                if prim and prim.IsValid() and not prim.IsActive():
                    killed += 1
                    b = mrec["bbox"]
                    if 0.5 * (b[0] + b[3]) < unit["cx"]:
                        front += 1
                    else:
                        rear += 1
        assert killed == self.stats.get("glass", 0) + self.stats.get("sash", 0)
        assert killed >= 1, "T3 seed=7 on the 5-row is expected to blow at least one pane"
        assert rear == 0, "a blowout must never happen on the leeward/rear side"
        assert front > rear, (
            "windward (front, W) must lose more than leeward: "
            "{0} vs {1}".format(front, rear))

    def test_cracked_panes_stay_active_with_the_cracked_look(self):
        # a pane that was HIT but did not blow out stays ACTIVE and wears
        # the dark translucent `aec_cracked_glass` material, never the
        # asset's own clean glass and never `_kill`ed
        n_checked = n_cracked = 0
        for unit in self.intact["units"]:
            for mrec in unit["meshes"]:
                if mrec["cat"] != "Windows":
                    continue
                if not (mrec["mat"].startswith("Clear_Glass")
                        or mrec["mat"] == ""):
                    continue
                prim = self.stage.GetPrimAtPath(mrec["path"])
                if not (prim and prim.IsValid()):
                    continue
                n_checked += 1
                if not prim.IsActive():
                    continue                      # the rare blowout
                mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                if mat and mat.GetPath().name == "aec_cracked_glass":
                    n_cracked += 1
        assert n_checked > 0
        assert n_cracked == self.stats.get("glass_cracked", 0)

    def test_glass_crack_lines_authored_under_the_building(self):
        # the standoff crack decal is ONE merged mesh under the BUILDING's
        # own root (identity-frame authoring: it has to move with the
        # window it is stuck to when the launcher seats the holder), bound
        # to the bright `aec_crack_line` look
        mesh = self.stage.GetPrimAtPath(self.root + "/TornadoCracks/glass_cracks")
        assert mesh and mesh.IsValid()
        pts = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        assert len(pts) > 0
        mat, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        assert mat and mat.GetPath().name == "aec_crack_line"
        assert str(mesh.GetPath()).startswith(self.root + "/"), (
            "a window decal must live under the BUILDING, not the cell")

    def test_no_forbidden_fitout_touched(self):
        # Floors/Ceilings/Casework/Stairs must all still be ACTIVE
        n_checked = 0
        for unit in self.intact["units"]:
            for mrec in unit["meshes"]:
                if mrec["cat"] not in _FORBIDDEN_CATS:
                    continue
                prim = self.stage.GetPrimAtPath(mrec["path"])
                assert prim and prim.IsValid() and prim.IsActive(), (
                    "tornado touched fit-out part {0} ({1})".format(
                        mrec["path"], mrec["cat"]))
                n_checked += 1
        assert n_checked > 0, "no fit-out parts found to check"

    def test_all_windward_railings_gone(self):
        # 2026-09-02 escalation: EVERY Railings/Top_Rails part (stoop AND
        # roof) on the windward elevation is gone, not a coin-flip.
        # "On the windward elevation" is PLANE proximity, matching the
        # module's own `RAIL_PLANE_TOL_M` test -- a stoop rail's SHAPE
        # (long across the wall, thin along the row) makes `_part_side`
        # (the window classifier) misread it as an S/N party-line part.
        perp = self.intact["perp"]
        windward = self.stats["windward"]
        n_windward = n_gone = 0
        for unit in self.intact["units"]:
            plane = (unit["plane_lo"] if windward in ("W", "S")
                     else unit["plane_hi"])
            for mrec in unit["meshes"]:
                cat = mrec["cat"]
                if not (cat.startswith("Railings") or cat.startswith("Top_Rails")):
                    continue
                b = mrec["bbox"]
                c_perp = 0.5 * (b[perp] + b[3 + perp])
                on_windward = ((c_perp <= plane + at.RAIL_PLANE_TOL_M)
                              if windward in ("W", "S") else
                              (c_perp >= plane - at.RAIL_PLANE_TOL_M))
                if not on_windward:
                    continue
                n_windward += 1
                prim = self.stage.GetPrimAtPath(mrec["path"])
                if prim and prim.IsValid() and not prim.IsActive():
                    n_gone += 1
        assert n_windward > 0, "no windward railings found in this asset"
        assert n_gone == n_windward, (
            "{0} of {1} windward railings survived".format(
                n_windward - n_gone, n_windward))

    def test_removed_railings_are_scattered_not_vanished(self):
        # 2026-09-02 round 2: "railings are gone but have them scattered or
        # something not just disappeared" -- every removed railing is FLOWN
        # as its own real mesh, exactly like the deck lumber
        assert self.stats.get("rail_flown", 0) > 0
        mesh = self.stage.GetPrimAtPath(CELL + "/tornado_debris/aec_railing_metal")
        assert mesh and mesh.IsValid()
        pts = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        assert len(pts) > 12 * self.stats["rail_flown"], (
            "flown railing reads like proxy boxes, not real member meshes")
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(mesh).ComputeAlignedRange()
        mn = r.GetMin()
        assert mn[2] >= -0.02, "flown railing sunk below the ground: {0}".format(mn[2])
        mat, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        assert mat, "flown railing has no material"

    def test_cornice_stripped_but_deck_membrane_stands(self):
        assert self.stats.get("cornice", 0) > 0
        # the big-footprint roof piece (the deck membrane) must NEVER be
        # touched -- only small-footprint Roofs pieces near the windward
        # plane may be killed
        for unit in self.intact["units"]:
            roofs = [(m, abs((m["bbox"][3] - m["bbox"][0]) *
                             (m["bbox"][4] - m["bbox"][1])))
                     for m in unit["meshes"] if m["cat"].startswith("Roofs")]
            if not roofs:
                continue
            deck_mesh, _area = max(roofs, key=lambda t: t[1])
            prim = self.stage.GetPrimAtPath(deck_mesh["path"])
            assert prim and prim.IsValid() and prim.IsActive(), (
                "the deck membrane {0} was touched".format(deck_mesh["path"]))

    def test_corner_bite_is_guarded_to_one_unit_and_reads_bigger(self):
        # at most one unit's facade gets bitten, and the shell keeps
        # standing: bbox height of the bitten wall ~= the intact wall's.
        # 2026-09-02 round 2 ("no missing chunks or brick bite"): the bite
        # is now ~1.5 storeys tall and wider (`BITE_HEIGHT_STOREYS`,
        # `BITE_WIDTH_FRAC`) -- the tri count removed is well up from the
        # old one-storey/narrower guard.
        assert self.stats.get("corner_bite_tris", 0) >= 40, (
            "the corner bite should read as a real missing chunk, not a sliver")
        bite_scope = self.stage.GetPrimAtPath(self.root + "/TornadoBite")
        assert bite_scope and bite_scope.IsValid()
        children = list(bite_scope.GetChildren())
        assert len(children) == 1, (
            "corner bite touched {0} units, expected 1".format(len(children)))
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bitten = bc.ComputeWorldBound(children[0]).ComputeAlignedRange()
        bn, bx = bitten.GetMin(), bitten.GetMax()
        # the matching intact unit's own Walls_Exterior mesh
        tag = children[0].GetName().rsplit("_", 1)[-1]
        unit = next(u for u in self.intact["units"]
                   if u["name"].rsplit("_", 1)[-1] == tag)
        wall = next(m for m in unit["meshes"] if m["cat"] == "Walls_Exterior")
        ih = wall["bbox"][5] - wall["bbox"][2]
        bh = bx[2] - bn[2]
        assert abs(bh - ih) < 0.1, (
            "bitten wall height {0:.3f} vs intact {1:.3f} -- reads like a "
            "collapse, not a corner bite".format(bh, ih))
        # every OTHER unit's own facade must be completely untouched
        for u in self.intact["units"]:
            if u["name"].rsplit("_", 1)[-1] == tag:
                continue
            w = next((m for m in u["meshes"] if m["cat"] == "Walls_Exterior"),
                     None)
            if w is None:
                continue
            prim = self.stage.GetPrimAtPath(w["path"])
            assert prim and prim.IsValid() and prim.IsActive(), (
                "corner bite touched more than one unit: {0}".format(u["name"]))

    def test_flown_lumber_is_real_geometry_on_the_ground(self):
        assert self.stats.get("lumber_flown", 0) > 0
        mesh = self.stage.GetPrimAtPath(CELL + "/tornado_debris/aec_flown_lumber")
        assert mesh and mesh.IsValid()
        pts = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        # a real flown member (not a small proxy board box) has FAR more
        # than the 8-corners-per-box a `_author_boxes` population would
        assert len(pts) > 12 * self.stats["lumber_flown"], (
            "flown lumber reads like proxy boxes, not real member meshes")
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(mesh).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        assert mn[2] >= -0.02, "flown lumber sunk below the ground: {0}".format(mn[2])
        # 2026-09-02 round 2: the OLD unbounded tumble (up to 90 degrees)
        # stood a 2-3 m board on end (bbox z 2.69 m, live-render review).
        # `LUMBER_TUMBLE_DEG` is capped low now -- the whole population's
        # bbox must read nowhere near that.
        assert mx[2] < 2.2, (
            "flown lumber reads as standing/impaled, not settled: "
            "bbox top at {0:.2f} m".format(mx[2]))

    def test_every_flown_piece_is_grounded_individually(self):
        # 2026-09-02 round 2: "the rear lumber is floating" -- a merged
        # mesh's GLOBAL min touching ~0 is not enough proof if only ONE
        # piece in the population actually reaches the ground. Every
        # CONNECTED piece (found by welding coincident vertices, since none
        # of these triangles share an index) must have its OWN low point at
        # the ground.
        for name in ("aec_flown_lumber", "aec_railing_metal"):
            mesh = self.stage.GetPrimAtPath(CELL + "/tornado_debris/" + name)
            assert mesh and mesh.IsValid()
            zranges = _piece_zranges(mesh)
            assert len(zranges) > 1, "{0}: expected more than one piece".format(name)
            for zmin, zmax in zranges:
                assert zmin <= 0.02, (
                    "{0}: a piece's own low point floats at {1:.3f}".format(
                        name, zmin))

    def test_debris_seated_not_floating(self):
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_])
        scope = self.stage.GetPrimAtPath(CELL + "/tornado_debris")
        assert scope and scope.IsValid() and self.stats["n_debris"] > 0
        deck_cap = 16.0                      # rows are 14.6-15.3 m tall
        # a tumbled timber member or railing run can stand at a shallow
        # lean once dropped near one end -- it is grounded, not floating,
        # so it gets a taller allowance than the flat box debris
        # populations. Both are well down from their pre-fix highs (a
        # standing-on-end board used to reach ~2.7 m; these caps still
        # catch a regression back toward that without being flaky).
        tall_ok = {"aec_bricks": deck_cap, "aec_flown_lumber": 2.5,
                   "aec_railing_metal": 6.5}
        for child in scope.GetChildren():
            r = bc.ComputeWorldBound(child).ComputeAlignedRange()
            assert not r.IsEmpty()
            mn, mx = r.GetMin(), r.GetMax()
            assert mn[2] >= -0.02, "{0} sunk {1}".format(
                child.GetName(), mn[2])
            cap = tall_ok.get(child.GetName(), 1.0)
            assert mx[2] <= cap, "{0} floating at {1}".format(
                child.GetName(), mx[2])

    def test_debris_materials_bound(self):
        scope = self.stage.GetPrimAtPath(CELL + "/tornado_debris")
        for child in scope.GetChildren():
            mat, _ = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()
            assert mat, "{0} has no material".format(child.GetName())
            # every ground-debris look lives under the CELL so
            # `_seat_holder` moves it too
            assert str(mat.GetPath()).startswith(CELL + "/"), mat.GetPath()


@needs_assets
def test_t2_is_glass_only():
    _stage, _root, stats = _wreck(_ASSET_2, "T2")
    assert stats.get("glass_cracked", 0) >= 1
    assert stats.get("glass", 0) == 0            # T2 never blows a pane out
    assert stats.get("sash", 0) == 0
    assert stats.get("roof_plant", 0) == 0
    assert stats.get("deck_lumber", 0) == 0
    assert stats.get("cornice", 0) == 0
    assert stats.get("corner_bite_tris", 0) == 0
    assert stats.get("rail_flown", 0) == 0
    extra = set(stats) - ALLOWED_KEYS
    assert not extra


@needs_assets
def test_deterministic():
    _s1, _r1, a = _wreck(_ASSET_2, "T3", seed=11)
    _s2, _r2, b = _wreck(_ASSET_2, "T3", seed=11)
    assert a == b


@needs_assets
def test_deterministic_flown_lumber_geometry():
    # the stats dict alone doesn't prove the AUTHORED geometry is
    # byte-identical -- check the flown-lumber mesh's own point array too
    s1, r1, a = _wreck(_ASSET_5, "T4", seed=11)
    s2, r2, b = _wreck(_ASSET_5, "T4", seed=11)
    assert a == b
    m1 = UsdGeom.Mesh(s1.GetPrimAtPath(CELL + "/tornado_debris/aec_flown_lumber"))
    m2 = UsdGeom.Mesh(s2.GetPrimAtPath(CELL + "/tornado_debris/aec_flown_lumber"))
    p1, p2 = m1.GetPointsAttr().Get(), m2.GetPointsAttr().Get()
    assert len(p1) == len(p2) and len(p1) > 0
    assert all(tuple(x) == tuple(y) for x, y in zip(p1, p2))


@needs_assets
def test_region_clamp(monkeypatch):
    monkeypatch.setenv("TU_PLATE_REGION", "-20,-20,20,20")
    stage, _root, stats = _wreck(_ASSET_5, "T4")
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    scope = stage.GetPrimAtPath(CELL + "/tornado_debris")
    for child in scope.GetChildren():
        if child.GetName() == "aec_bricks":
            continue                         # deck bricks sit on the roof
        r = bc.ComputeWorldBound(child).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        # box half-diagonal (or a flown 2.7 m timber's half-length, or a
        # long flown railing run) can poke past the clamped CENTRE by a
        # few metres
        assert mn[0] >= -22 and mx[0] <= 22, child.GetName()
        assert mn[1] >= -22 and mx[1] <= 22, child.GetName()


def test_side_weights_convention():
    """The bench's own measured case: world bearing 57.56 makes S the
    windward side at yaw 0 (the launcher's pinned comment), and the E-row
    cell frame (bearing - 90) makes the street front W windward."""
    w0 = at.side_weights(57.56)
    assert max(w0, key=w0.get) == "S"
    w90 = at.side_weights(57.56 - 90.0)
    assert max(w90, key=w90.get) == "W"


def test_fly_piece_grounds_every_call_and_caps_the_tumble():
    """Direct, asset-independent unit test of the 2026-09-02 round-2 fix:
    every `_fly_piece` call sinks its OWN piece to `SINK_M`, and the capped
    `LUMBER_TUMBLE_DEG` upper bound keeps a long board from ever reading as
    standing on end (the old bug: a 2-3 m board tumbled up to 90 degrees
    reached bbox z 2.69 m)."""
    length, w, t = 2.7, 0.09, 0.04
    V, _N = ab._box_tris((0.0, 0.0, 5.0), (length, w, t), 0.0, 0.0)
    P = V.reshape(-1, 3)
    md = {"P": P, "tris": np.arange(P.shape[0]).reshape(-1, 3),
         "UV": np.zeros((P.shape[0], 2))}
    rng = random.Random(0)
    wdir = (1.0, 0.0)
    worst_top = 0.0
    for _ in range(300):
        out = at._fly_piece(md, rng, wdir, 0.8, None)
        Pr = out["P"]
        zmin, zmax = float(Pr[:, 2].min()), float(Pr[:, 2].max())
        assert abs(zmin - (-at.SINK_M)) < 1e-6, (
            "not grounded on its own: min z {0}".format(zmin))
        worst_top = max(worst_top, zmax)
    bound = (length * math.sin(math.radians(at.LUMBER_TUMBLE_DEG[1]))
            + t * math.cos(math.radians(at.LUMBER_TUMBLE_DEG[1])) + 0.05)
    assert worst_top <= bound, (
        "{0:.2f} exceeds the flat-tumble bound {1:.2f}".format(worst_top, bound))
    assert worst_top < 2.0, (
        "a 2.7 m board must never read as standing on end (got {0:.2f} m)".format(
            worst_top))


def test_bite_height_covers_about_one_and_half_storeys():
    """The corner bite's own `z0`/`z1` arithmetic (mirrored here from
    `_windward_corner_bite`), on a synthetic 4-storey unit: it must reach
    meaningfully more than one storey down from the deck, capped so it
    never eats a second full storey."""
    levels = [0.0, 3.1, 6.5, 9.7]
    deck_z = 12.9
    story_h = levels[-1] - levels[-2]
    z0 = max(levels[-2], deck_z - at.BITE_HEIGHT_STOREYS * story_h)
    span_h = deck_z - z0
    assert span_h > story_h * 1.2, "the bite should read taller than one storey"
    assert z0 >= levels[-2] - 1e-9, "the bite must never eat a second full storey"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
