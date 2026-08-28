#!/usr/bin/env python3
"""
test_tornado_logs.py — the tree debris a tornado leaves, pinned without Isaac.

Two reports off the first assembled 100 x 100 m plate, and they are two
different bugs that happen to be about the same objects:

    "the logs that are thrown around look like they're floating and also
     their material looks to be the 'burnt forest floor' one. I can't have
     that. I want it to be one of the plain wood materials."

THE MATERIAL. Nothing bound the burnt forest floor. What was bound — measured
straight out of `assets/archetypes_tornado/tree_*.usd` — was each tree's OWN
bark diffuse routed through `damage._pbr`, which carries a diffuse map and
NOTHING ELSE, at `texture_scale = (0.3, 0.3)`: repeats per metre under world
triplanar, so one tile per 3.33 m. On a 0.2 x 1.5 m stick that is a 6% x 45%
crop of one photograph with no normal map to give the cylinder any relief, and
the photograph in question was a lottery — Douglas_Fir drew `alter49_tree10`
at mean luma 0.195 (charcoal), Largetooth_Aspen drew `alter49_tree4` at 0.535
with sd 0.223 (white birch bark with 40 cm black blotches). A mottled dark
patch is exactly what "burnt forest floor" describes.

THE FLOATING. There is no z bug in the bake: every `log*` mesh has a world
minimum below zero, so inside its own archetype each piece is seated. What is
missing is GROUND. `suburb_scene.apply_ground` lays its sheet over exactly
`region`, and a tree archetype throws debris up to 26.7 m from the trunk, so
any tree within a reach of the boundary puts part of its bed over the void.
`scour_relief.clip_to_region` fixes the same defect for spoil heaps by dropping
the feature; a tree cannot be trimmed, because it is referenced as an INSTANCE
and USD forbids authoring inside one. So the levers are WHICH archetype and
WHICH YAW, and `tornado.tree_level_and_yaw` uses them in that order of
preference.

Six claims are worth pinning, because every one is invisible in a render until
it is wrong and then it costs a re-bake to find out:

  1. THE MAPS ARE REAL AND THE NORMAL IS THERE. Every texture a log material
     binds exists on disk, and both surfaces carry a normal map — an OmniPBR
     input pointed at a missing file renders as if it were absent rather than
     failing, so "no normal map" and "a normal map that is not there" look
     identical from a render and identical in the USD.
  2. THE TILE IS METRIC AND SIZED TO A LIMB, not to a plate.
  3. THE TORNADO SURFACES ARE PALE AND THE WILDFIRE ONE IS DARK. A wind event
     photographs light against green grass; a burn scar photographs dark. The
     two paths share `bark_material` and are separated only by their tint, so
     the tint is the thing to pin.
  4. `plain_wood` — the fallback that was the bug — now finds the normal and
     roughness maps that were sitting unread beside every bark diffuse in this
     library, for all five diffuses the archetypes actually use.
  5. BARK GOES ON THE PIECES THAT HAVE BARK. `wood_debris` cuts riven columns
     out of the bole, not branches off the outside, so most pieces never touch
     the bole surface; the share is derived from the real draw, sliced out of
     the module so the two cannot drift apart.
  6. NOTHING IS PLACED OFF THE PLATE, the drawn damage level is kept wherever a
     yaw can save it, and the ladder is only walked down as a last resort.

RUNS WITHOUT ISAAC. `disaster/vegetation.py`, `disaster/planks.py` and
`disaster/tornado.py` all import `pxr` and `scene_generator` INSIDE the
functions that need them, so the modules import clean; this file installs a
recording stub for both under those names and then calls the REAL material
functions, which is why it can assert on what they author rather than on a
copy of it. No stage, no GPU, no Isaac Sim.

USAGE
    python3 scene_gen/tests/test_tornado_logs.py
    pytest -s scene_gen/tests/test_tornado_logs.py
"""

import ast
import inspect
import math
import os
import random
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


# ---------------------------------------------------------------------------
# the recording stubs
#
# NOT A REIMPLEMENTATION OF THE MATERIALS — the point of stubbing rather than
# slicing is that the REAL `bark_material` / `split_wood_material` /
# `plain_wood` bodies run, so a change to any of them shows up here instead of
# quietly diverging from a copy. `pxr` and `scene_generator` are both imported
# inside those functions, which is what makes the substitution possible at all.
# ---------------------------------------------------------------------------

class _Path(str):
    def AppendChild(self, name):
        return _Path(self.rstrip("/") + "/" + str(name))


class _AssetPath(object):
    def __init__(self, path):
        self.path = str(path)

    def __repr__(self):
        return "@%s@" % self.path


class _Types(object):
    """`Sdf.ValueTypeNames.<anything>` — the name is never inspected."""

    def __getattr__(self, name):
        return "type:" + name


class _Out(object):
    def ConnectToSource(self, *a, **k):
        return True


class _Input(object):
    def __init__(self, bag, name):
        self._bag, self._name = bag, name

    def Set(self, v):
        self._bag[self._name] = v
        return True

    def Get(self):
        return self._bag.get(self._name)


class _Shader(object):
    def __init__(self, path):
        self.path = str(path)
        self.inputs = {}
        self.shader_id = None

    def CreateIdAttr(self, v):
        self.shader_id = v

    def SetSourceAsset(self, *a):
        return True

    def SetSourceAssetSubIdentifier(self, *a):
        return True

    def CreateInput(self, name, _type):
        self.inputs.setdefault(name, None)
        return _Input(self.inputs, name)

    def GetInput(self, name):
        return _Input(self.inputs, name)

    def ConnectableAPI(self):
        return self


class _Material(object):
    def __init__(self, path):
        self.path = str(path)

    def CreateSurfaceOutput(self, *a):
        return _Out()

    CreateDisplacementOutput = CreateSurfaceOutput
    CreateVolumeOutput = CreateSurfaceOutput


_SHADERS = {}


def _install_stubs():
    sdf = types.ModuleType("pxr.Sdf")
    sdf.Path = _Path
    sdf.AssetPath = _AssetPath
    sdf.ValueTypeNames = _Types()

    gf = types.ModuleType("pxr.Gf")
    gf.Vec2f = lambda *a: tuple(float(v) for v in a)
    gf.Vec3f = lambda *a: tuple(float(v) for v in a)
    gf.Vec3d = gf.Vec3f

    def _shader_define(_stage, path):
        sh = _Shader(path)
        _SHADERS[str(path)] = sh
        return sh

    usdshade = types.ModuleType("pxr.UsdShade")
    usdshade.Material = types.SimpleNamespace(
        # ALWAYS MISSING. Every one of these functions short-circuits on an
        # existing material at the same path, and a stub that hands one back
        # would make the test assert on nothing at all.
        Get=lambda _stage, _path: None,
        Define=lambda _stage, path: _Material(path))
    usdshade.Shader = types.SimpleNamespace(Define=_shader_define)
    usdshade.MaterialBindingAPI = lambda *a, **k: types.SimpleNamespace(
        Bind=lambda *a, **k: True)

    pxr = types.ModuleType("pxr")
    pxr.Sdf, pxr.Gf, pxr.UsdShade = sdf, gf, usdshade
    for name, mod in (("pxr", pxr), ("pxr.Sdf", sdf), ("pxr.Gf", gf),
                      ("pxr.UsdShade", usdshade)):
        sys.modules[name] = mod

    # `airstack://` resolves against the repo root exactly as
    # `scene_generator._expand_scheme` does, so `os.path.exists` on the result
    # is a real answer about a real file rather than about a stub's guess.
    sg = types.ModuleType("scene_generator")
    sg._join_asset_root = lambda p, _root: (
        os.path.join(_REPO, str(p).split("://", 1)[1].lstrip("/"))
        if "://" in str(p) and str(p).startswith("airstack://") else str(p))
    sys.modules["scene_generator"] = sg


_install_stubs()

from disaster import tornado as tn                     # noqa: E402
from disaster import vegetation as veg                 # noqa: E402


def _author(fn, path, **kw):
    """Run a material function and hand back the shader inputs it authored."""
    fn(None, path, **kw)
    return _SHADERS[path + "/Shader"].inputs


def _asset(v):
    return v.path if isinstance(v, _AssetPath) else (v or "")


# Mean linear RGB of each diffuse over the whole sheet, measured once with PIL
# at 512 x 512 (2026-08-27) and pinned here so the test needs no image library.
# Reproduce with:
#     a = np.asarray(Image.open(p).convert("RGB").resize((512, 512))) / 255
#     a[..., 0].mean(), a[..., 1].mean(), a[..., 2].mean()
_MEAN_RGB = {
    "bark_oak_diff.jpg":            (0.4785, 0.4359, 0.3937),   # luma 0.442
    "pine_diff.jpg":                (0.7075, 0.5981, 0.4965),   # luma 0.614
    "alter49_tree10_basecolor.png": (0.2113, 0.1927, 0.1717),   # luma 0.195
    "bark3_basecolor.png":          (0.3459, 0.3222, 0.2853),   # luma 0.324
    "alter49_tree7_basecolor.png":  (0.3862, 0.3576, 0.3268),   # luma 0.362
    "bark2_basecolor.png":          (0.3879, 0.3255, 0.2551),   # luma 0.334
    "alter49_tree4_basecolor.png":  (0.5410, 0.5330, 0.5170),   # luma 0.535
}

# What the wildfire path does to the SAME bark surface — `scene_api`'s
# `_load_burnt_wood`. Reproduced here rather than imported because `scene_api`
# pulls in `scene_generator` for real.
_BURNT_TINT = (0.30, 0.26, 0.23)


def _luma(rgb):
    return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]


def _tinted_luma(basename, tint):
    r, g, b = _MEAN_RGB[basename]
    return _luma((r * tint[0], g * tint[1], b * tint[2]))


# ---------------------------------------------------------------------------
# 1 + 2 + 3: the two log surfaces
# ---------------------------------------------------------------------------

def test_log_materials():
    print("\n[1] the log surfaces: real maps, a normal, a metric tile")

    bark = _author(veg.bark_material, "/W/Looks/bark",
                   tile_m=tn.LOG_BARK_TILE_M, tint=tn.LOG_BARK_TINT)
    split = _author(veg.split_wood_material, "/W/Looks/split",
                    tile_m=tn.LOG_SPLIT_TILE_M, tint=tn.LOG_SPLIT_TINT)

    for name, inp, want_rough in (("bark", bark, False),
                                  ("split", split, True)):
        maps = [("diffuse", _asset(inp.get("diffuse_texture"))),
                ("normal", _asset(inp.get("normalmap_texture")))]
        if want_rough:
            maps.append(("roughness",
                         _asset(inp.get("reflectionroughness_texture"))))
        for role, p in maps:
            check(bool(p) and os.path.exists(p),
                  "%s %s map is bound AND on disk (%s)"
                  % (name, role, os.path.basename(p) or "<unbound>"))

    # THE ONE THAT WOULD HAVE CAUGHT THE ORIGINAL BUG. `damage._pbr` authors a
    # diffuse and nothing else, so a log rendered through it is a cylinder lit
    # by one sun with no surface variation at all — "painted pipe", in
    # `planks.wood_material`'s words.
    check("normalmap_texture" in bark and "normalmap_texture" in split,
          "BOTH surfaces carry a normal map (the whole defect: `_pbr` "
          "carries diffuse only)")

    # The bark set's third map is NOT an ORM and must stay unbound — its
    # channels are R=rough, G=ao, B=height against OmniPBR's R=ao, G=rough,
    # B=metallic, so binding it drives METALLIC off a height map.
    check("ORM_texture" not in bark and "enable_ORM_texture" not in bark,
          "the mis-ordered `bark_oak_multi` sheet is NOT bound as an ORM")

    print("\n[2] the tile is repeats per metre, and it is sized to a limb")
    for name, inp, tile in (("bark", bark, tn.LOG_BARK_TILE_M),
                            ("split", split, tn.LOG_SPLIT_TILE_M)):
        sc = inp.get("texture_scale")
        check(inp.get("project_uvw") is True
              and inp.get("world_or_object") is True,
              "%s is world-triplanar (generated debris carries no UVs)" % name)
        check(sc is not None and abs(sc[0] - 1.0 / tile) < 1e-6
              and abs(sc[1] - sc[0]) < 1e-9,
              "%s texture_scale is 1/tile_m = %.3f repeats/m (%.2f m a tile)"
              % (name, 1.0 / tile, tile))
    # `bark_oak_diff` carries ~52 furrow ridges across the sheet and
    # `pine_diff` ~11 board runs; at these tiles that is the feature size a
    # broken LIMB actually has. The old 3.33 m tile put a bark ridge at 6.4 cm
    # — mature-trunk scale — on a 0.2 m stick.
    ridge_cm = 100.0 * tn.LOG_BARK_TILE_M / 52.0
    board_cm = 100.0 * tn.LOG_SPLIT_TILE_M / 11.0
    print("      bark ridge %.1f cm a furrow, split grain %.1f cm a board; "
          "the old (0.3, 0.3) put them at %.1f / %.1f cm"
          % (ridge_cm, board_cm, 100.0 * 3.333 / 52.0, 100.0 * 3.333 / 11.0))
    check(1.5 <= ridge_cm <= 4.0,
          "a bark furrow lands at limb scale, 1.5-4 cm (%.1f)" % ridge_cm)
    check(4.0 <= board_cm <= 10.0,
          "a grain run lands at 4-10 cm (%.1f), so a 0.10-0.30 m stick shows "
          "two to four of them across its face" % board_cm)

    print("\n[3] the tornado is PALE and the wildfire is DARK")
    l_bark = _tinted_luma("bark_oak_diff.jpg", tn.LOG_BARK_TINT)
    l_split = _tinted_luma("pine_diff.jpg", tn.LOG_SPLIT_TINT)
    l_burnt = _tinted_luma("bark_oak_diff.jpg", _BURNT_TINT)
    print("      tinted mean luma — bark %.3f, split %.3f, wildfire %.3f"
          % (l_bark, l_split, l_burnt))
    check(l_bark > 0.45 and l_split > 0.58,
          "both tornado surfaces sit above 0.45 luma (breaking exposes the "
          "pale inside of what it breaks)")
    check(l_burnt < 0.20 and l_bark > 3.0 * l_burnt,
          "the wildfire tint on the same sheet is under 0.20 and more than "
          "three times darker — the two paths cannot be confused")
    # The per-species diffuses the archetypes bind today, for the record: the
    # spread is what makes a single shared surface the right answer.
    lo = min(_luma(_MEAN_RGB[k]) for k in _MEAN_RGB
             if k not in ("bark_oak_diff.jpg", "pine_diff.jpg"))
    hi = max(_luma(_MEAN_RGB[k]) for k in _MEAN_RGB
             if k not in ("bark_oak_diff.jpg", "pine_diff.jpg"))
    print("      the per-species diffuses they replace span luma %.3f "
          "(Douglas_Fir) to %.3f (Largetooth_Aspen) — a %.1fx range"
          % (lo, hi, hi / lo))
    check(hi / lo > 2.0,
          "the per-species lottery really is a >2x value range, which is why "
          "one shared surface replaces it")


# ---------------------------------------------------------------------------
# 4: the fallback that was the bug
# ---------------------------------------------------------------------------

_ARCHETYPE_DIFFUSES = (
    ("Douglas_Fir", "aec/brownstone/Assets/Vegetation/Trees/materials/"
                    "textures/alter49_tree10_basecolor.png"),
    ("American_Beech", "aec/brownstone/Assets/Vegetation/Trees/materials/"
                       "textures/bark3_basecolor.png"),
    ("Largetooth_Aspen", "aec/brownstone/Assets/Vegetation/Trees/materials/"
                         "textures/alter49_tree4_basecolor.png"),
    ("Black_Oak", "aec/tower/Assets/Vegetation/Black_Oak/materials/"
                  "textures/alter49_tree7_basecolor.png"),
    ("Common_Apple", "aec/tower/Assets/Vegetation/Common_Apple/materials/"
                     "textures/bark2_basecolor.png"),
)


def test_plain_wood_finds_its_maps():
    print("\n[4] `plain_wood` recovers the maps that were sitting unread")
    # Every diffuse below was read straight out of a baked archetype's log
    # material, so this is the exact set the fallback path has to handle.
    for species, rel in _ARCHETYPE_DIFFUSES:
        tex = os.path.join(_SCENE_GEN, "assets", rel)
        if not os.path.exists(tex):
            check(False, "%s diffuse missing from the asset tree (%s)"
                  % (species, rel))
            continue
        # `plain_wood` names the prim off an md5 of the texture rather than
        # taking a path, so the shader is found by what it bound.
        veg.plain_wood(None, "/W/Wind", texture=tex, tile_m=1.1)
        got = [v for k, v in _SHADERS.items()
               if k.startswith("/W/Wind/WindLooks/wood_")
               and _asset(v.inputs.get("diffuse_texture")) == tex]
        check(len(got) == 1,
              "%s: exactly one material authored, at a stable md5-keyed path"
              % species)
        if not got:
            continue
        inp = got[-1].inputs
        nrm = _asset(inp.get("normalmap_texture"))
        rgh = _asset(inp.get("reflectionroughness_texture"))
        check(bool(nrm) and os.path.exists(nrm),
              "%s: normal map found beside the diffuse (%s)"
              % (species, os.path.basename(nrm) or "<none>"))
        check(bool(rgh) and os.path.exists(rgh),
              "%s: roughness map found beside the diffuse (%s)"
              % (species, os.path.basename(rgh) or "<none>"))
        sc = inp.get("texture_scale")
        check(sc is not None and abs(sc[0] - 1.0 / 1.1) < 1e-6,
              "%s: tile is the argument in metres, not the old (0.3, 0.3)"
              % species)

    # A diffuse with no siblings must yield "" rather than a path to nothing —
    # OmniPBR renders a missing map as if the input were absent, so a wrong
    # guess here is indistinguishable from the original bug.
    check(veg._sibling_map(os.path.join(_SCENE_GEN, "nope_basecolor.png"),
                           veg._NORMAL_SIBLINGS) == "",
          "a diffuse with no sibling on disk yields \"\", not a dead path")
    check(veg._sibling_map("", veg._NORMAL_SIBLINGS) == "",
          "...and so does no texture at all")

    # THE TWO NAMING CONVENTIONS IN THIS ASSET TREE, both of which the walk has
    # to survive. `bark_oak_diff` -> `_nor` does not exist and `_norm` does, so
    # the list has to keep going past a miss; and a Poly-Haven `_diff_2k` must
    # not be rewritten by the shorter `_diff` rule.
    tex_dir = os.path.join(_SCENE_GEN, "assets", "aec", "brownstone",
                           "Materials", "vMaterials_2", "Wood", "textures")
    got = veg._sibling_map(os.path.join(tex_dir, "bark_oak_diff.jpg"),
                           veg._NORMAL_SIBLINGS)
    check(os.path.basename(got) == "bark_oak_norm.jpg",
          "`_diff` -> `_nor` misses and the walk continues to `_norm` (%s)"
          % (os.path.basename(got) or "<none>"))
    ph = os.path.join(_SCENE_GEN, "assets", "aec", "brownstone", "Assets",
                      "Vegetation", "Trees", "materials", "textures",
                      "bark_brown_01_diff_2k.png")
    got = veg._sibling_map(ph, veg._NORMAL_SIBLINGS)
    check(os.path.basename(got) == "bark_brown_01_nor_2k.png",
          "`_diff_2k` is matched before `_diff` (%s)"
          % (os.path.basename(got) or "<none>"))


# ---------------------------------------------------------------------------
# 5: which pieces have bark on them
# ---------------------------------------------------------------------------

def _slice_bark_draw():
    """The real cross-section draw, lifted out of `vegetation.wood_debris`.

    Sliced rather than re-typed for the reason `test_fence_geometry` slices
    its tiler: the whole value of this check is that it fails when the FORMULA
    changes, and a hand copy of four lines is a hand copy that silently drifts.
    `wood_debris` itself cannot be called — it needs trimesh, a stage and a
    surveyed tree.
    """
    src = open(os.path.join(_SCENE_GEN, "disaster", "vegetation.py")).read()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "wood_debris")
    # BY AST, NOT BY REGEX. `_bark` is authored across two physical lines and
    # `wood_debris` goes on to close several other multi-line calls below it,
    # so a line-oriented slice that rejoins anything starting with an operator
    # swallows half the seating code and will not compile. Asking the parser
    # for each assignment's own source segment is exact.
    want, picked = ("t_w", "t_d", "pr", "_bark"), {}
    for n in sorted((n for n in ast.walk(fn) if isinstance(n, ast.Assign)),
                    key=lambda n: n.lineno):
        tgt = n.targets[0]
        if isinstance(tgt, ast.Name) and tgt.id in want and tgt.id not in picked:
            seg = ast.get_source_segment(src, n) or ""
            picked[tgt.id] = " ".join(s.strip() for s in seg.splitlines())
    lines = [picked.get(k, "") for k in want]
    # `thick_m` is `wind_tree`'s argument, and the wind recipe passes
    # `_WIND_THICK`; substituted here so the slice needs no caller.
    lines = [ln.replace("*thick_m", "*veg._WIND_THICK") for ln in lines]
    ns = {"math": math, "veg": veg}
    code = compile("def _draw(rng, r_trunk):\n"
                   + "".join("    %s\n" % ln for ln in lines)
                   + "    return _bark\n",
                   "<wood_debris:cross-section>", "exec")
    exec(code, ns)                                   # noqa: S102 — repo source
    return ns["_draw"], lines


def test_bark_share():
    print("\n[5] bark goes on the pieces that kept bark")
    draw, lines = _slice_bark_draw()
    check(len(lines) == 4 and lines[-1].startswith("_bark = min("),
          "the cross-section draw came out of `wood_debris` as four "
          "statements, not re-typed here (%s)" % " | ".join(
              ln.split("=")[0].strip() for ln in lines))
    bark_at = inspect.signature(veg.wind_tree).parameters["bark_at"].default
    print("      r_trunk    share with bark (wind_tree bark_at = %.2f)"
          % bark_at)
    shares = {}
    for r_trunk in (0.25, 0.35, 0.50, 0.70, 0.90):
        rng = random.Random(11)
        n, barked = 20000, 0
        for _ in range(n):
            if draw(rng, r_trunk) >= bark_at:
                barked += 1
        shares[r_trunk] = barked / float(n)
        print("      %.2f m     %5.1f%%" % (r_trunk, 100.0 * shares[r_trunk]))

    check(all(0.10 < v < 0.60 for v in shares.values()),
          "every trunk radius keeps SOME bark and never all of it — a stand "
          "of pure bark or pure split wood is the failure either way")
    check(shares[0.25] > shares[0.90] * 1.5,
          "a slim bole sheds proportionally more bark than a fat one, because "
          "a fixed-thickness column out of a small trunk cannot miss it")
    check(shares[0.90] < 0.5,
          "on a big bole the MAJORITY of pieces are riven interior wood — "
          "which is why dressing them all in bark made the field read dark")


# ---------------------------------------------------------------------------
# 6: nothing lands off the plate
# ---------------------------------------------------------------------------

REGION_100 = (-50.0, -50.0, 50.0, 50.0)      # `suburb_tornado_100`, region_m 100
SPECIES = ("American_Beech", "Black_Oak", "Common_Apple", "Douglas_Fir",
           "Largetooth_Aspen", "Shumard_Oak")
THROW_DEG = 38.0 + 18.0                      # heading + a typical curl


def test_reach_table():
    print("\n[6a] the reach table matches the archetypes the bake writes")
    want = set()
    for sp in SPECIES:
        for lv in ("limbed", "leaning", "fallen", "snapped"):
            if lv == "fallen" and sp in tn.NO_UPROOT:
                continue
            want.add((sp, lv))
    check(set(tn.TREE_REACH) == want,
          "one entry per baked archetype, and exactly the %d the bake writes "
          "(NO_UPROOT skips `fallen`)" % len(want))
    check(all(len(v) == tn.REACH_SECTORS for v in tn.TREE_REACH.values()),
          "every profile has %d sectors" % tn.REACH_SECTORS)
    check(tn.tree_reach("Shumard_Oak", "pristine") is None,
          "`pristine` is unbounded — it is the green species USD, not an "
          "archetype, and a standing canopy over the edge is not the bug")
    check(tn.tree_reach("Nothofagus_Cunninghamii", "fallen") is not None,
          "an unmeasured species falls back to the per-level envelope rather "
          "than to no constraint at all")

    # The reach is a LOBE about local +X, which is the whole reason this is
    # sixteen numbers instead of one radius: a circular test at the same
    # maximum would reject about six times the area it needs to.
    fwd, back = [], []
    for prof in tn.TREE_REACH.values():
        fwd.append(max(prof[5:11]))              # -68 .. +68 deg of local +X
        back.append(max(list(prof[:5]) + list(prof[11:])))
    print("      forward lobe %.1f-%.1f m, everything behind %.1f-%.1f m"
          % (min(fwd), max(fwd), min(back), max(back)))
    check(min(fwd) > 3.0 * max(back),
          "the shortest forward lobe still out-reaches the longest backward "
          "one threefold — the throw direction is the whole shape")
    check(max(fwd) > 25.0,
          "the worst archetype throws past 25 m (Black_Oak limbed, 26.7)")


def test_reach_fits():
    print("\n[6b] `reach_fits` is exact against the plate boundary")
    prof = tn.tree_reach("Shumard_Oak", "fallen")
    check(tn.reach_fits(prof, 0.0, 0.0, 0.0, REGION_100),
          "a tree in the middle of the plate fits at any yaw")
    check(not tn.reach_fits(prof, 40.0, 0.0, 0.0, REGION_100),
          "...and one 10 m from the east edge, throwing east, does NOT")
    check(tn.reach_fits(prof, 40.0, 0.0, 180.0, REGION_100),
          "...but the same tree turned to throw west does")
    check(not tn.reach_fits(prof, 0.0, 0.0, 0.0, REGION_100, margin_m=40.0),
          "a margin insets the plate")

    # THE TANGENT CASE, which is the reason `_arc_extremes` exists. A sector
    # whose ENDS both clear the boundary can still bulge past it in the middle,
    # where the arc runs parallel to the edge — testing only the two ends would
    # pass a piece that is over the void by up to r * (1 - cos(11.25 deg)).
    r = max(prof)
    k = list(prof).index(r)
    step = 360.0 / tn.REACH_SECTORS
    mid = -180.0 + step * (k + 0.5)
    # Put the tree so the sector's MIDPOINT is due east and just over the edge,
    # while both of its ends fall short of it.
    x = REGION_100[2] - r + 0.05
    yaw = -mid
    ends = [x + r * math.cos(math.radians(mid + yaw + d))
            for d in (-step / 2.0, step / 2.0)]
    check(max(ends) < REGION_100[2],
          "the tangent case is set up: both sector ends are inside (%.2f m "
          "clear)" % (REGION_100[2] - max(ends)))
    check(not tn.reach_fits(prof, x, 0.0, yaw, REGION_100),
          "...and it is still rejected, because the arc between them is not")


def test_placement_rule():
    print("\n[6c] turn it before you downgrade it, and never leave the plate")
    rng = random.Random(7)
    off = kept = turned = downgraded = pristine = 0
    old_off = 0
    total = 0
    # A 3 m lattice of tree sites over the whole plate, every species, at the
    # intensity the corridor core carries — the worst case, and about ten times
    # the tree count a 100 m plat actually plants.
    for gx in range(-48, 49, 3):
        for gy in range(-48, 49, 3):
            sp = SPECIES[(gx + gy) % len(SPECIES)]
            base_yaw = rng.uniform(0.0, 360.0)
            lv, yaw, info = tn.tree_level_and_yaw(
                0.92, rng, sp, float(gx), float(gy), REGION_100,
                track_yaw_deg=THROW_DEG, base_yaw_deg=base_yaw)
            total += 1
            if not tn.reach_fits(tn.tree_reach(sp, lv), float(gx), float(gy),
                                 yaw, REGION_100):
                off += 1
            if lv == "pristine":
                pristine += 1
            elif info["downgraded"]:
                downgraded += 1
            elif info["turned_deg"]:
                turned += 1
            else:
                kept += 1
            # WHAT THE LAUNCHER DID BEFORE: draw the level, yaw the tipped
            # ones to the track and leave the rest on their layout yaw.
            old_lv = info["drawn"]
            old_yaw = (THROW_DEG if old_lv in ("leaning", "fallen")
                       else base_yaw)
            if not tn.reach_fits(tn.tree_reach(sp, old_lv), float(gx),
                                 float(gy), old_yaw, REGION_100):
                old_off += 1

    print("      %d sites: %d untouched, %d turned, %d level-dropped, "
          "%d fell back to pristine" % (total, kept, turned, downgraded,
                                        pristine))
    print("      debris over the void — before %d (%.0f%%), after %d"
          % (old_off, 100.0 * old_off / total, off))
    check(off == 0, "NOT ONE placement leaves the plate")
    check(old_off > total * 0.3,
          "the old rule really did put a large share of the stand's debris "
          "over the void (%.0f%%), so the test bites"
          % (100.0 * old_off / total))
    check(turned + kept > downgraded + pristine,
          "most trees are saved by a YAW rather than by a level drop — the "
          "damage ladder keeps its statistics")
    check(pristine < total * 0.20,
          "and the last-resort fallback stays rare (%.0f%%), so the plate "
          "does not print a green border" % (100.0 * pristine / total))


def test_placement_invariants():
    print("\n[6d] the rule's invariants")
    # NO_UPROOT survives the walk DOWN the ladder. Walking from `snapped` the
    # obvious next rung is `fallen`, and for these species the bake never wrote
    # one — the assembly would reference a missing archetype and quietly plant
    # a green tree.
    rng = random.Random(3)
    seen = set()
    for _ in range(4000):
        gx = rng.uniform(-49.0, 49.0)
        gy = rng.uniform(-49.0, 49.0)
        lv, _yaw, _i = tn.tree_level_and_yaw(
            rng.uniform(0.5, 1.0), rng, "Black_Oak", gx, gy, REGION_100,
            track_yaw_deg=THROW_DEG, base_yaw_deg=rng.uniform(0, 360))
        seen.add(lv)
    check("fallen" not in seen,
          "a NO_UPROOT species never lands on `fallen`, on the way up OR on "
          "the way down (saw %s)" % ", ".join(sorted(seen)))

    # THE RNG DRAW IS FIXED-LENGTH. Two trees at different places must consume
    # the same number of values, or moving one tree re-rolls every tree after
    # it and a change of plate size reshuffles the whole stand.
    counts = []
    for xy in ((0.0, 0.0), (49.0, 49.0), (-49.0, 0.0), (10.0, -30.0)):
        r = random.Random(5)
        tn.tree_level_and_yaw(0.9, r, "Douglas_Fir", xy[0], xy[1], REGION_100,
                              track_yaw_deg=THROW_DEG, base_yaw_deg=0.0)
        counts.append(r.random())
    check(len(set(counts)) == 1,
          "every placement consumes exactly two rng draws whatever it decides")

    # A tree in open middle ground is untouched — the rule has to be inert
    # where it is not needed, or it rewrites a scene that was already right.
    r = random.Random(9)
    lv, yaw, info = tn.tree_level_and_yaw(
        0.92, r, "Douglas_Fir", 0.0, 0.0, REGION_100,
        track_yaw_deg=THROW_DEG, base_yaw_deg=123.0)
    check(info["turned_deg"] == 0.0 and not info["downgraded"],
          "a tree in the middle of the plate is left exactly as drawn")
    check(lv not in ("leaning", "fallen") or abs(yaw - THROW_DEG) <= 38.0,
          "a tipped tree still falls within the ladder's +-38 deg of the "
          "track bearing when it does not need turning")


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("tornado logs: %d tests, offline (stubbed pxr, no GPU)" % len(tests))
    broken = []
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print("    ERROR %s: %s" % (name, exc))
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
