#!/usr/bin/env python3
"""
test_scene_modularity.py — OFFLINE proof that a scene KIND, its SIZE and its
DISASTER are three independent axes.

The claim under test: pick a preset (the KIND — `suburb`, `downtown`,
`suburb_net`), then set REGION_M and DISASTER_TYPE freely, and get a correct
config and a correct layout every time — with no bespoke preset per
combination.

RUNS WITHOUT ISAAC. No `pxr`, no GPU, no stage. That rules out
`compile_disaster.load_scene_config()`, whose last two lines call
`scene_generator` (which imports `pxr` at module scope), and rules out
`suburb_scene.build_suburb`. Two layers below those are pure Python and are
what this file exercises:

    compile_disaster.compile_spec(spec, base)   spec  -> low-level config
    layout.suburb_net.generate                  size  -> street network
    detail.suburb_parcel.parcel_blocks          blocks -> lots and houses

The REGION_M parser is not imported — it lives inside the Isaac launch
script, whose module scope pulls in the sim. Its source is sliced out of that
file and exec'd, so test A runs the repo's real parser rather than a copy of
it.

USAGE
    python3 scene_gen/tests/test_scene_modularity.py     # plain, no pytest
    pytest -s scene_gen/tests/test_scene_modularity.py   # -s to see the tables

Several tests REPORT rather than assert — the disaster-shape table in test B
exists so the "has damage" predicate can be chosen from what the compiler
actually emits, not from a guess baked into an assertion.
"""

import math
import os
import random
import sys
import textwrap
import time
import traceback

import yaml

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TESTS_DIR)
_REPO_ROOT = os.path.dirname(_SCENE_GEN)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd          # noqa: E402  pure Python, no pxr
from detail import suburb_parcel as sp  # noqa: E402
from layout import suburb_net as sn     # noqa: E402

LAUNCH_SCRIPT = os.path.join(
    _REPO_ROOT, "simulation", "isaac-sim", "launch_scripts",
    "example_multi_drone_scene_import.py")

# The three axes.
KINDS = ("suburb", "downtown")          # region-override-clean presets
AUDIT_KINDS = ("suburb", "downtown", "suburb_net")
SIZES = (250, 500, 1000)                # m; 1000 m generates in ~4 s
DISASTERS = ("none", "wildfire", "tornado")
SEEDS = (3, 10, 42)


# ---------------------------------------------------------------------------
# Reporting — printed inline so both runners show it (pytest needs -s).
# ---------------------------------------------------------------------------

def _say(*a):
    print(*a, flush=True)


def _table(title, headers, rows, note=None):
    cols = [len(h) for h in headers]
    srows = [[str(c) for c in r] for r in rows]
    for r in srows:
        for i, c in enumerate(r):
            cols[i] = max(cols[i], len(c))
    line = "  ".join("-" * w for w in cols)
    _say("")
    _say(title)
    _say(line)
    _say("  ".join(h.ljust(w) for h, w in zip(headers, cols)))
    _say(line)
    for r in srows:
        _say("  ".join(c.ljust(w) for c, w in zip(r, cols)))
    _say(line)
    if note:
        for ln in note.strip().splitlines():
            _say("  " + ln.strip())


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _deepcopy(d):
    return yaml.safe_load(yaml.safe_dump(d))


def _base():
    with open(cd.DEFAULT_BASE) as f:
        return yaml.safe_load(f)


def _preset(name):
    with open(os.path.join(cd.DEFAULT_PRESET_DIR, name + ".yaml")) as f:
        return yaml.safe_load(f)


def _apply_spec_overrides(cfg, spec_overrides):
    """Mirror of compile_disaster.load_scene_config's override merge.

    `load_scene_config` itself cannot run here (it ends in `scene_generator`,
    which needs pxr), so its one interesting line is reproduced: a shallow
    `dict.update` of the non-None overrides onto the HIGH-LEVEL spec, before
    compilation. See compile_disaster.py:723-728.
    """
    if not spec_overrides:
        return dict(cfg)
    applied = {k: v for k, v in spec_overrides.items() if v is not None}
    out = dict(cfg)
    out.update(applied)
    return out


def compile_kind(kind, spec_overrides=None):
    """A KIND preset plus launcher-style spec overrides -> low-level config."""
    spec = _apply_spec_overrides(_preset(kind), spec_overrides)
    return cd.compile_spec(spec, _base())


def region_of(cfg):
    return list(cfg.get("layout", {}).get("region_m") or [])


def disaster_shape(d):
    """Every candidate 'has damage' signal the compiled block carries."""
    field = d.get("field") or {}
    deb = d.get("debris") or {}
    return {
        "has_fire": "fire" in d,
        "fire_enabled": bool((d.get("fire") or {}).get("enabled")),
        "has_tornado": "tornado" in d,
        "damaged_fraction": d.get("damaged_fraction"),
        "destroyed_fraction": d.get("destroyed_fraction"),
        "field_kind": field.get("kind"),
        "field_inside": field.get("inside"),
        "field_outside": field.get("outside"),
        "trees_toppled": d.get("trees_toppled_fraction"),
        "debris_piles": deb.get("piles_per_building"),
        "n_keys": len(d),
    }


def _fmt(v):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


# ---------------------------------------------------------------------------
# The REGION_M parser, sliced out of the Isaac launch script
# ---------------------------------------------------------------------------

def load_region_m_parser():
    """Return (fn, path, lineno) for `_region_m` in the launch script.

    Importing the launch script would import Isaac. The function is nested
    inside `main()` there, so it is extracted by indentation and exec'd on its
    own — the real source, not a transcription.
    """
    with open(LAUNCH_SCRIPT) as f:
        src = f.read().splitlines()
    start = None
    for i, ln in enumerate(src):
        if ln.strip().startswith("def _region_m("):
            start = i
            break
    assert start is not None, f"no `def _region_m(` in {LAUNCH_SCRIPT}"
    indent = len(src[start]) - len(src[start].lstrip())
    body = [src[start]]
    for ln in src[start + 1:]:
        if ln.strip() and (len(ln) - len(ln.lstrip())) <= indent:
            break
        body.append(ln)
    ns = {}
    exec(textwrap.dedent("\n".join(body)), ns)   # noqa: S102 — repo source
    return ns["_region_m"], LAUNCH_SCRIPT, start + 1


# ---------------------------------------------------------------------------
# Layout helpers — the offline half of suburb_scene.build_suburb
# ---------------------------------------------------------------------------

_LAYOUT_CACHE = {}


def _layout_cfgs():
    """The two street-network configs worth testing at every size."""
    net_cfg = compile_kind("suburb_net")
    return {
        "defaults": ({}, {}),
        "suburb_net": (dict(net_cfg.get("suburb_net") or {}),
                       dict(net_cfg.get("suburb_parcel") or {})),
    }


def build_layout(width_m, height_m, seed, layout_cfg, parcel_cfg):
    """Streets + lots + houses, exactly as suburb_scene.py:3310-3411 does it
    minus the passes that need a USD stage (the measured house catalogue, so
    no `house_sizes`; the rest is identical, one shared RNG included).
    """
    rng = random.Random(int(seed) + 7717)
    net, blocks, info = sn.generate(float(width_m), float(height_m),
                                    rng, dict(layout_cfg))
    buildable = [b for b in blocks if not b.get("undeveloped")]
    pcfg = dict(parcel_cfg)
    bulb_r = float(sn.DEFAULTS["bulb_radius_m"])
    margin = float(pcfg.get("bulb_margin_m", 3.0))
    pcfg.setdefault("keepout_discs",
                    [(e.pts[-1], bulb_r + margin) for e in net.edges.values()
                     if e.street_type == "lollipop"])
    parcels = sp.parcel_blocks(buildable, rng, pcfg)
    houses = [h for b in parcels for h in (b.get("houses") or [])]
    return {"net": net, "blocks": blocks, "buildable": buildable,
            "parcels": parcels, "houses": houses, "info": info}


def layout(size, seed, cfg_name="suburb_net"):
    key = (size, seed, cfg_name)
    if key not in _LAYOUT_CACHE:
        lc, pc = _layout_cfgs()[cfg_name]
        t0 = time.time()
        _LAYOUT_CACHE[key] = build_layout(size, size, seed, lc, pc)
        _LAYOUT_CACHE[key]["secs"] = time.time() - t0
    return _LAYOUT_CACHE[key]


def perp_ccw(u):
    return (-u[1], u[0])


# ===========================================================================
# A. REGION_M parsing
# ===========================================================================

def test_A1_region_m_parsing():
    fn, path, ln = load_region_m_parser()
    _say(f"\n[A] parser from {path}:{ln}")
    good = [
        ("250",      [250.0, 250.0]),
        ("250x250",  [250.0, 250.0]),
        ("250,250",  [250.0, 250.0]),
        ("300x200",  [300.0, 200.0]),
        ("300X200",  [300.0, 200.0]),
        ("1600,1200", [1600.0, 1200.0]),
        ("250.5",    [250.5, 250.5]),
        ("",         None),
    ]
    rows, bad = [], []
    for raw, want in good:
        try:
            got = fn(raw)
        except BaseException as e:                       # noqa: BLE001
            got = f"{type(e).__name__}: {e}"
        ok = got == want
        rows.append([repr(raw), _fmt(want), _fmt(got), "ok" if ok else "WRONG"])
        if not ok:
            bad.append(f"REGION_M={raw!r} -> {got!r}, expected {want!r}")
    _table("[A1] REGION_M accepted forms",
           ["input", "expected", "got", ""], rows)
    assert not bad, "REGION_M parse mismatches:\n  " + "\n  ".join(bad)

    junk = ["abc", "250xfoo", "!!", "250 250", "twenty", "nan%"]
    rows, bad = [], []
    for raw in junk:
        try:
            got = fn(raw)
            outcome, ok = repr(got), False
        except SystemExit as e:
            outcome, ok = f"SystemExit: {e}", True
        except BaseException as e:                       # noqa: BLE001
            outcome, ok = f"{type(e).__name__}: {e}", False
        rows.append([repr(raw), outcome, "clean" if ok else "NOT CLEAN"])
        if not ok:
            bad.append(f"REGION_M={raw!r} -> {outcome} (wanted SystemExit)")
    _table("[A1] REGION_M junk rejection", ["input", "outcome", ""], rows)
    assert not bad, "junk REGION_M not cleanly rejected:\n  " + "\n  ".join(bad)


def test_A2_region_m_delimiter_only():
    """Junk that is nothing but delimiters must also be a clean error."""
    fn, _, _ = load_region_m_parser()
    rows, bad = [], []
    for raw in [",", "x", "xx", ",,", "x,"]:
        try:
            got = fn(raw)
            outcome, ok = repr(got), False
        except SystemExit as e:
            outcome, ok = f"SystemExit: {e}", True
        except BaseException as e:                       # noqa: BLE001
            outcome, ok = f"{type(e).__name__}: {e}", False
        rows.append([repr(raw), outcome, "clean" if ok else "NOT CLEAN"])
        if not ok:
            bad.append(f"REGION_M={raw!r} -> {outcome} (wanted SystemExit)")
    _table("[A2] REGION_M delimiter-only input", ["input", "outcome", ""], rows,
           note="A returned [] is not None, so the launcher applies it as a "
                "spec override. compile_spec then writes layout.region_m = [] "
                "and, for disaster-type none, COMPILES CLEANLY -- the empty "
                "region only surfaces once the generator tries to use it. A "
                "region-reading disaster (tornado) raises `not enough values "
                "to unpack` from compile_disaster.py:222 instead.")
    assert not bad, ("delimiter-only REGION_M not cleanly rejected:\n  "
                     + "\n  ".join(bad))


def test_A3_region_m_degenerate_report():
    """REPORT ONLY — what the parser does with non-positive dimensions."""
    fn, _, _ = load_region_m_parser()
    rows = []
    for raw in ["0", "0x0", "-250", "250x0", "1e9", "250x200x100"]:
        try:
            got = repr(fn(raw))
        except BaseException as e:                       # noqa: BLE001
            got = f"{type(e).__name__}: {e}"
        rows.append([repr(raw), got])
    _table("[A3] REGION_M degenerate input (report only)", ["input", "got"],
           rows,
           note="No range check: a zero or negative region parses happily and "
                "is handed to the generator.")


# ===========================================================================
# B. Spec overrides land, and what each disaster actually emits
# ===========================================================================

def test_B_size_x_disaster_grid():
    rows, bad = [], []
    for kind in KINDS:
        for size in SIZES:
            for dtype in DISASTERS:
                sev = 0.0 if dtype == "none" else 0.6
                cfg = compile_kind(kind, {"region_m": [size, size],
                                          "disaster-type": dtype,
                                          "severity": sev})
                got = region_of(cfg)
                shape = disaster_shape(cfg["disaster"])
                if [float(v) for v in got] != [float(size), float(size)]:
                    bad.append(f"{kind} REGION_M={size} DISASTER_TYPE={dtype}"
                               f" -> layout.region_m={got}")
                rows.append([
                    kind, size, dtype, str(got),
                    _fmt(shape["has_fire"]), _fmt(shape["has_tornado"]),
                    _fmt(shape["damaged_fraction"]),
                    _fmt(shape["destroyed_fraction"]),
                    _fmt(shape["field_kind"]), _fmt(shape["field_inside"]),
                    _fmt(shape["field_outside"]), _fmt(shape["trees_toppled"]),
                    _fmt(shape["debris_piles"]), shape["n_keys"],
                ])
    _table("[B] KIND x SIZE x DISASTER -> compiled config",
           ["kind", "size", "disaster", "layout.region_m", "fire?", "tornado?",
            "damaged", "destroyed", "field.kind", "f.inside", "f.outside",
            "trees", "debris.piles", "#keys"], rows)
    assert not bad, "region_m did not follow the override:\n  " + "\n  ".join(bad)


def test_B2_disaster_section_shape_report():
    """REPORT ONLY — the full disaster vocabulary, every type x severity.

    This is the table for choosing a "has damage" predicate. Read the
    `wildfire` rows against `none`: they are identical except for `fire`.
    """
    rows = []
    for dtype in sorted(cd.DISASTERS):
        for sev in (0.0, 0.3, 0.6, 1.0):
            cfg = compile_kind("suburb", {"region_m": [500, 500],
                                          "disaster-type": dtype,
                                          "severity": sev})
            s = disaster_shape(cfg["disaster"])
            rows.append([
                dtype, sev,
                _fmt(s["has_fire"]), _fmt(s["fire_enabled"]),
                _fmt(s["has_tornado"]),
                _fmt(s["damaged_fraction"]), _fmt(s["destroyed_fraction"]),
                _fmt(s["field_kind"]), _fmt(s["field_inside"]),
                _fmt(s["field_outside"]), _fmt(s["trees_toppled"]),
                _fmt(s["debris_piles"]), s["n_keys"],
            ])
    _table("[B2] disaster section shape, all types x severity "
           "(500 m suburb) — REPORT ONLY",
           ["disaster", "sev", "fire?", "fire.on", "tornado?", "damaged",
            "destroyed", "field.kind", "f.inside", "f.outside", "trees",
            "debris.piles", "#keys"], rows,
           note="""
           compile_spec substitutes compile_none whenever severity == 0, so
           every type at sev 0 is the pristine block (compile_disaster.py:640).
           wildfire is compile_none + a `fire` key by design
           (compile_disaster.py:480-504): zero structural damage, uniform-zero
           field. A predicate of `damaged_fraction > 0 or destroyed_fraction >
           0` therefore reports NO DAMAGE for every wildfire scene.
           """)

    # Report which single predicates separate "something happened" from none.
    preds = {
        "damaged_fraction > 0":
            lambda d: (d.get("damaged_fraction") or 0) > 0,
        "destroyed_fraction > 0":
            lambda d: (d.get("destroyed_fraction") or 0) > 0,
        "damaged or destroyed > 0":
            lambda d: (d.get("damaged_fraction") or 0) > 0
            or (d.get("destroyed_fraction") or 0) > 0,
        "field.inside > 0":
            lambda d: ((d.get("field") or {}).get("inside") or 0) > 0,
        "'fire' in disaster":
            lambda d: "fire" in d,
        "field.inside > 0 or fire.enabled":
            lambda d: ((d.get("field") or {}).get("inside") or 0) > 0
            or bool((d.get("fire") or {}).get("enabled")),
    }
    prows = []
    for name, fn in preds.items():
        cells = [name]
        for dtype in sorted(cd.DISASTERS):
            cfg = compile_kind("suburb", {"region_m": [500, 500],
                                          "disaster-type": dtype,
                                          "severity": 0.6})
            cells.append("T" if fn(cfg["disaster"]) else ".")
        prows.append(cells)
    _table("[B2] candidate 'has damage' predicates at severity 0.6 "
           "(T = fires, . = does not)",
           ["predicate"] + list(sorted(cd.DISASTERS)), prows,
           note="Only the last row is true for every non-`none` type.")


def test_B3_bad_spec_values_rejected():
    for dtype in ("hurricane_but_typo", "", "TORNADO!"):
        try:
            compile_kind("suburb", {"disaster-type": dtype})
        except ValueError:
            pass
        else:
            raise AssertionError(f"disaster-type={dtype!r} was accepted")
    for sev in (-0.1, 1.5, 42):
        try:
            compile_kind("suburb", {"severity": sev})
        except ValueError:
            pass
        else:
            raise AssertionError(f"severity={sev!r} was accepted")
    # Case-insensitivity is documented behaviour and must keep working.
    assert "tornado" in str(
        compile_kind("suburb", {"disaster-type": "TORNADO",
                                "severity": 0.5})["disaster"])


# ===========================================================================
# C. Overrides do not leak between axes
# ===========================================================================

def _disaster_identity(cfg):
    """What makes a disaster THAT disaster, ignoring region-scaled numbers."""
    d = cfg["disaster"]
    return (tuple(sorted(d.keys())), (d.get("field") or {}).get("kind"),
            "fire" in d, "tornado" in d)


def test_C1_disaster_override_does_not_move_region():
    bad = []
    for kind in KINDS:
        baseline = region_of(compile_kind(kind))
        for dtype in sorted(cd.DISASTERS):
            got = region_of(compile_kind(kind, {"disaster-type": dtype,
                                                "severity": 0.7}))
            if got != baseline:
                bad.append(f"{kind}: DISASTER_TYPE={dtype} moved region_m "
                           f"{baseline} -> {got}")
    assert not bad, "\n  ".join(bad)


def test_C2_region_override_does_not_change_disaster_identity():
    bad, rows = [], []
    for kind in KINDS:
        for dtype in sorted(cd.DISASTERS):
            ident = None
            for size in SIZES:
                cfg = compile_kind(kind, {"region_m": [size, size],
                                          "disaster-type": dtype,
                                          "severity": 0.7})
                cur = _disaster_identity(cfg)
                if ident is None:
                    ident = cur
                elif cur != ident:
                    bad.append(f"{kind}/{dtype}: identity changed at "
                               f"{size} m: {ident} -> {cur}")
            rows.append([kind, dtype, ident[1], _fmt(ident[2]),
                         len(ident[0])])
    _table("[C2] disaster identity is size-invariant",
           ["kind", "disaster", "field.kind", "fire?", "#keys"], rows)
    assert not bad, "\n  ".join(bad)


def test_C3_omitted_override_leaves_preset_alone():
    bad = []
    for kind in AUDIT_KINDS:
        spec = _preset(kind)
        plain = compile_kind(kind)
        empty = compile_kind(kind, {})
        nones = compile_kind(kind, {"region_m": None, "disaster-type": None,
                                    "severity": None, "seed": None})
        if plain != empty:
            bad.append(f"{kind}: spec_overrides={{}} changed the config")
        if plain != nones:
            bad.append(f"{kind}: all-None spec_overrides changed the config")
        # And the preset's own declared values survived compilation.
        if spec.get("region_m") and region_of(plain) != list(spec["region_m"]):
            bad.append(f"{kind}: preset region_m {spec['region_m']} became "
                       f"{region_of(plain)}")
        if spec.get("seed") is not None and plain.get("seed") != spec["seed"]:
            bad.append(f"{kind}: preset seed {spec['seed']} became "
                       f"{plain.get('seed')}")
    assert not bad, "\n  ".join(bad)


def test_C4_axes_are_orthogonal():
    """region_m depends only on SIZE; disaster identity only on DISASTER."""
    regions, idents, bad = {}, {}, []
    for kind in KINDS:
        for size in SIZES:
            for dtype in DISASTERS:
                cfg = compile_kind(kind, {"region_m": [size, size],
                                          "disaster-type": dtype,
                                          "severity": 0.6})
                regions.setdefault((kind, size), set()).add(
                    tuple(region_of(cfg)))
                idents.setdefault((kind, dtype), set()).add(
                    _disaster_identity(cfg))
    for k, v in regions.items():
        if len(v) != 1:
            bad.append(f"region_m for {k} varied with disaster: {v}")
    for k, v in idents.items():
        if len(v) != 1:
            bad.append(f"disaster identity for {k} varied with size")
    assert not bad, "\n  ".join(bad)


def test_C5_other_spec_keys_untouched():
    """Setting one axis must not disturb locale, asset_set or seed."""
    bad = []
    for kind in AUDIT_KINDS:
        ref = compile_kind(kind)
        for ov in ({"region_m": [321, 123]},
                   {"disaster-type": "tornado", "severity": 0.4},
                   {"severity": 0.9}):
            cfg = compile_kind(kind, ov)
            for key in ("locale", "asset_set", "seed"):
                if cfg.get(key) != ref.get(key):
                    bad.append(f"{kind} + {ov}: {key} {ref.get(key)!r} -> "
                               f"{cfg.get(key)!r}")
    assert not bad, "\n  ".join(bad)


def test_C6_preset_override_shadowing_audit():
    """A preset's own `overrides.layout.region_m` deep-merges AFTER the
    spec-level region_m, so it silently wins. Audit every preset."""
    rows, bad = [], []
    for f in sorted(os.listdir(cd.DEFAULT_PRESET_DIR)):
        if not f.endswith((".yaml", ".yml")):
            continue
        name = os.path.splitext(f)[0]
        spec = _preset(name)
        shadow = ((spec.get("overrides") or {}).get("layout") or {}
                  ).get("region_m")
        cfg = compile_kind(name, {"region_m": [777, 555]})
        got = region_of(cfg)
        survived = [float(v) for v in got] == [777.0, 555.0]
        rows.append([name, _fmt(spec.get("region_m")), _fmt(shadow),
                     str(got), "ok" if survived else "IGNORED"])
        if not survived and name in AUDIT_KINDS:
            bad.append(f"{name}: REGION_M=777x555 ignored, compiled "
                       f"layout.region_m={got} (preset's "
                       f"overrides.layout.region_m={shadow})")
    _table("[C6] does a REGION_M=777x555 spec override survive compilation?",
           ["preset", "spec region_m", "overrides.layout.region_m",
            "compiled", ""], rows,
           note="""
           compile_spec writes the spec's region_m into layout.region_m
           (compile_disaster.py:622-623) and then deep-merges the preset's own
           `overrides:` block over the whole config
           (compile_disaster.py:644-645), so a preset that repeats region_m
           under `overrides.layout` cannot be resized from the launcher.
           Only presets listed in AUDIT_KINDS are asserted; the rest are
           reported.
           """)
    assert not bad, "\n  ".join(bad)


def test_C7_compilation_is_pure():
    """Compiling must not mutate the preset or the base config on disk."""
    before_base = _base()
    before_spec = _preset("suburb")
    spec = _preset("suburb")
    base = _base()
    cd.compile_spec(spec, base)
    assert _base() == before_base, "compile_spec mutated default.yaml on disk"
    assert _preset("suburb") == before_spec, "compile_spec mutated the preset"
    assert spec == before_spec, ("compile_spec mutated the spec dict it was "
                                 "handed")


# ===========================================================================
# D. Layout scales with the region
# ===========================================================================

def test_D_layout_scales():
    rows, bad = [], []
    for cfg_name in ("defaults", "suburb_net"):
        for seed in SEEDS:
            counts = []
            for size in SIZES:
                L = layout(size, seed, cfg_name)
                houses = L["houses"]
                half = size / 2.0
                out = [h for h in houses
                       if abs(h["c"][0]) > half + 1e-6
                       or abs(h["c"][1]) > half + 1e-6]
                xs = [h["c"][0] for h in houses] or [0.0]
                ys = [h["c"][1] for h in houses] or [0.0]
                counts.append(len(houses))
                rows.append([
                    cfg_name, seed, size, len(L["blocks"]),
                    len(L["buildable"]), len(houses),
                    f"{len(houses) / (size * size / 1e6):.0f}",
                    f"[{min(xs):.0f},{max(xs):.0f}]",
                    f"[{min(ys):.0f},{max(ys):.0f}]",
                    len(out), f"{L['secs']:.1f}",
                ])
                if not houses:
                    bad.append(f"{cfg_name} seed {seed} {size} m: 0 houses")
                if out:
                    bad.append(f"{cfg_name} seed {seed} {size} m: {len(out)} "
                               f"house centres outside +/-{half} m, worst "
                               f"{max(max(abs(h['c'][0]), abs(h['c'][1])) for h in out):.1f}")
            if not (counts[0] < counts[1] < counts[2]):
                bad.append(f"{cfg_name} seed {seed}: house count not "
                           f"increasing with area: {counts}")
    _table("[D] layout at each size",
           ["cfg", "seed", "size_m", "blocks", "buildable", "houses",
            "houses/km2", "x range", "y range", "outside", "secs"], rows)
    assert not bad, "\n  ".join(bad)


def test_D2_house_footprints_inside_region_report():
    """REPORT ONLY — centres are asserted in D; corners may legitimately
    overhang the crop edge, since the region is a CROP of a bigger suburb."""
    rows = []
    for size in SIZES:
        L = layout(size, SEEDS[0], "suburb_net")
        half = size / 2.0
        worst = 0.0
        n_over = 0
        for h in L["houses"]:
            for (x, y) in (h.get("corners") or []):
                d = max(abs(x) - half, abs(y) - half)
                if d > 1e-6:
                    worst = max(worst, d)
            if any(max(abs(x) - half, abs(y) - half) > 1e-6
                   for (x, y) in (h.get("corners") or [])):
                n_over += 1
        rows.append([size, len(L["houses"]), n_over, f"{worst:.2f}"])
    _table("[D2] house corners vs region edge (report only)",
           ["size_m", "houses", "houses w/ corner outside", "worst overhang m"],
           rows)


# ===========================================================================
# E. Lot frame invariant: n is a unit normal of u
# ===========================================================================

def test_E_lot_frame_invariant():
    rows, bad = [], []
    tot_plus = tot_minus = tot = 0
    for cfg_name in ("defaults", "suburb_net"):
        for seed in SEEDS:
            for size in SIZES:
                L = layout(size, seed, cfg_name)
                plus = minus = other = 0
                worst_dot = worst_norm = worst_yaw = 0.0
                for h in L["houses"]:
                    u, n = tuple(h["u"]), tuple(h["n"])
                    worst_dot = max(worst_dot, abs(u[0] * n[0] + u[1] * n[1]))
                    worst_norm = max(worst_norm,
                                     abs(math.hypot(*n) - 1.0),
                                     abs(math.hypot(*u) - 1.0))
                    p = perp_ccw(u)
                    if math.hypot(n[0] - p[0], n[1] - p[1]) < 1e-9:
                        plus += 1
                    elif math.hypot(n[0] + p[0], n[1] + p[1]) < 1e-9:
                        minus += 1
                    else:
                        other += 1
                    yaw = math.degrees(math.atan2(u[1], u[0]))
                    dy = (float(h["yaw_deg"]) - yaw + 180.0) % 360.0 - 180.0
                    worst_yaw = max(worst_yaw, abs(dy))
                n_h = len(L["houses"])
                tot += n_h
                tot_plus += plus
                tot_minus += minus
                rows.append([cfg_name, seed, size, n_h, plus, minus, other,
                             f"{worst_dot:.2e}", f"{worst_norm:.2e}",
                             f"{worst_yaw:.2e}"])
                if worst_dot > 1e-9:
                    bad.append(f"{cfg_name}/{seed}/{size}: max |dot(u,n)| "
                               f"= {worst_dot:.3e}")
                if worst_norm > 1e-9:
                    bad.append(f"{cfg_name}/{seed}/{size}: u or n not unit "
                               f"(err {worst_norm:.3e})")
                if other:
                    bad.append(f"{cfg_name}/{seed}/{size}: {other} houses "
                               f"whose n is neither +perp_ccw(u) nor -")
    _table("[E] lot frame: n vs perp_ccw(u) = (-uy, ux)",
           ["cfg", "seed", "size_m", "houses", "n=+perp", "n=-perp", "other",
            "max|u.n|", "max|len-1|", "max|yaw-atan2(u)| deg"], rows,
           note=f"""
           {tot_plus}/{tot} houses have n == +perp_ccw(u), {tot_minus}/{tot}
           have -perp_ccw(u). yaw_deg is atan2(u.y, u.x) in every case, so a
           yaw derived from u alone is safe only while the +perp convention
           holds; nothing in suburb_parcel asserts it.
           """)
    assert not bad, "\n  ".join(bad)
    assert tot_minus == 0, (f"{tot_minus}/{tot} houses use -perp_ccw(u); the "
                            "yaw derivation assumes a fixed handedness")


# ===========================================================================
# F. Determinism
# ===========================================================================

def _pose(h):
    return (tuple(round(v, 9) for v in h["c"]),
            tuple(round(v, 9) for v in h["u"]),
            tuple(round(v, 9) for v in h["n"]),
            round(float(h["yaw_deg"]), 9),
            round(float(h["w"]), 9), round(float(h["d"]), 9))


def test_F1_layout_determinism():
    rows, bad = [], []
    lc, pc = _layout_cfgs()["suburb_net"]
    for size in (250, 500):
        for seed in SEEDS[:2]:
            a = build_layout(size, size, seed, lc, pc)
            b = build_layout(size, size, seed, lc, pc)
            same_n = len(a["houses"]) == len(b["houses"])
            same_p = (bool(a["houses"]) and
                      _pose(a["houses"][0]) == _pose(b["houses"][0]))
            rows.append([size, seed, len(a["houses"]), len(b["houses"]),
                         "ok" if same_n and same_p else "DIFFERS"])
            if not same_n:
                bad.append(f"{size} m seed {seed}: house count "
                           f"{len(a['houses'])} != {len(b['houses'])}")
            elif not same_p:
                bad.append(f"{size} m seed {seed}: first-house pose differs")
            # A different seed must actually change the plat.
            c = build_layout(size, size, seed + 1000, lc, pc)
            if c["houses"] and a["houses"] and \
                    _pose(c["houses"][0]) == _pose(a["houses"][0]):
                bad.append(f"{size} m: seed {seed} and {seed + 1000} give the "
                           "same first house — seed is not wired through")
    _table("[F1] same seed + same cfg -> same plat",
           ["size_m", "seed", "houses run 1", "houses run 2", ""], rows)
    assert not bad, "\n  ".join(bad)


def test_F2_compile_determinism():
    bad = []
    for kind in KINDS:
        for size in SIZES:
            for dtype in DISASTERS:
                ov = {"region_m": [size, size], "disaster-type": dtype,
                      "severity": 0.6}
                a = compile_kind(kind, ov)
                b = compile_kind(kind, dict(ov))
                if a != b:
                    bad.append(f"{kind}/{size}/{dtype}: compile_spec is not "
                               "deterministic")
    assert not bad, "\n  ".join(bad)


def test_F3_layout_follows_compiled_region():
    """The end-to-end claim: REGION_M -> compiled config -> plat extent."""
    bad, rows = [], []
    for size in SIZES:
        cfg = compile_kind("suburb", {"region_m": [size, size],
                                      "disaster-type": "wildfire",
                                      "severity": 0.6, "seed": 3})
        w, h = (float(v) for v in region_of(cfg))
        L = build_layout(w, h, int(cfg["seed"]), {}, {})
        x0, y0, x1, y1 = L["info"]["region"]
        rows.append([size, f"{w:.0f}x{h:.0f}",
                     f"[{x0:.0f},{y0:.0f},{x1:.0f},{y1:.0f}]",
                     len(L["houses"])])
        if abs((x1 - x0) - size) > 1e-6 or abs((y1 - y0) - size) > 1e-6:
            bad.append(f"REGION_M={size} -> network region "
                       f"{x1 - x0:.1f} x {y1 - y0:.1f} m")
    _table("[F3] REGION_M -> compiled layout.region_m -> network extent",
           ["REGION_M", "layout.region_m", "network region", "houses"], rows)
    assert not bad, "\n  ".join(bad)


# ===========================================================================
# Plain-python runner
# ===========================================================================

TESTS = [
    test_A1_region_m_parsing,
    test_A2_region_m_delimiter_only,
    test_A3_region_m_degenerate_report,
    test_B_size_x_disaster_grid,
    test_B2_disaster_section_shape_report,
    test_B3_bad_spec_values_rejected,
    test_C1_disaster_override_does_not_move_region,
    test_C2_region_override_does_not_change_disaster_identity,
    test_C3_omitted_override_leaves_preset_alone,
    test_C4_axes_are_orthogonal,
    test_C5_other_spec_keys_untouched,
    test_C6_preset_override_shadowing_audit,
    test_C7_compilation_is_pure,
    test_D_layout_scales,
    test_D2_house_footprints_inside_region_report,
    test_E_lot_frame_invariant,
    test_F1_layout_determinism,
    test_F2_compile_determinism,
    test_F3_layout_follows_compiled_region,
]


def main():
    _say("=" * 78)
    _say("scene modularity — OFFLINE (no Isaac, no GPU, no pxr)")
    _say(f"  scene_gen : {_SCENE_GEN}")
    _say(f"  base cfg  : {os.path.relpath(cd.DEFAULT_BASE, _REPO_ROOT)}")
    _say(f"  presets   : {os.path.relpath(cd.DEFAULT_PRESET_DIR, _REPO_ROOT)}")
    _say(f"  kinds {list(KINDS)}  sizes {list(SIZES)} m  "
         f"disasters {list(DISASTERS)}  seeds {list(SEEDS)}")
    _say("=" * 78)

    results = []
    t_all = time.time()
    for fn in TESTS:
        t0 = time.time()
        try:
            fn()
            results.append((fn.__name__, "PASS", time.time() - t0, None))
        except AssertionError as e:
            results.append((fn.__name__, "FAIL", time.time() - t0, str(e)))
        except BaseException as e:                       # noqa: BLE001
            results.append((fn.__name__, "ERROR", time.time() - t0,
                            traceback.format_exc()))

    _say("")
    _say("=" * 78)
    _say("SUMMARY")
    _say("=" * 78)
    width = max(len(n) for n, _, _, _ in results)
    for name, status, secs, _ in results:
        _say(f"  {status:<5}  {name.ljust(width)}  {secs:6.2f}s")
    failed = [r for r in results if r[1] != "PASS"]
    if failed:
        _say("")
        _say("-" * 78)
        _say("FAILURES")
        _say("-" * 78)
        for name, status, _, msg in failed:
            _say(f"\n{status}: {name}")
            for ln in (msg or "").splitlines():
                _say("    " + ln)
    _say("")
    _say(f"{len(results) - len(failed)}/{len(results)} passed in "
         f"{time.time() - t_all:.1f}s")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
