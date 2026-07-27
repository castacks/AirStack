#!/usr/bin/env python3
"""
compile_disaster.py — turn a HIGH-LEVEL disaster spec into a LOW-LEVEL scene
config that scene_generator.py can build directly.

    presets/tornado.yaml          (high level: what happened, how bad)
              |
              |  compile_disaster.py
              v
    low_level/compiled/tornado.yaml   (low level: every knob the generator reads)
              |
              |  scene_generator.py
              v
    the scene

HIGH-LEVEL SPEC
---------------
    disaster-type: tornado    # none | earthquake | tornado | explosion
                              # | flood | hurricane
    severity: 0.6             # 0..1 — 0 is pristine, 1 is as bad as that
                              #        disaster gets

    seed: 42                  # optional — city layout + disaster RNG
    region_m: [400, 400]      # optional — city extent

    # optional, disaster-specific (sensible defaults derived from the region):
    epicenter: [0, 0]         # earthquake / explosion — where it struck
    heading_deg: 35           # tornado / hurricane — direction of travel

    overrides:                # optional escape hatch, deep-merged last:
      packing:                # any low-level setting, verbatim
        min_parks: 4

Compilation is: base low-level config (low_level/default.yaml)
              + the disaster function's output
              + the spec's `overrides`
and the result is written as a self-contained low-level config, so a scene
stays reproducible from that one file even if default.yaml later changes.

ADDING A DISASTER
-----------------
Write ``compile_<name>(sev, spec, region) -> dict`` returning a ``disaster``
block (fates, debris, aftermath, field), then register it in DISASTERS. All
disaster knobs are *maxima* reached where ``disaster.field`` reads 1.0 — the
field is what gives each disaster its shape (uniform / radial / path).

USAGE
-----
    python3 utils/compile_disaster.py                 # compile every preset
    python3 utils/compile_disaster.py tornado         # one, by name
    python3 utils/compile_disaster.py path/to/spec.yaml --output out.yaml
    python3 utils/compile_disaster.py --list          # show disaster types

AS A LIBRARY
------------
``load_scene_config(path_or_name)`` accepts a config at either level and
returns a validated low-level dict — high-level specs are compiled in memory.
Launch scripts use it so SCENE_CONFIG can name either kind, or just a bare
name ("tornado") resolved against presets/ and low_level/compiled/.
"""

import argparse
import datetime
import os
import sys

import yaml

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.join(_ISAAC_SIM_DIR, "config", "scene_generation")
DEFAULT_BASE = os.path.join(_SCENE_GEN_DIR, "low_level", "default.yaml")
DEFAULT_PRESET_DIR = os.path.join(_SCENE_GEN_DIR, "presets")
DEFAULT_OUT_DIR = os.path.join(_SCENE_GEN_DIR, "low_level", "compiled")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* in place: nested dicts merge
    key-by-key, everything else (scalars, lists) is replaced outright."""
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def lerp(lo, hi, t):
    """Interpolate lo->hi over t in [0, 1], rounded to 3 decimals."""
    return round(lo + (hi - lo) * max(0.0, min(1.0, t)), 3)


def lerp_pair(lo_pair, hi_pair, t):
    """Interpolate a [min, max] count range, as ints."""
    return [int(round(lerp(lo_pair[0], hi_pair[0], t))),
            int(round(lerp(lo_pair[1], hi_pair[1], t)))]


def _ramp(sev, knee=0.0):
    """Severity remapped so it only starts biting past *knee* (0..1)."""
    if sev <= knee:
        return 0.0
    return (sev - knee) / (1.0 - knee)


# ---------------------------------------------------------------------------
# Disaster compilers — one per type
#
# Each returns the low-level ``disaster`` block for its type at severity
# *sev* (0..1). *spec* is the high-level dict (for epicenter / heading /
# per-disaster extras); *region* is (width_m, height_m).
#
# The shared vocabulary, so the types stay comparable:
#   damaged_/destroyed_fraction  structural loss
#   debris.*                     rubble volume, spread, and lean/sink
#   *_toppled_fraction           street furniture knocked down
#   *_scatter_m / *_strewn       how far light things were moved
#   humans_*                     casualties
#   field                        WHERE all of the above applies
# ---------------------------------------------------------------------------

def compile_none(sev, spec, region):
    """No disaster: a pristine city. Severity is ignored."""
    return {
        "damaged_fraction": 0.0,
        "destroyed_fraction": 0.0,
        "debris": {"piles_per_building": [0, 0],
                   "pieces_per_building": [0, 0],
                   "tilt_chance": 0.0},
        "streetlights_toppled_fraction": 0.0,
        "traffic_lights_toppled_fraction": 0.0,
        "traffic_lights_leaning_fraction": 0.0,
        "trash_cans_toppled_fraction": 0.0,
        "trash_cans_scatter_m": 0.0,
        "cars_toppled_fraction": 0.0,
        "cars_strewn": [0, 0],
        "humans_prone_fraction": 0.0,
        "humans_strewn": [0, 0],
        "field": {"kind": "uniform", "inside": 0.0},
    }


def compile_earthquake(sev, spec, region):
    """Ground shaking: structures fail in place.

    Signature — buildings pancake, lean and sink (liquefaction); rubble drops
    straight down at the facades rather than flying; poles shake down but
    nothing is blown anywhere, so light objects tip over where they stood.
    Attenuates radially from the epicenter over a wide radius.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    return {
        "damaged_fraction": lerp(0.05, 0.35, sev),
        "destroyed_fraction": lerp(0.02, 0.55, sev),
        "debris": {
            # Collapse rubble: lots of it, piled tight against the ruin.
            "piles_per_building": lerp_pair([1, 2], [4, 7], sev),
            "pile_max_offset_m": lerp(2.0, 4.0, sev),
            "pieces_per_building": lerp_pair([4, 8], [14, 26], sev),
            "pieces_scatter_m": lerp(3.0, 7.0, sev),   # gravity, not wind
            # Leaning/sinking is the earthquake tell.
            "tilt_chance": lerp(0.25, 0.7, sev),
            "tilt_deg": [3, lerp(7.0, 12.0, sev)],
            "sink_m": [0.5, lerp(1.2, 2.2, sev)],
            "lean_piles": lerp_pair([2, 3], [3, 5], sev),
        },
        "streetlights_toppled_fraction": lerp(0.05, 0.5, sev),
        "traffic_lights_toppled_fraction": lerp(0.04, 0.45, sev),
        "traffic_lights_leaning_fraction": lerp(0.15, 0.5, sev),
        "traffic_lights_lean_deg": [8, lerp(20.0, 38.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.25, 0.8, sev),
        "trash_cans_scatter_m": lerp(0.5, 2.0, sev),   # tip, don't fly
        "cars_toppled_fraction": lerp(0.01, 0.15, sev),  # crushed, not flipped
        "cars_strewn": lerp_pair([0, 1], [2, 5], sev),
        "strewn_topple_fraction": 0.4,
        "humans_prone_fraction": lerp(0.05, 0.55, sev),
        "humans_strewn": lerp_pair([0, 2], [4, 10], sev),
        # Wide radial attenuation — the whole city feels it, the epicenter
        # feels it worst. Never fully zero anywhere.
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(0.15, 0.45, sev), 1),
            "falloff_m": round(max(w, h) * 0.55, 1),
            "inside": 1.0,
            "outside": lerp(0.1, 0.45, sev),
        },
    }


def compile_tornado(sev, spec, region):
    """A narrow track of extreme wind across an otherwise intact city.

    Signature — total destruction inside a corridor, near-nothing outside it.
    Everything light is thrown a long way: cans fly, cars are flipped and
    strewn, debris is scattered far downwind. Buildings are torn apart rather
    than settling, so tilt/sink stays low.
    """
    w, h = region
    return {
        "damaged_fraction": lerp(0.1, 0.3, sev),
        "destroyed_fraction": lerp(0.15, 0.65, sev),
        "debris": {
            "piles_per_building": lerp_pair([1, 2], [2, 4], sev),
            "pile_max_offset_m": lerp(2.0, 3.5, sev),
            # Signature: many fragments, thrown far.
            "pieces_per_building": lerp_pair([6, 12], [18, 34], sev),
            "pieces_scatter_m": lerp(6.0, 18.0, sev),
            "tilt_chance": lerp(0.05, 0.2, sev),    # ripped, not sunk
            "tilt_deg": [2, 5],
            "sink_m": [0.2, 0.6],
            "lean_piles": [1, 2],
        },
        "streetlights_toppled_fraction": lerp(0.15, 0.85, sev),
        "traffic_lights_toppled_fraction": lerp(0.12, 0.8, sev),
        "traffic_lights_leaning_fraction": lerp(0.2, 0.6, sev),
        "traffic_lights_lean_deg": [12, lerp(28.0, 45.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.4, 0.95, sev),
        "trash_cans_scatter_m": lerp(4.0, 14.0, sev),   # flung
        "cars_toppled_fraction": lerp(0.1, 0.6, sev),
        "cars_strewn": lerp_pair([2, 5], [10, 20], sev),
        "strewn_topple_fraction": 0.85,
        "humans_prone_fraction": lerp(0.1, 0.5, sev),
        "humans_strewn": lerp_pair([1, 3], [5, 12], sev),
        # The track: a corridor across the region, sharp edges, nothing
        # outside it. Wider and less sharply bounded as severity climbs.
        "field": {
            "kind": "path",
            "heading_deg": float(spec.get("heading_deg", 35.0)),
            "width_m": round(max(w, h) * lerp(0.08, 0.3, sev), 1),
            "falloff_m": round(max(w, h) * 0.08, 1),
            "inside": 1.0,
            "outside": 0.0,
        },
    }


def compile_explosion(sev, spec, region):
    """A blast: ground zero is obliterated, damage drops off fast with range.

    Signature — the tightest, most extreme gradient of any type. At the
    center nothing is left standing; a few hundred meters out the city is
    barely touched. Debris is thrown outward hard.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    return {
        "damaged_fraction": lerp(0.1, 0.25, sev),
        "destroyed_fraction": lerp(0.3, 0.75, sev),
        "debris": {
            "piles_per_building": lerp_pair([2, 3], [4, 7], sev),
            "pile_max_offset_m": lerp(2.5, 4.5, sev),
            "pieces_per_building": lerp_pair([8, 16], [20, 38], sev),
            "pieces_scatter_m": lerp(7.0, 16.0, sev),
            "tilt_chance": lerp(0.15, 0.45, sev),
            "tilt_deg": [3, lerp(8.0, 14.0, sev)],
            "sink_m": [0.4, lerp(1.0, 1.8, sev)],
            "lean_piles": lerp_pair([2, 3], [3, 5], sev),
        },
        "streetlights_toppled_fraction": lerp(0.3, 0.9, sev),
        "traffic_lights_toppled_fraction": lerp(0.25, 0.85, sev),
        "traffic_lights_leaning_fraction": lerp(0.3, 0.6, sev),
        "traffic_lights_lean_deg": [10, 40],
        "trash_cans_toppled_fraction": lerp(0.5, 0.95, sev),
        "trash_cans_scatter_m": lerp(5.0, 12.0, sev),
        "cars_toppled_fraction": lerp(0.2, 0.7, sev),
        "cars_strewn": lerp_pair([2, 5], [8, 16], sev),
        "strewn_topple_fraction": 0.9,
        "humans_prone_fraction": lerp(0.25, 0.75, sev),
        "humans_strewn": lerp_pair([2, 5], [8, 16], sev),
        # Small full-strength core, fast falloff, clean outside.
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(0.06, 0.22, sev), 1),
            "falloff_m": round(max(w, h) * lerp(0.15, 0.3, sev), 1),
            "inside": 1.0,
            "outside": 0.0,
        },
    }


def compile_flood(sev, spec, region):
    """Water came through: little structural loss, everything light displaced.

    Signature — buildings mostly stand (some undermined), but anything that
    floats has been carried off and dumped: cars strewn and rolled, bins
    washed away, debris deposited in the streets. No wind, so poles stand.
    """
    w, h = region
    return {
        "damaged_fraction": lerp(0.05, 0.35, sev),
        "destroyed_fraction": lerp(0.0, 0.12, sev),
        "debris": {
            "piles_per_building": lerp_pair([1, 2], [2, 4], sev),
            "pile_max_offset_m": lerp(2.5, 5.0, sev),   # deposited, not dropped
            "pieces_per_building": lerp_pair([3, 7], [10, 20], sev),
            "pieces_scatter_m": lerp(5.0, 12.0, sev),   # floated outward
            "tilt_chance": lerp(0.1, 0.4, sev),         # undermined footings
            "tilt_deg": [2, lerp(5.0, 9.0, sev)],
            "sink_m": [0.3, lerp(0.8, 1.5, sev)],
            "lean_piles": [1, 3],
        },
        "streetlights_toppled_fraction": lerp(0.02, 0.15, sev),   # poles hold
        "traffic_lights_toppled_fraction": lerp(0.02, 0.12, sev),
        "traffic_lights_leaning_fraction": lerp(0.05, 0.3, sev),
        "traffic_lights_lean_deg": [5, 20],
        "trash_cans_toppled_fraction": lerp(0.5, 0.95, sev),      # all float
        "trash_cans_scatter_m": lerp(5.0, 15.0, sev),
        "cars_toppled_fraction": lerp(0.1, 0.45, sev),            # rolled
        "cars_strewn": lerp_pair([2, 5], [10, 18], sev),
        "strewn_topple_fraction": 0.5,
        "humans_prone_fraction": lerp(0.1, 0.45, sev),
        "humans_strewn": lerp_pair([1, 3], [5, 12], sev),
        # Broad and even — water finds everywhere, so only a gentle radial
        # bias toward the low ground at the epicenter.
        "field": {
            "kind": "radial",
            "center": [float(c) for c in spec.get("epicenter", [0.0, 0.0])],
            "radius_m": round(max(w, h) * 0.5, 1),
            "falloff_m": round(max(w, h) * 0.5, 1),
            "inside": 1.0,
            "outside": lerp(0.3, 0.7, sev),
        },
    }


def compile_hurricane(sev, spec, region):
    """City-wide wind and rain: broad, even damage without a sharp edge.

    Signature — tornado-like damage mechanisms (things blown over and moved)
    but spread across the whole region at lower intensity, with no untouched
    zone and no narrow track.
    """
    return {
        "damaged_fraction": lerp(0.08, 0.4, sev),
        "destroyed_fraction": lerp(0.03, 0.3, sev),
        "debris": {
            "piles_per_building": lerp_pair([1, 2], [3, 5], sev),
            "pile_max_offset_m": lerp(2.0, 3.5, sev),
            "pieces_per_building": lerp_pair([4, 9], [14, 26], sev),
            "pieces_scatter_m": lerp(5.0, 12.0, sev),
            "tilt_chance": lerp(0.1, 0.3, sev),
            "tilt_deg": [2, 7],
            "sink_m": [0.3, 0.9],
            "lean_piles": [1, 3],
        },
        "streetlights_toppled_fraction": lerp(0.1, 0.6, sev),
        "traffic_lights_toppled_fraction": lerp(0.08, 0.5, sev),
        "traffic_lights_leaning_fraction": lerp(0.25, 0.6, sev),
        "traffic_lights_lean_deg": [10, lerp(25.0, 40.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.45, 0.9, sev),
        "trash_cans_scatter_m": lerp(3.0, 10.0, sev),
        "cars_toppled_fraction": lerp(0.05, 0.35, sev),
        "cars_strewn": lerp_pair([1, 3], [6, 12], sev),
        "strewn_topple_fraction": 0.7,
        "humans_prone_fraction": lerp(0.08, 0.45, sev),
        "humans_strewn": lerp_pair([0, 2], [4, 10], sev),
        # Uniform: the whole region is in the storm.
        "field": {"kind": "uniform", "inside": 1.0},
    }


DISASTERS = {
    "none": compile_none,
    "earthquake": compile_earthquake,
    "tornado": compile_tornado,
    "explosion": compile_explosion,
    "flood": compile_flood,
    "hurricane": compile_hurricane,
}


# ---------------------------------------------------------------------------
# Compilation
# ---------------------------------------------------------------------------

def compile_spec(spec: dict, base: dict) -> dict:
    """High-level *spec* + *base* low-level config -> low-level config."""
    dtype = str(spec.get("disaster-type", spec.get("disaster_type", "none"))).lower()
    if dtype not in DISASTERS:
        raise ValueError(
            f"unknown disaster-type {dtype!r}; expected one of "
            f"{', '.join(sorted(DISASTERS))}")

    sev = float(spec.get("severity", 1.0))
    if not 0.0 <= sev <= 1.0:
        raise ValueError(f"severity must be in [0, 1], got {sev}")

    cfg = yaml.safe_load(yaml.safe_dump(base))   # deep copy

    # City-level passthroughs, before the disaster reads the region.
    if "seed" in spec:
        cfg["seed"] = spec["seed"]
    if "region_m" in spec:
        cfg.setdefault("layout", {})["region_m"] = spec["region_m"]

    region = tuple(float(v) for v in cfg["layout"]["region_m"])

    # Severity 0 means untouched, whatever the type claims to be.
    fn = DISASTERS[dtype] if sev > 0.0 else compile_none
    cfg["disaster"] = fn(sev, spec, region)

    # Escape hatch: raw low-level overrides win over everything.
    if spec.get("overrides"):
        deep_merge(cfg, spec["overrides"])

    # Compilation is a build step; leaving the high-level keys in the output
    # would suggest editing them there has an effect. Keep them only as
    # provenance in the header comment.
    for k in ("disaster-type", "disaster_type", "severity", "overrides",
              "epicenter", "heading_deg", "presets_path", "preset_file"):
        cfg.pop(k, None)
    return cfg


def is_high_level(cfg: dict) -> bool:
    """True if *cfg* is a high-level disaster spec rather than a scene config."""
    return bool(cfg.get("disaster-type") or cfg.get("disaster_type"))


def resolve_config_path(name_or_path: str) -> str:
    """Accept a path, or a bare name looked up in the known config dirs.

    Search order: the path as given, then ``presets/`` (high level), then
    ``low_level/compiled/`` and ``low_level/`` (low level), each with and
    without a ``.yaml`` / ``.yml`` suffix. Raises FileNotFoundError listing
    what *is* available, so a typo says so instead of failing obscurely.
    """
    if os.path.isfile(name_or_path):
        return name_or_path

    search = [DEFAULT_PRESET_DIR, DEFAULT_OUT_DIR,
              os.path.join(_SCENE_GEN_DIR, "low_level")]
    stem = os.path.basename(name_or_path)
    for d in search:
        for cand in (os.path.join(d, stem),
                     os.path.join(d, stem + ".yaml"),
                     os.path.join(d, stem + ".yml")):
            if os.path.isfile(cand):
                return cand

    available = []
    for d in search:
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith((".yaml", ".yml")):
                available.append(f"    {os.path.relpath(os.path.join(d, f), _ISAAC_SIM_DIR)}")
    raise FileNotFoundError(
        f"scene config not found: {name_or_path!r}\n"
        "  available:\n" + "\n".join(available))


def load_scene_config(name_or_path: str, base_path: str = None) -> dict:
    """Load a scene config at **either** level and return a low-level dict.

    A high-level disaster spec is compiled in memory against *base_path*
    (default ``low_level/default.yaml``); a low-level config is returned as
    is. Either way the result is validated the way scene_generator expects,
    so launch scripts can point ``SCENE_CONFIG`` at whichever is convenient:

        SCENE_CONFIG = ".../presets/tornado.yaml"            # high level
        SCENE_CONFIG = ".../low_level/compiled/tornado.yaml" # low level
        SCENE_CONFIG = "tornado"                             # by name

    Compiling in memory means a high-level spec always reflects the current
    ``default.yaml``, with no stale compiled artifact in between — at the
    cost of no on-disk record of what ran. Run ``compile_disaster.py`` to
    get that record.
    """
    path = resolve_config_path(name_or_path)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: config must be a mapping")

    if is_high_level(cfg):
        base_path = base_path or DEFAULT_BASE
        with open(base_path) as f:
            base = yaml.safe_load(f)
        cfg = compile_spec(cfg, base)
        print(f"[compile_disaster] compiled high-level spec in memory: "
              f"{os.path.relpath(path, _ISAAC_SIM_DIR)} "
              f"(base {os.path.relpath(base_path, _ISAAC_SIM_DIR)})")
    else:
        print(f"[compile_disaster] loaded low-level config: "
              f"{os.path.relpath(path, _ISAAC_SIM_DIR)}")

    # Lazy so `--list` and plain compilation stay free of the pxr dependency.
    import scene_generator
    return scene_generator.validate_config(cfg, path)


def _header(spec: dict, source: str, base: str) -> str:
    dtype = spec.get("disaster-type", spec.get("disaster_type", "none"))
    sev = spec.get("severity", 1.0)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    extras = {k: v for k, v in spec.items()
              if k not in ("disaster-type", "disaster_type", "severity")}
    lines = [
        "# GENERATED FILE — do not edit by hand.",
        "#",
        "# Low-level scene config compiled by utils/compile_disaster.py.",
        "# Edit the high-level spec (or low_level/default.yaml) and recompile:",
        f"#     python3 utils/compile_disaster.py {os.path.basename(source)}",
        "#",
        f"# disaster-type : {dtype}",
        f"# severity      : {sev}",
        f"# spec          : {source}",
        f"# base          : {base}",
        f"# compiled      : {stamp}",
    ]
    if extras:
        lines.append(f"# spec extras   : {extras}")
    return "\n".join(lines) + "\n\n"


def compile_file(spec_path: str, base_path: str, out_path: str) -> str:
    with open(spec_path) as f:
        spec = yaml.safe_load(f) or {}
    if not isinstance(spec, dict):
        raise ValueError(f"{spec_path}: high-level spec must be a mapping")
    with open(base_path) as f:
        base = yaml.safe_load(f)

    cfg = compile_spec(spec, base)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(_header(spec, os.path.relpath(spec_path, _ISAAC_SIM_DIR),
                        os.path.relpath(base_path, _ISAAC_SIM_DIR)))
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False,
                       width=100)
    return out_path


def main():
    ap = argparse.ArgumentParser(
        description="Compile high-level disaster specs into low-level scene configs.")
    ap.add_argument("specs", nargs="*",
                    help="high-level spec paths or bare names (default: every "
                         f"*.yaml in {os.path.relpath(DEFAULT_PRESET_DIR, _ISAAC_SIM_DIR)})")
    ap.add_argument("--base", default=DEFAULT_BASE,
                    help="low-level base config to build on")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR,
                    help="where compiled configs are written")
    ap.add_argument("--output", default=None,
                    help="explicit output path (single spec only)")
    ap.add_argument("--list", action="store_true",
                    help="list the known disaster types and exit")
    args = ap.parse_args()

    if args.list:
        print("disaster types:")
        for name, fn in sorted(DISASTERS.items()):
            summary = (fn.__doc__ or "").strip().splitlines()[0]
            print(f"  {name:<11} {summary}")
        return

    specs = args.specs
    if not specs:
        specs = sorted(os.path.join(DEFAULT_PRESET_DIR, f)
                       for f in os.listdir(DEFAULT_PRESET_DIR)
                       if f.endswith((".yaml", ".yml")))
        if not specs:
            raise SystemExit(f"no specs found in {DEFAULT_PRESET_DIR}")
    else:
        # Accept bare names ("tornado") as well as paths.
        resolved = []
        for s in specs:
            if os.path.isfile(s):
                resolved.append(s)
                continue
            cand = os.path.join(DEFAULT_PRESET_DIR, s)
            for c in (cand, cand + ".yaml", cand + ".yml"):
                if os.path.isfile(c):
                    resolved.append(c)
                    break
            else:
                raise SystemExit(f"spec not found: {s}")
        specs = resolved

    if args.output and len(specs) > 1:
        raise SystemExit("--output takes a single spec")

    for spec_path in specs:
        name = os.path.splitext(os.path.basename(spec_path))[0]
        out = args.output or os.path.join(args.out_dir, f"{name}.yaml")
        compile_file(spec_path, args.base, out)
        print(f"[compile_disaster] {os.path.relpath(spec_path, _ISAAC_SIM_DIR)}"
              f"  ->  {os.path.relpath(out, _ISAAC_SIM_DIR)}")


if __name__ == "__main__":
    sys.exit(main())
