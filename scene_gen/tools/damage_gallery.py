#!/usr/bin/env python3
"""Preview what a disaster does to a building — one asset at a time, or a grid.

Builds a tiny scene per (building, disaster type): the building at the origin,
wrecked by the real damage pipeline, standing in the debris the real disaster
stage would drop around it. Then renders them into a contact sheet where each
ROW is a building and each COLUMN is a disaster type, pristine on the far left.

    ┌──────────┬────────────┬─────────┬───────────┬──────┬───────────┬───────┐
    │ pristine │ earthquake │ tornado │ hurricane │ fire │ explosion │ flood │
    ├──────────┼────────────┼─────────┼───────────┼──────┼───────────┼───────┤
    │  house A │            │         │           │      │           │       │
    │  house B │            │         │           │      │           │       │
    └──────────┴────────────┴─────────┴───────────┴──────┴───────────┴───────┘

`--bare-columns` pairs each disaster with a debris-free twin, so the two things
a damaged cell shows can be read apart — what the mesh damage did to the
building, and what the disaster stage scattered around it. They are produced by
different code and tuned by different knobs, and with a full debris ring in
place the building is largely hidden behind it.

    ┌──────────┬───────────────┬────────────┬───────────────┬─────────┬─────┐
    │ pristine │ earthquake    │ earthquake │ tornado       │ tornado │ ... │
    │          │ (mesh only)   │            │ (mesh only)   │         │     │
    └──────────┴───────────────┴────────────┴───────────────┴─────────┴─────┘

The twin is not a re-roll. The disaster stage runs in full and the debris is
discarded AFTERWARDS, so the building in the two cells is bit-identical and the
pair differs in exactly one variable — see `build_cell`.

WHY THE REAL PIPELINE AND NOT A MOCK
------------------------------------
Every cell is built by calling `disaster_stage.apply_to_buildings` and
`mesh_damage.apply_to_stage` on a one-building placement list — the same two
functions `generate_scene` calls, in the same order, off the same compiled
config `compile_disaster.py` emits for that type. So the sheet is evidence about
the generator rather than about this script: if a failure field stops
breaking anything or a debris knob stops being read, the gallery shows it.

The one thing forced is the FATE. `apply_to_buildings` normally rolls
damaged/destroyed/untouched per building against the damage field, which for a
gallery would leave cells randomly blank. Here the field is pinned to 1.0 and
the fate to `--fate`, so intensity is exactly `--severity` and every cell is
populated. Nothing else is special-cased.

WHY TWO INTERPRETERS
--------------------
This half needs `pxr` and runs on the host python (usd-core). Rendering needs
`bpy`, which ships only for 3.13 and bundles its own USD — the two cannot share
a process (see the note at the top of `disaster/mesh_damage.py`). So this writes
USDs plus a manifest, then shells out to `render_damage_gallery.py` under
`uv run --script`, exactly as `render_usd.py` is run. `--no-render` stops after
the USDs.

ASSETS MUST BE LOCAL
--------------------
Nucleus (`omniverse://`) does not resolve under plain usd-core or under Blender,
so a gallery can only be built from assets that are on disk: the `objaverse://`
cache (`prepare_assets.py`) and the repo's own `airstack://` packs. `--list`
says which of an asset set's buildings qualify.

**suburban** works out of the box — all 15 of its houses and all 8 of its
debris assets are cached. The **urban** library lives on Nucleus and reports
`0/9`; for that side, `tools/localize_nucleus_assets.py` mirrors it down (from
inside the isaac-sim container, the only place with the resolver) and
`--asset-set urban_intact_local` points at the mirror.

WHAT A SHEET IS EVIDENCE OF
---------------------------
Both halves of the library are worth looking at, because they fail differently.
The objaverse houses are hollow SHELLS, so `solidify` is doing most of the work
in a suburban sheet — compare against `--wall-thickness 0` and the rubble goes
from chunks to paper. The nine `urban_intact` buildings are already closed
solids (7-34x the volume a 0.25 m slab of their surface would enclose), so
`solidify` declines every one of them and the sheet reports `thickened=0`.
That is the operator working, not failing, and the sheet is where you can see
which case an asset is in.

Usage
-----
    python3 tools/damage_gallery.py                    # the default sheet
    python3 tools/damage_gallery.py --list             # what can be built
    python3 tools/damage_gallery.py --rows 6 --severity 0.9
    python3 tools/damage_gallery.py --one 3 --disaster tornado    # one cell
    python3 tools/damage_gallery.py --no-render        # USDs only
    python3 tools/damage_gallery.py --wall-thickness 0 # the "before" sheet
    python3 tools/damage_gallery.py --bare-columns     # + debris-free twins
    python3 tools/damage_gallery.py --asset-set urban_intact_local --rows 9

Run with the system `python3`, not `AirStack/.venv` — same reason
`preset_report.py` gives: `scene_generator` imports `pxr` at module scope and
the system interpreter is the one with `usd-core`.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS_DIR)
sys.path.insert(0, _SCENE_GEN)

import yaml                                                     # noqa: E402

import compile_disaster as cd                                   # noqa: E402
import scene_generator as sg                                    # noqa: E402
from disaster import disaster_stage, mesh_damage                 # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdShade                       # noqa: E402

#: Pristine first, then the taxonomy. `none` is not a column — the pristine
#: cell IS the `none` case, and rendering it twice would waste a sixth of the
#: sheet on two identical tiles.
COLUMNS = ["pristine", "earthquake", "tornado", "hurricane", "fire",
           "explosion", "flood"]

DEFAULT_OUT = os.path.join(_SCENE_GEN, "galleries", "damage")

#: Keys in `mesh_damage`'s tally that count things rather than name the
#: failure field that ran. Everything else in it is a field name.
_TALLY_COUNTERS = ("fragments", "loose", "shattered", "thickened",
                   "already_solid", "unbroken")


# ---------------------------------------------------------------------------
# asset selection
# ---------------------------------------------------------------------------


def load_asset_set(asset_set: str) -> dict:
    """A validated config carrying *asset_set*'s pools and conventions."""
    with open(cd.DEFAULT_BASE) as fh:
        cfg = yaml.safe_load(fh)
    cfg["asset_set"] = asset_set
    anchor = os.path.join(_SCENE_GEN, "config", "asset_sets",
                          f"{asset_set}.yaml")
    with contextlib.redirect_stdout(io.StringIO()):
        cfg = sg.resolve_asset_set(cfg, anchor)
        cfg = sg.validate_config(cfg, anchor)
    return cfg


def _slug(label: str) -> str:
    """A column label as a filename. Labels carry spaces and parentheses now
    that a column can be `earthquake (mesh only)`, and those are a nuisance on
    disk and in the shell for no gain — the manifest keeps the real label."""
    return re.sub(r"[^A-Za-z0-9._-]+", "_", label).strip("_") or "cell"


def _on_disk(path: str) -> bool:
    """Whether a resolved USD reference actually exists locally."""
    local = sg._expand_scheme(path) or (path if path.startswith("/") else None)
    return bool(local) and os.path.exists(local)


#: `objaverse://8dbf...` on one line, `# Bungalow The Chase` at the end of it.
_LABEL_RE = re.compile(r"""(?:objaverse://)?([0-9a-fA-F]{32}|[\w.-]+\.usdc?)"""
                       r"""["' ]*\s*#\s*(.+?)\s*$""")


def asset_labels() -> dict:
    """Human names for assets, scraped off the asset sets' own comments.

    A row labelled `6644de89c2f0449db3de934744162b63` says nothing; the set
    that references it already says "Bungalow The Chase" in the comment beside
    it, and that comment is the only place the name exists — an Objaverse asset
    is identified by uid and nothing else. Cheap to read, and wrong only in the
    direction of falling back to the uid.
    """
    out: dict = {}
    d = os.path.join(_SCENE_GEN, "config", "asset_sets")
    for fn in sorted(os.listdir(d)):
        if not fn.endswith(".yaml"):
            continue
        with open(os.path.join(d, fn)) as fh:
            for line in fh:
                m = _LABEL_RE.search(line.strip())
                if m and not m.group(2).startswith(("noqa", "TODO")):
                    out.setdefault(m.group(1), m.group(2)[:34])
    return out


def buildings_of(cfg: dict) -> list:
    """``[{name, usd, scale, axis_up, local}, ...]`` from `buildings.intact`.

    Keeps unresolvable entries in the list rather than dropping them, so
    `--list` can say *why* a set produces a short gallery instead of silently
    producing one.
    """
    usds = cfg.get("usds") or {}
    pool = (usds.get("buildings") or usds.get("houses") or {}).get("intact")
    paths, scale_ovr, axis_ovr, _yaw, _tags = sg._normalize_usd_list(
        pool or [], float(cfg.get("asset_scale", 1.0)),
        str(cfg.get("asset_root", "") or ""))
    labels = asset_labels()
    out = []
    for p in paths:
        stem = os.path.splitext(os.path.basename(p))[0]
        out.append({
            "name": labels.get(stem) or labels.get(os.path.basename(p))
            or stem[:28],
            "usd": p,
            "scale": float(scale_ovr.get(p, cfg.get("asset_scale", 1.0))),
            "axis_up": str(axis_ovr.get(p, "Z")),
            "local": _on_disk(p),
        })
    return out


# ---------------------------------------------------------------------------
# one cell
# ---------------------------------------------------------------------------


def cell_config(base: dict, disaster: str, severity: float, seed: int,
                fate: str, wall_m: float = None) -> dict:
    """*base* with the compiled `disaster:` block for one gallery cell.

    Two deliberate overrides, both so the cell is populated and comparable
    rather than left to chance:

    * **the field is pinned uniform at 1.0**, so the local intensity every knob
      is scaled by is exactly *severity*. Left alone, a lone building at the
      origin would read whatever that type's field says there — 1.0 for a
      radial epicentre, but a tornado corridor is placed by heading and a
      building can miss it entirely.
    * **the fate is forced**, because `apply_to_buildings` otherwise rolls
      damaged/destroyed/untouched per building and a third of the sheet would
      come out pristine.

    `heading_deg` is carried across onto the replacement field, because that is
    where the wind profiles read the storm bearing from; dropping it would give
    every cell its own random bearing.

    The third override is not a choice: `default.yaml` keeps a 10 m clutter
    keep-out at the origin for the takeoff pad, and a gallery stands its
    building at the origin — so the pad silently ate almost every piece of
    debris, and the cells came out with wrecked buildings on clean ground. A
    gallery has no drone to take off.
    """
    cfg = dict(base)
    cfg["seed"] = int(seed)
    cfg["exclusions"] = []
    if disaster == "pristine":
        cfg["disaster"] = {"type": "none", "severity": 0.0,
                           "field": {"kind": "uniform", "inside": 0.0}}
        return cfg

    blk = cd.DISASTERS[disaster](float(severity), {}, (60.0, 60.0))
    blk["type"] = disaster
    blk["severity"] = float(severity)
    heading = (blk.get("field") or {}).get("heading_deg", 35.0)
    blk["field"] = {"kind": "uniform", "inside": 1.0, "heading_deg": heading}
    blk["damaged_fraction"] = 0.0 if fate == "destroyed" else 1.0
    blk["destroyed_fraction"] = 1.0 if fate == "destroyed" else 0.0
    if wall_m is not None:
        # The fourth override, and the only one that is a knob rather than a
        # correction: `--wall-thickness 0` is how the sheet is rebuilt as the
        # before picture, since a gallery is the only place the two are
        # comparable side by side.
        md = dict(blk.get("mesh_damage") or {})
        md["thickness"] = dict(md.get("thickness") or {},
                               enabled=wall_m > 0.0, wall_m=float(wall_m))
        blk["mesh_damage"] = md
    cfg["disaster"] = blk
    return cfg


def topple_settling_props(placements: list, seed: int) -> None:
    """Lay flat everything the real pipeline would have let PhysX rest.

    `scene_prep.settle_rigid_props` drops every `settle`-marked prop under
    physics and freezes it where it lands. That is an Isaac Sim pass — it plays
    the timeline — so a host-side gallery cannot run it, and without *something*
    in its place the preview lies in a specific and very visible way: the
    debris planks are scanned standing on end (the suburban set says so in a
    comment beside them, and pins their length rather than their footprint for
    exactly this reason), and every piece is authored floating 0.4 m up waiting
    to be dropped. The first sheet came out with wrecked houses ringed by
    what looked like fence posts hovering over clean ground.

    So this rolls them over and `drop_to_ground` puts them down. It is an
    approximation of the pass, not the pass, and the sheet says so.
    """
    import random as _random

    rng = _random.Random(int(seed) + 991)
    for p in placements:
        if not p.get("settle"):
            continue
        p["roll_deg"] = float(p.get("roll_deg", 0.0)) + \
            rng.choice([-1.0, 1.0]) * rng.uniform(72.0, 108.0)
        p["pitch_deg"] = float(p.get("pitch_deg", 0.0)) + rng.uniform(-20, 20)


def drop_to_ground(stage, prim_paths, ground_z: float = 0.0) -> int:
    """Sit each prim's lowest point on the ground. Returns how many moved.

    The other half of the settle stand-in, and the one that matters for
    FRAGMENTS: they are cut where the geometry was and thrown from there, so
    until physics rests them a torn-off roof hangs in mid-air exactly where it
    used to be. Dropping each piece straight down is not a pile — pieces do not
    stack or collide — but it puts the debris on the ground, which is the
    difference between a preview that reads and one that does not.
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    moved = 0
    for path in prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        dz = ground_z - box.GetMin()[2]
        if abs(dz) < 1e-6:
            continue
        # The drop is measured in WORLD metres but a translate op is applied in
        # the prim's own space, and a fragment's parent is the building — which
        # carries the placement's scale. On the centimetre-authored packs
        # (`scale: 0.01`) a world metre is a hundred local units, so writing the
        # world delta straight in would fling the debris a hundred times too
        # far. Rotate and scale it into the parent's frame first.
        delta = Gf.Vec3d(0.0, 0.0, dz)
        parent = prim.GetParent()
        if parent and parent.IsValid():
            to_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            delta = to_world.GetInverse().TransformDir(delta)

        # Fragments are meshes with no transform of their own, so they take a
        # translate op; placed props already have one and must be added to
        # rather than overwritten.
        xf = UsdGeom.Xformable(prim)
        existing = next((o for o in xf.GetOrderedXformOps()
                         if o.GetOpType() == UsdGeom.XformOp.TypeTranslate),
                        None)
        if existing is None:
            xf.AddTranslateOp().Set(delta)
        else:
            cur = existing.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
            existing.Set(Gf.Vec3d(cur[0] + delta[0], cur[1] + delta[1],
                                  cur[2] + delta[2]))
        moved += 1
    return moved


def soot_map(stage) -> dict:
    """``{material name: albedo multiplier}`` for everything `scorch` darkened.

    Soot is authored where USD composition will actually consult it — as
    `inputs:scale` on the `UsdUVTexture` feeding albedo, because the shader's
    `diffuseColor` is *connected* and a value there is ignored. Hydra and Isaac
    honour that. **Blender's USD importer silently drops it**: it builds an
    Image Texture straight into Base Color and no multiply anywhere, so a
    burnt-out building imports at full brightness and the gallery would show a
    fire that chars nothing.

    Rather than let the preview under-report the effect, the factor is read
    back off the stage here and re-applied by the renderer. Multipliers at or
    above 1.0 are ignored: assets ship non-unit scales on their *normal* maps
    (2,2,2,2 is common) and those are not soot.
    """
    out: dict = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        if shader.GetIdAttr().Get() != "UsdUVTexture":
            continue
        scale = shader.GetInput("scale")
        if not scale:
            continue
        val = scale.Get()
        if val is None or float(val[0]) >= 1.0:
            continue
        name = prim.GetParent().GetName()
        out[name] = min(out.get(name, 1.0), round(float(val[0]), 4))
    return out


#: Placement categories the disaster stage scatters AROUND a building, as
#: opposed to the building itself. `disaster_stage.apply_to_buildings` tags
#: them as it appends them, and these are the two it uses.
_DEBRIS_CATEGORIES = ("debris", "debris_pile")


def build_cell(base: dict, building: dict, disaster: str, severity: float,
               seed: int, fate: str, out_path: str, settle: bool = True,
               wall_m: float = None, debris: bool = True) -> dict:
    """Wreck one building into one USD. Returns what happened to it.

    The order is `generate_scene_on_stage`'s, and it is load-bearing: the
    disaster stage runs in pure Python first (it decides fate and drops
    debris), `apply_placements` composes the references, and only then does
    mesh damage run — it authors overrides on geometry inside a referenced
    layer, and there is nothing to override until the reference is composed.
    """
    cfg = cell_config(base, disaster, severity, seed, fate, wall_m)
    resolver = sg._make_resolver(cfg)
    fp = resolver.get(building["usd"], "house", scale=building["scale"],
                      axis_up=building["axis_up"])
    placements = [{
        "usd": building["usd"], "x_m": 0.0, "y_m": 0.0, "z_m": fp["base"],
        "yaw_deg": 0.0, "roll_deg": 90.0 if building["axis_up"] == "Y" else 0.0,
        "pitch_deg": 0.0, "scale": building["scale"], "category": "house",
        "axis_up": building["axis_up"],
        "_footprint_m": (fp["sx"], fp["sy"]),
    }]
    # A region big enough that the debris ring is never clipped by it.
    span = max(fp["sx"], fp["sy"]) * 6.0 + 40.0
    layout = {"region": (-span / 2, -span / 2, span / 2, span / 2)}

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    with contextlib.redirect_stdout(io.StringIO()):
        if disaster != "pristine":
            disaster_stage.apply_to_buildings(cfg, layout, placements, resolver)
        if not debris:
            # DROPPED AFTER THE FACT, not suppressed in the config, and that is
            # the whole reason this is trustworthy as a comparison. Turning the
            # debris counts down would change how many draws the disaster stage
            # takes off the RNG, so the building beside it would be damaged
            # differently and the pair would no longer differ in one variable.
            # Letting the stage run in full and then discarding what it
            # scattered leaves the building bit-identical to its twin.
            #
            # Order is preserved and the building was the only entry before the
            # stage appended to it, so it keeps index 0 — which matters,
            # because `mesh_damage.apply_to_stage` seeds each building off its
            # placement index.
            placements[:] = [p for p in placements
                             if p.get("category") not in _DEBRIS_CATEGORIES]
        if settle:
            topple_settling_props(placements, seed)
        sg.apply_placements(stage, placements, "/World/gen", 1.0, None,
                            resolver)
        mesh = (mesh_damage.apply_to_stage(stage, cfg, placements)
                if disaster != "pristine" else {"tally": {}, "fragments": []})
        if settle:
            # Only the LOOSE fragments fall — the anchored ones are still
            # standing on their footings, and dropping those is exactly the
            # mistake `fracture_to_stage` documents.
            drop_to_ground(stage, list(mesh.get("loose", ()))
                           + [p["prim_path"] for p in placements
                              if p.get("settle") and p.get("prim_path")])
    soot = soot_map(stage)
    stage.GetRootLayer().Save()

    cats: dict = {}
    for p in placements:
        cats[p["category"]] = cats.get(p["category"], 0) + 1
    return {
        "usd": out_path,
        "footprint_m": [round(fp["sx"], 2), round(fp["sy"], 2)],
        "height_m": round(fp["sz"], 2),
        "debris": cats.get("debris", 0),
        "debris_piles": cats.get("debris_pile", 0),
        "fragments": len(mesh["fragments"]),
        # How many of them came free of the structure, which is the number that
        # says whether this building collapsed or merely cracked.
        "loose": len(mesh.get("loose", ())),
        # The failure field that ran, which is the only non-counter key the
        # tally carries.
        "field": next((k for k in mesh["tally"]
                       if k not in _TALLY_COUNTERS), None),
        # Whether the walls were given volume before being broken. Reported per
        # cell rather than taken on trust from the flag, because `solidify`
        # declines meshes that already enclose material.
        "thickened": int(mesh["tally"].get("thickened", 0)),
        # Charring, for the renderer to re-apply — see `soot_map`.
        "soot": soot,
    }


# ---------------------------------------------------------------------------
# the sheet
# ---------------------------------------------------------------------------


#: Suffix marking a column that shows the building alone. Also how the
#: renderer tells the two apart when it colours the headings.
BARE_SUFFIX = " (mesh only)"


def _twin(col: tuple) -> list:
    """*col*, preceded by a debris-free copy of itself.

    The bare one goes FIRST so a row reads left to right as "what the mesh
    damage did" then "what it looks like once the disaster stage has dressed
    it" — the debris only ever adds, so that is the order in which the sheet
    explains itself.
    """
    label, dtype, sev, _debris = col
    return [(label + BARE_SUFFIX, dtype, sev, False), col]


def disaster_columns(severity: float, types=None, bare: bool = False) -> list:
    """The default axis: pristine, then one column per disaster type.

    A column is ``(label, disaster, severity, debris)``. Keeping the axis as
    data rather than as a list of type names is what lets the same builder
    produce a severity sweep, where the columns are one disaster at several
    severities — the other question worth asking of this generator, since "the
    same city at different damage levels" is the invariant the whole project
    rests on.

    *bare* pairs each disaster with a debris-free twin. Worth the doubled
    width because the two things a damaged cell shows — what the mesh damage
    did to the building, and what the disaster stage scattered around it — are
    produced by different code and tuned by different knobs, and with the
    debris in place the building is largely hidden behind it.
    """
    cols = [(t, t, float(severity), True) for t in (types or COLUMNS[1:])]
    if bare:
        cols = [c for col in cols for c in _twin(col)]
    return [("pristine", "pristine", 0.0, True)] + cols


def severity_columns(disaster: str, severities, bare: bool = False) -> list:
    """One disaster across several severities — pristine still on the left."""
    cols = [(f"{float(s):g}", disaster, float(s), True) for s in severities]
    if bare:
        cols = [c for col in cols for c in _twin(col)]
    return [("pristine", "pristine", 0.0, True)] + cols


def build_gallery(asset_set="suburban", rows=5, seed=42, fate="damaged",
                  specs=None, out_dir=DEFAULT_OUT, pick=None, settle=True,
                  title="Procedural building damage", quiet=False,
                  wall_m=None) -> dict:
    """Build every cell and write the manifest the renderer consumes.

    *pick* is a list of indices into the set's local buildings, for looking at
    one asset closely; otherwise the first *rows* of them (0 for all).
    """
    specs = list(specs or disaster_columns(0.8))
    base = load_asset_set(asset_set)
    base["measure_usds"] = True

    pool = [b for b in buildings_of(base) if b["local"]]
    if not pool:
        raise SystemExit(
            f"no locally-resolvable buildings in asset set {asset_set!r}. "
            "Run `prepare_assets.py` for its objaverse assets, or pick a set "
            "whose buildings are on disk — see --list.")
    chosen = ([pool[i % len(pool)] for i in pick] if pick
              else (pool[:rows] if rows > 0 else pool))

    specs = [tuple(sp) + (True,) * (4 - len(sp)) for sp in specs]
    sevs = sorted({sev for _, d, sev, _ in specs if d != "pristine"})
    sev_txt = (f"severity {sevs[0]:g}" if len(sevs) == 1
               else f"severity {sevs[0]:g}–{sevs[-1]:g}")
    settled = ("debris settled (approximated — Isaac's PhysX pass is not "
               "available host-side)" if settle
               else "debris NOT settled: authored poses")
    walls = ("walls left as shipped: zero-thickness shells" if wall_m == 0.0
             else f"walls thickened to {wall_m:g} m before breaking"
             if wall_m else "walls thickened (pipeline default)")
    manifest = {
        "title": title,
        "subtitle": (f"asset set {asset_set} · {sev_txt} · fate {fate} · "
                     f"seed {seed} · {walls} · {settled}"),
        "asset_set": asset_set, "seed": seed, "fate": fate,
        "settle": bool(settle), "wall_m": wall_m,
        "bare_suffix": BARE_SUFFIX,
        "columns": [label for label, _, _, _ in specs], "rows": [],
    }
    for r, b in enumerate(chosen):
        row = {"name": b["name"], "cells": {}, "stats": {}}
        for label, dtype, sev, debris in specs:
            path = os.path.join(out_dir, "usd", f"{r:02d}_{b['name']}",
                                f"{_slug(label)}.usda")
            # One seed per ROW, shared by its columns: the point of a row is
            # comparing damage on the same building, so anything not driven by
            # the column itself must be held still across it.
            info = build_cell(base, b, dtype, sev, seed + r * 1000, fate,
                              path, settle=settle, wall_m=wall_m,
                              debris=debris)
            row["cells"][label] = os.path.relpath(path, out_dir)
            row["stats"][label] = info
            if not quiet:
                bits = (f"frag={info['fragments']:3d} "
                        f"debris={info['debris']:3d}+{info['debris_piles']} "
                        f"solid={info['thickened']}"
                        if dtype != "pristine" else "—")
                print(f"  {b['name'][:24]:24s} {label[:24]:24s} {bits}")
        manifest["rows"].append(row)

    os.makedirs(out_dir, exist_ok=True)
    mpath = os.path.join(out_dir, "manifest.json")
    with open(mpath, "w") as fh:
        json.dump(manifest, fh, indent=1)
        fh.write("\n")
    return manifest


def render_gallery(out_dir=DEFAULT_OUT, res=520, samples=48, az=38.0, el=20.0,
                   cpu=False) -> int:
    """Hand the manifest to the bpy-side renderer. Returns its exit code."""
    script = os.path.join(_TOOLS_DIR, "render_damage_gallery.py")
    cmd = ["uv", "run", "--script", script,
           os.path.join(out_dir, "manifest.json"),
           "--res", str(res), "--samples", str(samples),
           "--az", str(az), "--el", str(el)]
    if cpu:
        cmd.append("--cpu")
    print("[gallery] " + " ".join(cmd))
    return subprocess.call(cmd)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--asset-set", default="suburban")
    ap.add_argument("--rows", type=int, default=5,
                    help="buildings in the sheet; 0 for every local one")
    ap.add_argument("--severity", type=float, default=0.8)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--fate", choices=("damaged", "destroyed"),
                    default="damaged",
                    help="'damaged' previews the mesh-damage path (the "
                         "default the generator prefers); 'destroyed' the "
                         "ruin-asset swap")
    ap.add_argument("--one", type=int, metavar="N",
                    help="only building N of the set (with --disaster)")
    ap.add_argument("--disaster", action="append",
                    help="restrict the columns; repeatable")
    ap.add_argument("--sweep", metavar="TYPE",
                    help="columns become severities of one disaster instead "
                         "of the disaster types — the sweep that has to stay "
                         "comparable for a severity study to mean anything")
    ap.add_argument("--severities", default="0.2,0.4,0.6,0.8,1.0",
                    help="with --sweep")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--res", type=int, default=520)
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--az", type=float, default=38.0)
    ap.add_argument("--el", type=float, default=20.0)
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--bare-columns", action="store_true",
                    help="pair every disaster with a debris-free twin column "
                         "showing the building alone, so mesh damage can be "
                         "read separately from what the disaster stage "
                         "scattered around it. Doubles the sheet width")
    ap.add_argument("--title",
                    help="heading on the sheet; defaults to one describing "
                         "the axis (the disaster types, or the sweep)")
    ap.add_argument("--wall-thickness", type=float, default=None,
                    metavar="M",
                    help="wall thickness in metres for `solidify`, which gives "
                         "the shell volume before it is broken. 0 disables it, "
                         "which is how the 'before' sheet is built; omit to "
                         "take the pipeline default")
    ap.add_argument("--no-settle", action="store_true",
                    help="skip the stand-in for Isaac's PhysX settle pass, "
                         "leaving debris and fragments in their authored "
                         "poses (floating, and planks on end)")
    ap.add_argument("--list", action="store_true",
                    help="which of the set's buildings can be built, and why")
    args = ap.parse_args()

    if args.list:
        cfg = load_asset_set(args.asset_set)
        rows = buildings_of(cfg)
        n = sum(1 for b in rows if b["local"])
        print(f"{args.asset_set}: {n}/{len(rows)} buildings resolve locally")
        for i, b in enumerate(rows):
            print(f"  [{i:2d}] {'ok ' if b['local'] else 'REMOTE'}  "
                  f"{b['name']:30s} {b['usd']}")
        if n < len(rows):
            print("\nREMOTE entries live on Nucleus, which neither usd-core "
                  "nor Blender can resolve.\nRun prepare_assets.py for "
                  "objaverse:// assets, or use --asset-set suburban.")
        return 0

    if args.sweep:
        specs = severity_columns(
            args.sweep, [float(v) for v in args.severities.split(",")],
            bare=args.bare_columns)
        title = f"{args.sweep.capitalize()} across severity"
    else:
        types = [t for t in (args.disaster or COLUMNS[1:])
                 if t != "pristine"]
        specs = disaster_columns(args.severity, types, bare=args.bare_columns)
        title = "Procedural building damage"

    man = build_gallery(asset_set=args.asset_set, rows=args.rows,
                        seed=args.seed, fate=args.fate, specs=specs,
                        out_dir=args.out, settle=not args.no_settle,
                        title=args.title or title,
                        wall_m=args.wall_thickness,
                        pick=[args.one] if args.one is not None else None)
    print(f"[gallery] {len(man['rows'])} rows x {len(specs)} columns "
          f"-> {args.out}")
    if args.no_render:
        return 0
    return render_gallery(args.out, args.res, args.samples, args.az, args.el,
                          args.cpu)


if __name__ == "__main__":
    raise SystemExit(main())
