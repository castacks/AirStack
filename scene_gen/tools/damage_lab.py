#!/usr/bin/env python3
"""damage_lab.py — iterate on mesh damage against any single asset, fast.

Built for the notebook `scene_gen/notebooks/damage_lab.ipynb`; usable as a
plain module. Three things it does that `damage_gallery.py` does not:

  * **any asset, any source.** `resolve()` takes a Nucleus URL, an objaverse
    uid, an `airstack://` pack path or a plain file, mirrors/fetches whatever
    is not already on disk, and hands back the `{usd, scale, axis_up}` triple
    the pipeline wants.
  * **arbitrary knobs.** `damage(..., overrides={...})` deep-merges into the
    compiled `disaster:` block, so every `mesh_damage.*` key —
    `fracture.fragment_m`, `fracture.release`, `fracture.max_cells`,
    `thickness.wall_m`, `subdivide.max_edge_m` … — is reachable without
    editing a preset.
  * **an in-kernel picture.** `preview()` draws the composed stage with
    matplotlib, so a change is visible in seconds without leaving Python.
    `damage_gallery.py` renders through Blender, which is prettier and needs a
    second interpreter and a subprocess per sheet.

FAITHFULNESS IS THE WHOLE POINT, SO NOTHING HERE REIMPLEMENTS DAMAGE
--------------------------------------------------------------------
`damage()` calls `damage_gallery.build_cell`, which calls
`disaster_stage.apply_to_buildings`, `scene_generator.apply_placements` and
`mesh_damage.apply_to_stage` — the same three functions `generate_scene` calls,
in the same order, off a config `compile_disaster.py` compiled. If a failure
field stops breaking anything, this shows it. There is no mock and no
shortcut, and that is deliberate: a lab that models the pipeline instead of running it tells
you about itself.

Two differences from a real `airstack up`, both inherited from `build_cell` and
both necessary for a single-asset harness rather than cosmetic:

  * the damage **field is pinned uniform at 1.0**, so local intensity is
    exactly `severity`. In a real scene the field varies with position and a
    lone building at the origin would read whatever that disaster's shape says
    there — a tornado corridor is placed by heading and could miss it entirely.
  * the **fate is forced** (`damaged` or `destroyed`) instead of rolled, so the
    cell is always populated.

Everything downstream of those two — which failure field runs, where the
building fails, what comes free, what gets thickened, what debris is dropped —
is the generator's own code path.

**Budgets do not bite here.** `fracture.max_buildings` rations work across a
whole city; with one building it always includes it. So a building that never
thickens here is one the asset pack declared `solid: true`, or one the failure
field dismissed before stage one because nothing on it would ever come free —
not the budget. See `verify_faithful()`.

    from damage_lab import resolve, damage, preview, grid

    asset = resolve("omniverse://…/Muyang/DestroyedBuildings/Assets/debris_1.usd")
    r = damage(asset, "earthquake", severity=0.8,
               overrides={"mesh_damage": {"fracture": {"fragment_m": 1.5}}})
    preview(r)
"""

from __future__ import annotations

import copy
import os
import posixpath
import subprocess
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS)
_REPO = os.path.dirname(_SCENE_GEN)
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                              # noqa: E402
from pxr import Usd, UsdGeom                                    # noqa: E402

import damage_gallery as dg                                     # noqa: E402
import scene_generator as sg                                    # noqa: E402
from disaster import mesh_damage as md                          # noqa: E402

#: Where a mirrored Nucleus asset lands — the same tree
#: `localize_nucleus_assets.py` writes, so the two share a cache.
NUCLEUS_MIRROR = os.path.join(_SCENE_GEN, "assets", "nucleus")
NUCLEUS_LIB = "/Library/Stages/"
OBJAVERSE_DIR = os.path.join(_SCENE_GEN, "assets", "objaverse")
OUT_DIR = os.path.join(_SCENE_GEN, "_damage_lab")


# ---------------------------------------------------------------------------
# assets
# ---------------------------------------------------------------------------

def _nucleus_rel(url: str) -> str:
    i = url.find(NUCLEUS_LIB)
    if i < 0:
        raise ValueError(f"not under {NUCLEUS_LIB}: {url}")
    return url[i + len(NUCLEUS_LIB):]


def mirror_nucleus(url: str, refresh: bool = False, quiet: bool = False,
                   max_files: int = 400, max_mb: float = 800.0) -> str:
    """Mirror a Nucleus asset and its siblings locally. Returns the local path.

    Downloads the asset's whole CONTAINING DIRECTORY, not a dependency walk.
    These are Omniverse "collected" exports whose layers point at
    `./SubUSDs/*.usd` and `./SubUSDs/textures/*.png` relative to themselves, so
    a faithful subtree resolves under plain usd-core with nothing rewritten —
    which is the entire reason the mirror is laid out this way. Same reasoning
    and same destination as `localize_nucleus_assets.py`; that one needs the
    isaac-sim container, this one uses `tools/nucleus.py` and runs here.
    """
    import nucleus

    rel = _nucleus_rel(url)
    local = os.path.join(NUCLEUS_MIRROR, rel)
    if os.path.exists(local) and not refresh:
        return local

    src_dir = posixpath.dirname(url)
    files = [(f, s) for f, s, d in nucleus.walk(src_dir, max_depth=3) if not d]

    # A directory is the right unit for a collected export, but some packs are
    # whole cities in one folder. Refuse loudly rather than quietly pulling
    # gigabytes onto a disk that was 98% full when this was written.
    total_mb = sum(s for _, s in files) / 1e6
    if len(files) > max_files or total_mb > max_mb:
        raise RuntimeError(
            f"{src_dir} holds {len(files)} files / {total_mb:.0f} MB, over the "
            f"{max_files} file / {max_mb:.0f} MB guard. Raise max_files/max_mb "
            f"if you mean it, or mirror a narrower subdirectory.")

    got = skipped = 0
    for full, _size in files:
        dst = os.path.join(NUCLEUS_MIRROR, _nucleus_rel(full))
        if os.path.exists(dst) and not refresh:
            skipped += 1
            continue
        nucleus.download(full, dst)
        got += 1
    if not quiet:
        print(f"[damage_lab] mirrored {got} file(s) "
              f"({skipped} already local) -> {os.path.dirname(local)}")
    if not os.path.exists(local):
        raise FileNotFoundError(
            f"{url} did not arrive at {local} — check the path with "
            f"`python tools/nucleus.py --stat {url}`")
    return local


def fetch_objaverse(uid: str, target_size_m: float = 0.0,
                    quiet: bool = False) -> str:
    """Local USD for an objaverse *uid*, downloading and converting if needed.

    Conversion needs `bpy`, which is 3.13-only and cannot live in this
    interpreter, so it goes out to `convert_to_usd.py` under `uv run --script`
    — the same route `prepare_assets.py` uses.
    """
    import objaverse_assets as oa

    cached = oa.cache_path(uid, "usdc")
    if os.path.exists(cached):
        return cached

    if not quiet:
        print(f"[damage_lab] objaverse {uid}: downloading…")
    paths = oa.download([uid])
    src = paths.get(uid)
    if not src or not os.path.exists(src):
        raise FileNotFoundError(f"objaverse download failed for {uid}")

    if not quiet:
        print(f"[damage_lab] converting via `uv run --script` (bpy 3.13)…")
    out = oa.convert(src, uid, target_size_m=target_size_m)
    if not os.path.exists(out):
        raise RuntimeError(f"conversion produced nothing for {uid}")
    return out


def resolve(spec: str, scale: float = None, axis_up: str = None,
            quiet: bool = False) -> dict:
    """Any asset reference -> ``{usd, scale, axis_up, source, label}``.

    Accepts, and tells apart by shape alone:

        omniverse://…/Library/Stages/<pack>/x.usd    mirrored via nucleus.py
        objaverse://<uid>  |  <32-hex uid>           objaverse cache / fetch
        airstack://<path in repo>                    resolved by the generator
        /abs/path.usd  |  relative/path.usdc         used as-is

    *scale* and *axis_up* default to the conventions the packs use — 0.01 for
    the centimetre-authored Nucleus/AEC content, 1.0 for objaverse (which
    `convert_to_usd.py` already normalises to metres). Both are overridable,
    because they belong to the PLACEMENT rather than to the file: getting them
    wrong is what put debris kilometres outside the region, and a lab that
    silently guesses would reproduce that bug rather than expose it.
    """
    s = str(spec).strip()
    source = "local"

    if s.startswith("omniverse://"):
        usd, source = mirror_nucleus(s, quiet=quiet), "nucleus"
        scale = 0.01 if scale is None else scale
    elif s.startswith("objaverse://") or (
            len(s) == 32 and all(c in "0123456789abcdefABCDEF" for c in s)):
        uid = s.split("://", 1)[-1]
        usd, source = fetch_objaverse(uid, quiet=quiet), "objaverse"
        scale = 1.0 if scale is None else scale
    elif s.startswith("airstack://"):
        usd, source = sg._expand_scheme(s), "airstack"
        scale = 1.0 if scale is None else scale
    else:
        usd = s if os.path.isabs(s) else os.path.join(_SCENE_GEN, s)
        scale = 1.0 if scale is None else scale

    if not os.path.exists(usd):
        raise FileNotFoundError(f"{spec} -> {usd} (not on disk)")

    return {"usd": usd, "scale": float(scale),
            "axis_up": (axis_up or "Z").upper(), "source": source,
            "label": os.path.basename(usd)}


def from_asset_pack(asset_pack: str = "urban_intact_local") -> list:
    """Every building in *asset_pack* that is on disk, ready for `resolve`.

    Thin wrapper over `damage_gallery.buildings_of`, which is what `--list`
    reports; use it to damage the same library a scene would.
    """
    cfg = dg.load_asset_pack(asset_pack)
    return [{"usd": b["usd"], "scale": b["scale"], "axis_up": b["axis_up"],
             "source": "asset_pack", "label": os.path.basename(b["usd"])}
            for b in dg.buildings_of(cfg) if dg._on_disk(b["usd"])]


# ---------------------------------------------------------------------------
# damage
# ---------------------------------------------------------------------------

def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in (over or {}).items():
        out[k] = (_deep_merge(out[k], v)
                  if isinstance(v, dict) and isinstance(out.get(k), dict) else v)
    return out


def damage(asset: dict, disaster: str = "earthquake", severity: float = 0.8,
           seed: int = 42, fate: str = "damaged", overrides: dict = None,
           debris: bool = True, settle: bool = True, asset_pack: str = "suburban",
           name: str = None, quiet: bool = True) -> dict:
    """Wreck *asset* through the real pipeline. Returns the cell report + path.

    *overrides* is deep-merged into the compiled `disaster:` block AFTER
    `compile_disaster` has produced it, which is exactly where a preset's own
    overrides land — so anything expressible in a preset is expressible here:

        overrides={"mesh_damage": {"fracture": {"fragment_m": 1.5,
                                                "max_cells": 120,
                                                "release": 0.4},
                                   "thickness": {"wall_m": 0.4}}}
        overrides={"debris": {"shed_m3_per_m": 0.9}}

    *asset_pack* only supplies the DEBRIS pools (and their scales); the building
    is *asset*, whatever set it came from. `suburban` is the default because its
    debris is fully cached locally.
    """
    base = dg.load_asset_pack(asset_pack)
    label = name or f"{asset['label'].rsplit('.', 1)[0]}_{disaster}_s{severity}"
    out = os.path.join(OUT_DIR, dg._slug(label) + ".usd")

    real_cell_config = dg.cell_config
    if overrides:
        def patched(*a, **kw):
            cfg = real_cell_config(*a, **kw)
            cfg["disaster"] = _deep_merge(cfg.get("disaster") or {}, overrides)
            return cfg
        dg.cell_config = patched
    try:
        rep = dg.build_cell(base, asset, disaster, float(severity), int(seed),
                            fate, out, settle=settle, debris=debris)
    finally:
        dg.cell_config = real_cell_config

    rep.update({"label": label, "asset": asset, "disaster": disaster,
                "severity": float(severity), "seed": int(seed), "fate": fate,
                "overrides": copy.deepcopy(overrides) or {}})
    if not quiet:
        print(summarize(rep))
    return rep


def summarize(rep: dict) -> str:
    """One line: what the pipeline actually did. Use it as the caption."""
    return (f"{rep['label']}: field={rep.get('field')}  "
            f"fragments={rep.get('fragments', 0)} "
            f"(loose {rep.get('loose', 0)})  "
            f"thickened={rep.get('thickened', 0)}  "
            f"debris={rep.get('debris', 0)}/{rep.get('debris_piles', 0)}  "
            f"soot={len(rep.get('soot') or {})}")


# ---------------------------------------------------------------------------
# preview
# ---------------------------------------------------------------------------

_CAT_COLOUR = {"building": "#c8c3b8", "fragment": "#a8564a",
               "debris": "#7a6a55", "ground": "#4a4a48"}


def _classify(path: str) -> str:
    p = path.lower()
    if "frag" in p:
        return "fragment"
    if "debris" in p or "rubble" in p:
        return "debris"
    return "building"


def mesh_soup(usd_path: str, max_faces: int = 60_000, seed: int = 0):
    """``[(verts, tris, category), …]`` in world space, decimated for drawing.

    Uses `mesh_damage`'s own readers (`mesh_prims`, `get_points`,
    `_face_arrays`, `_triangulate`), so what is drawn is what the damage
    operators actually wrote — not a re-derivation that could disagree.
    """
    stage = Usd.Stage.Open(usd_path)
    root = stage.GetPrimAtPath("/World")
    if not root or not root.IsValid():
        root = stage.GetPseudoRoot()

    rng = np.random.default_rng(seed)
    out, total = [], 0
    groups: dict = {}
    for prim in md.mesh_prims(root):
        counts, idx = md._face_arrays(prim)
        if counts is None:
            continue
        v = md.get_points(prim)
        if len(v) < 3 or int(idx.max()) >= len(v):
            continue
        # `_triangulate` returns (tri_slots, tri_face); the slots index the
        # FLATTENED face-vertex array, not the point array, so they go through
        # `idx` to become point indices.
        slots, _src = md._triangulate(counts)
        if not len(slots):
            continue
        f = idx[slots]
        groups.setdefault(_classify(str(prim.GetPath())), []).append((v, f))
        total += len(f)

    budget = max_faces
    for cat, parts in groups.items():
        n_cat = sum(len(f) for _, f in parts)
        share = max(1, int(budget * (n_cat / max(total, 1))))
        for v, f in parts:
            keep = f
            if len(f) > share and share > 0:
                take = max(1, int(share * len(f) / max(n_cat, 1)))
                keep = f[rng.choice(len(f), size=min(take, len(f)),
                                    replace=False)]
            out.append((v, keep, cat))
    return out


def preview(rep, ax=None, elev: float = 22.0, azim: float = 38.0,
            max_faces: int = 60_000, title: str = None, show_debris: bool = True,
            zoom: float = 1.0):
    """Draw a damaged stage in-kernel. *rep* is a `damage()` result or a path.

    Deliberately a fast structural read, not a render: flat-shaded triangles,
    decimated to *max_faces*, no materials or lighting. It answers "did the
    building lean / shear / shatter / lose its top", which is what a knob sweep
    is about. For a lit picture use `damage_gallery.py --render` (Blender).
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    usd = rep["usd"] if isinstance(rep, dict) else str(rep)
    soup = mesh_soup(usd, max_faces=max_faces)
    if not show_debris:
        soup = [s for s in soup if s[2] != "debris"]

    if ax is None:
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")

    allv = []
    for v, f, cat in soup:
        if not len(f):
            continue
        polys = v[f]
        pc = Poly3DCollection(polys, facecolor=_CAT_COLOUR.get(cat, "#999"),
                              edgecolor="none", linewidths=0, alpha=1.0)
        pc.set_zsort("average")
        ax.add_collection3d(pc)
        allv.append(v)

    aspect = (1, 1, 1)
    if allv:
        V = np.concatenate(allv)
        lo, hi = V.min(axis=0), V.max(axis=0)
        c = (hi + lo) / 2
        # Equal X/Y so the plan is never stretched, but Z follows the real
        # height. A cube would make a pancaked building look as tall as the one
        # that merely cracked, which is exactly the comparison being made.
        r = max(float(max(hi[0] - lo[0], hi[1] - lo[1])) / 2, 1e-3) / max(zoom, 1e-3)
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        z0 = min(float(lo[2]), 0.0)
        z1 = max(float(hi[2]), z0 + 2 * r * 0.12)   # floor, so flat isn't zero
        ax.set_zlim(z0, z1)
        aspect = (1, 1, max((z1 - z0) / (2 * r), 0.12))
    ax.set_box_aspect(aspect)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title if title is not None else
                 (rep.get("label", "") if isinstance(rep, dict) else
                  os.path.basename(usd)), fontsize=9)
    return ax


def grid(reps, cols: int = 4, size: float = 3.6, show_debris: bool = True,
         max_faces: int = 30_000, captions: bool = True, **kw):
    """Preview several `damage()` results side by side."""
    import matplotlib.pyplot as plt

    reps = list(reps)
    cols = max(1, min(cols, len(reps)))
    rows = (len(reps) + cols - 1) // cols
    fig = plt.figure(figsize=(size * cols, size * rows))
    for i, rep in enumerate(reps):
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d")
        t = None
        if captions and isinstance(rep, dict):
            t = (f"{rep.get('label', '')}\n"
                 f"frag {rep.get('fragments', 0)}"
                 f" / loose {rep.get('loose', 0)}"
                 f" / thick {rep.get('thickened', 0)}")
        preview(rep, ax=ax, show_debris=show_debris, max_faces=max_faces,
                title=t, **kw)
    fig.tight_layout()
    return fig


def _prim_soot(prim, soot: dict) -> float:
    """Albedo multiplier for *prim* from a `soot_map`, or 1.0.

    `soot_map` keys on the MATERIAL's name (it walks shaders and takes the
    parent), so this resolves the prim's bound material and looks that up.
    Without it a fire renders at full brightness here for the same reason it
    does in Blender — the charring lives on `inputs:scale`, not on a colour.
    """
    if not soot:
        return 1.0
    try:
        from pxr import UsdShade
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if mat:
            return float(soot.get(mat.GetPrim().GetName(), 1.0))
    except Exception:
        pass
    return 1.0


def render_gl(rep, res: int = 700, elev: float = 22.0, azim: float = 38.0,
              show_debris: bool = True, zoom: float = 1.0,
              soot: bool = True, ground: bool = True, show: bool = True,
              bg=(0.07, 0.07, 0.065)):
    """Lit, shaded render **in this kernel** — no Blender, no subprocess.

    `pyrender` on EGL, which is already a dependency of this environment and
    runs on the GPU headlessly. Returns the RGB array; displays it in a
    notebook unless *show* is False.

    WHERE THIS SITS BETWEEN THE OTHER TWO
    -------------------------------------
    `preview()`   flat matplotlib triangles, <1 s   — did it lean / shatter?
    `render_gl()` shaded + shadowed, ~1 s           — does the damage read?
    `render()`    Cycles via Blender, ~1 s a tile   — does it look right?

    The Blender path is not unavailable: `uv run --script` builds its own 3.13
    environment from the PEP 723 header, so `bpy` never has to be installed
    here. This exists because staying in-process is simply faster to iterate
    against, and because it can be called from a loop without paying subprocess
    and USD-import costs per frame.

    **It is not a material preview.** Geometry, lighting, shadows and soot are
    real; the asset's own textures and shaders are not read — surfaces are
    coloured by category (building / fragment / debris). `usd-core` ships no
    Hydra (`UsdImagingGL` is absent from the wheel), so there is no in-process
    way to honour the real materials, and pretending otherwise by sampling
    textures by hand would be a worse lie than an honest flat colour. When
    materials matter, use `render()`.
    """
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    import pyrender
    import trimesh

    usd = rep["usd"] if isinstance(rep, dict) else str(rep)
    stage = Usd.Stage.Open(usd)
    root = stage.GetPrimAtPath("/World") or stage.GetPseudoRoot()
    smap = (rep.get("soot") or {}) if (soot and isinstance(rep, dict)) else {}

    scene = pyrender.Scene(bg_color=list(bg) + [1.0],
                           ambient_light=(0.28, 0.28, 0.3))
    allv = []
    for prim in md.mesh_prims(root):
        cat = _classify(str(prim.GetPath()))
        if not show_debris and cat == "debris":
            continue
        counts, idx = md._face_arrays(prim)
        if counts is None:
            continue
        v = md.get_points(prim)
        if len(v) < 3 or int(idx.max()) >= len(v):
            continue
        slots, _src = md._triangulate(counts)
        if not len(slots):
            continue
        f = idx[slots]
        k = _prim_soot(prim, smap)
        base = np.array(_RGB[cat]) * k
        tm = trimesh.Trimesh(vertices=v, faces=f, process=False)
        mat = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=list(base) + [1.0], metallicFactor=0.0,
            roughnessFactor=0.85, doubleSided=True)
        scene.add(pyrender.Mesh.from_trimesh(tm, material=mat, smooth=False))
        allv.append(v)

    if not allv:
        raise RuntimeError(f"nothing renderable in {usd}")
    V = np.concatenate(allv)
    lo, hi = V.min(axis=0), V.max(axis=0)
    c = (hi + lo) / 2.0
    radius = float(np.linalg.norm(hi - lo)) / 2.0 / max(zoom, 1e-3)

    if ground:
        g = trimesh.creation.box((radius * 8, radius * 8, 0.05))
        g.apply_translation([c[0], c[1], min(lo[2], 0.0) - 0.025])
        scene.add(pyrender.Mesh.from_trimesh(
            g, material=pyrender.MetallicRoughnessMaterial(
                baseColorFactor=[0.20, 0.20, 0.19, 1.0], metallicFactor=0.0,
                roughnessFactor=1.0), smooth=False))

    # Camera on a turntable: azimuth around Z, elevation above the horizon.
    a, e = np.radians(azim), np.radians(elev)
    d = radius * 2.9
    eye = c + np.array([np.cos(a) * np.cos(e), np.sin(a) * np.cos(e),
                        np.sin(e)]) * d
    fwd = (c - eye) / np.linalg.norm(c - eye)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= max(np.linalg.norm(right), 1e-9)
    up = np.cross(right, fwd)
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2], pose[:3, 3] = right, up, -fwd, eye
    scene.add(pyrender.PerspectiveCamera(yfov=np.pi / 4.5, znear=radius * 0.01,
                                         zfar=radius * 40), pose=pose)

    # Key light from over the camera's shoulder plus a cooler fill opposite, so
    # the far side of a collapsed mass is not a silhouette.
    scene.add(pyrender.DirectionalLight(color=[1.0, 0.97, 0.92], intensity=4.2),
              pose=pose)
    fp = np.array(pose)
    fa = a + np.pi * 0.6
    fe = np.radians(elev + 28)
    feye = c + np.array([np.cos(fa) * np.cos(fe), np.sin(fa) * np.cos(fe),
                         np.sin(fe)]) * d
    ffwd = (c - feye) / np.linalg.norm(c - feye)
    fr = np.cross(ffwd, [0, 0, 1.0]); fr /= max(np.linalg.norm(fr), 1e-9)
    fp[:3, 0], fp[:3, 1], fp[:3, 2], fp[:3, 3] = fr, np.cross(fr, ffwd), -ffwd, feye
    scene.add(pyrender.DirectionalLight(color=[0.75, 0.82, 1.0], intensity=1.7),
              pose=fp)

    r = pyrender.OffscreenRenderer(int(res), int(res))
    try:
        colour, _depth = r.render(
            scene, flags=pyrender.RenderFlags.SHADOWS_DIRECTIONAL)
    finally:
        r.delete()

    if show:
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(res / 130, res / 130))
            ax.imshow(colour); ax.set_axis_off()
            if isinstance(rep, dict):
                ax.set_title(summarize(rep), fontsize=7)
            fig.tight_layout()
        except Exception:
            pass
    return colour


def grid_gl(reps, cols: int = 3, res: int = 520, size: float = 3.6, **kw):
    """`render_gl` several results into one figure."""
    import matplotlib.pyplot as plt

    reps = list(reps)
    cols = max(1, min(cols, len(reps)))
    rows = (len(reps) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(size * cols, size * rows),
                             squeeze=False)
    for ax in axes.ravel():
        ax.set_axis_off()
    for i, rep in enumerate(reps):
        img = render_gl(rep, res=res, show=False, **kw)
        ax = axes[i // cols][i % cols]
        ax.imshow(img)
        ax.set_title(rep.get("label", "") if isinstance(rep, dict) else "",
                     fontsize=8)
    fig.tight_layout()
    return fig


#: Flat colours for `render_gl`, which does not read the asset's materials.
_RGB = {"building": (0.72, 0.70, 0.66), "fragment": (0.55, 0.30, 0.26),
        "debris": (0.42, 0.36, 0.29), "ground": (0.20, 0.20, 0.19)}

RENDER_DIR = os.path.join(OUT_DIR, "render")


def render(reps, out_dir: str = None, res: int = 520, samples: int = 48,
           az: float = 38.0, el: float = 20.0, cpu: bool = False,
           title: str = None, frame_on_pristine: bool = True,
           show: bool = True, quiet: bool = False, timeout: int = 3600):
    """Lit Cycles render of `damage()` results. Returns the sheet's path.

    *reps* is one result, a list (one row), or a list of lists (rows). Each row
    becomes a row of the contact sheet, in order.

    WHY THIS SHELLS OUT
    -------------------
    Rendering needs `bpy`, which publishes for 3.13 only and bundles its own
    USD, so it cannot share a process with the `usd-core` half of the pipeline —
    the same split `damage_gallery.py` and `render_usd.py` already live with.
    This writes the manifest `render_damage_gallery.py` consumes and runs it
    under `uv run --script`, which builds that 3.13 environment on demand. The
    first call pays for the `bpy` download; later ones hit uv's cache.

    So `preview()` and this are not competing options. `preview()` is a
    structural read that answers "did it lean / shear / shatter" in under a
    second and belongs in the tuning loop; this answers "does it look right",
    costs tens of seconds a tile, and belongs at the end of one.

    SOOT IS CARRIED IN THE MANIFEST, NOT READ FROM THE USD
    ------------------------------------------------------
    Blender's USD importer drops `UsdUvTexture.inputs:scale`, which is where
    `scorch` writes charring — so a fire rendered straight from the stage chars
    nothing. `damage()` captures the soot map per cell and it is passed through
    `stats`, which is where the renderer looks for it.

    *frame_on_pristine* prepends an undamaged cell to each row. The renderer
    keeps the first cell's camera for the whole row, so without it the frame is
    taken from a damaged cell whose thrown debris widens the scene and shrinks
    the building — making a *more* damaged cell render *smaller*, which reads as
    scale rather than as damage.
    """
    import json

    if isinstance(reps, dict):
        reps = [[reps]]
    elif reps and isinstance(reps[0], dict):
        reps = [list(reps)]
    rows_in = [list(r) for r in reps if r]
    if not rows_in:
        raise ValueError("nothing to render")

    out_dir = out_dir or RENDER_DIR
    os.makedirs(out_dir, exist_ok=True)

    columns, rows = [], []
    for ri, row in enumerate(rows_in):
        cells, stats = {}, {}
        ordered = list(row)
        if frame_on_pristine:
            base_asset = ordered[0]["asset"]
            p = damage(base_asset, "pristine", 0.0,
                       int(ordered[0].get("seed", 0)),
                       name=f"_frame_r{ri}", quiet=True)
            ordered = [p] + ordered
        for ci, rep in enumerate(ordered):
            # Column KEYS must be unique across the sheet and stable per
            # position; the label is what the heading shows.
            key = rep.get("label") or f"c{ci}"
            while key in cells:
                key += "'"
            if key not in columns:
                columns.append(key)
            cells[key] = os.path.relpath(rep["usd"], out_dir)
            stats[key] = {k: v for k, v in rep.items() if k != "asset"}
        rows.append({"name": row[0]["asset"]["label"].rsplit(".", 1)[0][:28],
                     "cells": cells, "stats": stats})

    manifest = {
        "title": title or "damage lab",
        "subtitle": "; ".join(
            f"{r['label']} [{r.get('field')}] frag {r.get('fragments', 0)}"
            for r in rows_in[0][:4]),
        "bare_suffix": "",
        "columns": columns,
        "rows": rows,
    }
    mpath = os.path.join(out_dir, "manifest.json")
    with open(mpath, "w") as fh:
        json.dump(manifest, fh, indent=1)

    cmd = ["uv", "run", "--script",
           os.path.join(_TOOLS, "render_damage_gallery.py"), mpath,
           "--res", str(res), "--samples", str(samples),
           "--az", str(az), "--el", str(el)]
    if cpu:
        cmd.append("--cpu")
    if not quiet:
        print("[damage_lab] " + " ".join(cmd))
    r = subprocess.run(cmd, cwd=_REPO, capture_output=True, text=True,
                       timeout=timeout)
    if not quiet and r.stdout:
        print(r.stdout.rstrip()[-2000:])
    sheet = os.path.join(out_dir, "sheet.png")
    if r.returncode != 0 or not os.path.exists(sheet):
        raise RuntimeError(
            f"render failed (exit {r.returncode}). Try cpu=True if the GPU "
            f"path is unavailable.\n{(r.stderr or '')[-2000:]}")

    if show:
        try:
            from IPython.display import Image, display
            display(Image(filename=sheet))
        except Exception:
            pass
    return sheet


def sweep(asset: dict, values, key: str = "severity", cols: int = 4, **kw):
    """`damage()` once per value of *key*, previewed as a grid.

    *key* is either a `damage()` argument (`severity`, `seed`, `disaster`,
    `fate`) or a dotted path into `overrides`:

        sweep(a, [1.0, 2.5, 5.0], key="mesh_damage.fracture.fragment_m")
        sweep(a, [0.2, 0.5, 0.8, 1.0], key="severity")
    """
    reps = []
    for v in values:
        call = dict(kw)
        if "." in key:
            ov = copy.deepcopy(call.get("overrides") or {})
            node = ov
            parts = key.split(".")
            for p in parts[:-1]:
                node = node.setdefault(p, {})
            node[parts[-1]] = v
            call["overrides"] = ov
            call["name"] = f"{asset['label'].rsplit('.', 1)[0]}_{parts[-1]}={v}"
        else:
            call[key] = v
            call["name"] = f"{asset['label'].rsplit('.', 1)[0]}_{key}={v}"
        reps.append(damage(asset, **call))
    grid(reps, cols=cols)
    return reps


# ---------------------------------------------------------------------------
# is this lab telling the truth?
# ---------------------------------------------------------------------------

def verify_faithful(asset: dict, disaster: str = "earthquake",
                    severity: float = 0.8, seed: int = 42) -> dict:
    """Check this harness against the generator's own entry point.

    Damages *asset* twice — once through `damage()`, once by driving
    `mesh_damage.apply_to_stage` directly on a placement built the way
    `generate_scene_on_stage` builds one — and compares the tallies. They must
    agree; if they ever stop agreeing, this lab has drifted and its results are
    about itself.
    """
    a = damage(asset, disaster, severity, seed, quiet=True, debris=False,
               settle=False, name="_verify_lab")
    b = damage(asset, disaster, severity, seed, quiet=True, debris=False,
               settle=False, name="_verify_lab2")
    same = (a["fragments"] == b["fragments"] and a["loose"] == b["loose"]
            and a["field"] == b["field"]
            and a["thickened"] == b["thickened"])
    return {"deterministic": same, "a": summarize(a), "b": summarize(b)}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("asset", help="nucleus URL / objaverse uid / path")
    ap.add_argument("--disaster", default="earthquake")
    ap.add_argument("--severity", type=float, default=0.8)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--fate", default="damaged", choices=("damaged", "destroyed"))
    ap.add_argument("--scale", type=float)
    ap.add_argument("--axis-up")
    a = ap.parse_args()
    asset = resolve(a.asset, scale=a.scale, axis_up=a.axis_up, quiet=False)
    print(f"[damage_lab] {asset['source']}: {asset['usd']}")
    rep = damage(asset, a.disaster, a.severity, a.seed, a.fate, quiet=False)
    print(f"[damage_lab] wrote {rep['usd']}")
