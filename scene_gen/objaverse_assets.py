#!/usr/bin/env python3
"""Objaverse -> USD asset pipeline for the scene generator.

The scene generator references USDs. Most of the library lives on Nucleus
(``omniverse://…``), which is fine but finite — building out a new asset pack
(suburban, rural, …) is bottlenecked on finding art. Objaverse 1.0 has ~800k
Sketchfab objects with rich metadata, but ships **glb**, not USD, so it can't
be referenced directly.

This module closes that gap. **Assets are identified by their Objaverse uid**,
and an asset pack references one directly::

    - usd: "objaverse://6644de89c2f0449db3de934744162b63"
      target-size-m: 12

The uid is the whole identity: the cache lives at
``scene_gen/assets/objaverse/<uid>/<uid>.usdc``, so it is a pure function of
what the config asked for. There is no local name to keep in sync, two sets
referencing the same object share one copy, and the config stays meaningful
even with an empty cache.

The cache is **derived state, not a checked-in asset**. ``prepare_assets.py``
walks the asset packs and downloads + converts whatever is missing — run it on
the host before ``airstack up``, which then just reads the cache through the
repo bind-mount. (It has to be the host: the sim container has no ``uv``,
``objaverse`` or Blender.)

The metric scale is **baked into the cached USD** (see ``target-size-m``), so
the generator gets a metric asset and no config carries a scale factor that
could drift from the geometry.

``scene_generator.LOCAL_ASSET_ROOTS`` maps ``objaverse://`` to that cache
directory, so a config never names a host path — the repo is mounted at
``/isaac-sim/AirStack`` in the sim container and the same entry resolves
correctly in both places.

Why Objaverse 1.0 and not Objaverse-XL
--------------------------------------
XL is ~10M objects but its annotations carry almost no metadata — the only
searchable text is the file path, and most GitHub-hosted meshes are untextured
low-poly game props. Objaverse 1.0 (the Sketchfab subset) ships per-object
name, tags, categories, face/vertex counts, **texture count and resolution**,
and license — all without downloading a single mesh. Since untextured assets
are useless here, that metadata is what makes filtering possible.

Usage (CLI)
-----------
    # what's out there (metadata only — no downloads)
    python3 scene_gen/objaverse_assets.py search house --limit 20
    python3 scene_gen/objaverse_assets.py search house --tags suburban,residential

    # cache one object by uid, sized to a 12 m plan dimension
    python3 scene_gen/objaverse_assets.py ensure <uid> --target-size 12

    # what's cached, and paste-ready asset-pack entries
    python3 scene_gen/objaverse_assets.py list --yaml

Usage (library)
---------------
    from objaverse_assets import load_catalog, search, ensure
    cat = load_catalog()                       # cached DataFrame
    hits = search(cat, "house", min_textures=1)
    ensure(hits.iloc[0].uid, target_size_m=12.0)

The converted USDs and downloaded glbs are **git-ignored** (textures run to
tens of MB). ``manifest.yaml`` is committed for provenance and attribution —
uid, title, author, license, baked scale and measured size — but it is a
record, not the source of truth: the asset packs are, and ``prepare_assets.py``
rebuilds the cache from them.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys

_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))

ASSET_DIR = os.path.join(_SCENE_GEN_DIR, "assets", "objaverse")
ASSET_PACKS_DIR = os.path.join(_SCENE_GEN_DIR, "config", "asset_packs")
MANIFEST = os.path.join(ASSET_DIR, "manifest.yaml")
CATALOG_CACHE = os.path.join(ASSET_DIR, ".catalog.parquet")
CONVERTER = os.path.join(_SCENE_GEN_DIR, "convert_to_usd.py")

SKETCHFAB_URL = "https://sketchfab.com/3d-models/{uid}"


# ---------------------------------------------------------------------------
# Catalog — Objaverse 1.0 annotations as a DataFrame
# ---------------------------------------------------------------------------

def load_catalog(refresh: bool = False, quiet: bool = False):
    """Return the Objaverse 1.0 annotation catalog as a pandas DataFrame.

    One row per object with the fields worth filtering on. The first call
    downloads ~160 annotation shards (a few minutes, cached by objaverse under
    ``~/.objaverse``); the flattened table is then cached as parquet here so
    subsequent calls take about a second.

    Columns: ``uid name tags categories faces vertices animations textures
    texture_res glb_size license author likes views text``.

    ``text`` is the lowercased name + tag + category slugs joined together —
    the field :func:`search` matches against.
    """
    import pandas as pd

    if os.path.exists(CATALOG_CACHE) and not refresh:
        return pd.read_parquet(CATALOG_CACHE)

    import objaverse

    if not quiet:
        print("[objaverse] loading annotations (first run downloads ~160 "
              "shards, this takes a few minutes)…")
    uids = objaverse.load_uids()
    anns = objaverse.load_annotations(uids)

    rows = []
    for uid, a in anns.items():
        glb = (a.get("archives") or {}).get("glb") or {}
        tags = [t.get("slug") for t in (a.get("tags") or []) if t.get("slug")]
        cats = [c.get("slug") for c in (a.get("categories") or []) if c.get("slug")]
        rows.append({
            "uid": uid,
            "name": (a.get("name") or "").strip(),
            "tags": tags,
            "categories": cats,
            "faces": a.get("faceCount"),
            "vertices": a.get("vertexCount"),
            "animations": a.get("animationCount"),
            "textures": glb.get("textureCount"),
            "texture_res": glb.get("textureMaxResolution"),
            "glb_size": glb.get("size"),
            "license": a.get("license"),
            "author": ((a.get("user") or {}).get("displayName")
                       or (a.get("user") or {}).get("username") or ""),
            "likes": a.get("likeCount"),
            "views": a.get("viewCount"),
        })

    df = pd.DataFrame(rows)
    # One searchable text field; hyphens/underscores -> spaces so word-boundary
    # regexes work against slugs like "cars-vehicles".
    df["text"] = (
        df["name"] + " "
        + df["tags"].str.join(" ") + " "
        + df["categories"].str.join(" ")
    ).str.replace(r"[-_/]+", " ", regex=True).str.lower()

    os.makedirs(ASSET_DIR, exist_ok=True)
    df.to_parquet(CATALOG_CACHE, index=False)
    if not quiet:
        print(f"[objaverse] catalog: {len(df):,} objects -> {CATALOG_CACHE}")
    return df


# ---------------------------------------------------------------------------
# Search
# ---------------------------------------------------------------------------

def search(catalog, query: str = "", *, exclude: str = "", tags: str = "",
           min_textures: int = 1, min_texture_res: int = 0,
           min_faces: int = 0, max_faces: int = 400_000,
           no_animation: bool = True, licenses: str = "",
           sort: str = "likes", limit: int = 0):
    """Filter *catalog* down to plausible scene assets.

    *query* / *exclude* are regexes matched against the combined name+tags+
    categories text; ``|``-separated alternatives are the normal idiom. Word
    boundaries are added around bare alphanumeric words so ``car`` doesn't
    match ``carpet``.

    Defaults encode what the scene generator needs: **at least one texture**
    (untextured assets read as grey blobs and are not usable), no animation
    rig, and a face budget that keeps a few hundred instances loadable.

    ``sort`` is ``likes`` | ``views`` | ``faces`` | ``texture_res``.
    """
    df = catalog

    if query:
        df = df[df["text"].str.contains(_wordify(query), regex=True, na=False)]
    if exclude:
        df = df[~df["text"].str.contains(_wordify(exclude), regex=True, na=False)]
    if tags:
        want = {t.strip().lower() for t in tags.split(",") if t.strip()}
        df = df[df["tags"].apply(lambda ts: bool(want & {t.lower() for t in ts}))]

    if min_textures:
        df = df[df["textures"].fillna(0) >= min_textures]
    if min_texture_res:
        df = df[df["texture_res"].fillna(0) >= min_texture_res]
    if min_faces:
        df = df[df["faces"].fillna(0) >= min_faces]
    if max_faces:
        df = df[df["faces"].fillna(0) <= max_faces]
    if no_animation:
        df = df[df["animations"].fillna(0) == 0]
    if licenses:
        keep = {s.strip().lower() for s in licenses.split(",") if s.strip()}
        df = df[df["license"].str.lower().isin(keep)]

    if sort in df.columns:
        df = df.sort_values(sort, ascending=False)
    return df.head(limit) if limit else df


def _wordify(pattern: str) -> str:
    """Wrap bare alphanumeric alternatives in word boundaries, leaving anything
    that already looks like a regex alone. ``"house|cottage"`` becomes
    ``r"\\bhouse\\b|\\bcottage\\b"`` so it can't match inside another word."""
    parts = []
    for alt in pattern.split("|"):
        alt = alt.strip()
        if not alt:
            continue
        parts.append(rf"\b{alt}\b" if re.fullmatch(r"[\w ]+", alt) else alt)
    return "|".join(parts)


# ---------------------------------------------------------------------------
# Download + measure
# ---------------------------------------------------------------------------

def download(uids, processes: int = 4) -> dict:
    """Download glbs for *uids*. Returns ``{uid: local glb path}``.

    objaverse caches under ``~/.objaverse``, so re-downloads are free and this
    is safe to call repeatedly.
    """
    import objaverse

    uids = list(uids)
    return objaverse.load_objects(uids=uids, download_processes=processes)


def measure(mesh_path: str) -> dict:
    """Cheap pre-screen of a source mesh, in **source** coordinates.

    Useful for triage before committing to a conversion (is there geometry at
    all? how many faces?), but *not* for sizing: which source axis ends up
    being "up" in the USD depends on the root transform inside the glTF, so
    predicting the converted extents from here gets it wrong on real assets.
    Sizing uses the bbox the converter reports back — see :func:`convert`.

    Returns ``{"extents": [x, y, z], "faces": n}``.
    """
    import trimesh

    scene = trimesh.load(mesh_path, force="scene")
    geoms = [g for g in scene.geometry.values()
             if isinstance(g, trimesh.Trimesh) and len(g.faces)]
    if not geoms:
        raise ValueError(f"no triangle geometry in {mesh_path}")
    merged = trimesh.util.concatenate(geoms)
    return {"extents": [float(v) for v in merged.extents],
            "faces": int(len(merged.faces))}


def scale_for(extents, target_size_m: float, fit: str = "footprint") -> float:
    """Scale factor mapping *extents* (USD/Z-up model units) to *target_size_m*.

    glb has no canonical unit — Sketchfab art is authored at whatever size the
    artist liked — so every imported asset needs a scale to reach real-world
    size. ``fit`` picks the dimension being pinned: ``footprint`` (longest of
    x/y — right for buildings, which are sized by their plan), ``height``
    (z), or ``max`` (longest of all three).
    """
    x, y, z = extents
    ref = {"footprint": max(x, y), "height": z, "max": max(x, y, z)}[fit]
    if ref <= 0:
        raise ValueError(f"degenerate extents {extents}")
    return float(target_size_m) / float(ref)


# ---------------------------------------------------------------------------
# Convert
# ---------------------------------------------------------------------------

def cache_path(uid: str, fmt: str = "usdc") -> str:
    """Where the converted USD for *uid* lives: ``<uid>/<uid>.usdc``.

    Keyed by the Objaverse uid and nothing else, so an asset pack can name
    ``objaverse://<uid>`` and the cache is a pure function of that — no local
    slug to keep in sync, and two sets referencing the same object share one
    copy.
    """
    return os.path.join(ASSET_DIR, uid, f"{uid}.{fmt}")


def convert(src: str, uid: str, *, target_size_m: float = 0.0,
            fit: str = "footprint", fmt: str = "usdc") -> tuple:
    """Convert *src* into the cache slot for *uid*.

    Returns ``(usd_path, extents, scale)``. *extents* is the converted asset's
    real-world ``[x, y, z]`` bounding box and *scale* the factor applied to get
    there — both reported by the converter from Blender's world coordinates,
    the same coordinates it writes to USD. That is authoritative; it is not
    inferable from the source mesh.

    With *target_size_m* the metric scale is **baked into the USD**, so the
    cached asset is already in meters and an asset pack never carries a scale
    factor that could drift from the geometry.

    Runs :mod:`convert_to_usd` under ``uv run --script``: it needs headless
    Blender, whose ``bpy`` wheel is CPython 3.13-only, so it gets its own
    ephemeral environment rather than polluting this one.
    """
    dest_dir = os.path.join(ASSET_DIR, uid)
    os.makedirs(dest_dir, exist_ok=True)

    # The converter names outputs after the input file, so stage the source
    # under the uid to get <uid>/<uid>.usdc rather than <download hash>.usdc.
    staged = os.path.join(dest_dir, uid + os.path.splitext(src)[1])
    shutil.copyfile(src, staged)

    cmd = ["uv", "run", "--script", CONVERTER, staged, "--report",
           "--format", fmt, "--flat", "--overwrite", "-o", dest_dir]
    if target_size_m:
        cmd += ["--target-size", str(target_size_m), "--fit", fit]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    os.remove(staged)

    usd = cache_path(uid, fmt)
    if proc.returncode != 0 or not os.path.exists(usd):
        raise RuntimeError(
            f"conversion of {src} failed (exit {proc.returncode})\n"
            f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}")

    report = None
    for line in proc.stdout.splitlines():
        if line.startswith("BBOX "):
            report = json.loads(line[5:])
    if report is None:
        raise RuntimeError(f"converter reported no bounding box for {src}")
    return usd, report["size"], report.get("scale", 1.0)


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------

def load_manifest() -> dict:
    import yaml
    if not os.path.exists(MANIFEST):
        return {"assets": {}}
    with open(MANIFEST) as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("assets", {})
    return data


def save_manifest(data: dict) -> None:
    import yaml
    os.makedirs(ASSET_DIR, exist_ok=True)
    with open(MANIFEST, "w") as f:
        f.write("# Objaverse assets cached locally, written by "
                "scene_gen/objaverse_assets.py.\n"
                "# Keyed by Objaverse uid — the same id the asset packs "
                "reference as objaverse://<uid>.\n"
                "#\n"
                "# A record for provenance and attribution, NOT the source of "
                "truth: the asset\n"
                "# sets are. The USDs are git-ignored; "
                "`prepare_assets.py` rebuilds them\n"
                "# from whatever the asset packs reference.\n"
                "#\n"
                "# `baked_scale` is already applied to the USD — do not "
                "re-apply it in a config.\n"
                "# `size_m` is the asset's real-world bounding box as cached."
                "\n\n")
        yaml.safe_dump(data, f, sort_keys=True, default_flow_style=False)


def ensure(uid: str, *, target_size_m: float = 0.0, fit: str = "footprint",
           force: bool = False, catalog=None, quiet: bool = False) -> dict:
    """Make sure *uid* is present in the local cache as a metric USD.

    Downloads and converts only when needed, so this is the cheap call to make
    unconditionally on every launch: a warm cache costs a manifest read.
    Re-converts when the cached asset was built for a different
    ``target_size_m``, since the scale is baked into the USD.

    Returns the manifest entry.
    """
    data = load_manifest()
    entry = data["assets"].get(uid)
    if entry is not None and is_cached(uid, target_size_m, fit) and not force:
        return entry

    if not quiet:
        why = "not cached" if not os.path.exists(cache_path(uid)) \
            else "cached at a different size/fit"
        print(f"[objaverse] {uid}: {why} — downloading and converting…")

    paths = download([uid])
    if uid not in paths:
        raise RuntimeError(f"download failed for {uid}")
    src = paths[uid]

    faces = measure(src)["faces"]
    usd, extents, scale = convert(src, uid, target_size_m=target_size_m, fit=fit)

    entry = {
        "uid": uid,
        "usd": os.path.relpath(usd, ASSET_DIR),
        "target_size_m": float(target_size_m) or None,
        "fit": fit if target_size_m else None,
        # Baked into the USD, recorded for provenance only — an asset pack does
        # not (and must not) re-apply it.
        "baked_scale": round(scale, 6),
        "size_m": [round(e, 3) for e in extents],
        "faces": faces,
        "source": SKETCHFAB_URL.format(uid=uid),
    }

    if catalog is None:
        catalog = _catalog_if_cached()
    if catalog is not None:
        row = catalog[catalog["uid"] == uid]
        if len(row):
            r = row.iloc[0]
            entry.update({
                "title": str(r["name"]),
                "author": str(r["author"]),
                "license": str(r["license"]),
                "textures": int(r["textures"] or 0),
                "texture_res": int(r["texture_res"] or 0),
            })

    data["assets"][uid] = entry
    save_manifest(data)

    if not quiet:
        sz = entry["size_m"]
        print(f"[objaverse] {uid}: {sz[0]} x {sz[1]} x {sz[2]} m  "
              f"{entry['faces']:,} faces  {entry.get('title', '')}")
    return entry


def _catalog_if_cached():
    """The catalog, but only if it is already built — never pay the multi-minute
    first build just to annotate a manifest entry with a title and license."""
    if not os.path.exists(CATALOG_CACHE):
        return None
    try:
        return load_catalog(quiet=True)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Sync — what `airstack up` calls
# ---------------------------------------------------------------------------

OBJAVERSE_RE = re.compile(r"objaverse://([0-9a-fA-F]{32})")


def scan_asset_packs(paths=None) -> dict:
    """Collect every ``objaverse://<uid>`` referenced by the asset packs.

    Returns ``{uid: target_size_m}``. Reads the YAML as a plain nested
    structure rather than resolving it through the generator, so this stays
    usable without ``pxr`` — the host running ``airstack up`` has no USD.
    """
    import yaml

    if paths is None:
        paths = sorted(glob.glob(os.path.join(ASSET_PACKS_DIR, "*.yaml"))
                       + glob.glob(os.path.join(ASSET_PACKS_DIR, "*.yml")))

    wanted: dict = {}

    def walk(node):
        if isinstance(node, dict):
            usd = node.get("usd")
            if isinstance(usd, str) and OBJAVERSE_RE.search(usd):
                uid = OBJAVERSE_RE.search(usd).group(1)
                size = node.get("target-size-m", node.get("target_size_m"))
                wanted[uid] = {
                    "target_size_m": float(size) if size else 0.0,
                    "fit": str(node.get("fit", "footprint")),
                }
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
        elif isinstance(node, str):
            m = OBJAVERSE_RE.search(node)
            if m:
                wanted.setdefault(m.group(1),
                                  {"target_size_m": 0.0, "fit": "footprint"})

    for p in paths:
        with open(p) as f:
            walk(yaml.safe_load(f))
    return wanted


def is_cached(uid: str, target_size_m: float = 0.0,
              fit: str = "footprint") -> bool:
    """True when *uid* is cached **at the requested size and fit**.

    Both are baked into the USD, so a config that changes either leaves a
    cached file that exists but is the wrong shape — checking existence alone
    would silently keep serving the stale one.
    """
    if not os.path.exists(cache_path(uid)):
        return False
    entry = load_manifest()["assets"].get(uid)
    if entry is None:
        return False
    if abs(float(entry.get("target_size_m") or 0.0)
           - float(target_size_m or 0.0)) >= 1e-6:
        return False
    # `fit` is only meaningful when a target size was requested.
    return not target_size_m or (entry.get("fit") or "footprint") == fit


def pending(paths=None) -> list:
    """Uids the asset packs reference that need downloading or re-converting.

    What ``prepare_assets.py`` iterates over, and what ``--list`` reports.
    """
    return [uid for uid, spec in sorted(scan_asset_packs(paths).items())
            if not is_cached(uid, spec["target_size_m"], spec["fit"])]


def asset_pack_entries(uids=None) -> str:
    """Render cached assets as YAML ready to paste into an asset pack."""
    data = load_manifest()
    out = []
    for uid, e in sorted(data["assets"].items(),
                         key=lambda kv: kv[1].get("title", "")):
        if uids and uid not in uids:
            continue
        size = e.get("target_size_m")
        out.append(f'      - usd: "objaverse://{uid}"'
                   + (f'\n        target-size-m: {size:g}' if size else '')
                   + f'   # {e.get("title", "?")}'
                   f' — {e["size_m"][0]}x{e["size_m"][1]}x{e["size_m"][2]} m')
    return "\n".join(out)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _cmd_search(args):
    cat = load_catalog(refresh=args.refresh)
    hits = search(cat, args.query, exclude=args.exclude, tags=args.tags,
                  min_textures=args.min_textures,
                  min_texture_res=args.min_texture_res,
                  max_faces=args.max_faces, licenses=args.licenses,
                  sort=args.sort, limit=args.limit)
    total = len(search(cat, args.query, exclude=args.exclude, tags=args.tags,
                       min_textures=args.min_textures,
                       min_texture_res=args.min_texture_res,
                       max_faces=args.max_faces, licenses=args.licenses))
    print(f"{total:,} matches; showing {len(hits)}\n")
    print(f"{'uid':34s} {'faces':>8s} {'tex':>4s} {'res':>5s} {'likes':>6s}  name")
    for _, r in hits.iterrows():
        print(f"{r['uid']:34s} {int(r['faces'] or 0):8,d} "
              f"{int(r['textures'] or 0):4d} {int(r['texture_res'] or 0):5d} "
              f"{int(r['likes'] or 0):6d}  {r['name'][:44]}")


def _cmd_ensure(args):
    ensure(args.uid, target_size_m=args.target_size, fit=args.fit,
           force=args.force)


def _cmd_list(args):
    data = load_manifest()
    if not data["assets"]:
        print("nothing cached yet — run `prepare_assets.py`")
        return
    print(f"{'uid':34s} {'size (m)':22s} {'faces':>9s}  {'lic':<9s} title")
    for uid, e in sorted(data["assets"].items(),
                         key=lambda kv: kv[1].get("title", "")):
        present = "" if os.path.exists(cache_path(uid)) else "  [NOT CACHED]"
        size = "x".join(str(v) for v in e["size_m"])
        print(f"{uid:34s} {size:22s} {e['faces']:9,d}  "
              f"{e.get('license', '?'):<9s} {e.get('title', '')[:32]}{present}")
    if args.yaml:
        print("\n--- asset-pack entries ---")
        print(asset_pack_entries())


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Search Objaverse and import assets as USD for the scene generator.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("search", help="search the catalog (metadata only)")
    s.add_argument("query", nargs="?", default="", help="regex over name+tags+categories")
    s.add_argument("--exclude", default="", help="regex to reject")
    s.add_argument("--tags", default="", help="comma-separated tags, any-of")
    s.add_argument("--min-textures", type=int, default=1)
    s.add_argument("--min-texture-res", type=int, default=0)
    s.add_argument("--max-faces", type=int, default=400_000)
    s.add_argument("--licenses", default="", help="comma-separated, e.g. by,cc0")
    s.add_argument("--sort", default="likes",
                   choices=["likes", "views", "faces", "texture_res"])
    s.add_argument("--limit", type=int, default=25)
    s.add_argument("--refresh", action="store_true", help="rebuild the catalog cache")
    s.set_defaults(func=_cmd_search)

    e = sub.add_parser("ensure", help="cache one object by uid")
    e.add_argument("uid")
    e.add_argument("--target-size", type=float, default=0.0,
                   help="bake a real-world size in m (0 = keep model units)")
    e.add_argument("--fit", default="footprint",
                   choices=["footprint", "height", "max"],
                   help="which dimension --target-size pins")
    e.add_argument("--force", action="store_true", help="re-convert even if cached")
    e.set_defaults(func=_cmd_ensure)

    l = sub.add_parser("list", help="show cached assets")
    l.add_argument("--yaml", action="store_true", help="also print asset-pack entries")
    l.set_defaults(func=_cmd_list)

    args = ap.parse_args()
    return args.func(args) or 0


if __name__ == "__main__":
    raise SystemExit(main())
