#!/usr/bin/env python
"""nucleus_fetch.py — mirror the round-4 rubble debris catalogue from Nucleus
to the local checkout, so `disaster.quake_rubble_usd.author()` can run with a
LOCAL `asset_root` (previews, host-side tests, an offline Isaac Sim mount)
instead of resolving `omniverse://` on every reference.

Must run INSIDE the isaac-sim container on the bare `pxr` + `omni.client`
python (no Kit, no SimulationApp — see `tools/_t_pxr.sh`):

    scene_gen/tools/_t_pxr.sh scene_gen/tools/nucleus_fetch.py

The container mounts the repo at /isaac-sim/AirStack, so writing under
/isaac-sim/AirStack/scene_gen/assets/... lands directly in the host checkout
— this script never needs to know it is in a container beyond that path.

What is mirrored (relative to
`omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/`,
matching the local `scene_gen/assets/` tree exactly):

    concrete_rubble_debris/split/<dir>/          every asset dir EXCEPT the
                                                  `*_hp` (high-poly) siblings
                                                  — same prop, more triangles,
                                                  never worth the tris budget.
    standalone/debris/pieces/<dir>/               all 34 (flat colour, no
                                                   textures/ folder to fetch)
    standalone/debris/piles/<dir>/                2 (textured)

Each asset dir's `.usdc` and (if present) `textures/*` are copied via
`omni.client.read_file` + a local write — the same pattern already proven in
this repo by `tools/_g_tex_fetch.py` (`omni.client.copy` needs both ends to
be Omniverse URLs on some client builds; read+write works for any local
destination). A file already on disk with the SAME byte size is skipped, so
a re-run after a partial fetch only pulls what is missing.
"""
import os
import sys

REPO = "/isaac-sim/AirStack"
sys.path.insert(0, os.path.join(REPO, "scene_gen"))

NUCLEUS_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                "Projects/SEI-COA/scene_gen/assets/")
LOCAL_ROOT = os.path.join(REPO, "scene_gen", "assets")

# (nucleus subdir under NUCLEUS_ROOT, whether to skip "*_hp" siblings)
FOLDERS = [
    ("concrete_rubble_debris/split/", True),
    ("standalone/debris/pieces/", False),
    ("standalone/debris/piles/", False),
]


def _is_dir(oc, entry):
    return bool(entry.flags & oc.ItemFlags.CAN_HAVE_CHILDREN)


def _list(oc, url):
    r, ents = oc.list(url)
    if r != oc.Result.OK:
        return None
    return ents


def _fetch_file(oc, url, local_path, table):
    """Read one file from Nucleus and write it locally unless an
    identically-sized copy already exists. Returns bytes actually written
    (0 if skipped or failed)."""
    r, _ver, content = oc.read_file(url)
    if r != oc.Result.OK:
        print("  FAIL read {0}: {1}".format(url, r))
        table.append((url, 0, "FAIL " + str(r)))
        return 0
    data = bytes(memoryview(content))
    size = len(data)
    if os.path.exists(local_path) and os.path.getsize(local_path) == size:
        table.append((url, size, "skip (up to date)"))
        return 0
    d = os.path.dirname(local_path)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    with open(local_path, "wb") as fh:
        fh.write(data)
    table.append((url, size, "fetched"))
    return size


def _mirror_asset_dir(oc, asset_url, asset_local, table):
    """Copy every plain file directly under `asset_url` (the .usdc) plus
    everything under its `textures/` child (if any)."""
    ents = _list(oc, asset_url)
    if ents is None:
        print("  LIST FAILED", asset_url)
        return
    for e in ents:
        name = e.relative_path
        if _is_dir(oc, e):
            if name.lower() != "textures":
                continue
            tex_url = asset_url + name + "/"
            tex_ents = _list(oc, tex_url)
            if not tex_ents:
                continue
            for te in tex_ents:
                if _is_dir(oc, te):
                    continue
                _fetch_file(oc, tex_url + te.relative_path,
                            os.path.join(asset_local, "textures", te.relative_path),
                            table)
        else:
            _fetch_file(oc, asset_url + name, os.path.join(asset_local, name), table)


def main():
    import omni.client as oc

    table = []
    for sub, skip_hp in FOLDERS:
        base_url = NUCLEUS_ROOT + sub
        base_local = os.path.join(LOCAL_ROOT, *sub.strip("/").split("/"))
        print("== {0}".format(sub))
        ents = _list(oc, base_url)
        if ents is None:
            print("  LIST FAILED", base_url)
            continue
        for e in ents:
            if not _is_dir(oc, e):
                continue
            name = e.relative_path
            if skip_hp and name.endswith("_hp"):
                print("  skip (hp): {0}".format(name))
                continue
            asset_url = base_url + name + "/"
            asset_local = os.path.join(base_local, name)
            _mirror_asset_dir(oc, asset_url, asset_local, table)

    fetched = [t for t in table if t[2] == "fetched"]
    skipped = [t for t in table if t[2].startswith("skip")]
    failed = [t for t in table if t[2].startswith("FAIL")]

    print("\n{0:<78} {1:>10}  {2}".format("relative path", "bytes", "status"))
    print("-" * 100)
    for url, size, status in table:
        rel = url.replace(NUCLEUS_ROOT, "")
        print("{0:<78} {1:>10}  {2}".format(rel, size, status))

    total_mb = sum(t[1] for t in fetched) / (1024.0 * 1024.0)
    print("-" * 100)
    print("fetched {0} files, {1:.2f} MB; skipped {2} (already present); {3} failed"
          .format(len(fetched), total_mb, len(skipped), len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
