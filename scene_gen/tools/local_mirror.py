"""Where an `omniverse://` asset lives on this disk.

STDLIB ONLY, ON PURPOSE. Two interpreters need this answer and neither can
import the other's world: the host env has `pxr` and no `bpy`, the bpy env has
`bpy` and no `pxr` (see `ENVIRONMENTS.md`). A resolver with no dependencies is
importable from both, which is the only way `tools/asset_properties.py` and
`tools/render_archetypes.py` can agree about which file they are looking at.

WHY A MIRROR IS NEEDED AT ALL
-----------------------------
`omni.usd_resolver` is what turns an `omniverse://` URL into bytes, and it is
only present inside Kit. Anything outside Kit — plain `pxr`, Blender's USD —
sees the URL as a path that does not exist: `pxr` raises, and Blender silently
drops the texture and renders the asset grey. Nearly all of this pack is also
on disk, either because it was published FROM the repo or because it was
mirrored INTO `assets/nucleus/`, so the fix is a path rewrite rather than a
download.
"""
from __future__ import annotations

import os

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(_HERE)
AIRSTACK = os.path.dirname(SCENE_GEN)

#: `shared.yaml`'s `asset_root`. Every relative entry in a pack hangs off it.
NUCLEUS_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                "Projects/SEI-COA/")

#: Where to look for a file by BASENAME when no prefix rule finds it. A last
#: resort, and worth having for previews: a plausible-but-wrong texture is a
#: much smaller lie than a grey building.
INDEX_ROOTS = ("assets/standalone", "assets/nucleus", "assets/downtowncity",
               "assets/aec", "assets/objaverse")

_IMAGE_EXT = (".png", ".jpg", ".jpeg", ".exr", ".tga", ".hdr")
_INDEX: dict = {}


def to_url(source: str) -> str:
    """A pack entry (relative or absolute) as the URL Stage A referenced."""
    s = str(source)
    if "://" in s:
        return s
    return NUCLEUS_ROOT + s.lstrip("/")


def local_for(url: str):
    """The on-disk twin of *url*, or None.

    THE CANDIDATES ARE TRIED IN ORDER because the mirror is not always at the
    server's depth: `scene_gen/assets/...` was published from the repo and
    round-trips exactly, while `DownTown/Assets/...` was mirrored to
    `assets/nucleus/Muyang/DownTown/Assets/...` — one level deeper than the
    URL says.
    """
    s = str(url)
    if not s:
        return None
    if "://" not in s and os.path.exists(s):
        return s
    # Blender normalises `omniverse://x` to `omniverse:/x`; accept both.
    s = s.replace("omniverse:/", "omniverse://", 1) if s.startswith(
        "omniverse:/") and not s.startswith("omniverse://") else s
    if not s.startswith(NUCLEUS_ROOT):
        return None
    rest = s[len(NUCLEUS_ROOT):]
    for cand in (os.path.join(AIRSTACK, rest),
                 os.path.join(SCENE_GEN, "assets", "nucleus", rest),
                 os.path.join(SCENE_GEN, "assets", "nucleus", "Muyang", rest)):
        if os.path.exists(cand):
            return cand
    return _index().get(os.path.basename(rest).lower())


def _index() -> dict:
    """`{lowercased filename: path}` over the local mirrors. Built once."""
    if _INDEX:
        return _INDEX
    for rel in INDEX_ROOTS:
        root = os.path.join(SCENE_GEN, rel)
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            for f in files:
                if f.lower().endswith(_IMAGE_EXT):
                    _INDEX.setdefault(f.lower(), os.path.join(dirpath, f))
    return _INDEX
