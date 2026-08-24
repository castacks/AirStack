"""library — what the archetype library holds, and how it is addressed.

Pure Python: no `pxr`, no Kit. Both the baker (which needs Isaac Sim) and the
assembler (which does not, until it writes a reference) agree on layout and
naming through this module, so the two cannot drift apart. A mismatch here is
the failure mode that costs a whole bake: Stage A writes `house_ranch_snag.usd`
and Stage B looks for `ranch_snag.usd`, and nothing notices until a scene comes
up full of holes.

LAYOUT ON DISK
--------------
    assets/archetypes/<disaster>/<type>_<level>.usd
    assets/archetypes/<disaster>/manifest.json

The disaster is the DIRECTORY rather than part of the stem, so one asset type's
ladder can be compared across disasters by listing sibling folders, and a
disaster can be re-baked by deleting one directory.

`<type>` identifies the thing, not the file it came from: a modular house style
(`ranch`), a tree species (`Black_Oak`), or a slug derived from a library USD's
path. Slugging matters because urban buildings are library assets whose paths
carry uids and spaces — see `type_slug`.
"""

from __future__ import annotations

import json
import os
import re

#: Where the library lives, relative to `scene_gen/`. Gitignored: it is
#: rebuilt by the baker and is far too large to track.
DEFAULT_ROOT = os.path.join("assets", "archetypes")

MANIFEST_NAME = "manifest.json"


def root_dir(scene_gen_dir: str, root: str = "") -> str:
    return root or os.path.join(scene_gen_dir, DEFAULT_ROOT)


def disaster_dir(scene_gen_dir: str, disaster: str, root: str = "") -> str:
    return os.path.join(root_dir(scene_gen_dir, root),
                        str(disaster or "none").lower())


def type_slug(name: str) -> str:
    """A filename-safe, stable identifier for an asset type.

    Library assets arrive as `objaverse://<uid>` or as Nucleus paths with
    spaces and mixed case (`Bungalow The Chase`). Both have to become one
    filename component that survives a round trip through a manifest and a
    shell, and that stays the SAME across runs — a slug derived from anything
    order-dependent (an index into a pool) renames every archetype the moment
    an asset is added, which silently invalidates the whole library.
    """
    s = str(name or "")
    if "://" in s:
        s = s.split("://", 1)[1]
    s = os.path.splitext(os.path.basename(s.rstrip("/")))[0]
    s = re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_")
    return s or "asset"


def archetype_name(asset_type: str, level: str) -> str:
    """The file stem: ``<type>_<level>``."""
    return f"{type_slug(asset_type)}_{level}"


def archetype_path(scene_gen_dir: str, disaster: str, asset_type: str,
                   level: str, root: str = "") -> str:
    return os.path.join(disaster_dir(scene_gen_dir, disaster, root),
                        archetype_name(asset_type, level) + ".usd")


def write_manifest(path: str, records: list, meta: dict = None) -> str:
    """Write the manifest for one disaster's library.

    Records carry enough to assemble WITHOUT reopening every USD: the type, the
    level, the source asset, and the measured bounds. Stage B places by
    footprint, so making it open a few hundred USDs to find that out would put
    the cost the bake exists to remove straight back into assembly.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    doc = {"version": 1, **(meta or {}), "archetypes": records}
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    return path


def read_manifest(path: str) -> dict:
    if not os.path.isfile(path):
        return {}
    with open(path) as fh:
        return json.load(fh) or {}


class Library:
    """Read side: what was baked for one disaster, addressed by (type, level).

    Falls back down the ladder rather than raising when a level is missing —
    an assembler that dies because one archetype failed to export turns a
    partial bake into no scene at all, when the honest answer is "use the worst
    level that DID bake and say so". `misses` records every fallback so a run
    can report how complete its library was.
    """

    def __init__(self, path: str = "", doc: dict = None):
        self.path = path
        #: Where the USDs actually live. A record's `usd` is a bare filename
        #: relative to this, so a library stays valid when it is moved,
        #: copied, or published to Nucleus.
        self.root = os.path.dirname(os.path.abspath(path)) if path else ""
        self.doc = doc if doc is not None else read_manifest(path)
        self.misses: dict = {}
        self._by_key = {}
        for rec in self.doc.get("archetypes", ()):
            self._by_key[(str(rec.get("type")), str(rec.get("level")))] = rec

    def __len__(self):
        return len(self._by_key)

    @property
    def disaster(self) -> str:
        return str(self.doc.get("disaster", "none"))

    def types(self) -> list:
        return sorted({t for t, _ in self._by_key})

    def get(self, asset_type: str, level: str) -> dict:
        """The record for (type, level), or ``{}``."""
        return self._by_key.get((type_slug(asset_type), str(level)), {})

    def usd_path(self, rec: dict) -> str:
        """The absolute path to a record's USD."""
        usd = str((rec or {}).get("usd", ""))
        if not usd:
            return ""
        return usd if os.path.isabs(usd) else os.path.join(self.root, usd)

    def resolve(self, asset_type: str, level: str, ladder_names: list) -> dict:
        """(type, level), falling back down *ladder_names* toward pristine.

        *ladder_names* is in intensity order, so walking it backwards from the
        requested level finds the closest baked level that is no WORSE than
        asked for — never a more damaged asset than the field called for, which
        would be visibly wrong in the other direction.
        """
        rec = self.get(asset_type, level)
        if rec:
            return rec
        names = list(ladder_names)
        if str(level) in names:
            for cand in reversed(names[:names.index(str(level))]):
                rec = self.get(asset_type, cand)
                if rec:
                    key = f"{type_slug(asset_type)}:{level}"
                    self.misses[key] = self.misses.get(key, 0) + 1
                    return rec
        key = f"{type_slug(asset_type)}:{level}"
        self.misses[key] = self.misses.get(key, 0) + 1
        return {}
