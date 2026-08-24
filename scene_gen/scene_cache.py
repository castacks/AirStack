"""scene_cache — the two-tier offline scene cache from SPEC's Entrypoint 1.

    seed, asset pack, locale            ->  layout + pristine scene
    + disaster type and severity        ->  disaster scene (child of the above)

WHY TWO TIERS AND NOT ONE KEY
-----------------------------
Because that split is the generator's central invariant, not a storage detail:
a locale and a seed fully specify the layout, and severity only decides what
happens to it. A severity sweep is therefore the SAME city at several damage
levels — which is the entire reason comparing a search algorithm across
severities means anything (`tests/test_layout_decoupling.py` enforces it).

Keying the cache the same way makes that structural. The pristine scene is
computed once and every severity hangs off it, so a five-point sweep pays for
one layout rather than five, and two severities that disagreed about the layout
would be a cache-shape error rather than a subtle visual one.

WHAT IS STORED
--------------
    assets/scenes/<pristine-key>/meta.json      the spec that produced it
                                /layout.json    blocks, roads, region
                                /pristine.usd
                                /<disaster-key>/meta.json
                                                /scene.usd

Keys are short hashes of the fields that actually change the output, so an
unrelated edit to a preset's comments does not invalidate anything.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
It does not decide whether a cached scene is still *valid* against changed
asset packs or changed generator code. A pack edit changes `pack_digest` and
misses; a code change does not, and pretending otherwise would need a hash of
the whole tree on every lookup. `--clear` is the honest answer, and the key is
printed on every hit so a suspicious scene can be traced to its entry.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil

_SCENE_GEN = os.path.dirname(os.path.abspath(__file__))

#: Gitignored — derived, large, and rebuilt by `bake_scene.py`.
DEFAULT_ROOT = os.path.join(_SCENE_GEN, "assets", "scenes")

LAYOUT_NAME = "layout.json"
PRISTINE_NAME = "pristine.usd"
SCENE_NAME = "scene.usd"
META_NAME = "meta.json"


def _digest(obj) -> str:
    return hashlib.sha256(
        json.dumps(obj, sort_keys=True, separators=(",", ":"),
                   default=str).encode()).hexdigest()


def pack_digest(config: dict) -> str:
    """A hash of the RESOLVED asset lists, not of the pack's filename.

    Two configs naming `urban` are only the same scene if `urban` still means
    the same assets. Hashing the resolved `usds` block catches an edited pack,
    a re-pointed `extends:`, and a pool that gained an entry — all of which
    change the scene while leaving the pack name identical.
    """
    return _digest(config.get("usds") or {})[:12]


def pristine_key(config: dict) -> str:
    """Tier 1: seed, asset pack, locale — plus the layout knobs they drive."""
    return _digest({
        "seed": config.get("seed"),
        "locale": config.get("locale"),
        "pack": config.get("asset_pack"),
        "pack_digest": pack_digest(config),
        # The region and the subdivision settings are what a locale *means*
        # here; two "urban" scenes at different region sizes are not one scene.
        "layout": config.get("layout"),
        "packing": config.get("packing"),
    })[:16]


def disaster_key(config: dict) -> str:
    """Tier 2: what the event was and how bad.

    The whole compiled `disaster:` block, not just type and severity: a preset
    that overrides the field geometry (as `urban_quake_tiny` does) produces a
    different scene at the same nominal severity, and silently serving the
    other one would be the worst failure this cache could have.
    """
    return _digest(config.get("disaster") or {})[:16]


def is_pristine(config: dict) -> bool:
    dis = config.get("disaster") or {}
    return (str(dis.get("type", "none")) == "none"
            or float(dis.get("severity", 0.0) or 0.0) <= 0.0)


class SceneCache:
    def __init__(self, root: str = ""):
        self.root = root or DEFAULT_ROOT

    # -- paths ------------------------------------------------------------
    def pristine_dir(self, config: dict) -> str:
        return os.path.join(self.root, pristine_key(config))

    def disaster_dir(self, config: dict) -> str:
        return os.path.join(self.pristine_dir(config), disaster_key(config))

    def scene_dir(self, config: dict) -> str:
        """Where THIS config's finished scene lives — tier 1 when pristine."""
        return (self.pristine_dir(config) if is_pristine(config)
                else self.disaster_dir(config))

    def scene_usd(self, config: dict) -> str:
        return os.path.join(
            self.scene_dir(config),
            PRISTINE_NAME if is_pristine(config) else SCENE_NAME)

    def layout_json(self, config: dict) -> str:
        return os.path.join(self.pristine_dir(config), LAYOUT_NAME)

    # -- read -------------------------------------------------------------
    def get(self, config: dict) -> str:
        """The cached scene USD for *config*, or "" on a miss."""
        p = self.scene_usd(config)
        return p if os.path.isfile(p) else ""

    def layout(self, config: dict) -> dict:
        """The cached layout, or {}. Available even when only tier 1 is warm."""
        try:
            with open(self.layout_json(config)) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    # -- write ------------------------------------------------------------
    def put_layout(self, config: dict, layout: dict) -> str:
        d = self.pristine_dir(config)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, LAYOUT_NAME)
        with open(path, "w") as fh:
            json.dump(layout, fh, default=list)
        self._write_meta(d, {
            "tier": "pristine", "key": pristine_key(config),
            "seed": config.get("seed"), "locale": config.get("locale"),
            "asset_pack": config.get("asset_pack"),
            "pack_digest": pack_digest(config),
            "region_m": (config.get("layout") or {}).get("region_m"),
        })
        return path

    def reserve(self, config: dict) -> str:
        """Create this config's scene directory and return the USD path."""
        d = self.scene_dir(config)
        os.makedirs(d, exist_ok=True)
        if not is_pristine(config):
            dis = config.get("disaster") or {}
            self._write_meta(d, {
                "tier": "disaster", "key": disaster_key(config),
                "parent": pristine_key(config),
                "type": dis.get("type"), "severity": dis.get("severity"),
            })
        return self.scene_usd(config)

    @staticmethod
    def _write_meta(d: str, meta: dict) -> None:
        with open(os.path.join(d, META_NAME), "w") as fh:
            json.dump(meta, fh, indent=2, sort_keys=True, default=str)

    # -- admin ------------------------------------------------------------
    def entries(self) -> list:
        """``[(pristine_meta, [disaster_meta, ...]), ...]``."""
        out = []
        if not os.path.isdir(self.root):
            return out
        for name in sorted(os.listdir(self.root)):
            d = os.path.join(self.root, name)
            if not os.path.isdir(d):
                continue
            meta = _read(os.path.join(d, META_NAME))
            kids = []
            for sub in sorted(os.listdir(d)):
                sd = os.path.join(d, sub)
                if os.path.isdir(sd):
                    kids.append(_read(os.path.join(sd, META_NAME)))
            out.append((meta, kids))
        return out

    def clear(self) -> int:
        n = len(self.entries())
        shutil.rmtree(self.root, ignore_errors=True)
        return n


def _read(path: str) -> dict:
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def main():
    import argparse

    ap = argparse.ArgumentParser(description="Inspect the offline scene cache.")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--root", default="")
    args = ap.parse_args()

    c = SceneCache(args.root)
    if args.clear:
        print(f"cleared {c.clear()} pristine entr(ies) from {c.root}")
        return
    entries = c.entries()
    if not entries:
        print(f"empty ({c.root})")
        return
    for meta, kids in entries:
        print(f"{meta.get('key', '?')}  seed={meta.get('seed')} "
              f"locale={meta.get('locale')} pack={meta.get('asset_pack')} "
              f"region={meta.get('region_m')}")
        for k in kids:
            print(f"    └ {k.get('key', '?')}  {k.get('type')} "
                  f"sev={k.get('severity')}")


if __name__ == "__main__":
    main()
