"""measure_cache — persist asset footprint measurements between runs.

SPEC's pipeline names "measure assets / use cached" as a step of its own, in
both Stage A and Stage B. The measuring existed; the caching did not.

WHAT IT COSTS WITHOUT THIS
--------------------------
`SizeResolver` memoises within one process, so a run measures each distinct
asset once — but a measurement is `Usd.Stage.Open` plus a bbox compute, and for
a Nucleus asset that is a network round trip. Every fresh process pays the
whole bill again: every preset compile, every `plan.py --used-only`, every test
run, every scene launch, every Stage A bake.

WHAT IS AND IS NOT SAFE TO CACHE
--------------------------------
Only the UNIT-SCALE measurement, keyed on ``(path, axis_up)`` — the same key
`SizeResolver._raw` already uses. Footprint is exactly linear in scale, so the
per-placement scale is applied afterwards and never enters the key. Caching the
scaled result instead would be a cache miss per randomised debris scale, which
is the bug `SizeResolver`'s own docstring records costing 10m37s of a 78-minute
launch.

STALENESS
---------
Keyed on the asset's mtime and size where it is a local file, so re-running
`prepare_assets.py` at a new `target-size-m` invalidates exactly the entries it
changed — the failure mode that left a manhole 8 m across for weeks. A remote
(`omniverse://`) asset has no cheap stat, so those entries are keyed on path
alone and trusted; clear the cache with `--clear` if a Nucleus asset is
republished.

A FAILED MEASUREMENT IS ONLY CACHED WHEN IT IS A PROPERTY OF THE ASSET
----------------------------------------------------------------------
"Could not measure" means two different things. For a local file it means the
file is missing or broken — stable, worth remembering, and self-healing
because creating the file changes its stat token. For a REMOTE asset it means
"I could not reach Nucleus", which is a property of the machine and the
moment.

Caching the remote case is actively destructive, and it is not hypothetical:
building the cache on the host — where Nucleus is unreachable — poisoned 156
of 214 entries with None. The container, which CAN reach Nucleus, then read
that bind-mounted file, skipped measuring, and packed the whole city against
`fallback_sizes`. Buildings overflowed their blocks and ground tiles were laid
with gaps between them, because every footprint was a 4x4 m guess.

So a failure is stored only when the path can be stat-ed. Unreachable remote
assets are re-tried every run, which is the cost of being right.

A corrupt or unreadable cache is ignored rather than fatal. It is an
optimisation, and refusing to build a scene because a cache file got truncated
would be a worse failure than the one it prevents.

WRITE-THROUGH, NOT AT EXIT
--------------------------
A successful measurement is flushed to disk as soon as it is made, rather than
by the `atexit` handler alone. The handler is still registered — it catches the
local assets, where a write per measurement would be pure overhead — but it
cannot be the only path, because **the process that does the measuring worth
keeping never exits normally**. An Isaac launch script loops until the app
closes and the documented iteration loop kills it with `C-c`, so the expensive
half (Nucleus round trips, which are the only measurements a host process
cannot reproduce at all) was never reaching disk.

Only REMOTE successes are written through. Local ones are cheap to redo and
would make a scene with thousands of props write the file thousands of times.
"""

from __future__ import annotations

import json
import os
import tempfile

_SCENE_GEN = os.path.dirname(os.path.abspath(__file__))

#: Gitignored: it is derived, machine-specific and grows with every asset seen.
DEFAULT_PATH = os.path.join(_SCENE_GEN, "assets", ".measurements.json")

#: Bumped when the stored shape changes OR when previously-written entries are
#: known bad, so an old file is discarded rather than misread.
#: v2: v1 cached FAILED measurements for remote assets. See `put`.
VERSION = 2


def _stat_key(path: str) -> str:
    """A cheap staleness token for *path*, or "" when it cannot be stat'd."""
    try:
        st = os.stat(path)
    except OSError:
        return ""
    return f"{int(st.st_mtime)}:{st.st_size}"


class MeasureCache:
    """On-disk `(path, axis_up) -> footprint | None`.

    `None` is a real, cacheable answer: it means "this asset could not be
    measured", and re-deriving it costs the same failed open every time.
    """

    def __init__(self, path: str = "", autosave: bool = True):
        self.path = path or DEFAULT_PATH
        self.hits = 0
        self.misses = 0
        self._dirty = False
        self._data = self._load()
        if autosave:
            # Flush at exit rather than making every caller remember. A run
            # that measures 150 assets and then crashes should still leave the
            # next one warm, and the write is atomic so a half-file is not a
            # state this can reach.
            import atexit
            atexit.register(self.save)

    def _load(self) -> dict:
        try:
            with open(self.path) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            return {}
        if doc.get("version") != VERSION:
            return {}
        return doc.get("entries") or {}

    @staticmethod
    def _key(usd_path: str, axis_up: str) -> str:
        return f"{usd_path}\x00{str(axis_up or 'Z').upper()}"

    def get(self, usd_path: str, axis_up: str):
        """``(hit, footprint)``. *footprint* may legitimately be None."""
        rec = self._data.get(self._key(usd_path, axis_up))
        if rec is None:
            self.misses += 1
            return False, None
        # Local file changed underneath us -> treat as a miss.
        if rec.get("stat", "") != _stat_key(usd_path):
            self.misses += 1
            return False, None
        self.hits += 1
        return True, rec.get("fp")

    def put(self, usd_path: str, axis_up: str, fp) -> None:
        stat = _stat_key(usd_path)
        if fp is None and not stat:
            # Unstattable AND unmeasurable: almost certainly a remote asset on
            # a machine that cannot reach the server. Remembering that would
            # make every later run on a machine that CAN reach it skip
            # measuring and fall back to guessed footprints. See the module
            # docstring — this exact entry poisoned a whole city's packing.
            return
        self._data[self._key(usd_path, axis_up)] = {"stat": stat, "fp": fp}
        self._dirty = True
        # Write through on a remote success. `stat` is empty exactly for the
        # assets a host process cannot measure at all, and those are the ones
        # a launcher killed with C-c would otherwise never persist. See the
        # module docstring.
        if fp is not None and not stat:
            self.save()

    def save(self) -> bool:
        """Atomic write. Returns whether anything was written."""
        if not self._dirty:
            return False
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path),
                                       suffix=".tmp")
            with os.fdopen(fd, "w") as fh:
                json.dump({"version": VERSION, "entries": self._data}, fh)
            os.replace(tmp, self.path)
        except OSError as exc:
            print(f"[measure_cache] could not save: {exc}")
            return False
        self._dirty = False
        return True

    def __len__(self):
        return len(self._data)


def clear(path: str = "") -> bool:
    try:
        os.remove(path or DEFAULT_PATH)
        return True
    except OSError:
        return False


def main():
    import argparse

    ap = argparse.ArgumentParser(description="Inspect the measurement cache.")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--path", default="")
    args = ap.parse_args()
    if args.clear:
        print("cleared" if clear(args.path) else "nothing to clear")
        return
    c = MeasureCache(args.path)
    print(f"{len(c)} measurement(s) in {c.path}")


if __name__ == "__main__":
    main()
