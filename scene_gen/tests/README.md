# scene_gen guardrail tests

Fast, host-side tests for the scene generator's **layout**. No Isaac Sim, no
Nucleus, no Docker, no stage — a full run is a few seconds.

These are separate from the repo-root [`tests/`](../../tests/), which is the
containerized system-test suite run by `airstack test`. That suite's
`pytest.ini` scopes collection to its own directory, so the two never collect
each other.

## Running them

```bash
cd scene_gen
python3 -m pytest tests/ -v
```

**Use the system `python3`, not `AirStack/.venv`.** `scene_generator.py`
imports `pxr` at module scope; the system interpreter has `usd-core`
installed and the project venv does not. This is the same reason
`preset_report.py` says "plain python3".

## What's here

| File | Role |
|------|------|
| [`snapshot.py`](snapshot.py) | Turns a generation run into a stable, diffable signature. Also a CLI. |
| [`test_layout_decoupling.py`](test_layout_decoupling.py) | The assertions. |
| `snapshots/*.json` | Committed baselines — digests, category counts, and the street plan. |

## The snapshot CLI

```bash
python3 tests/snapshot.py                          # compare against baselines
python3 tests/snapshot.py --write                  # re-baseline (commit the diff)
python3 tests/snapshot.py --case tornado@s42@0.8   # one case
python3 tests/snapshot.py --explain earthquake 0.0 0.8   # what moved, and why
```

Baselines store **digests, not the full placement arrays** — verbatim those run
to ~1 MB per case and produce unreadable diffs. Every case is reproducible from
`(preset, seed, severity)`, so `--explain` rebuilds both sides on demand
instead. The street plan *is* stored verbatim: it is small and worth reading in
a review.

When a change to the generator is intended, re-run `--write` and let the
committed diff show what it did.

## The failing test is intentional

`test_layout_invariant_to_severity` is marked `xfail(strict=True)`. It encodes
requirement 4a of the three-stage refactor — *a locale and a seed fully
specify the layout; damage only adds to it* — which is not true yet:

```
earthquake seed 42:  severity 0 -> 0.8
  street plan : SAME
  layout      : CHANGED
  unmoved: 1454   moved/removed: 2609   new/relocated: 2609
  most-affected categories: plant=1212, tree=842, concrete=156, trail=97
```

Two causes, and the second is much the larger:

1. `_anchor_ok` (`scene_generator.py:1613`) gates which block a large building
   may anchor into by local damage intensity.
2. `build_city` runs on **one RNG** (`scene_generator.py:1247`) across all 130
   of its draw sites, so any damage-conditional draw shifts every subsequent
   placement. That is why 1,212 *plants* move when severity changes —
   building anchoring cannot explain that.

The street plan is already invariant, and `test_geometry_invariant_to_severity`
passes today; it is here to keep it that way.

Because the marker is `strict`, the suite will **fail** once the requirement is
met — that is the signal to delete the marker.
