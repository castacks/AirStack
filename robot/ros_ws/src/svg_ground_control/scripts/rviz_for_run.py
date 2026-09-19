#!/usr/bin/env python3
"""Strip an RViz config down to the drones a run actually has.

    rviz_for_run.py <master.rviz> <out.rviz> drone_1 drone_2 drone_3

The shipped config carries camera panels for every scenario, so running one of
them leaves blank panels for drones that were never spawned. This keeps the
displays whose topic names a live drone, plus everything that is not
drone-specific (grid, markers), and writes the result somewhere temporary.

Text in, text out: RViz configs carry a Qt geometry blob that a YAML
round-trip would reformat, so the file is edited as lines.
"""

from __future__ import annotations

import re
import sys

BLOCK_START = "    - Class:"


def block_topic(block: list[str]) -> str | None:
    for line in block:
        m = re.match(r"\s*Value: (/\S+)", line)
        if m:
            return m.group(1)
    return None


def keep(block: list[str], drones: set[str]) -> bool:
    topic = block_topic(block)
    if topic is None:
        return True                       # no topic (Grid) — always keep
    m = re.match(r"/([^/]+)/", topic)
    if not m:
        return True
    owner = m.group(1)
    if not re.fullmatch(r"(drone|robot)_\d+", owner):
        return True                       # /svg/viz/markers and friends
    return owner in drones


def main(argv: list[str]) -> int:
    if len(argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2
    src, dst, drones = argv[1], argv[2], set(argv[3:])

    lines = open(src).read().split("\n")

    # Scope to the Displays list only: "    - Class:" also matches Tools
    # entries further down, and merging those into Displays corrupts the file.
    try:
        first = next(i for i, l in enumerate(lines) if l.strip() == "Displays:")
    except StopIteration:
        open(dst, "w").write("\n".join(lines))
        return 0
    end = next((i for i in range(first + 1, len(lines))
                if lines[i] and not lines[i].startswith("    ")), len(lines))

    starts = [i for i in range(first, end) if lines[i].startswith(BLOCK_START)]
    if not starts:
        open(dst, "w").write("\n".join(lines))
        return 0

    head = lines[:starts[0]]
    bounds = starts + [end]

    kept, dropped = [], []
    for a, b in zip(bounds, bounds[1:]):
        block = lines[a:b]
        (kept if keep(block, drones) else dropped).append(block)

    out = head + [l for blk in kept for l in blk] + lines[end:]
    open(dst, "w").write("\n".join(out))

    for blk in dropped:
        name = next((l.split("Name: ", 1)[1] for l in blk
                     if l.strip().startswith("Name: ")), "?")
        print(f"dropped: {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
