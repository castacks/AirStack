#!/usr/bin/env python3
"""Generate `downtown_gac_probe.yaml` from `downtown_gac.yaml`.

    python3 scene_gen/tools/make_probe_preset.py

A PROBE SCENE IS THE PARENT SCENE WITH MOST OF IT SWITCHED OFF — the point is
to look at two districts in isolation and decide whether their placement is
right, without waiting for a whole city to build and without the rest of the
city competing for attention. So it MUST carry the parent's district tuning
exactly: the terrace bands, the block targets, the repeat knobs, the ring
radii. A hand-written copy would drift from the parent the first time either
is touched, and a drifted probe answers a question nobody asked.

The config system has no preset-level `extends` (`compile_spec` merges a preset
onto the low-level base, not onto another preset), so the copy is made here
instead and REGENERATED rather than edited. Edit `downtown_gac.yaml` and re-run
this.

What it changes, and nothing else:

  districts.probe          zone only the named typologies, that many blocks
                           each, and leave every other block EMPTY
  parks / park chance      off — a park is a block of trees to load and there
                           is nothing to review in it here
  layout.region_m          unchanged at 800 m, so block sizes and the ring
                           radii (which are a fraction of the HALF-DIAGONAL,
                           see the parent) are identical to the real scene
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PRESETS = os.path.join(HERE, "..", "config", "presets")
SRC = os.path.join(PRESETS, "downtown_gac.yaml")
DST = os.path.join(PRESETS, "downtown_gac_probe.yaml")

# typology -> how many blocks.
#
# FIVE highrise blocks, not the two or three the review first asked for, and
# the number is MEASURED rather than chosen: the user wants every skyscraper on
# the plate so they can see how each one lands, and a sweep of the block count
# against the `highrise` pool shows how many slots that actually takes —
#     3 blocks -> 10/11 (SM_Building_20 never drawn)
#     4 blocks -> 10/11 (SM_Building_15 never drawn)
#     5 blocks -> 11/11
#     6 blocks -> 11/11
# Re-run the sweep if the pool or the tall-separation knobs change; fewer
# towers per block means more blocks are needed for the same coverage.
PROBE = {"brick_midrise": 3, "highrise": 5}

HEADER = '''# GENERATED FILE — DO NOT EDIT.
#     python3 scene_gen/tools/make_probe_preset.py
# regenerates it from `downtown_gac.yaml`. Edit THAT and re-run; see the
# script's docstring for why this is generated rather than hand-copied.
#
# THE TWO NEW DISTRICTS, ALONE, ON AN OTHERWISE EMPTY 800 m PLATE.
# `districts.probe` zones {probe} and leaves every
# other block unbuilt, so the tall glass towers and the brick terrace can be
# reviewed without a whole city to load or to look past. Roads, kerbs and
# sidewalks are still built — a blank wall facing a street cannot be judged
# without the street.
#
#     SCENE_CONFIG=downtown_gac_probe \\
#     ISAAC_SIM_SCRIPT_NAME=scene_launch_script.py \\
#     airstack up isaac-sim
#
# The timeline does NOT autoplay; export AUTOPLAY=1 if you want it running.
'''


def main():
    src = open(SRC).read()
    if "districts:" not in src:
        sys.exit("no districts block in %s" % SRC)

    # `probe` goes at the top of the districts block, where it is impossible to
    # miss when reading the file.
    anchor = "  districts:\n"
    if src.count(anchor) != 1:
        sys.exit("expected exactly one '  districts:' in %s" % SRC)
    probe_yaml = "".join(
        "      %s: %d\n" % (k, v) for k, v in PROBE.items())
    src = src.replace(anchor, anchor + (
        "    # PROBE MODE — see this file's header. Only these typologies are\n"
        "    # zoned, on at most this many blocks each; every other block is\n"
        "    # left empty (no buildings, no infill).\n"
        "    probe:\n" + probe_yaml), 1)

    # Parks off: nothing to review in one, and it is a lot of geometry.
    src = src.replace("      parks:\n        enabled: true\n",
                      "      parks:\n        enabled: false\n")
    for a, b in (("    park_block_chance: ", "    park_block_chance: 0.0  # probe: "),
                 ("    min_parks: ", "    min_parks: 0  # probe: "),
                 ("    max_parks: ", "    max_parks: 0  # probe: ")):
        i = src.find(a)
        if i >= 0:
            j = src.find("\n", i)
            src = src[:i] + b + src[i + len(a):j].strip() + src[j:]

    open(DST, "w").write(HEADER.format(probe=PROBE) + "\n" + src)
    print("wrote %s" % os.path.normpath(DST))


if __name__ == "__main__":
    main()
