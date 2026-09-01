"""sky_probe.py — bare-pxr Nucleus stat of mid-day sky HDRI candidates.

WHY THIS EXISTS. `sky_presets.py`'s `mid_day` preset is fully procedural (no
HDRI dependency, on purpose — see its own module docstring), but a real sky
texture reads better than a flat dome once one is confirmed to resolve. This
repo has THREE different guesses at what AirLab's Nucleus mirror actually
calls its sky library (see `scene_gen/_plans/isaac_overcast_sky.md` for the
full trace), and nobody has run a probe against any of them. This is that
probe, extended to also try the public NVIDIA content CDN over HTTPS, which
depends on outbound internet rather than the Nucleus server at all.

NUCLEUS WAS DOWN (full TCP outage) when this was written — it has NEVER been
run. Every path below is a CANDIDATE, not a confirmed asset. Run it once
Nucleus is back:

    docker cp sky_probe.py isaac-sim:/tmp/sky_probe.py
    docker exec isaac-sim bash -c 'cd /isaac-sim && \
        PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ./python.sh /tmp/sky_probe.py \
        >/tmp/sky_probe_stdout.txt 2>&1; cat /tmp/sky_probe.txt'

Same "bare python.sh, no SimulationApp" pattern as `_nuc_ls.py` /
`nucleus_browse.py` in this directory — `omni.client` imports fine without
booting Kit, so this is safe to run ALONGSIDE a live Isaac Sim process (it
does not touch the GPU, does not open a stage, does not import `pxr`).

Env:
    NUC_SERVER   Nucleus host[:port] (default airlab-nucleus.andrew.cmu.edu:443)
    SKY_OUT      output file (default /tmp/sky_probe.txt); stdout is also
                 swallowed by the Kit logger inside the container, same
                 reason `nucleus_browse.py` writes to a file.
"""
import os

import omni.client as oc

SERVER = os.environ.get("NUC_SERVER", "airlab-nucleus.andrew.cmu.edu:443").strip()
OUT = os.environ.get("SKY_OUT", "/tmp/sky_probe.txt")

# ---------------------------------------------------------------------------
# ROOT candidates — the "version-root problem" from
# scene_gen/_plans/isaac_overcast_sky.md. Ranked by confidence, highest
# first; #1 and #2 come from this repo's OWN currently-active asset root
# (example_multi_drone_scene_import.py sets
# `/persistent/isaac/asset_root/default` to
# `omniverse://<server>/NVIDIA/Assets/Isaac/5.1`), #3/#4 are the legacy
# content-pack layout the Kit log's `SunsetSkyMat.mdl` path implies
# (`Environments/2023_1/DomeLights/...`) and the never-run `_sky_stat.py`
# guess (`2024_1`) sitting next to it, #5 is a known-good control (already
# confirmed live and in use — RetroNeighborhood's own borrowed sky rig) so a
# probe run that returns all-FAIL still proves the SERVER, not just the
# paths, is reachable.
# ---------------------------------------------------------------------------
ROOTS = [
    ("new-doubled",  "omniverse://{0}/NVIDIA/Assets/Isaac/5.1/NVIDIA/Assets/Skies/".format(SERVER)),
    ("new-plain",    "omniverse://{0}/NVIDIA/Assets/Isaac/5.1/Skies/".format(SERVER)),
    ("legacy-2023",  "omniverse://{0}/NVIDIA/Environments/2023_1/DomeLights/".format(SERVER)),
    ("legacy-2024",  "omniverse://{0}/NVIDIA/Environments/2024_1/DomeLights/".format(SERVER)),
]
CONTROL = ("retro-control",
          "omniverse://{0}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd".format(SERVER))

# ---------------------------------------------------------------------------
# FILE candidates, ranked for a MID-DAY look (task: "more of a mid day
# vibe"). Merged from the vendored-in-this-repo confirmed listing
# (infinigen_sdg_utils.py, pose_generation configs, PegasusSimulator
# examples) — see isaac_overcast_sky.md's category table. `AVOID` entries
# are included anyway so the probe output can confirm/deny them too (one of
# them, mealie_road, is independently confirmed BY THIS REPO to read as
# evening — downtown_gac.yaml's own "---- LIGHT ----" comment — so it is
# useful as a negative control, not a candidate to pick).
# ---------------------------------------------------------------------------
CANDIDATES = [
    # rank, category/file, why
    (1, "Clear/noon_grass_4k.hdr",
     "name says noon; already the confirmed default HDRI in this exact "
     "repo (quadruped_scene_launch_script.py's DOME_LIGHT_TEXTURE)"),
    (2, "Clear/syferfontein_18d_clear_4k.hdr",
     "PolyHaven open veld, clear sky, sun well up -- good midday candidate"),
    (3, "Clear/sunflowers_4k.hdr",
     "bright open field, sun fairly high"),
    (4, "Clear/white_cliff_top_4k.hdr",
     "bright coastal/overcast-leaning -- neutral fallback if noon_grass 404s"),
    (5, "Cloudy/champagne_castle_1_4k.hdr",
     "bright partly-cloudy mountain scene -- midday WITH some sky texture"),
    (6, "Cloudy/kloofendal_48d_partly_cloudy_4k.hdr",
     "partly cloudy, still visible sun -- weaker of the two Cloudy picks"),
    (7, "Clear/qwantani_4k.hdr",
     "PolyHaven savanna -- warmer/lower sun than noon_grass, rank below it"),
    # -- AVOID: sunset/evening/sunrise by name, or independently confirmed
    #    bad in this repo. Probed anyway, as negative controls.
    (90, "Clear/mealie_road_4k.hdr",
     "AVOID -- confirmed (downtown_gac.yaml) to make the city read as "
     "evening; a clear-sky HDRI bakes in its own low sun, no intensity "
     "knob can raise it"),
    (91, "Clear/evening_road_01_4k.hdr", "AVOID -- evening by name"),
    (92, "Clear/venice_sunset_4k.hdr", "AVOID -- sunset by name"),
    (93, "Clear/signal_hill_sunrise_4k.hdr", "AVOID -- sunrise, low sun"),
    (94, "Clear/kloppenheim_02_4k.hdr", "AVOID -- also filed under Night/"),
]

# Public NVIDIA content CDN — no Nucleus dependency at all (outbound HTTPS
# instead). Confirmed URL SHAPE and the `4.5` version segment are live in
# this repo's own vendored PegasusSimulator example
# (5_python_multi_vehicle.py); see isaac_overcast_sky.md option 1.
CDN_ROOT = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
           "/Assets/Isaac/4.5/NVIDIA/Assets/Skies/")


def stat(url):
    try:
        r = oc.stat(url)
        res = r[0] if isinstance(r, tuple) else r
        size = getattr(r[1], "size", None) if isinstance(r, tuple) and len(r) > 1 else None
        return res, size
    except Exception as exc:  # a bad/unsupported scheme should not kill the run
        return "EXC({0})".format(exc), None


def main():
    lines = []
    lines.append("sky_probe — server {0}".format(SERVER))
    lines.append("=" * 78)

    lines.append("\n-- control (expected OK; proves the server itself is up) --")
    label, url = CONTROL
    res, size = stat(url)
    lines.append("{0:<8} {1:<14} {2}".format(str(res), label, url))

    lines.append("\n-- Nucleus roots x mid-day file candidates "
                 "(rank, lower = more mid-day) --")
    for root_label, root in ROOTS:
        for rank, fname, why in sorted(CANDIDATES, key=lambda c: c[0]):
            url = root + fname
            res, size = stat(url)
            ok = str(res).endswith("OK")
            lines.append("{0:<8} r{1:<3} {2:<14} {3}{4}".format(
                str(res), rank, root_label, url,
                " ({0}b)".format(size) if ok and size else ""))
        lines.append("")

    lines.append("-- public NVIDIA CDN (HTTPS, no Nucleus dependency) --")
    for rank, fname, why in sorted(CANDIDATES, key=lambda c: c[0]):
        url = CDN_ROOT + fname
        res, size = stat(url)
        lines.append("{0:<8} r{1:<3} {2}".format(str(res), rank, url))

    lines.append("\n-- candidate rationale --")
    for rank, fname, why in sorted(CANDIDATES, key=lambda c: c[0]):
        lines.append("  r{0:<3} {1:<44} {2}".format(rank, fname, why))

    text = "\n".join(lines)
    with open(OUT, "w") as fh:
        fh.write(text + "\n")
    print(text)
    print("\n[sky_probe] wrote {0}".format(OUT))


main()
