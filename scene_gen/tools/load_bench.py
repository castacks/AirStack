#!/usr/bin/env python3
"""Measure `airstack up` -> interactive scene, and where the time goes.

THE NUMBER THIS PRODUCES is the one `.agents/MISSIONS.md` calls the benchmark:
wall-clock from the launch command to a loaded, settled, rendered scene. Run it
from the AirStack repo root:

    python3 scene_gen/tools/load_bench.py --config urban_quake_tiny            # warm
    python3 scene_gen/tools/load_bench.py --config urban_quake_tiny --cold     # from `down`

WARM vs COLD, and why both are needed
-------------------------------------
`airstack up` on an EXISTING container does nothing at all (compose sees the
container and returns), so "time an `airstack up`" is ambiguous until you say
which of these you mean:

  cold  `airstack down isaac-sim` then `up`: container created, Kit started
        from scratch, RTX pipeline cache cold. What a colleague pays on a
        fresh clone. Measured at 133 s just to `Simulation App Startup
        Complete` on a cold shader cache versus 14 s warm, so a cold number
        is dominated by things the scene code cannot fix.
  warm  the container is up; the launcher is re-sent to the tmux pane
        (`tmux send-keys`, the documented iteration loop). Kit restarts, the
        shader cache is hot. This is the inner loop every mission iterates
        against, and the number that moves when scene code gets faster.

Report both. Gate on cold, iterate on warm.

NO INSTRUMENTATION REQUIRED
---------------------------
The phase breakdown is read out of Kit's own log, where every `print()` the
launcher makes is mirrored as a `... [N,NNNms] ... [py stdout]: ...` line stamped
against Kit's clock. So this measures the launcher as it ships — nothing to
add, nothing to keep in sync, and it works on any launch script whose prints
appear in `PHASES` below.

Host wall-clock supplies the two ends Kit cannot see: `t0` (before the launch
command) and the banner. Kit's zero point is placed on the host timeline as
`t_banner_host - kit_clock_at_banner`, which is what turns the Kit-relative
stamps into a share of the end-to-end number and exposes the pre-Kit slice
(container create, `python.sh`, interpreter startup) as a phase like any other.

OUTPUT: a phase table, `scene_gen/_bench/<stamp>.json`, and a markdown row
ready to paste into the board's timing table.
"""

import argparse
import ast
import calendar
import json
import os
import re
import subprocess
import sys
import time

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CONTAINER = "isaac-sim"
KIT_LOG_DIR = "/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"
# Mounted at $HOME/docker/isaac-sim/logs on the host — see the compose file.
PANE_LOG = "/isaac-sim/.nvidia-omniverse/logs/load_bench.log"
HOST_PANE_LOG = os.path.expanduser("~/docker/isaac-sim/logs/load_bench.log")

# Ordered phase boundaries. Each entry is (label, regex); the FIRST Kit-log
# line matching the regex ends that phase. A marker that never appears is
# reported as a gap rather than an error, so this survives a launcher that
# skips a stage (no settle, no targets) without needing to know which.
PHASES = [
    ("kit app ready",     r"\bapp ready\b"),
    ("kit startup",       r"Simulation App Startup Complete"),
    ("env + base stage",  r"\[scene_gen\] env clutter:"),
    ("base colliders",    r"\[scene_gen\] colliders:"),
    ("config compile",    r"\[compile_disaster\]"),
    ("asset pack",        r"\[scene_gen\] asset pack:"),
    ("generate + place",  r"\[scene_gen\] Applied \d+ placements"),
    # Live fracture runs AFTER the placements are applied — it operates on the
    # stage — so without its own boundary it is charged to whatever comes next.
    # It was 64 s of a 96 s `urban_quake_live` run hiding inside "colliders".
    # Absent on the archetype path, where the damage is a reference swap.
    ("live mesh damage",  r"\[mesh_damage\] .*fragments="),
    ("scene colliders",   r"\[scene_gen\] colliders:"),
    # `settle` used to be ONE phase ending at its first print, which charged it
    # with everything between the colliders and it. That is not physics: the
    # launcher pumps `app.update()` after generating, and the renderer spends
    # that pump compiling MDL materials for every new prim — 27 s of the 30 s
    # on a cold run. The settle's own summary line reports setup/cook/sim/freeze
    # and totals ~2.8 s cold. Splitting them keeps the gate honest.
    ("renderer warmup (MDL)", r"\[scene_prep\] settle: physics scene"),
    ("settle (physics)",   r"\[scene_prep\] settle: \d+ props \|"),
    ("sky + targets",     r"={10,}"),
]
BANNER = re.compile(r"(READY|COMPLETE)\s*$")
# ONLY a Python traceback. Kit logs `[Error]` constantly for missing MDL
# modules and textures on a scene that loads perfectly well, so treating
# `[Error]` as failure aborts every healthy run.
FAIL = re.compile(r"Traceback \(most recent call last\)")
# Kit log lines are `2026-08-25T03:52:55Z [5,403ms] [Info] [omni...] ...`.
# The ms field is Kit-relative and millisecond-resolution; the ISO stamp is
# absolute but only to the second, so the ms field is what the phases use.
STAMP = re.compile(r"\[([\d,]+)ms\]")
ISO = re.compile(r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)Z")


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kw)


def dexec(cmd):
    return sh(f"docker exec {CONTAINER} bash -c {json_quote(cmd)}")


def json_quote(s):
    return "'" + s.replace("'", "'\\''") + "'"


def container_up():
    r = sh(f"docker ps --format '{{{{.Names}}}}' --filter name=^{CONTAINER}$")
    return CONTAINER in r.stdout


def launch_line(script, config, extra_env):
    env = " ".join([f"SCENE_CONFIG={config}"] + extra_env)
    return (f'clear; {env} PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh '
            f'/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/{script} '
            f'--ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts')


def start_pipe():
    """Pipe the pane to a host-readable file, fresh for this run.

    `pipe-pane -o` is a no-op if a pipe is already open, so the pipe is closed
    first — otherwise a second run appends to whatever the last one left and
    the banner search finds a STALE banner and reports a 0-second load.
    """
    dexec(f"tmux pipe-pane -t isaac")            # close any existing pipe
    dexec(f": > {PANE_LOG}")
    dexec(f"tmux clear-history -t isaac")
    dexec(f"tmux pipe-pane -t isaac -o 'cat >> {PANE_LOG}'")


def stop_running_kit():
    """C-c whatever is in the pane and wait for the prompt back."""
    dexec("tmux send-keys -t isaac C-c")
    for _ in range(60):
        pane = dexec("tmux capture-pane -p -J -t isaac | grep -v '^$' | tail -1").stdout
        if pane.rstrip().endswith("#"):
            return True
        time.sleep(1)
    return False


def wait_for_banner(timeout_s, poll_s=0.5):
    """Poll the host-side pane log until the launcher's readiness banner.

    Returns (host_seconds, banner_line) or (None, reason)."""
    t0 = time.time()
    deadline = t0 + timeout_s
    seen = ""
    while time.time() < deadline:
        try:
            with open(HOST_PANE_LOG, errors="replace") as f:
                seen = f.read()
        except FileNotFoundError:
            seen = ""
        clean = re.sub(r"\x1b\[[0-9;?]*[a-zA-Z]", "", seen)
        for line in clean.splitlines():
            if BANNER.search(line.strip()) and "=" not in line:
                return time.time(), line.strip()
        if FAIL.search(clean):
            return None, "launcher failed — see " + HOST_PANE_LOG
        time.sleep(poll_s)
    return None, f"timed out after {timeout_s}s"


def _kit_log_lines():
    """Lines of the newest Kit log, or [] when it cannot be read."""
    r = dexec(f'D={json_quote(KIT_LOG_DIR)}; ls -t "$D" | head -1')
    name = r.stdout.strip()
    if not name:
        return []
    return dexec(f'cat {json_quote(KIT_LOG_DIR + "/" + name)}').stdout.splitlines()


def kit_log_phases():
    """Read the newest Kit log and pull the first stamp for each phase marker.

    Kit stamps every line with seconds since ITS start, and mirrors the
    launcher's stdout into the same file, so one pass over it dates every
    phase boundary on a single clock.
    """
    r = dexec(f'D={json_quote(KIT_LOG_DIR)}; ls -t "$D" | head -1')
    name = r.stdout.strip()
    if not name:
        return [], None, "", {"distinct_materials": 0, "rtpso_wait_s": 0,
                              "gprims": 0}
    r = dexec(f'cat {json_quote(KIT_LOG_DIR + "/" + name)}')
    lines = r.stdout.splitlines()

    # Scanning forward from the last marker found keeps the two identical
    # `colliders:` lines apart. A marker the launcher never prints must NOT
    # consume the scan position, or every later phase reads as missing too —
    # so `i` only advances on a hit.
    stats = renderer_stats(lines)
    marks, i = [], 0
    for label, pat in PHASES:
        rx = re.compile(pat)
        j = next((k for k in range(i, len(lines)) if rx.search(lines[k])), None)
        marks.append((label, None if j is None else _stamp_at(lines, j)))
        if j is not None:
            i = j + 1
    return marks, _kit_epoch(lines), name, stats


def scene_manifest(lines):
    """What the in-Kit run actually placed, for the bake-cache acceptance test.

    A host bake and an in-Kit generate must agree, and they demonstrably did
    not — a plain interpreter cannot open a Nucleus asset, footprints fall back
    to guesses, and footprints drive block sizing. Only Kit can produce the
    reference, so it is captured here from the run's own prints and committed
    as a fixture the host-runnable test compares against.
    """
    out = {}
    for line in lines:
        m = re.search(r"Placements by category: (\{[^}]*\})", line)
        if m:
            try:
                out["by_category"] = ast.literal_eval(m.group(1))
            except (ValueError, SyntaxError):
                pass
        m = re.search(r"Applied (\d+) placements under '([^']*generated)'", line)
        if m:
            out["applied"] = int(m.group(1))
    return out


def renderer_stats(lines):
    """What the renderer-warmup slice is actually made of.

    Two thirds of a cold load is the renderer compiling things, and both halves
    scale with content the scene code chooses: MDL compilation with the number
    of DISTINCT materials in the scene (not the number of prims — identical
    materials compile once), and the RtPso wait with the shader permutations
    those materials need. Reporting both makes a materials change comparable
    before and after, which a wall-clock total alone does not.
    """
    blob = "\n".join(lines)
    mats = set(re.findall(r"MaterialPool/(mat_[0-9a-f]+)", blob))
    rtpso = [int(m) for m in re.findall(
        r"RtPso async group async compilation: (\d+) seconds", blob)]
    # Colliders are applied to every gprim in the generated scene, so this line
    # is the closest thing the launcher prints to a mesh count. It is the
    # measure that ACTUALLY tracked the renderer slice: on urban_quake_tiny the
    # distinct-material count stayed at 18 across the archetype fix while
    # renderer warmup went 4.3s -> 25.8s, because what changed was ~120 new
    # fragment meshes to composite and bind, not the materials on them.
    gprims = [int(m) for m in re.findall(r"colliders: (\d+) applied", blob)]
    return {"distinct_materials": len(mats), "rtpso_wait_s": max(rtpso or [0]),
            "gprims": max(gprims or [0])}


def _stamp_at(lines, j, look_back=8):
    """Kit clock, in seconds, for the log line at *j*.

    A `print()` whose text contains a newline is logged as ONE stamped entry
    followed by bare continuation lines — `print("\n" + "=" * 70)` puts the
    rule on an unstamped line — so a marker that lands on a continuation has
    to borrow the stamp of the entry it belongs to.
    """
    for k in range(j, max(-1, j - look_back), -1):
        m = STAMP.search(lines[k])
        if m:
            return int(m.group(1).replace(",", "")) / 1000.0
    return None


def _kit_epoch(lines):
    """Host epoch seconds at which Kit's own clock reads zero.

    Every log line carries BOTH an absolute UTC stamp (1 s resolution) and a
    Kit-relative one (1 ms), so subtracting the second from the first anchors
    the whole Kit-relative timeline onto the host clock. Without this the
    pre-Kit slice can only be inferred from the last marker, which silently
    charges everything after that marker to container startup.
    """
    for line in lines:
        mi, ms = ISO.match(line), STAMP.search(line)
        if mi and ms:
            t = calendar.timegm(time.strptime(mi.group(1), "%Y-%m-%dT%H:%M:%S"))
            return t - int(ms.group(1).replace(",", "")) / 1000.0
    return None


def report(args, t0, t_ready, marks, kit_epoch, log_name, banner, stats):
    total = t_ready - t0
    # Kit's clock zero, as an offset from t0. Everything before it is container
    # start + python.sh + interpreter import, which no Kit-side instrumentation
    # can see; everything after the last marker is the play/first-frame tail.
    kit_zero = None if kit_epoch is None else kit_epoch - t0

    rows = []
    if kit_zero is not None:
        rows.append(("container + python.sh", kit_zero, kit_zero))
    prev = 0.0
    for label, s in marks:
        if s is None:
            rows.append((label + "  (not printed)", None, None))
            continue
        rows.append((label, s - prev, (kit_zero or 0) + s))
        prev = s
    if kit_zero is not None:
        rows.append(("play + first frame", total - (kit_zero + prev), total))

    w = max(len(r[0]) for r in rows)
    print("\n" + "=" * (w + 26))
    print(f"{'phase':<{w}}  {'seconds':>9}  {'cumulative':>11}")
    print("-" * (w + 26))
    for label, dt, cum in rows:
        if dt is None:
            print(f"{label:<{w}}  {'-':>9}  {'-':>11}")
        else:
            print(f"{label:<{w}}  {dt:>9.1f}  {cum:>11.1f}")
    print("-" * (w + 26))
    print(f"{'TOTAL (' + args.mode + ')':<{w}}  {total:>9.1f}  {'':>11}")
    print("=" * (w + 26))
    print(f"renderer cost: {stats['gprims']} gprims, "
          f"{stats['distinct_materials']} distinct MDL materials compiled, "
          f"{stats['rtpso_wait_s']}s RtPso shader wait")
    print(f"banner: {banner}")
    print(f"kit log: {log_name}")
    print("note: `container + python.sh` is anchored on Kit's 1 s absolute log "
          "clock, so it carries +-1 s and can read slightly negative on a warm "
          "run where the true slice is about a second.")

    stamp = time.strftime("%Y-%m-%dT%H-%M-%S")
    out_dir = os.path.join(REPO, "scene_gen", "_bench")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{args.config}-{args.mode}-{stamp}.json")
    with open(path, "w") as f:
        json.dump({
            "date": time.strftime("%Y-%m-%d %H:%M"),
            "config": args.config, "seed": args.seed, "mode": args.mode,
            "script": args.script, "env": args.env,
            "total_s": round(total, 1),
            "phases": [{"phase": l, "seconds": None if d is None else round(d, 1)}
                       for l, d, _ in rows],
            "renderer": stats,
            "kit_log": log_name, "banner": banner,
        }, f, indent=2)
    print(f"wrote {os.path.relpath(path, REPO)}")

    print("\nboard row (paste under the timing table):")
    envs = " ".join(args.env) or "-"
    print(f"| {time.strftime('%Y-%m-%d')} | {args.config} | {args.seed} | "
          f"{args.mode} | {total:.0f} | {envs} | coasei-a1 [b06dcc] |")
    return total


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="urban_quake_tiny")
    p.add_argument("--seed", default="42", help="recorded, not injected — it lives in the preset")
    p.add_argument("--script", default="scene_launch_script.py")
    p.add_argument("--cold", action="store_true",
                   help="`airstack down isaac-sim` first, so the container and the "
                        "shader cache are created from scratch")
    p.add_argument("--env", nargs="*", default=[], metavar="K=V",
                   help="extra env for the run, e.g. SCENE_ARCHETYPES=0")
    p.add_argument("--emit-manifest", default="", metavar="PATH",
                   help="also write what this in-Kit run placed, as the "
                        "reference fixture for the bake-cache acceptance test")
    p.add_argument("--timeout", type=float, default=1800)
    args = p.parse_args()
    args.mode = "cold" if args.cold else "warm"

    if args.cold:
        print(f"[bench] cold: removing the {CONTAINER} container")
        sh(f"cd {REPO} && ./airstack.sh down isaac-sim")
        env = " ".join([f"SCENE_CONFIG={args.config}",
                        f"ISAAC_SIM_SCRIPT_NAME={args.script}"] + args.env)
        t0 = time.time()
        r = sh(f"cd {REPO} && {env} ./airstack.sh up isaac-sim")
        if r.returncode:
            sys.exit(f"[bench] up failed:\n{r.stderr}")
        # The pane exists only once the container's entrypoint has made it.
        for _ in range(60):
            if dexec("tmux has-session -t isaac").returncode == 0:
                break
            time.sleep(1)
        else:
            sys.exit("[bench] tmux session never appeared")
        # AUTOLAUNCH already sent the launcher; pipe the pane and read from here.
        # Anything it printed before the pipe opened is still in the Kit log,
        # which is where the phase breakdown comes from, so nothing is lost.
        start_pipe()
    else:
        if not container_up():
            sys.exit(f"[bench] {CONTAINER} is not running — use --cold, or "
                     f"`./airstack.sh up isaac-sim` first")
        print("[bench] warm: stopping the running launcher")
        if not stop_running_kit():
            sys.exit("[bench] the pane never came back to a prompt")
        start_pipe()
        t0 = time.time()
        dexec("tmux send-keys -t isaac " + json_quote(
            launch_line(args.script, args.config, args.env)) + " ENTER")

    print(f"[bench] waiting for the readiness banner (timeout {args.timeout:.0f}s)")
    t_ready, banner = wait_for_banner(args.timeout)
    if t_ready is None:
        sys.exit(f"[bench] {banner}")

    marks, kit_epoch, log_name, stats = kit_log_phases()
    report(args, t0, t_ready, marks, kit_epoch, log_name, banner, stats)

    if args.emit_manifest:
        man = scene_manifest(_kit_log_lines())
        if not man.get("by_category"):
            sys.exit("[bench] no placement counts in the Kit log — nothing to emit")
        man.update(config=args.config, seed=args.seed, kit_log=log_name)
        os.makedirs(os.path.dirname(os.path.abspath(args.emit_manifest)),
                    exist_ok=True)
        with open(args.emit_manifest, "w") as f:
            json.dump(man, f, indent=2, sort_keys=True)
        print(f"wrote in-Kit manifest -> {args.emit_manifest}")


if __name__ == "__main__":
    main()
