#!/usr/bin/env python3
"""Generate a re-run mission containing only the cells that FAILED.

A frozen sweep is 24 (environment x method) cells and a failure costs one cell,
not the run — the runner marks it failed and moves on. Rather than resubmit all
24, this emits a mission with only the named environments, so a re-run costs
~2.5 h per cell instead of days.

WHY A GENERATOR AND NOT A HAND-EDITED FILE. The environment entries carry
SPAWN_CONFIGS (eight per-robot spawn poses), FROZEN_SCENE, RESULTS_SCENE and the
per-method server flags. Retyping any of that is how a re-run silently measures
a different scene than the original. This copies the entries verbatim from the
source mission and changes nothing but the envelope.

USAGE
    python3 osmo/missions/make_rerun_mission.py \
        --failed firesuburbanl1v1_conavgpt2_team \
        --set-env ISAAC_SIM_ACTIVE_GPU=1 \
        --set-env OFFBOARD_COMPUTE_GPU=1 \
        --out osmo/missions/frozen_suburban_8robot_rerun.yaml

    # or let it read the failures straight out of a run's mission log:
    python3 osmo/missions/make_rerun_mission.py --from-log /path/to/mission.log

The `iterations` count is set to the number of cells and `environment_order`
to `round_robin`, so each named cell runs exactly once, in order
(`mission_runner.load_mission` only accepts `round_robin`/`grouped` --
`sequential` is not a valid value and crashes the runner before anything
runs).
"""
import argparse
import re
import sys
from pathlib import Path

import yaml

DEFAULT_SRC = Path(__file__).with_name("frozen_suburban_8robot.yaml")


def failures_from_log(path):
    """Environment names whose LAST attempt failed.

    A cell that failed attempt 1 and passed attempt 2 is NOT a failure — that
    is the retry working as designed, and three of the first six cells did
    exactly that. Only `iteration N failed on attempt 2/2` (or an abort with no
    later success for that iteration) counts.
    """
    text = Path(path).read_text(errors="ignore")
    env_of = {}
    cur = None
    failed_final = set()
    for ln in text.splitlines():
        # Missions range from a one-cell focused retry to the original
        # 24-cell sweep.  Pinning this to `/24` made `--from-log` silently
        # return no failures for the newer four-cell per-level missions.
        m = re.search(r"iteration (\d+)/(\d+)", ln)
        if m:
            cur = int(m.group(1))
        m = re.search(r"environment: (\S+)", ln)
        if m and cur:
            env_of[cur] = m.group(1)
        # the runner logs this only when it gives up on the iteration
        if cur and re.search(r"iteration \d+ failed on attempt 2/2", ln):
            failed_final.add(cur)
        if cur and "ERROR: iteration aborted" in ln:
            failed_final.add(cur)
        # A later successful final step means an automatic retry recovered.
        if cur and re.search(r"step \d+: OK", ln):
            failed_final.discard(cur)
    return [env_of[i] for i in sorted(failed_final) if i in env_of]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(DEFAULT_SRC))
    ap.add_argument("--failed", nargs="*", default=[],
                    help="environment names to re-run")
    ap.add_argument("--from-log", help="mission log to scrape failures from")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default=None)
    ap.add_argument(
        "--set-env", action="append", default=[], metavar="KEY=VALUE",
        help="override a top-level mission env value (repeatable; values stay strings)")
    args = ap.parse_args()

    spec = yaml.safe_load(open(args.src))
    by_name = {e["name"]: e for e in spec["environments"]}

    wanted = list(args.failed)
    if args.from_log:
        wanted += [e for e in failures_from_log(args.from_log) if e not in wanted]
    if not wanted:
        print("no failed environments given — nothing to do", file=sys.stderr)
        return 2

    missing = [w for w in wanted if w not in by_name]
    if missing:
        print(f"unknown environment(s): {missing}\n"
              f"known: {sorted(by_name)}", file=sys.stderr)
        return 1

    out = dict(spec)
    out["name"] = args.name or (spec["name"] + "_rerun")
    out["environments"] = [by_name[w] for w in wanted]
    out["iterations"] = len(wanted)
    out["environment_order"] = "round_robin"
    overrides = []
    for item in args.set_env:
        if "=" not in item or not item.split("=", 1)[0].strip():
            print(f"invalid --set-env {item!r}; expected KEY=VALUE", file=sys.stderr)
            return 2
        key, value = item.split("=", 1)
        key = key.strip()
        out.setdefault("env", {})[key] = value
        overrides.append((key, value))
    # `nas_dest` is a BASE the upload rsyncs `osmo/results/` into, which
    # already nests <mission name>/<stamp>/ itself -- appending "_rerun" here
    # made a base that was never created on the NAS (confirmed 2026-09-03,
    # same bug independently found in hand-written mission specs). The rerun's
    # own distinct `name` (set above) already gives it a separate NAS
    # subdirectory under the SAME base, so just inherit it unchanged.

    Path(args.out).write_text(
        "# GENERATED by make_rerun_mission.py — do not hand-edit.\n"
        f"# Re-runs {len(wanted)} failed cell(s) from {Path(args.src).name}:\n"
        + "".join(f"#   - {w}\n" for w in wanted)
        + "".join(f"# Pod override: {k}={v}\n" for k, v in overrides)
        + yaml.safe_dump(out, sort_keys=False, width=100)
    )
    print(f"wrote {args.out} with {len(wanted)} cell(s):")
    for w in wanted:
        print(f"  {w}  (method={by_name[w]['method']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
