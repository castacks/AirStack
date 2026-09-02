#!/usr/bin/env bash
# stage2_env_2robot.sh — print the exact `edit_env.py --set ...` invocation for
# the TWO-drone Stage-2 flight (robot_1 20 m from the fully-exposed casualty,
# robot_2 20 m from the partially-covered one). Nothing is applied.
#
#   scripts/raven_live/stage2_env_2robot.sh          # rationale + command
#   scripts/raven_live/stage2_env_2robot.sh --cmd    # just the command
#   scripts/raven_live/stage2_env_2robot.sh --cmd | bash
#
# Needs scripts/raven_live/out/spawns_2robot.json — written by
# scripts/raven_live/validate_freeze.sh.
set -euo pipefail
NROBOTS=2 exec bash "$(dirname "$0")/_stage2_env.sh" "$@"
