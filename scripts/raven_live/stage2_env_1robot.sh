#!/usr/bin/env bash
# stage2_env_1robot.sh — print the exact `edit_env.py --set ...` invocation for
# the ONE-drone Stage-2 flight (robot_1 only, 20 m from the fully-exposed
# casualty). Nothing is applied.
#
#   scripts/raven_live/stage2_env_1robot.sh          # rationale + command
#   scripts/raven_live/stage2_env_1robot.sh --cmd    # just the command
#   scripts/raven_live/stage2_env_1robot.sh --cmd | bash
#
# Needs scripts/raven_live/out/spawns_1robot.json — written by
# scripts/raven_live/validate_freeze.sh.
set -euo pipefail
NROBOTS=1 exec bash "$(dirname "$0")/_stage2_env.sh" "$@"
