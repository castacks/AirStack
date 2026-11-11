#!/usr/bin/env bash
set -euo pipefail

# Pair 1 (run in parallel)

ros2 service call /turn_square std_srvs/srv/Trigger {} & p1=$!

wait $p1
echo "square command sent"
