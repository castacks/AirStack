#!/usr/bin/env bash
set -euo pipefail

# Pair 1 (run in parallel)

ros2 service call /offboard_hover std_srvs/srv/Trigger {} & p1=$!

wait $p1
echo "square command sent"
