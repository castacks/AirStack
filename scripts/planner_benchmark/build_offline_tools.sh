#!/usr/bin/env bash
# Build the two ROS-free planner CLIs on the host, no ROS or colcon needed:
#   scripts/planner_benchmark/bin/mtl_search_plan     (mtl_search_planner adapter + vendored mtl::planner)
#   scripts/planner_benchmark/bin/tigris_search_plan  (tigris_search_planner core)
# Needs a C++17 compiler and Eigen 3 headers (libeigen3-dev). Usage: build_offline_tools.sh [-j N]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
P="$REPO/robot/ros_ws/src/global/planners"
CXX="${CXX:-g++}"
EIGEN="${EIGEN_INCLUDE:-$(pkg-config --cflags-only-I eigen3 2>/dev/null | sed 's/-I//g' | awk '{print $1}' || true)}"
[ -z "$EIGEN" ] && EIGEN=/usr/include/eigen3
[ -d "$EIGEN/Eigen" ] || { echo "Eigen headers not found (set EIGEN_INCLUDE or install libeigen3-dev)"; exit 1; }
mkdir -p "$HERE/bin"
FLAGS="-O2 -std=c++17 -pthread -DNDEBUG"

M="$P/mtl_search_planner"; V="$M/third_party/mtl_planner"
MTL_SRC=$(ls "$V"/src/*.cpp "$V"/src/core/*.cpp "$V"/src/mapping/*.cpp "$V"/src/routing/*.cpp \
             "$V"/src/planning/*.cpp "$V"/src/trajectory/*.cpp "$V"/src/sensing/*.cpp)
echo "[build_offline_tools] mtl_search_plan ..."
$CXX $FLAGS -I"$M/include" -I"$V/include" -I"$EIGEN" "$M/src/search_problem.cpp" "$M/src/mtl_search_plan_cli.cpp" \
    $MTL_SRC -o "$HERE/bin/mtl_search_plan"

T="$P/tigris_search_planner"
echo "[build_offline_tools] tigris_search_plan ..."
$CXX $FLAGS -I"$T/include" -I"$EIGEN" $(ls "$T"/src/*.cpp | grep -v _node.cpp) -o "$HERE/bin/tigris_search_plan"
echo "[build_offline_tools] done -> $HERE/bin"
