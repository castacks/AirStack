#!/usr/bin/env bash
# Build the two ROS-free planner CLIs on the host, no ROS or colcon needed:
#   scripts/planner_benchmark/bin/mtl_search_plan     (mtl_search_planner adapter + vendored mtl::planner
#                                                      + vendored mtl::curve planner; planner.type picks one)
#   scripts/planner_benchmark/bin/tigris_search_plan  (tigris_search_planner core)
# Needs a C++17 compiler and Eigen 3 headers (libeigen3-dev). Usage: build_offline_tools.sh
#
# The vendored source lists are read from mtl_search_planner/cmake/mtl_vendored.cmake
# (MTL_VENDOR_PLANNER_SOURCES, MTL_CURVE_VENDOR_PLANNER_SOURCES), so this build compiles exactly
# what the ROS package compiles. -O2, no -fopenmp (MTLC_HAVE_OPENMP / MTL_HAVE_OPENMP stay
# undefined): every plan is serial and deterministic, like the stack build.
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

M="$P/mtl_search_planner"
V="$M/third_party/mtl_planner"
C="$M/third_party/mtl_curve_planner"
CMK="$M/cmake/mtl_vendored.cmake"

# Print the entries of `set(<VAR> ...)` in mtl_vendored.cmake, one per line.
cmake_list() {
  awk -v var="$1" '
    $0 ~ "^set\\(" var "([ \t]|$)" {on=1; next}
    on && /\)/ {sub(/\).*/, ""); if ($1 != "") print $1; on=0; next}
    on {for (i = 1; i <= NF; i++) print $i}
  ' "$CMK"
}
MTL_SRC=$(cmake_list MTL_VENDOR_PLANNER_SOURCES | sed "s|^|$V/|")
CURVE_SRC=$(cmake_list MTL_CURVE_VENDOR_PLANNER_SOURCES | sed "s|^|$C/|")
[ -n "$MTL_SRC" ] && [ -n "$CURVE_SRC" ] || { echo "could not read the source lists from $CMK"; exit 1; }
for f in $MTL_SRC $CURVE_SRC; do [ -f "$f" ] || { echo "missing source: $f"; exit 1; }; done
echo "[build_offline_tools] mtl_search_plan ($(echo $MTL_SRC | wc -w) mtl + $(echo $CURVE_SRC | wc -w) mtl::curve sources) ..."

# Compile each translation unit into its own object dir: mtl/ and mtl_curve/ share basenames
# (numeric.cpp, kmeans.cpp, ...), so they cannot share one flat directory.
OBJ="$HERE/bin/obj"; rm -rf "$OBJ"; mkdir -p "$OBJ/mtl" "$OBJ/curve" "$OBJ/adapter"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 2)}"
compile() {  # src out incs...
  local s="$1" o="$2"; shift 2
  $CXX $FLAGS "$@" -I"$EIGEN" -c "$s" -o "$o"
}
export -f compile; export CXX FLAGS EIGEN
{
  for s in $MTL_SRC;   do r=${s#$V/};  echo "$s|$OBJ/mtl/${r//\//__}.o|-I$V/include"; done
  for s in $CURVE_SRC; do r=${s#$C/};  echo "$s|$OBJ/curve/${r//\//__}.o|-I$C/include"; done
  for s in "$M/src/search_problem.cpp" "$M/src/mtl_search_plan_cli.cpp"; do
    echo "$s|$OBJ/adapter/$(basename "$s").o|-I$M/include -I$V/include -I$C/include"; done
} | xargs -P "$JOBS" -I{} bash -c 'IFS="|" read -r s o inc <<< "{}"; compile "$s" "$o" $inc'
$CXX $FLAGS "$OBJ"/adapter/*.o "$OBJ"/mtl/*.o "$OBJ"/curve/*.o -o "$HERE/bin/mtl_search_plan"
rm -rf "$OBJ"

T="$P/tigris_search_planner"
echo "[build_offline_tools] tigris_search_plan ..."
$CXX $FLAGS -I"$T/include" -I"$EIGEN" $(ls "$T"/src/*.cpp | grep -v _node.cpp) -o "$HERE/bin/tigris_search_plan"
echo "[build_offline_tools] done -> $HERE/bin"
