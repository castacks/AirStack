#!/usr/bin/env bash
# Run tools/wall_thickness_census.py inside the isaac-sim container on a BARE
# python — no SimulationApp, no Kit, no GPU — so it is safe beside a running
# sim.  Same three-extension bootstrap as tools/measure_fences.sh:
#   omni.usd.libs        the pxr python modules and libusd_ms.so
#   omni.usd_resolver    the `omniverse://` ArResolver plugin
#   omni.client.lib      libomniclient.so, which that plugin dlopens
# Nucleus credentials (OMNI_USER / OMNI_PASS) are already in the container env.
#
#   scene_gen/tools/wall_thickness_census.sh [container]
# Env passed through: CENSUS_REFRESH=1 (ignore the Nucleus cache),
#                     ARCH_SEED (default 4, the seed the archetype bake uses),
#                     CENSUS_OUT / CENSUS_CACHE.
set -euo pipefail
CONTAINER="${1:-isaac-sim}"
docker exec \
  -e CENSUS_REFRESH="${CENSUS_REFRESH:-}" \
  -e ARCH_SEED="${ARCH_SEED:-4}" \
  -e CENSUS_OUT="${CENSUS_OUT:-}" \
  -e CENSUS_CACHE="${CENSUS_CACHE:-}" \
  "$CONTAINER" bash -lc '
set -euo pipefail
L=$(ls -d /isaac-sim/extscache/omni.usd.libs-* | head -1)
R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-* | head -1)
C=/isaac-sim/kit/extscore/omni.client.lib
export PYTHONPATH="$L:$R:$C:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$L/bin:$R/lib:$C/bin:${LD_LIBRARY_PATH:-}"
export PXR_PLUGINPATH_NAME="$R/lib/usd/omni_usd_resolver/resources"
[ -n "${CENSUS_OUT:-}" ] || unset CENSUS_OUT
[ -n "${CENSUS_CACHE:-}" ] || unset CENSUS_CACHE
cd /isaac-sim && ./python.sh /isaac-sim/AirStack/scene_gen/tools/wall_thickness_census.py
'
