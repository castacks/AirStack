#!/usr/bin/env bash
# Run tools/measure_fences.py inside the isaac-sim container on a BARE python —
# no SimulationApp, no Kit, no GPU, so it is safe to run beside a sim that is
# already up (which is the normal state of this machine).
#
# `python.sh` alone has no `pxr`: Isaac ships USD inside extensions and relies
# on the SimulationApp bootstrap to put them on the path. The three extensions
# below are all that a bbox off Nucleus needs —
#   omni.usd.libs        the pxr python modules and libusd_ms.so
#   omni.usd_resolver    the `omniverse://` ArResolver plugin
#   omni.client.lib      libomniclient.so, which that plugin dlopens
# — and the Nucleus credentials are already in the container's environment
# (OMNI_USER / OMNI_PASS), so nothing has to be logged in interactively.
#
#   scene_gen/tools/measure_fences.sh [container]     # default: isaac-sim
set -euo pipefail
CONTAINER="${1:-isaac-sim}"
docker exec "$CONTAINER" bash -lc '
set -euo pipefail
L=$(ls -d /isaac-sim/extscache/omni.usd.libs-* | head -1)
R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-* | head -1)
C=/isaac-sim/kit/extscore/omni.client.lib
export PYTHONPATH="$L:$R:$C:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$L/bin:$R/lib:$C/bin:${LD_LIBRARY_PATH:-}"
export PXR_PLUGINPATH_NAME="$R/lib/usd/omni_usd_resolver/resources"
cd /isaac-sim && ./python.sh /isaac-sim/AirStack/scene_gen/tools/measure_fences.py
'
