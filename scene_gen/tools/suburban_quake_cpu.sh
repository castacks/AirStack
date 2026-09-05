#!/usr/bin/env bash
# Run INSIDE isaac-sim. This loads USD/Nucleus libraries, not SimulationApp.
set -euo pipefail
for quake_usd in /isaac-sim/extscache/omni.usd.libs-*/; do break; done
for quake_resolver in /isaac-sim/extscache/omni.usd_resolver-*/; do break; done
for quake_yaml in /isaac-sim/extscache/omni.services.pip_archive-*/pip_prebundle; do break; done
quake_client=/isaac-sim/kit/extscore/omni.client.lib
export LD_LIBRARY_PATH="/isaac-sim/kit:${quake_usd}bin:${quake_resolver}lib:$quake_client/bin"
export PYTHONPATH="$quake_usd:$quake_client:/isaac-sim/AirStack/scene_gen:/isaac-sim/kit/kernel/py:$quake_yaml:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/isaac-sim/kit/data/Kit/Isaac-Sim Python/5.1/pip3-envs/default"
export PXR_PLUGINPATH_NAME="${quake_resolver}usd/omni_usd_resolver/resources"
exec /isaac-sim/kit/python/bin/python3 "$@"
