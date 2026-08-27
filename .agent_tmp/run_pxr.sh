#!/bin/bash
U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-*/ | head -1)
C=/isaac-sim/kit/extscore/omni.client.lib
export LD_LIBRARY_PATH="/isaac-sim/kit:${U}bin:${R}lib:$C/bin"
export PYTHONPATH="$U:$C"
export PXR_PLUGINPATH_NAME="${R}usd/omni_usd_resolver/resources"
exec /isaac-sim/kit/python/bin/python3 /isaac-sim/AirStack/.agent_tmp/nmeasure.py "$@"
