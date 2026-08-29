cd /isaac-sim/AirStack
U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-*/ | head -1)
C=/isaac-sim/kit/extscore/omni.client.lib
NP=/isaac-sim/extscache/omni.kit.pip_archive-0.0.0+69cbf6ad.lx64.cp311/pip_prebundle
TM=/isaac-sim/exts/omni.pip.compute/pip_prebundle
SP=/isaac-sim/kit/python/lib/python3.11/site-packages
export LD_LIBRARY_PATH="/isaac-sim/kit:${U}bin:${R}lib:$C/bin"
export PYTHONPATH="$U:$C:$NP:$TM:$SP"
export PXR_PLUGINPATH_NAME="${R}usd/omni_usd_resolver/resources"
exec /isaac-sim/kit/python/bin/python3 "$@"
