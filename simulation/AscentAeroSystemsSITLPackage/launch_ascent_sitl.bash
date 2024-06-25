#!/bin/bash
set -ex

# kill all child processes on interrupt
trap 'pkill -P $$; exit' SIGINT SIGTERM

ASCENT_SITL_PORT=14552  # port to talk to QGC
ISAAC_SIM_PORT=14553    # port for our autonomy stack

# launch QGC, SITL, and mavproxy
./AscentQLinux/AscentQ & \
./spirit_sitl -S --model coaxial -I0 & \
\
mavproxy.py --streamrate=100 --master tcp:127.0.0.1:5760 \
    --out udp:127.0.0.1:$ASCENT_SITL_PORT \
    --out udp:127.0.0.1:$ISAAC_SIM_PORT & \

wait