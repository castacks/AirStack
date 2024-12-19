#!/bin/bash

# Description: this script launches the AscentAeroSystems SITL package
# Author: Andrew Jong (ajong@andrew.cmu.edu)
# Copyright (c) 2024. This file is developed as part of software from the AirLab at the
# Robotics Institute at Carnegie Mellon University (https://theairlab.org).

set -ex

# kill all child processes on interrupt
trap 'pkill -P $$; exit' SIGINT SIGTERM


SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ASCENT_DIR="$SCRIPT_DIR/AscentAeroSystemsSITLPackage"

ASCENT_SITL_PORT=14552  # port to talk to QGC
ISAAC_SIM_PORT=14553    # port for Isaac Sim
AUTONOMY_STACK_PORT=14554    # port for our autonomy stack


# launch QGC, SITL, and mavproxy as child processes

cd $ASCENT_DIR
# ./AscentQLinux/AscentQ & \
./spirit_sitl -S --model coaxial -I0 & \
\
mavproxy.py --streamrate=100 --master tcp:127.0.0.1:5760 \
    --out udp:127.0.0.1:$ASCENT_SITL_PORT \
    --out udp:127.0.0.1:$ISAAC_SIM_PORT \
    --out udp:127.0.0.1:$AUTONOMY_STACK_PORT

wait
