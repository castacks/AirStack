#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/AscentAeroSystemsSITLPackage

export ASCENT_SITL_PORT=14552  # port to talk to QGC
export ISAAC_SIM_PORT=14553    # port for Isaac Sim
export AUTONOMY_STACK_PORT=14554    # port for our autonomy stack

tmuxp load -d ascent_sitl.yaml
