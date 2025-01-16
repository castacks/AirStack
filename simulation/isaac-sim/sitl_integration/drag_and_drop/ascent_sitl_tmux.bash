#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/

#export BASE_PORT=5760
#export ASCENT_SITL_PORT=14552  # port to talk to QGC
#export ISAAC_SIM_PORT=14553    # port for Isaac Sim
#export AUTONOMY_STACK_PORT=14554    # port for our autonomy stack

export BASE_PORT=$1
export ASCENT_SITL_PORT=$2  # port to talk to QGC
export ISAAC_SIM_PORT=$3    # port for Isaac Sim
export AUTONOMY_STACK_PORT=$4    # port for our autonomy stack
export MAVROS_LAUNCH_PORT=$5
export SESSION_NAME=$6
export DOMAIN_ID=$7
export NAMESPACE=$8

tmuxp load -d ascent_sitl.yaml -s $SESSION_NAME
