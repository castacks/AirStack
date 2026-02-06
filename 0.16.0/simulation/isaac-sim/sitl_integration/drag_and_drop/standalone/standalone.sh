#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/

#export BASE_PORT=5760
#export ASCENT_SITL_PORT=14552  # port to talk to QGC
#export ISAAC_SIM_PORT=14553    # port for Isaac Sim
#export AUTONOMY_STACK_PORT=14554    # port for our autonomy stack

export BASE_PORT=5760
export ASCENT_SITL_PORT=14552  # port to talk to QGC
export ISAAC_SIM_PORT=14553    # port for Isaac Sim
export AUTONOMY_STACK_PORT=14554    # port for our autonomy stack
export MAVROS_LAUNCH_PORT=14555
export SESSION_NAME=1
export DOMAIN_ID=1
export NAMESPACE=robot_1

tmuxp load standalone.yaml -s $SESSION_NAME
