## Introduction
This is a README file to set up AirStack for RAVEN. AirStack serves as a core aerial autonomy stack of RAVEN, which handles planning, control, interface with simulation, etc. For more comprehensive information about the AirStack, please go to the <a href="https://github.com/castacks/AirStack">main branch</a> and <a href="https://docs.theairlab.org/main/docs/getting_started/">documentation</a>.

## Requirements

AirStack was tested on Ubuntu 22.04. A GPU of GeForce RTX 4080 or higher is recommended for the best performance.

## Setup

    cd ~/RAVEN/AirStack
    ./airstack.sh install
    ./airstack.sh setup

## Build Docker Images

    cd ~/RAVEN/AirStack
    docker compose build 