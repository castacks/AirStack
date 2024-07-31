#!/bin/bash

#docker build --no-cache --network=host -t core_isaac_sim_4 -f dockerfiles/core_isaac_sim_4.dockerfile .
docker build --network=host -t core_isaac_sim_4 -f dockerfiles/core_isaac_sim_4.dockerfile .
