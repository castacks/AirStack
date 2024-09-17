#!/bin/bash

container_name=$(curl -s --unix-socket /var/run/docker.sock http://localhost/containers/$HOSTNAME/json | jq -r .Name)

ROBOT_NAME=$(echo "$container_name" | sed 's#/docker-##')
ROS_DOMAIN_ID=$(echo "$ROBOT_NAME" | awk -F'-' '{print $NF}')

export ROBOT_NAME
export ROS_DOMAIN_ID