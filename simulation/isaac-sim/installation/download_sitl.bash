#!/bin/bash

echo "Downloading the Ascent Aerosystems SITL package from AirLab Storage..."

ANDREWID=""
# Prompt for ANDREWID
read -p "Please enter your Andrew ID: " ANDREWID

# Check if ANDREWID is provided
if [ -z "$ANDREWID" ]; then
    log_error "Error: Andrew ID cannot be empty"
    return 1
fi

# Set ANDREWID as environment variable
rsync --progress -avz ${ANDREWID}@airlab-storage.andrew.cmu.edu:/volume1/airstack/ascent_sitl/AscentAeroSystemsSITLPackage.zip /tmp/

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

UNZIP_DIR=$SCRIPT_DIR/../sitl_integration

echo "Unzipping the SITL package to $(readlink -f $UNZIP_DIR)"

unzip /tmp/AscentAeroSystemsSITLPackage.zip -d $UNZIP_DIR

rm /tmp/AscentAeroSystemsSITLPackage.zip
