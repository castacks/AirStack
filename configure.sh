#!/bin/bash

# setup

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BOLDCYAN="\e[1;96m"
YELLOW="\e[;33m"
ENDCOLOR="\e[0m"

confirm_no() { #https://stackoverflow.com/questions/3231804/in-bash-how-to-add-are-you-sure-y-n-to-any-command-or-alias
    read -r -p "${1:-Are you sure? [y/N]} " response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            true
            ;;
        *)
            false
            ;;
    esac
}


# Generate user.config.json
USER_CONFIG_JSON_SOURCE=${SCRIPT_DIR}/simulation/isaac-sim/docker/user_TEMPLATE.config.json
USER_CONFIG_JSON_DESTINATION=${SCRIPT_DIR}/simulation/isaac-sim/docker/user.config.json

echo -e "${BOLDCYAN}1. Generating Default IsaacSim Config ($USER_CONFIG_JSON_DESTINATION)${ENDCOLOR}"

if [ -d $USER_CONFIG_JSON_DESTINATION ] && [ $(ls -A $USER_CONFIG_JSON_DESTINATION | wc -l) == 0 ]; then
    # delete an empty directory with the same name as $USER_CONFIG_JSON_DESTINATION which gets created when
    # docker compose up is run before this script. Doing this will create a directory name user.config.json because
    # it is being mounted as a volume but it doesn't exist yet.
    rm -rf $USER_CONFIG_JSON_DESTINATION
fi

if [ -f $USER_CONFIG_JSON_DESTINATION ]; then
    echo -e "${YELLOW}WARNING: The file $USER_CONFIG_JSON_DESTINATION already exists.${ENDCOLOR}"
    confirm_no "Do you want to reset it to the default? [y/N]" && cp $USER_CONFIG_JSON_SOURCE $USER_CONFIG_JSON_DESTINATION
else
    cp $USER_CONFIG_JSON_SOURCE $USER_CONFIG_JSON_DESTINATION
fi


# AirLab Nucleus Login Config
OMNI_PASS_SOURCE=${SCRIPT_DIR}/simulation/isaac-sim/docker/omni_pass_TEMPLATE.env
OMNI_PASS_DESTINATION=${SCRIPT_DIR}/simulation/isaac-sim/docker/omni_pass.env

echo -e "${BOLDCYAN}2. Configure AirLab Nucleus Login ($OMNI_PASS_DESTINATION)${ENDCOLOR}"

echo "Go to https://airlab-storage.andrew.cmu.edu:8443/omni/web3/, log in, then right click on the cloud and click the \"API Tokens\" window to generate an API token and paste it here. Leave this blank if you want to skip this step: "
if [ -f $OMNI_PASS_DESTINATION ]; then
    echo -e "${YELLOW}WARNING: The file $USER_CONFIG_JSON_DESTINATION already exists, leave it blank to skip.${ENDCOLOR}"
fi
read -r -p "API Token: " API_TOKEN

if [ ! -z "${API_TOKEN}" ]; then
    sed "s/PASTE-YOUR-API-TOKEN/$API_TOKEN/g" $OMNI_PASS_SOURCE > $OMNI_PASS_DESTINATION
fi
