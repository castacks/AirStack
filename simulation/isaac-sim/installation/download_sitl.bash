#!/bin/bash

echo "Installing gdown package"

pip install gdown

echo "Downloading the SITL package from Google Drive"

gdown https://drive.google.com/uc\?id\=1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

UNZIP_DIR=$SCRIPT_DIR/../sitl_integration

echo "Unzipping the SITL package to $(readlink -f $UNZIP_DIR)"

unzip AscentAeroSystemsSITLPackage.zip -d $UNZIP_DIR

rm AscentAeroSystemsSITLPackage.zip
