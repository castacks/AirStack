#!/bin/bash

SDKFile="NatNetSDK.tar"

# Check if sdk was already downloaded
#if [  -d "deps/NatNetSDK" ] 
if [[ -d "deps/NatNetSDK/lib" ]] && [[ -d "deps/NatNetSDK/include" ]]; then
        echo "SDK already installed."
        exit 1
fi

# Check for architecture
ARCH=$(uname -m)
if [[ "$ARCH" == "x86_64" ]]; then
        echo "System architecture is AMD64."
        SDK_URL="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu/NatNet_SDK_4.4_ubuntu.tar"
elif [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm"* ]]; then
        echo "System architecture is ARM."
        SDK_URL="https://d2mzlempwep3hb.cloudfront.net/NatNetSDKLinux/ubuntu_arm/NatNet_SDK_4.4_ubuntu_ARM.tar"
else
        echo "Unsupported architecture: $ARCH"
        exit 1
fi

# Download sources.
wget ${SDK_URL} -O ${SDKFile}

# Uncompressing
tar xf ${SDKFile} -C deps/NatNetSDK/ && rm -rf ${SDKFile}


