#!/bin/bash

XAUTH=/tmp/.docker.xauth
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
	echo $xauth_list | xauth -f $XAUTH nmerge -
    else
	touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
       -e "PRIVACY_CONSENT=Y" \
       -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
       -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
       -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
       -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
       -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
       -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
       -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
       -v ~/docker/isaac-sim/documents:/root/Documents:rw \
       -v $SCRIPT_DIR/extras/kit-app-template/source/extensions/:/root/Documents/Kit/shared/exts/ \
       -v $SCRIPT_DIR/extras:/extras:rw \
       -v $SCRIPT_DIR/extras/.bashrc:/root/.bashrc \
       -v $SCRIPT_DIR/extras/.bash_history:/root/.bash_history \
       -v $SCRIPT_DIR/extras/inputrc:/etc/inputrc \
       --env "ACCEPT_EULA=Y" \
       --env "PRIVACY_CONSENT=Y" \
       --env="DISPLAY=$DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --env="XAUTHORITY=$XAUTH" \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       -v $XAUTH:$XAUTH \
       --cap-add=SYS_PTRACE \
       core_isaac_sim_4:latest
       #nvcr.io/nvidia/isaac-sim:4.0.0
