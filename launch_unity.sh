#!/bin/bash

# Build the container
./unity_ws/build_container.sh

# Setup the X server
xhost +local:

# Run the contianer
docker run -it \
    --privileged \
    --pid host \
    --net host \
    --cap-add SYS_ADMIN \
    --security-opt apparmor:unconfined \
    --device /dev/fuse:rw \
    --env "DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /opt/unity:/opt/unity \
    -v ./unity_ws/.unityhub:/opt/unityhub \
    -v ./unity_ws/.config/unityhub:/root/.config/unityhub \
    -v ./unityStuffs:/projects/unityStuffs \
    mob_rob_unity-20-04

# docker run -it \
#     --privileged \
#     --pid host \
#     --net host \
#     --cap-add SYS_ADMIN \
#     --entrypoint /bin/bash \
#     --security-opt apparmor:unconfined \
#     --device /dev/fuse:rw \
#     --env "DISPLAY=$DISPLAY" \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
#     -v /opt/unity:/opt/unity \
#     -v ./unity_ws/.unityhub:/opt/unityhub \
#     -v ./unity_ws/.config/unityhub:/root/.config/unityhub \
#     -v ./unity_ws/UnityHubNative:/root/UnityHubNative \
#     -v ./unityStuffs:/projects/unityStuffs \
#     mob_rob_unity-22-04
