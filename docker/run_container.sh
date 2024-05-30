#!/bin/bash
xhost +
IMAGE="orbslam3-ros2:latest"
NAME="orbslam3-ros2-container"
docker run --name $NAME -it --rm \
-e "ACCEPT_EULA=Y" --privileged --network=host \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
$IMAGE