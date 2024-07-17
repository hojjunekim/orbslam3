#!/bin/bash
xhost +local:root # enable GUI use
IMAGE="orbslam3-ros2:latest"
NAME="orbslam3-ros2-container"
docker run --name $NAME -it --rm \
-e "ACCEPT_EULA=Y" --privileged \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
--net host \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $HOME/ros2_ws:/home/ros2_ws \
$IMAGE