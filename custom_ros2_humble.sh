#!/bin/bash

CONTAINER_NAME="custom_yahboom"

# Start X server access
xhost +

# Check if container is already running
if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    echo "Attaching to running container..."
    docker exec -it $CONTAINER_NAME bash
elif docker ps -aq -f name="$CONTAINER_NAME" | grep -q .; then
    echo "Starting existing container..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating and starting new container..."
    docker run -it --rm \
    --privileged=true \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tem/.X11-unix:/tmp/.X11-unix \
    --security-opt apparmor:unconfined \
    -v /dev/input:/dev/input \
    -v /dev/video0:/dev/video0 \
    -v /home/pi/Documents/custom:/custom \
    --name $CONTAINER_NAME \
    yahboomtechnology/ros-humble:4.1.0 /bin/bash -c "/root/1.sh & /custom/startup.sh"
fi
