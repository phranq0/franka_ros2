#!/usr/bin/bash
docker run -it --tty --device /dev/dri --user ubuntu --name $1 -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host -e DISPLAY=$DISPLAY -e XAUTHORITY wngfra/franka_ros2:latest