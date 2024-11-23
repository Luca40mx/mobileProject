#!/bin/bash

# Build the container
./ros2_ws/build_container.sh

# Run the contianer
docker run -p 6000:6000/udp -p 7400-7431:7400-7431/udp -v./ros2_ws/workspace:/workspace -it mob_rob_ros2  
