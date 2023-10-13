#!/bin/bash

fastdds discovery --server-id 0
echo "Discovery Serers are launched"

export ROS_DISCOVERY_SERVER=127.0.0.1:11811
echo "Environment Variables are set"

ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
echo "The listener node is launched"

export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
echo "Talker node is launched"

ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener
echo "Discovery server execution"

ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker
echo "check for verification is done."

fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888
echo "establish a communication with redundant servers."



