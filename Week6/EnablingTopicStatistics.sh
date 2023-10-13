#!/bin/bash
ros2 run cpp_pubsub listener_with_topic_statistics
echo "Running the subscriber with statistics enabled node"

ros2 run cpp_pubsub talker
echo "Talker is running"

ros2 topic list
echo " List of current active topics"


echo " Statistics data published"
ros2 topic echo /statistics
