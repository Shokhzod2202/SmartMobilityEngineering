#!/bin/sh

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "Source is added to your shell startup script"

echo "Your environmental variables:"
printenv | grep -i ROS

export ROS_DOMAIN_ID=2
echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
echo "The ROS_DOMAIN_ID is set and saved to the bashrc file"

#Using turtlesim, ros2, and rqt

sudo apt update

sudo apt install ros-humble-turtlesim
echo "List of executable turtlesim:"
ros2 pkg executables turtlesim

ros2 run turtlesim turtlesim_node
echo "turtlesim_node is started"

ros2 run turtlesim turtle_teleop_key
echo "turtle_teleop_key node is started"

ros2 node list
ros2 topic list
ros2 service list
ros2 action list

sudo apt update

sudo apt install ~nros-humble-rqt*
echo "rqt is installed"

rqt
echo "rqt is running"

ros2 node info /my_turtle
echo "List of subscibers are printed"

ros2 topic list
echo "List of all topics currently active"

ros2 topic list -t
echo "List of topics with topic type"

ros2 service type /clear
echo "Showing the type of the service"

ros2 service list -t
echo "Printing the list of services with type info"

ros2 service find std_srvs/srv/Empty
echo "Finding all the Empty type services like //clear, //rest"


