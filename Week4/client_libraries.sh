#!/bin/sh

sudo apt install python3-colcon-common-extensions
echo "Installing colcon"

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
echo "Creating the workspace"

colcon build --symlink-install
echo "Building the workspace and installing the symlink"

colcon test
echo "Running the Test colcon"

source install/setup.bash
echo "Sourcing the setup"

ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
echo "Running the subscriber demo"

ros2 run examples_rclcpp_minimal_publisher publisher_member_function
echo "Running the publisher demo"

echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "Saving the sourcing into the bashrc"


