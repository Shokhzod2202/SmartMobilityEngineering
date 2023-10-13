sudo apt install ros-humble-rosbag2
echo "Install rosbag2"

cd ros2_ws/src
ros2 pkg create --build-type ament_python bag_recorder_nodes_py --dependencies rclpy rosbag2_py example_interfaces std_msgs
echo "New Package is created"

ros2 run bag_recorder_nodes_py simple_bag_recorder

ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_cpp listener

ros2 bag play my_bag

ros2 run bag_recorder_nodes_py data_generator_node

ros2 bag play timed_synthetic_bag

ros2 topic echo /synthetic



