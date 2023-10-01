mkdir launch
echo "Write the launch file"


cd launch
ros2 launch turtlesim_mimic_launch.py

ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"


rqt_graph


