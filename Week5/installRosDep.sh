pip install rosdep
echo "ROS dep is installed"

sudo rosdep init
rosdep update

rosdep install --from-paths src -y --ignore-src

rosdep install --from-paths src -y --ignore-src


