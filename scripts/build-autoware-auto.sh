. /opt/AutowareAuto/setup.bash

vcs import . < autoware.auto.foxy.repos
sudo apt update; rosdep update; rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select recordreplay_planner_nodes
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select autoware_demos

