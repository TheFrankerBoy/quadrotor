cd ../../opt/ros/humble


source setup.sh


cd /ros2_ws
apt-get update
apt-get install -y python3-pip
pip3 install pandas


colcon build --packages-select sjtu_drone_control --symlink-install


source install/setup.bash


source ../../opt/ros/humble/setup.bash

		
source install/setup.sh

ros2 run sjtu_drone_control trajectory_follower --ros-args -r __ns:=/drone

