cd ../../opt/ros/humble
source setup.sh
cd /ros2_ws
colcon build --packages-select sjtu_drone_control --symlink-install
source install/setup.bash
ros2 run sjtu_drone_control trajectory_follower --ros-args -r __ns:=/drone


