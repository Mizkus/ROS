colcon build
source install/setup.bash
ros2 launch robot_depth robot_depth.launch.py
rm -rf install build log