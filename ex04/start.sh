colcon build
source install/setup.bash
ros2 launch robot_lidar robot_lidar.launch.py
rm -rf install build log