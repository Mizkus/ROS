colcon build
source install/setup.bash
ros2 launch robot_bringup robot_lidar.launch.py
rm -rf install build log