colcon build
source install/setup.bash 
ros2 launch ex02 turtlesim_with_carrot.launch.py 
rm -rf install log build
