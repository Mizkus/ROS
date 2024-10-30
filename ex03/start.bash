colcon build
source install/setup.bash 
gnome-terminal -- bash -c "ros2 run turtlesim turtle_teleop_key; exec bash"
ros2 launch ex03 turtle_time_travel.launch.py delay:=$1
rm -rf install log build
