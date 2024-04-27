#!bin/bash
colcon build
source ./install/setup.bash
cd ./asabe_2024
ros2 launch asabe_2024 arm.launch.py