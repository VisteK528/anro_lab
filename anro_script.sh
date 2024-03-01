#!/usr/bin/bash
echo "Bulding the workspace..."
colcon build
echo "Sourcing the setup.bash..."
source install/setup.bash
echo "Running the anro_turtle node from lab2 package..."
ros2 run lab2 anro_turtle
