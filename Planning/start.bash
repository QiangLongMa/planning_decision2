#!/bin/bash

# Source the ROS 2 setup script
source /opt/ros/foxy/setup.bash

# Source the workspace setup script
source install/setup.bash

# Run the ROS 2 node
ros2 run planning planning_main_node