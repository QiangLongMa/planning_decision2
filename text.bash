#! /bin/bash


source  /opt/ros/foxy/setup.bash
sleep 1
source install/setup.bash 



ros2 bag play /home/ring/bag/rosbag2_2024_10_22-04_10_46/rosbag2_2024_10_22-04_10_46_0.db3 
ros2 run text_global text_global_node


