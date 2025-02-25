#! /bin/bash


source  /opt/ros/foxy/setup.bash
sleep 1
source install/setup.bash 

# 使用 xterm 启动 ros2 节点，并保持终端窗口开启
xterm -hold -e "ros2 run text_global text_global_node" &

ros2 bag play /home/ring/bag/rosbag2_2024_10_22-04_10_46/rosbag2_2024_10_22-04_10_46_0.db3 

# 退出时关闭所有终端窗口和相关进程
wait
#Function to kill processes
kill_ros2_processes() {
    pid1=$(pgrep -f text_global_node)
    if [ -n "$pid1" ]; then
        echo "Killing text_global_node process with PID $pid1"
        kill -15 $pid1
    else
        echo "text_global_node process not found."
    fi
}
# Call the function to kill processes
trap 'trap - SIGTERM; kill_ros2_processes; kill -- -$$' SIGINT SIGTERM EXIT
# End of file



