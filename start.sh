#! /bin/bash

#! /bin/bash

pid5=$(pgrep -f test1)
if [ -n "$pid5" ]; then
    echo "Killing test1 process with PID $pid5"
    kill -15 $pid5
else
    echo "test1 process not found."
fi

pid6=$(pgrep -f can_node)
if [ -n "$pid6" ]; then
    echo "Killing can_node process with PID $pid6"
    kill -15 $pid6
else
    echo "can_node process not found."
fi

pid7=$(pgrep -f real_time_plot)
if [ -n "$pid7" ]; then
    echo "Killing real_time_plot process with PID $pid7"
    kill -15 $pid7
else
    echo "real_time_plot process not found."
fi

source  /opt/ros/foxy/setup.bash
sleep 1
source install/setup.bash 

# 获取当前工作路径
current_path=$(pwd)
# 打印当前工作路径
echo "当前工作路径是: $current_path"
# 检查 output_txt 文件夹是否存在，如果不存在则创建它
output_dir="${current_path}/output_txt"
if [ ! -d "$output_dir" ]; then
  echo "output_txt 文件夹不存在，正在创建..."
  mkdir -p "$output_dir"
else
  echo "output_txt 文件夹已存在，忽略创建。"
fi
# 获取当前日期和时间，格式为 yyyy-mm-dd_HH-MM-SS
log_filename="${current_path}/output_txt/$(date +"%Y-%m-%d_%H-%M-%S").txt"
# 打印生成的日志文件名
echo "日志文件名是: $log_filename"
# 运行第一个命令并将日志输出到动态命名的文件
ros2 run hmi hmi -> "$log_filename" &
PID1=$!

rviz2 -d ${current_path}/src/myrviz/default.rviz >/dev/null 2>&1  &
PID2=$!

#ros2 bag play /home/ring/bag/rosbag2_2024_10_22-04_10_46/rosbag2_2024_10_22-04_10_46_0.db3 --loop


# Optionally, you can wait for both commands to finish
wait
#Function to kill processes
kill_ros2_processes() {
    echo "Killing ROS 2 processes..."
    # Kill all ros2 related processes
    kill -15 $(ps aux | grep '[r]os2' | awk '{print $2}')

    pid=$(pgrep -f Lidar_Processing_node)
    if [ -n "$pid" ]; then
        echo "Killing Lidar_Processing_node process with PID $pid"
        kill -15 $pid
    else
        echo "Lidar_Processing_node process not found."
    fi
    pid2=$(pgrep -f points_and_lines_publisher)
    if [ -n "$pid2" ]; then
        echo "Killing points_and_lines_publisher process with PID $pid2"
        kill -15 $pid2
    else
        echo "points_and_lines_publisher process not found."
    fi

    pid3=$(pgrep -f can_node)
    if [ -n "$pid3" ]; then
        echo "Killing can_node process with PID $pid3"
        kill -15 $pid3
    else
        echo "can_node process not found."
    fi

    pid4=$(pgrep -f gnss_node)
    if [ -n "$pid4" ]; then
        echo "Killing gnss_node process with PID $pid4"
        kill -15 $pid4
    else
        echo "gnss_node process not found."
    fi

    pid5=$(pgrep -f gnss_sync_node)
    if [ -n "$pid5" ]; then
        echo "Killing gnss_sync_node process with PID $pid5"
        kill -15 $pid5
    else
        echo "gnss_sync_node process not found."
    fi

    pid8=$(pgrep -f real_time_plot)
    if [ -n "$pid8" ]; then
        echo "Killing real_time_plot process with PID $pid8"
        kill -15 $pid8
    else
        echo "real_time_plot process not found."
    fi

    pidlocal=$(pgrep -f planning_main_node)
    if [ -n "$pidlocal" ]; then
        echo "Killing planning_main_node process with PID $pidlocal"
        kill -15 $pidlocal
    else
        echo "local_node4 process not found."
    fi




}


# Trap function to ensure processes are killed on script exit
trap "kill_ros2_processes" EXIT
echo "Both commands started successfully. Waiting for processes to finish..."
sleep 1
echo "Exiting script."


# # 启动hmi
# ros2 run hmi hmi

# # 启动global节点
# ros2 run global global_node

# # 播放gps录制包，文件位于/home/ring/bag/rosbag2_2024_10_22-04_10_46/rosbag2_2024_10_22-04_10_46_0.db3
# ros2 bag play /home/ring/bag/rosbag2_2024_10_22-04_10_46/rosbag2_2024_10_22-04_10_46_0.db3 --loop





