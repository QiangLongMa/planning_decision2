#! /usr/bin/env bash

# 查找正在运行的 planning_main_node 进程
PID=$(pgrep -f planning_main_node)

if [ -z "$PID" ]; then
    echo "Error: planning_main_node process not found!"
    exit 1
fi

echo "Found planning_main_node PID: $PID"

# 把 PID 导出到环境变量 ROS2_NODE_PID
export ROS2_NODE_PID=$PID

# 也可将 PID 写入临时文件，供后续读取
echo "$PID" > /tmp/planning_main_node.pid
echo "ROS2_NODE_PID=$PID"