#!/bin/bash
# eva0n.sh

# 设置信号捕获
trap 'echo "正在退出..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT SIGTERM

echo "设置环境变量..."
source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch fcu_url:="/dev/ttyS0:921600" &

echo "等待节点初始化..."
sleep 5

echo "启动cartographer..."
source install/local_setup.sh
ros2 launch cartographer cartographer_imu.launch.py

# 当cartographer退出时，杀死所有后台作业
kill $(jobs -p) 2>/dev/null