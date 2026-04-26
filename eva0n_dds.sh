#!/bin/bash

# 设置信号捕获
trap 'echo "正在退出..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT SIGTERM

echo "设置环境变量..."
source /opt/ros/humble/setup.bash

MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600 &

echo "等待节点初始化..."
sleep 5

echo "启动cartographer..."
source install/local_setup.sh
ros2 launch cartographer cartographer_imu_dds.launch.py

echo "等待节点初始化..."
sleep 15

ros2 run ekf2_trans node_ekf2_link_dds

# 当cartographer退出时，杀死所有后台作业
kill $(jobs -p) 2>/dev/null