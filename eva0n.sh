#!/bin/bash
# eva0n.sh
# sudo date -s "$(date -u +%Y-%m-%d\ %H:%M:%S)"
# 设置信号捕获
trap 'echo "正在退出..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT SIGTERM

echo "设置环境变量..."
source /opt/ros/humble/setup.bash

ros2 run mavros mavros_node --ros-args \
-p fcu_url:="/dev/ttyS0:921600" \
-p time.use_system_time:=true \
-p time.timesync_mode:="off" \
-p plugin_allowlist:="['sys_status', 'sys_time', 'imu', 'param', 'command', 'local_position', 'vision_pose','setpoint_position']" \
--log-level warn&

echo "等待节点初始化..."
sleep 5

echo "启动cartographer..."
source install/local_setup.sh
ros2 launch cartographer cartographer_imu.launch.py

# echo "等待节点初始化..."
# sleep 5

# ros2 run ekf2_trans ekf2_link

# 当cartographer退出时，杀死所有后台作业
kill $(jobs -p) 2>/dev/null