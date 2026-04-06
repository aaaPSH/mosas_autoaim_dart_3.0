#!/bin/bash

sleep 5

PWD="nvidia"

# 检查并关闭已存在的接口
echo "$PWD" | sudo -S ip link set can0 down 2>/dev/null
# 设置波特率并开启 CAN 接口，同时扩展缓存长度
echo "$PWD" | sudo -S ip link set can0 type can bitrate 1000000
echo "$PWD" | sudo -S ip link set can0 txqueuelen 1000
echo "$PWD" | sudo -S ip link set can0 up

# 1. 加载 ROS 2 Humble 的底层环境变量
source /opt/ros/humble/setup.bash

# 2. 加载你自己的工作空间环境变量 (假设你的 Dart 工作空间在主目录下，请根据实际路径修改)
source /home/nvidia/mosas_autoaim_dl_dart_3.0/install/setup.bash

# 3. 执行你的 launch 文件
# 格式: ros2 launch <你的包名> <你的launch文件名>
ros2 launch mosas_bringup start.launch.py


