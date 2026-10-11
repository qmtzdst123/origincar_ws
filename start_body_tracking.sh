#!/bin/bash
# OriginCar人体跟随功能一键启动脚本

echo "========================================="
echo "OriginCar人体跟随功能启动"
echo "========================================="

# 设置环境变量（如果是USB相机请改为：export CAM_TYPE=usb）
export CAM_TYPE=mipi

# 检查是否已编译人体跟随包
if [ ! -d "install/origincar_body_tracking" ]; then
    echo "首次使用，正在编译人体跟随功能包..."
    colcon build --packages-select origincar_body_tracking
fi

# 加载工作空间环境
source install/setup.bash

# 启动小车底盘（后台运行）
echo "1. 启动小车底盘..."
ros2 launch origincar_base origincar_bringup.launch.py &
sleep 2

# 检查并准备人体检测模型配置
echo "2. 准备人体检测环境..."
if [ -d "/opt/tros/lib/mono2d_body_detection/config" ]; then
    cp -r /opt/tros/lib/mono2d_body_detection/config/* ./config/ 2>/dev/null || cp -r /opt/tros/lib/mono2d_body_detection/config/ .
elif [ -d "/opt/tros/humble/lib/mono2d_body_detection/config" ]; then
    cp -r /opt/tros/humble/lib/mono2d_body_detection/config/* ./config/ 2>/dev/null || cp -r /opt/tros/humble/lib/mono2d_body_detection/config/ .
fi

# 加载TROS环境
if [ -f "/opt/tros/setup.bash" ]; then
    source /opt/tros/setup.bash
elif [ -f "/opt/tros/humble/setup.bash" ]; then
    source /opt/tros/humble/setup.bash
fi

# 启动人体检测（后台运行）
echo "3. 启动人体检测节点..."
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py &
sleep 3

# 启动人体跟随
echo "4. 启动人体跟随控制策略..."
source install/setup.bash
ros2 launch origincar_body_tracking body_tracking.launch.py

echo "========================================="
echo "人体跟随功能已退出"
echo "========================================="
