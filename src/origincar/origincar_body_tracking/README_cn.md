# OriginCar - 智能循迹避障小车

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![平台](https://img.shields.io/badge/平台-地平线%20RDK-orange)](https://developer.horizon.ai)
[![许可证](https://img.shields.io/badge/许可证-Apache%202.0-green)](LICENSE)

基于ROS2框架与AI视觉技术的智能自主循迹避障小车，具备线路跟随、障碍物避障、人体追踪、二维码识别等功能。

## 功能特性

- **智能循迹**：基于YOLOv5的视觉线路检测，结合PID控制算法实现精准跟随
- **自主避障**：实时锥桶检测与智能避障策略
- **人体跟随**：基于地平线Mono2d人体检测的追踪功能
- **二维码识别**：ZBar二维码解码，触发任务指令
- **AI图像分析**：集成豆包视觉大模型的高级图像理解


## 软件模块


`origincar_base`底盘控制节点，串口通信，里程计与IMU发布
`origincar_competition`竞赛控制节点，循迹与避障决策
`origincar_body_tracking`人体跟随节点，视觉追踪策略
`origincar_description`URDF机器人模型，用于可视化
`origincar_msg`自定义消息定义
`qr_decoder`二维码识别节点
`image_upload_analyzer`大模型图像分析节点

## 安装说明

### 环境要求

- ROS2 Humble
- 地平线RDK开发板（已安装TROS）
- Python 3.8+

### 依赖安装

```bash
# 安装Python依赖
pip install pyzbar
pip install volcengine-python-sdk[ark]

# 安装TROS功能包（RDK开发板）
sudo apt install tros-mono2d-body-detection
```

### 编译

```bash
cd ~/Desktop/dev_ws
colcon build
source install/setup.bash
```

## 快速启动

### 人体跟随（一键启动）

```bash
cd ~/Desktop/dev_ws
chmod +x start_body_tracking.sh
./start_body_tracking.sh


### 人体跟随（分步启动）

### 循迹避障

```bash
# 步骤1：启动底盘节点
ros2 launch origincar_base origincar_bringup.launch.py

# 步骤2：启动视觉处理
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo

# 步骤3：启动竞赛控制（已包含相机与二维码识别）
ros2 launch origincar_competition start.launch.py

# 步骤4：启动上位机界面（可选）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 人体跟随（分步启动）

```bash
# 步骤1：启动底盘节点
ros2 launch origincar_base origincar_bringup.launch.py

# 步骤2：设置相机类型
export CAM_TYPE=mipi  # USB相机改为：export CAM_TYPE=usb

# 步骤3：复制模型配置（如需要）
cp -r /opt/tros/lib/mono2d_body_detection/config/ .

# 步骤4：启动人体检测
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py

# 步骤5：启动人体跟随
ros2 launch origincar_body_tracking body_tracking.launch.py
```

### 二维码识别

```bash
ros2 run origincar_qr li4
```

### 大模型图像分析

```bash
ros2 run image_upload_analyzer image_upload_analyzer
```

## ROS2话题

/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |`/odom` | `nav_msgs/Odometry` | 里程计数据 |
/imu/data_raw` | `sensor_msgs/Imu` | IMU原始数据 |
origincar_competition` | `ai_msgs/PerceptionTargets` | YOLO检测结果 |
/hobot_mono2d_body_detection` | `ai_msgs/PerceptionTargets` | 人体检测结果 |

## 配置说明

### YOLO模型配置

编辑 `config/yolov5sconfig.json`：
```json
{
    "model_file": "<模型文件路径>",
    "class_num": 2,
    "cls_names_list": "config/origincar.list"
}
```

### 人体跟随参数

`origincar_body_tracking` 关键参数：
- `activate_robot_rotate_thr`：激活旋转阈值
- `move_step`：移动步长
- `track_serial_lost_num_thr`：连续丢失帧数阈值

## 常见问题

**Q：小车不运动？**
- 检查人体检测是否启动：`ros2 topic list | grep body`
- 应看到话题：`/hobot_mono2d_body_detection`

**Q：找不到串口？**
- 确认连接：`ls /dev/ttyACM*`
- 检查权限：`sudo chmod 666 /dev/ttyACM0`

**Q：YOLO检测不工作？**
- 检查配置文件中的模型路径
- 确认相机流：`ros2 topic hz /image`

**Q：二维码无法识别？**
- 确保已安装 `pyzbar`
- 检查图像质量和光照条件

## 启动顺序说明

**人体跟随必须按顺序启动**：
底盘 → 人体检测 → 人体跟随

人体跟随功能依赖人体检测结果，如果没有人体检测节点，小车不会运动！

## 验证启动成功

启动后检查话题：
```bash
ros2 topic list
```

应看到以下话题：
- `/cmd_vel` - 小车控制
- `/hobot_mono2d_body_detection` - 人体检测结果

站在小车前方，小车应该：
1. 自动转向你
2. 跟随你移动
3. 离开视野时停止

## 许可证

Apache License 2.0

## 致谢

- [地平线机器人](https://developer.horizon.ai) - TROS框架与BPU加速
- [ROS2](https://docs.ros.org) - 机器人操作系统
- [YOLOv5](https://github.com/ultralytics/yolov5) - 目标检测模型
- [ZBar](http://zbar.sourceforge.net) - 二维码解码器

