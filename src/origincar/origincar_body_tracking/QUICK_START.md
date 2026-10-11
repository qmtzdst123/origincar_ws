# OriginCar人体跟随功能包 - 快速启动指南

## 🎯 功能实现效果

你的OriginCar小车将实现以下功能：

1. **自动检测人体**：通过相机实时检测视野中的人体
2. **智能跟随**：自动选择面积最大的人体作为目标进行跟随
3. **旋转追踪**：当人体偏离小车正前方时，自动旋转保持目标在正前方
4. **距离控制**：根据人体距离自动前进或后退，保持合适的跟随距离
5. **丢失处理**：目标消失时停止运动并寻找新目标

## 📦 融合内容

本功能包已将开源的`body_tracking`追踪包完整融合并适配到你的小车：

### 核心文件
- `body_tracking.cpp/h` - 核心跟踪算法
- `robot_ctrl_node.h` - 机器人控制节点
- `param_node.h` - 参数配置节点
- `smart_subscriber.h/cpp` - 消息订阅管理
- `util.cpp/h` - 工具函数
- `time_helper.h` - 时间辅助
- `common.h` - 通用定义

### 配置文件
- `CMakeLists.txt` - ROS2编译配置
- `package.xml` - ROS2包描述
- `body_tracking.launch.py` - 启动文件

## 🚀 快速启动步骤

### 步骤1: 编译功能包

```bash
cd ~/Desktop/dev
colcon build --packages-select origincar_body_tracking
source install/setup.bash
```

### 步骤2: 启动小车底盘

```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

### 步骤3: 启动人体检测

**重要提示**：人体跟随功能需要人体检测结果！你需要根据实际相机配置启动人体检测节点。

示例（如果使用mono2d_body_detection）：

```bash
# 根据你的相机类型设置环境变量
export CAM_TYPE=mipi  # 或 usb

# 复制模型文件（如果需要）
cp -r /opt/tros/lib/mono2d_body_detection/config/ .

# 启动人体检测
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

### 步骤4: 启动人体跟随

```bash
ros2 launch origincar_body_tracking body_tracking.launch.py
```

## ⚙️ 参数调整

如果需要调整跟随性能，可以在启动文件中修改参数：

```python
parameters=[
    {"linear_velocity": 0.2},     # 前进速度（增大可更快跟随）
    {"angular_velocity": 0.4},    # 旋转速度（增大可更快转向）
    {"activate_robot_rotate_thr": 15},  # 降低阈值可更灵敏旋转
    {"track_serial_lost_num_thr": 20},  # 降低可更快寻找新目标
]
```

## 🔧 接口对接说明

### 与origincar_base的对接

本功能包完美适配你的小车底盘驱动（origincar_base）：

- **发布话题**：`/cmd_vel` (geometry_msgs/msg/Twist)
- **订阅话题**：origincar_base已订阅`/cmd_vel`并控制电机
- **兼容性**：完全兼容，无需修改origincar_base代码

### 工作流程

```
相机 → 人体检测 → 发布检测结果 → 人体跟随算法 → 发布速度指令 → origincar_base接收 → 控制电机 → 小车运动
```

## 📋 文件结构

```
origincar_body_tracking/
├── include/
│   ├── body_tracking.h          # 核心跟踪算法头文件
│   ├── common.h                 # 通用定义
│   ├── param_node.h             # 参数节点
│   ├── robot_ctrl_node.h        # 控制节点
│   ├── smart_subscriber.h       # 消息订阅
│   ├── time_helper.h            # 时间辅助
│   └── util.h                   # 工具函数
├── src/
│   ├── main.cpp                 # 主入口
│   ├── body_tracking.cpp        # 核心跟踪算法实现
│   ├── smart_subscriber.cpp     # 消息订阅实现
│   └── util.cpp                 # 工具函数实现
├── launch/
│   └ body_tracking.launch.py    # 启动文件
├── CMakeLists.txt               # 编译配置
├── package.xml                  # 包描述
├── README.md                    # 详细说明
└── QUICK_START.md               # 本文件
```

## ❗ 常见问题

### Q1: 小车不运动？

**检查清单**：
1. 人体检测是否正常运行？
   ```bash
   ros2 topic list | grep body
   # 应看到 /hobot_mono2d_body_detection
   ```

2. 跟随节点是否接收到检测结果？
   ```bash
   ros2 topic echo /hobot_mono2d_body_detection
   # 应看到人体检测结果数据
   ```

3. 小车底盘是否正常运行？
   ```bash
   ros2 topic info /cmd_vel
   # 应看到origincar_base订阅该话题
   ```

### Q2: 小车跟丢目标？

**解决方案**：
- 降低速度参数（linear_velocity和angular_velocity）
- 增加丢失阈值（track_serial_lost_num_thr）
- 确保相机视野充足

### Q3: 人体检测节点找不到？

**解决方案**：
本功能包只包含跟随算法，人体检测需要单独安装。请根据你的硬件环境安装对应的人体检测功能包。

## 🎉 成功标志

当功能正常运行时，你应该看到：

1. **日志信息**：
   ```
   Tracking body start!, track_id: xxx
   Do rotate move...
   ```

2. **小车行为**：
   - 站在小车前方，小车会自动转向你
   - 移动时小车会跟随移动
   - 离开视野时小车停止运动

## 📞 技术支持

如有问题，请检查：
1. README.md - 详细技术文档
2. 源代码注释 - 算法实现细节
3. 原开源包文档 - body_tracking原始实现

---

**祝你使用愉快！你的小车现在具备了智能人体跟随功能！** 🤖✨