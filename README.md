# Flapping Platform - 扑翼机实验平台

基于 ROS2 Humble 的扑翼机自动化实验平台。通过 RM63 机械臂搭载扑翼机模型在转台上进行扑翼飞行数据采集与运动控制实验。

## 系统架构

```
rm_driver (TCP ↔ 硬件) ──→ /arm_joint_states ──┐
                                                ├──→ joint_state_merger ──→ /joint_states
/turntable_angle ──────────────────────────────┘                            │
                                                                            ├──→ robot_state_publisher
                                                                            └──→ move_group

move_group ──→ /rm_group_controller/follow_joint_trajectory
                    │
                    v
              rm_control (三次样条插值)
                    │
                    v
              rm_driver ──→ TCP ──→ 硬件
```

## 硬件组成

- **RM63 机械臂**：6 自由度协作机械臂，末端搭载扑翼机模型
- **转台**：提供绕 Z 轴旋转自由度，配合机械臂实现 7 轴运动
- **传感器接口**：支持 STM32+MPU 真实传感器数据接入

## 软件包结构

| 包名 | 说明 |
|------|------|
| `flapping_platform_description` | URDF/XACRO 机器人模型描述，含仿真和真实硬件两个版本 |
| `flapping_platform_gazebo` | Gazebo 仿真环境、世界文件和模型 |
| `flapping_platform_moveit_config` | MoveIt2 运动规划配置，支持 Gazebo 仿真和真实硬件 |
| `flapping_platform_bringup` | 系统启动脚本和转台速度控制 GUI |
| `flapping_platform_sensor_interface` | 传感器接口节点（关节状态合并等） |
| `flapping_platform_ros2` | 元包，聚合以上所有子包 |
| `flapping_platform_system_tests` | 系统集成测试 |

### 外部依赖

| 包名 | 说明 |
|------|------|
| `ros2_rm_robot-humble` | RM 系列机械臂 ROS2 驱动（含 rm_driver、rm_control、rm_description 等） |
| `ROS-TCP-Endpoint` | Unity ↔ ROS2 的 TCP 通信桥接 |
| `experiment` | 实验数据采集（force_collector）和可视化（force_visualizer）节点 |

## 环境要求

| 组件 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| Gazebo | Ignition Fortress (gz-sim7) |
| MoveIt2 | ROS2 Humble 对应版本 |
| 编译器 | GCC 11+, CMake 3.22+ |
| Python | 3.10+ (含 tkinter) |
| 机械臂 | RM63 (睿尔曼 6 轴) |

## 快速开始

### 1. 安装依赖

```bash
# 安装 ROS2 Humble（完整桌面版）
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装 MoveIt2 和相关包
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools

# 安装 Gazebo Ignition
sudo apt install gz-sim7

# 安装 Python 依赖
pip install pyserial
```

### 2. 克隆并编译

```bash
cd ~/flapping_platform_ws/src
git clone <this-repo-url> .

# 安装依赖 rosdep
cd ~/flapping_platform_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install
source install/setup.bash
```

### 3. 仿真环境运行

```bash
# 启动 Gazebo 仿真 + MoveIt2
ros2 launch flapping_platform_moveit_config gazebo_moveit.launch.py

# 运行笛卡尔直线往复运动 Demo
ros2 run flapping_platform_moveit_config vertical_move_demo \
  --ros-args -p start_x:=-0.4 -p start_y:=0.0 -p start_z:=0.3 -p target_z:=0.7 -p num_cycles:=5
```

### 4. 真实硬件运行

```bash
# 一键启动真实机械臂控制链路
ros2 launch flapping_platform_bringup flapping_platform_bringup.launch.py

# 启动转台速度控制 GUI
ros2 run flapping_platform_bringup turntable_control.py
```

## 常用操作

### 机械臂直线往复运动

```bash
# 参数说明：
#   start_x/y/z: 起始位置 (m)
#   target_z   : 目标 Z 高度 (m)
#   num_cycles : 往复次数

# 最长行程配置（RM63 推荐: x=0.3~0.5, y=0）
ros2 run flapping_platform_moveit_config vertical_move_demo \
  --ros-args -p start_x:=0.4 -p start_y:=0.0 -p start_z:=0.3 -p target_z:=0.9 -p num_cycles:=5
```

### 实验数据采集

```bash
ros2 launch experiment data_collection.launch.py
```

### 转台速度控制

| 操作 | 按键/按钮 |
|------|-----------|
| 逆时针旋转 | ← 方向键 / "◀ 逆时针" 按钮 |
| 顺时针旋转 | → 方向键 / "顺时针 ▶" 按钮 |
| 停止 | 空格键 / "■ 停止" 按钮 |
| 微调加速 | ↑ 方向键 (+0.1 rad/s) |
| 微调减速 | ↓ 方向键 (-0.1 rad/s) |

## 仿真 vs 真实硬件

| 启动文件 | 用途 |
|----------|------|
| `flapping_platform_description/launch/robot_state_publisher_gazebo.launch.py` | Gazebo 仿真用 robot_state_publisher |
| `flapping_platform_description/launch/robot_state_publisher_real.launch.py` | 真实硬件用 robot_state_publisher |
| `flapping_platform_moveit_config/launch/gazebo_moveit.launch.py` | 仿真环境 MoveIt 启动 |
| `flapping_platform_moveit_config/launch/real_moveit.launch.py` | 真实硬件 MoveIt 启动 |
| `flapping_platform_moveit_config/launch/mock_moveit.launch.py` | 无硬件 Mock 测试 |

## 目录结构

```
flapping_platform_ws/
├── src/
│   ├── flapping_platform/               # 扑翼机平台核心包
│   │   ├── flapping_platform_bringup/         # 系统启动
│   │   ├── flapping_platform_description/     # URDF 模型
│   │   │   └── urdf/robots/                   #   - gazebo / real 两个版本
│   │   ├── flapping_platform_gazebo/          # Gazebo 仿真
│   │   │   ├── worlds/                        #   - empty.world
│   │   │   └── models/                        #   扑翼机 SDF 模型
│   │   ├── flapping_platform_moveit_config/   # MoveIt2 配置
│   │   │   ├── config/real/                   #   真实硬件参数
│   │   │   ├── config/gazebo/                 #   仿真参数
│   │   │   └── src/                           #   - vertical_move_demo.cpp
│   │   ├── flapping_platform_ros2/            # 元包
│   │   ├── flapping_platform_sensor_interface/# 传感器接口
│   │   │   └── scripts/
│   │   │       └── joint_state_merger.py      #   关节状态合并
│   │   └── flapping_platform_system_tests/    # 系统测试
│   ├── ros2_rm_robot-humble/             # RM 机械臂驱动（外部依赖）
│   ├── ROS-TCP-Endpoint/                 # Unity TCP 桥接（外部依赖）
│   └── experiment/                       # 实验采集与控制
│       ├── src/                          #   - force_collector.cpp
│       │                                 #   - experiment_control.cpp
│       ├── scripts/                      #   - force_visualizer.py
│       │                                 #   - experiment_gui.py
│       └── launch/                       #   实验启动文件
├── build/     # colcon 编译产物
├── install/   # colcon 安装产物
└── log/       # 运行日志
```
