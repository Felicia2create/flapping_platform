# ROS2 + MoveIt2 + ros2_control 控制链路完整解析

> 本文以 **扑翼机平台 (flapping_platform)** 项目为例，涵盖 Gazebo 仿真、mock_components 虚拟、睿尔曼真机三种控制模式，从 URDF 配置到 Launch 文件编写，逐一拆解。

---

## 目录

1. [核心概念速览](#1-核心概念速览)
2. [配置文件分层](#2-配置文件分层)
3. [三种控制链路详解](#3-三种控制链路详解)
4. [Launch 文件编写模式](#4-launch-文件编写模式)
5. [关键参数详解](#5-关键参数详解)
6. [实战：从零搭建新机器人配置](#6-实战从零搭建新机器人配置)
7. [调试与排查](#7-调试与排查)

---

## 1. 核心概念速览

### 1.1 控制链路的三大组件

```
┌────────────────────────────────────────────────────────────┐
│                      MoveIt2 (规划层)                        │
│  move_group: 运动规划、碰撞检测、轨迹生成                      │
│  SRDF: 规划组定义、碰撞矩阵、被动关节                          │
│  kinematics.yaml: 逆运动学求解器配置                           │
└──────────────────────┬─────────────────────────────────────┘
                       │ FollowJointTrajectory Action
                       ▼
┌────────────────────────────────────────────────────────────┐
│                  ros2_control (控制层)                        │
│  controller_manager: 加载和管理控制器生命周期                  │
│  JointTrajectoryController: 接收轨迹→插值→发送指令             │
│  JointStateBroadcaster: 读取硬件状态→发布 /joint_states        │
│  hardware_plugin: 与真实/虚拟硬件通信                          │
└──────────────────────┬─────────────────────────────────────┘
                       │ command_interface
                       ▼
┌────────────────────────────────────────────────────────────┐
│                   Hardware (硬件层)                           │
│  Gazebo: gz_ros2_control/GazeboSimSystem → 物理引擎           │
│  mock: mock_components/GenericSystem → 零延迟镜像              │
│  Real: rm_driver → TCP/SDK → 睿尔曼机械臂                      │
└────────────────────────────────────────────────────────────┘
```

### 1.2 数据流核心：`/joint_states` 话题

所有模式都围绕 `sensor_msgs/msg/JointState` 消息展开：

```
/joint_states (JointState)
  ├── name: ["joint1", "joint2", ..., "plate_joint"]   ← 关节名称（字符串匹配）
  ├── position: [0.0, 0.0, ...]                         ← 弧度
  └── velocity: [0.0, 0.0, ...]                         ← 弧度/秒
```

**两个消费者**：
- **robot_state_publisher** → 订阅 `/joint_states` → 发布 `/tf`（TF 树）
- **move_group** → 订阅 `/joint_states` → 更新内部机器人状态 → 用于规划

---

## 2. 配置文件分层

### 2.1 配置全景图

```
你的项目
├── URDF (urdf/robots/*.urdf.xacro)          ← 机器人几何 + ros2_control 标签
│   ├── urdf/mech/*.urdf.xacro               ← 机械臂/平台/关节宏
│   └── urdf/control/*.ros2_control.xacro    ← 硬件插件 + 关节接口定义
│
├── MoveIt 配置 (config/)
│   ├── *.srdf                                ← 规划组 + 碰撞对 + 被动关节
│   ├── kinematics.yaml                       ← IK 求解器（KDL/TRAC-IK）
│   ├── joint_limits.yaml                     ← 关节限位
│   ├── moveit_controllers.yaml               ← MoveIt → ros2_control 的桥梁
│   ├── ros2_controllers.yaml                 ← ros2_control 控制器参数
│   └── initial_positions.yaml                ← 初始位置（仅虚拟/仿真用）
│
└── Launch 文件 (launch/*.launch.py)
    ├── demo_xxx.launch.py                    ← 自包含演示（虚拟/仿真）
    ├── real_xxx.launch.py                    ← 真机精简版（move_group + rviz）
    └── xxx_bringup.launch.py                 ← 整合启动（真机全部组件）
```

### 2.2 URDF 中的 ros2_control 标签

这是 ros2_control 与 URDF 的**唯一接触点**。`<ros2_control>` 标签告诉 `resource_manager`：
- 用哪个硬件插件
- 每个关节有哪些命令/状态接口

#### 仿真版（Gazebo）

```xml
<!-- flapping_platform.ros2_control.xacro -->
<xacro:macro name="flapping_platform_ros2_control" params="use_gazebo initial_positions_file">
    <ros2_control name="RobotSystem" type="system">
        <hardware>
            <xacro:if value="${use_gazebo}">
                <plugin>gz_ros2_control/GazeboSimSystem</plugin>  <!-- Gazebo 插件 -->
            </xacro:if>
        </hardware>
        <joint name="arm1_joint1">          <!-- 带前缀 arm1_ -->
            <command_interface name="position"/>   <!-- 接受位置指令 -->
            <state_interface name="position">      <!-- 汇报位置状态 -->
                <param name="initial_value">0.0</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
        <!-- ... arm1_joint2 到 arm1_joint6 同理 ... -->
        <joint name="plate_joint">
            <command_interface name="velocity"/>   <!-- 转台用速度控制 -->
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
</xacro:macro>
```

#### 虚拟真机版（mock_components）

```xml
<!-- flapping_platform_real.ros2_control.xacro -->
<xacro:macro name="flapping_platform_real_ros2_control" params="initial_positions_file">
    <ros2_control name="RobotSystem" type="system">
        <hardware>
            <plugin>mock_components/GenericSystem</plugin>  <!-- 虚拟硬件：指令即状态 -->
        </hardware>
        <joint name="joint1">                 <!-- 无前缀，匹配真机 -->
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
        <!-- ... joint2 到 joint6 同理 ... -->
    </ros2_control>
</xacro:macro>
```

**关键差异**：
| 属性 | Gazebo 仿真 | mock_components | 真机 |
|------|------------|-----------------|------|
| 硬件插件 | `gz_ros2_control/GazeboSimSystem` | `mock_components/GenericSystem` | URDF 中不启动 ros2_control |
| 关节名 | `arm1_joint1..6`（有前缀） | `joint1..6`（无前缀） | `joint1..6`（硬件发布） |
| 状态来源 | Gazebo 物理引擎计算 | 指令值即时镜像 | 机械臂 encoder 回传 |

---

## 3. 三种控制链路详解

### 3.1 模式一：Gazebo 仿真

```
                        ┌─────────────────────────────┐
                        │        Gazebo 进程           │
                        │  ┌───────────────────────┐   │
                        │  │  gz_ros2_control 插件  │   │
                        │  │  ↓ 写关节力矩           │   │
                        │  │  物理引擎 (ODE/Bullet)  │   │
                        │  │  ↓ 计算位置/速度        │   │
                        │  │  ↑ 读关节状态           │   │
                        │  └───────────────────────┘   │
                        └─────────────────────────────┘
                                  ↑ command    │ state
                                  │ interfaces │ interfaces
┌─────────────────────────────────┼────────────┼──────────────────┐
│                          ros2_control 进程                      │
│  ┌──────────────────┐  ┌──────────────────┐  ┌───────────────┐  │
│  │ JointTrajectory  │  │ JointState       │  │ controller_   │  │
│  │ Controller       │  │ Broadcaster      │  │ manager       │  │
│  │                  │  │                  │  │               │  │
│  │ 接收轨迹→插值     │  │ 读状态→发布话题   │  │ 加载/切换     │  │
│  │ →发送位置指令     │  │ /joint_states    │  │ 控制器        │  │
│  └────────┬─────────┘  └────────┬─────────┘  └───────────────┘  │
│           │ command             │ state                          │
└───────────┼─────────────────────┼────────────────────────────────┘
            │                     │
            ▼                     ▼
   /rm_group_controller/    /joint_states
   follow_joint_trajectory    │
            ▲                 ▼
            │          ┌──────────────┐
            │          │ robot_state  │
┌───────────┴──────────┤ publisher    │
│     move_group        │ (URDF→TF)   │
│  (MoveIt2 规划)       └──────────────┘
│                              │
│  用户拖拽目标位姿             ▼
│  → 逆运动学求解             /tf
│  → 碰撞检测                 (base_link→link1→...→link6)
│  → 轨迹规划 (OMPL)
│  → 发送 Action
└──────────────────────────────┘
```

**Gazebo 模式特点**：
- MoveIt 发布的轨迹**间接**影响关节状态（经过物理引擎）
- 有物理延迟、可以模拟摩擦力/重力/碰撞
- `use_sim_time:=true`（使用 Gazebo 的 `/clock` 而非系统时间）
- 需要先启动 Gazebo + spawn 机器人模型

### 3.2 模式二：mock_components 虚拟模式

```
┌──────────────────────────────────────────────────────────────┐
│                     ros2_control_node 进程                      │
│                                                                │
│  ┌───────────────────────┐    ┌────────────────────────────┐  │
│  │ mock_components/      │    │ controller_manager         │  │
│  │ GenericSystem         │    │                            │  │
│  │                       │    │ ┌────────────────────────┐ │  │
│  │  command_interface ◄──┼────┤ JointTrajectoryController│ │  │
│  │  (position/velocity)  │    │ │ (轨迹→位置指令)         │ │  │
│  │       │               │    │ └────────────────────────┘ │  │
│  │       │ 即时镜像       │    │ ┌────────────────────────┐ │  │
│  │       ▼               │    │ │ JointStateBroadcaster  │ │  │
│  │  state_interface ─────┼───►│ │ (读状态→/joint_states)  │ │  │
│  │  (position/velocity)  │    │ └────────────────────────┘ │  │
│  └───────────────────────┘    └────────────────────────────┘  │
│                                                                │
│   数据流: moveit指令 → controller → GenericSystem → 即时返回    │
│   延迟: ~0ms (纯内存操作)                                       │
│   时间: 系统时钟 (不用 use_sim_time)                             │
└──────────────────────────────────────────────────────────────┘
              ▲ command                      │ state
              │                              ▼
     /rm_group_controller/           /joint_states
     follow_joint_trajectory              │
              ▲                           ▼
       ┌──────┴────────┐          ┌──────────────┐
       │  move_group    │          │ robot_state  │
       │  + rviz2       │          │ publisher    │
       └───────────────┘          └──────────────┘
```

**mock_components 本质**：
- `GenericSystem` 是一个**零延迟理想硬件模拟器**
- 写入 `command_interface` 的值会被**立即复制**到 `state_interface`
- 不需要 Gazebo、不需要物理引擎
- 用于**纯软件验证**：MoveIt 配置、URDF 正确性、TF 树完整性

**对应你的项目文件**：
- URDF: [flapping_platform_real.urdf.xacro](src/flapping_platform/flapping_platform_description/urdf/robots/flapping_platform_real.urdf.xacro)
- ros2_control: [flapping_platform_real.ros2_control.xacro](src/flapping_platform/flapping_platform_description/urdf/control/flapping_platform_real.ros2_control.xacro)
- Launch: [demo_moveit.launch.py](src/flapping_platform/flapping_platform_moveit_config/launch/demo_moveit.launch.py)

### 3.3 模式三：睿尔曼真机

```
┌────────────────────────────────────────────────────────────────┐
│                    RM 机械臂硬件                                 │
│  Ethernet (TCP/IP)                                              │
│  ↑ 接收 movej_canfd 指令 (CAN-FD over TCP)                       │
│  ↓ 返回关节状态 (encoder 读数)                                   │
└────────────────────┬───────────────────────────────────────────┘
                     │ TCP
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  rm_driver 节点                                                  │
│  ├── SDK 层: 睿尔曼 C++ API 封装                                  │
│  ├── 发布: /joint_states (关节状态)                               │
│  └── 订阅: /rm_driver/movej_canfd_cmd (原始运动指令)              │
└────────────────────┬───────────────────────────────────────────┘
                     │ /joint_states (由 driver 发布!)
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  rm_control 节点 (Action Server)                                 │
│  ├── Action: /rm_group_controller/follow_joint_trajectory        │
│  ├── 接收 MoveIt 轨迹 → cubic spline 插值                         │
│  └── 发布: /rm_driver/movej_canfd_cmd → rm_driver → 硬件          │
└────────────────────┬───────────────────────────────────────────┘
                     ▲ Action Goal
                     │
┌────────────────────┴───────────────────────────────────────────┐
│  move_group (MoveIt2)                                            │
│  ├── 订阅 /joint_states → 跟踪当前状态 (关节名匹配!)              │
│  ├── 规划: 逆运动学 + 碰撞检测 + 轨迹生成                          │
│  ├── 通过 FollowJointTrajectory Action 发送轨迹给 rm_control      │
│  ├── moveit_manage_controllers: False  ← 关键!                   │
│  └── 不启动 ros2_control_node                                    │
└────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  robot_state_publisher (单独启动)                                 │
│  ├── 输入: /joint_states (来自 rm_driver)                        │
│  ├── 输入: 含转台的 flapping_platform_real.urdf.xacro             │
│  └── 输出: /tf (完整 TF 树: world→base_link→...→link6)            │
└────────────────────────────────────────────────────────────────┘
```

**真机模式核心要点**：

1. **不启动 ros2_control_node** — 硬件由 rm_driver 直接管理
2. **`moveit_manage_controllers: False`** — MoveIt 不管理 controller 生命周期，只发 Action
3. **轨迹执行参数** — 真机需要更宽松的容差：
   ```yaml
   trajectory_execution.allowed_execution_duration_scaling: 1.2
   trajectory_execution.allowed_goal_duration_margin: 0.5
   trajectory_execution.allowed_start_tolerance: 0.15
   ```
4. **robot_state_publisher 单独启动** — 包含转台的完整 URDF

**对应你的项目文件**：
- Bringup: [flapping_platform_bringup.launch.py](src/flapping_platform/flapping_platform_bringup/launch/flapping_platform_bringup.launch.py)
- MoveIt: [real_moveit.launch.py](src/flapping_platform/flapping_platform_moveit_config/launch/real_moveit.launch.py)
- RSP: [robot_state_publisher_real.launch.py](src/flapping_platform/flapping_platform_description/launch/robot_state_publisher_real.launch.py)

### 3.4 三种模式对比总结

| 维度 | Gazebo 仿真 | mock_components | 真机 |
|------|------------|----------------|------|
| **硬件插件** | `gz_ros2_control/GazeboSimSystem` | `mock_components/GenericSystem` | 无（rm_driver 代替） |
| **状态来源** | 物理引擎计算 | 指令即时镜像 | 机械臂 encoder |
| **ros2_control_node** | 启动 | 启动 | **不启动** |
| **controller_manager** | 管理所有 controller | 管理所有 controller | 无（rm_control 代替） |
| **moveit_manage_controllers** | True（默认） | True（默认） | **False** |
| **轨迹执行** | JointTrajectoryController | JointTrajectoryController | rm_control Action Server |
| **/joint_states 发布者** | JointStateBroadcaster | JointStateBroadcaster | rm_driver |
| **use_sim_time** | True | False（或用系统时钟） | False |
| **robot_state_publisher** | 在 demo launch 中 | 在 demo launch 中 | 单独 launch 文件 |
| **物理模拟** | 有（重力/摩擦/碰撞） | 无 | 真实物理 |
| **延迟** | 模拟延迟 | ~0ms | 网络 + 机械延迟 |

---

## 4. Launch 文件编写模式

### 4.1 模式对比

MoveIt2 提供了 `moveit_configs_utils` 工具库，`MoveItConfigsBuilder` 是核心入口。Launch 文件有两种编写风格：

#### 风格 A：自包含函数式（本项目采用）

```python
# demo_moveit.launch.py — 把所有组件打包成独立函数
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(...).to_moveit_configs()
    ld = LaunchDescription()
    generate_static_virtual_joint_tfs_launch(ld, moveit_config)  # 虚关节 TF
    generate_rsp_launch(ld, moveit_config)                        # robot_state_publisher
    generate_move_group_launch(ld, moveit_config)                 # move_group
    generate_moveit_rviz_launch(ld, moveit_config)                # rviz
    # 手动添加 ros2_control_node + spawner
    return ld
```

**优点**：自包含，一个文件启动全部。适合开发调试。

#### 风格 B：模块化 IncludeLaunchDescription（RM 参考采用）

```python
# rm_63_III_6fb_bringup.launch.py — 用 IncludeLaunchDescription 组合
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(rm_driver),        # 独立包
        IncludeLaunchDescription(rm_control),       # 独立包
        IncludeLaunchDescription(rsp_display),      # 独立包
        IncludeLaunchDescription(real_moveit),      # 独立包
    ])
```

**优点**：每个子 launch 独立可测，真机部署灵活。适合生产环境。

### 4.2 虚拟/仿真 Launch 的核心组件

一个自包含的虚拟/仿真 demo launch 需要以下组件：

| 序号 | 组件 | 包 | 作用 |
|------|------|-----|------|
| 1 | `static_transform_publisher` | `tf2_ros` | 发布 SRDF 中 virtual_joint 的静态 TF |
| 2 | `robot_state_publisher` | `robot_state_publisher` | `/joint_states` → `/tf` |
| 3 | `move_group` | `moveit_ros_move_group` | 运动规划核心 |
| 4 | `rviz2` | `rviz2` | 可视化 + MotionPlanning 插件 |
| 5 | `ros2_control_node` | `controller_manager` | 加载硬件插件 + 控制器 |
| 6 | `spawner` (×N) | `controller_manager` | 启动每个控制器 |

### 4.3 真机 Launch 的精简

真机 launch 只保留核心：

| 序号 | 组件 | 原因 |
|------|------|------|
| 3 | `move_group` | 规划必需 |
| 4 | `rviz2` | 可视化 + 交互 |
| — | 去掉 ros2_control_node | 硬件由 rm_driver 管理 |
| — | 去掉 spawner | 不需要 ros2_control controllers |
| — | 去掉 rsp | 移到单独的 launch 文件（含转台） |
| — | 去掉 static_transform_publisher | 真机 SRDF 通常无 virtual_joint |

### 4.4 MoveItConfigsBuilder 详解

`MoveItConfigsBuilder` 是 MoveIt2 的**配置聚合器**，通过链式调用加载所有配置：

```python
moveit_config = (
    MoveItConfigsBuilder("flapping_platform", package_name="flapping_platform_moveit_config")
    # 1. robot_description: URDF xacro → 机器人几何 + ros2_control 标签
    .robot_description(
        file_path="path/to/robot.urdf.xacro",       # xacro 文件路径
        mappings={                                   # xacro 参数
            "initial_positions_file": "...",
        },
    )
    # 2. robot_description_semantic: SRDF → 规划组、碰撞、被动关节
    .robot_description_semantic(
        file_path="config/xxx/xxx.srdf"
    )
    # 3. trajectory_execution: moveit_controllers.yaml → MoveIt 如何发轨迹
    .trajectory_execution(
        file_path="config/xxx/moveit_controllers.yaml"
    )
    # 4. robot_description_kinematics: kinematics.yaml → IK 求解器
    .robot_description_kinematics(
        file_path="config/xxx/kinematics.yaml"
    )
    # 5. joint_limits: joint_limits.yaml → 关节限位
    .joint_limits(file_path="config/xxx/joint_limits.yaml")
    # 6. planning_pipelines: 规划器（ompl, chomp, stomp 等）
    .planning_pipelines(pipelines=["ompl"])
    # 7. 生成最终配置对象
    .to_moveit_configs()
)
```

**MoveItConfigs 对象的主要属性**：
- `moveit_config.robot_description` → `{"robot_description": "<urdf xml string>"}`
- `moveit_config.robot_description_semantic` → `{"robot_description_semantic": "<srdf xml string>"}`
- `moveit_config.robot_description_kinematics` → `{"robot_description_kinematics": {...}}`
- `moveit_config.planning_pipelines` → `{"planning_pipelines": {...}}`
- `moveit_config.trajectory_execution` → `{"moveit_simple_controller_manager": {...}}`
- `moveit_config.joint_limits` → `{"robot_description_planning": {"joint_limits": {...}}}`
- `moveit_config.to_dict()` → 所有配置的扁平化字典（传给 move_group）

---

## 5. 关键参数详解

### 5.1 move_group 参数

```python
move_group_configuration = {
    # 将 SRDF 发布到 /robot_description_semantic 话题（RViz 需要）
    "publish_robot_description_semantic": True,

    # 是否允许执行轨迹（False = 只规划不执行）
    "allow_trajectory_execution": True,

    # 加载额外 MoveGroup 功能（如 "move_group/MoveGroupCartesianPathService"）
    "capabilities": "",

    # 禁用默认功能
    "disable_capabilities": "",

    # 发布 planning scene（RViz MotionPlanning 插件需要）
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,

    # 是否从 /joint_states 复制速度和加速度到内部监控
    "monitor_dynamics": False,
}
```

### 5.2 轨迹执行参数

```python
trajectory_execution = {
    # ★ 最关键的参数：是否由 MoveIt 管理 ros2_control 控制器生命周期
    # True (默认): MoveIt 启动/停止 controller，检查 controller 状态
    # False (真机): MoveIt 直接发 FollowJointTrajectory Action，不管 controller
    "moveit_manage_controllers": False,

    # 允许的执行时间缩放 (轨迹时长 × 1.2 内算成功)
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,

    # 目标到达容差 (秒) — 轨迹结束后多久算到达
    "trajectory_execution.allowed_goal_duration_margin": 0.5,

    # 起始位置容差 (弧度) — 当前关节位置离轨迹起点多远算失败
    "trajectory_execution.allowed_start_tolerance": 0.15,
}
```

### 5.3 moveit_controllers.yaml — MoveIt 与 Controller 的桥梁

```yaml
# 指定控制器管理器类型
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - rm_group_controller              # 控制器名称（必须与 ros2_controllers.yaml 一致）

  rm_group_controller:
    type: FollowJointTrajectory        # Action 类型
    action_ns: follow_joint_trajectory # Action 命名空间
    default: true                      # 是否为默认控制器
    joints:                            # 该控制器管理的关节（必须与 URDF joint name 完全一致!）
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
```

**最终 Action 全名**：`/rm_group_controller/follow_joint_trajectory`

### 5.4 ros2_controllers.yaml — ros2_control 控制器参数

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100                  # 控制循环频率 (Hz)

    # 声明控制器类型
    rm_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    turntable_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

# JointStateBroadcaster 参数
joint_state_broadcaster:
  ros__parameters:
    joints:                           # 要发布的关节（可选，Humble 有 bug 会忽略）
      - joint1
      - joint2
      # ...

# JointTrajectoryController 参数
rm_group_controller:
  ros__parameters:
    joints:                           # 控制的关节
      - joint1
      # ...
    command_interfaces:               # 使用的命令接口（与 URDF 中定义的对应）
      - position
    state_interfaces:                 # 使用的状态接口
      - position
      - velocity
```

### 5.5 SRDF 文件

```xml
<robot name="flapping_platform">
    <!-- 规划组: chain 从 base_link 到 link6 -->
    <group name="rm_group">
        <chain base_link="base_link" tip_link="link6"/>
    </group>

    <!-- 预定义姿态 -->
    <group_state name="home" group="rm_group">
        <joint name="joint1" value="0"/>
        <!-- ... -->
    </group_state>

    <!-- 被动关节（MoveIt 不规划，只读取状态） -->
    <passive_joint name="plate_joint"/>

    <!-- 碰撞禁用（跳过已知安全的碰撞对，加速规划） -->
    <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="link3" reason="Never"/>
    <!-- ... -->
</robot>
```

---

## 6. 实战：从零搭建新机器人配置

### 6.1 配置文件清单（必选 6 个）

| # | 文件 | 作用 |
|---|------|------|
| 1 | `urdf/robots/xxx.urdf.xacro` | 机器人几何 + ros2_control 标签 |
| 2 | `config/xxx/xxx.srdf` | 规划组 + 碰撞 + 被动关节 |
| 3 | `config/xxx/kinematics.yaml` | IK 求解器配置 |
| 4 | `config/xxx/joint_limits.yaml` | 关节限位 |
| 5 | `config/xxx/moveit_controllers.yaml` | MoveIt → Controller 映射 |
| 6 | `config/xxx/ros2_controllers.yaml` | Controller 参数 |

### 6.2 配置文件清单（按需）

| # | 文件 | 何时需要 |
|---|------|----------|
| 7 | `urdf/control/xxx.ros2_control.xacro` | 有自定义硬件插件时 |
| 8 | `config/xxx/initial_positions.yaml` | 虚拟/仿真模式（mock_components 需要初始值） |
| 9 | `config/xxx/moveit.rviz` | 自定义 RViz 布局 |

### 6.3 Launch 文件清单

| 场景 | Launch 文件 | 包含组件 |
|------|------------|----------|
| 虚拟验证 | `demo_xxx.launch.py` | rsp + move_group + rviz + ros2_control + spawners |
| 真机 | `real_xxx.launch.py` | move_group + rviz |
| 真机 | `xxx_bringup.launch.py` | driver + control + rsp + moveit (组合以上) |

### 6.4 配置一致性检查清单

- [ ] URDF 关节名 == moveit_controllers.yaml joints 列表 == ros2_controllers.yaml joints 列表
- [ ] SRDF chain 中的 link 名存在于 URDF
- [ ] ros2_control 标签中 `command_interface` 与 ros2_controllers.yaml 中 `command_interfaces` 匹配
- [ ] ros2_control 标签中 `state_interface` 与 ros2_controllers.yaml 中 `state_interfaces` 匹配
- [ ] `moveit_controllers.yaml` 中 `controller_names` 与 `ros2_controllers.yaml` 中 `controller_manager` 下声明的 controller 名称一致
- [ ] 真机模式下 `moveit_manage_controllers: False`
- [ ] initial_positions.yaml 中关节名与 URDF 一致

---

## 7. 调试与排查

### 7.1 验证关节名匹配

```bash
# 查看当前 /joint_states 发布哪些关节
ros2 topic echo /joint_states --once | grep -A 20 name

# 查看 URDF 定义了哪些关节
ros2 run xacro xacro src/xxx/urdf/robots/xxx.urdf.xacro 2>&1 | grep 'joint name='

# 查看 SRDF 定义了哪些规划组
cat src/xxx/config/xxx/xxx.srdf | grep -E '<group|<joint'
```

### 7.2 验证 TF 树完整性

```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf，检查链: world→platform_base_link→platform_plate_Link→base_link→link1→...→link6
```

### 7.3 验证 MoveIt → Controller 的连接

```bash
# 检查 Action Server 是否存在
ros2 action list | grep follow_joint_trajectory
# 虚拟/仿真: /rm_group_controller/follow_joint_trajectory (由 JointTrajectoryController 提供)
# 真机:     /rm_group_controller/follow_joint_trajectory (由 rm_control 提供)

# 查看 Action 接口
ros2 action info /rm_group_controller/follow_joint_trajectory
```

### 7.4 常见问题

| 症状 | 原因 | 检查 |
|------|------|------|
| RViz 机器人不显示 | robot_state_publisher 没启动或 URDF 路径错误 | `ros2 node list \| grep robot_state` |
| Planning scene 不更新 | move_group 没订阅到 /joint_states | `ros2 topic echo /joint_states` |
| Plan 成功但 Execute 失败 | 关节名不匹配或 controller 没启动 | 对比 URDF/controller 的关节名 |
| 真机机器人不动 | `moveit_manage_controllers: True` 或 Action 不存在 | 改为 False，检查 `ros2 action list` |
| TF 树不完整 | 缺少转台或虚关节的 TF 发布 | `ros2 run tf2_tools view_frames` |
| RViz 无 MotionPlanning 面板 | 未加载 MoveIt RViz 插件配置 | 确认 rviz_config 路径指向 moveit.rviz |
| controller_manager 找不到 controller | ros2_controllers.yaml 中未声明 | 检查 `controller_manager.ros__parameters` 下是否有对应条目 |

---

## 附录：本项目文件索引

### 仿真模式（Gazebo + arm1_ 前缀）
- URDF: `flapping_platform_description/urdf/robots/flapping_platform.urdf.xacro`
- ros2_control: `flapping_platform_description/urdf/control/flapping_platform.ros2_control.xacro`
- Config: `flapping_platform_moveit_config/config/flapping_platform/`

### 虚拟真机模式（mock_components + 空前缀）
- URDF: `flapping_platform_description/urdf/robots/flapping_platform_real.urdf.xacro`
- ros2_control: `flapping_platform_description/urdf/control/flapping_platform_real.ros2_control.xacro`
- Config: `flapping_platform_moveit_config/config/flapping_platform_real/`
- Launch: `flapping_platform_moveit_config/launch/demo_moveit.launch.py`

### 真机模式（rm_driver + rm_control）
- Bringup: `flapping_platform_bringup/launch/flapping_platform_bringup.launch.py`
- MoveIt: `flapping_platform_moveit_config/launch/real_moveit.launch.py`
- RSP: `flapping_platform_description/launch/robot_state_publisher_real.launch.py`

### RM 官方参考
- 虚拟 demo: `ros2_rm_robot-humble/rm_moveit2_config/rm_63_config/launch/demo_III_6fb.launch.py`
- 真机 demo: `ros2_rm_robot-humble/rm_moveit2_config/rm_63_config/launch/real_moveit_demo_III_6fb.launch.py`
- Gazebo demo: `ros2_rm_robot-humble/rm_moveit2_config/rm_63_config/launch/gazebo_moveit_demo_III_6fb.launch.py`
- Driver: `ros2_rm_robot-humble/rm_driver/launch/rm_63_driver.launch.py`
- Control: `ros2_rm_robot-humble/rm_control/launch/rm_63_control.launch.py`
