# agilex_ws/src 工作空间子项目总览

本工作空间是一个 ROS 1 (catkin) 工作空间，集成了移动机器人底盘驱动、机械臂控制、升降机构、夹爪、激光雷达、深度相机、SLAM 建图与导航、RViz 插件等功能模块，用于构建一套完整的移动操作机器人（Loco-Manipulation）系统。

---

## 当前设备配置

> **以下为本项目实际使用的硬件设备，相关子项目在目录中以 `[当前设备]` 标注。**

| 角色 | 设备型号 | 对应子项目 | 关键启动命令 |
|------|----------|-----------|-------------|
| **移动底盘** | AgileX Ranger | [ranger_ros](#1-ranger_ros----ranger-系列底盘驱动) `[当前设备]` | `roslaunch ranger_bringup ranger.launch` |
| **机械臂** | Dobot CR10 | [TCP-IP-ROS-6AXis](#3-tcp-ip-ros-6axis----dobot-六轴机械臂-sdk) `[当前设备]` | `export DOBOT_TYPE=cr10` 后 `roslaunch dobot_v4_bringup bringup_v4.launch` |
| **夹爪** | DH-Robotics AG95 | [gripper](#4-gripper----dh-robotics-夹爪驱动) `[当前设备]` | `roslaunch dh_gripper_driver dh_gripper.launch GripperModel:=AG95_MB` |

### 快速启动流程（Ranger + CR10 + AG95）

```bash
# 1. 使能 CAN 口（每次开机执行一次）
rosrun ranger_bringup bringup_can2usb.bash

# 2. 启动 Ranger 底盘
roslaunch ranger_bringup ranger.launch

# 3. 启动 CR10 机械臂（新终端）
export DOBOT_TYPE=cr10
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1

# 4. 启动 AG95 夹爪（新终端）
roslaunch dh_gripper_driver dh_gripper.launch GripperModel:=AG95_MB Connectport:=/dev/ttyUSB0

# 5. 启动 MoveIt 运动规划（新终端）
export DOBOT_TYPE=cr10
roslaunch dobot_moveit moveit.launch

# 6. 启动激光雷达 + 导航（可选）
roslaunch rslidar_sdk start.launch
roslaunch ranger_bringup navigation_4wd.launch
```

---

## 目录

1. [ranger_ros -- Ranger 系列底盘驱动 `[当前设备]`](#1-ranger_ros----ranger-系列底盘驱动-当前设备)
2. [ugv_sdk -- AgileX/Weston Robot 底盘通信 SDK](#2-ugv_sdk----agilexweston-robot-底盘通信-sdk)
3. [TCP-IP-ROS-6AXis -- Dobot 六轴机械臂 SDK `[当前设备]`](#3-tcp-ip-ros-6axis----dobot-六轴机械臂-sdk-当前设备)
4. [gripper -- DH-Robotics 夹爪驱动 `[当前设备]`](#4-gripper----dh-robotics-夹爪驱动-当前设备)
5. [lifting_ctrl -- 电动升降柱控制](#5-lifting_ctrl----电动升降柱控制)
6. [bt_task_msgs -- 自定义消息/服务定义](#6-bt_task_msgs----自定义消息服务定义)
7. [rslidar_sdk -- RoboSense 激光雷达驱动](#7-rslidar_sdk----robosense-激光雷达驱动)
8. [realsense-ros -- Intel RealSense 深度相机驱动](#8-realsense-ros----intel-realsense-深度相机驱动)
9. [SLAM -- 建图与激光里程计](#9-slam----建图与激光里程计)
10. [plugin -- RViz 多目标导航插件](#10-plugin----rviz-多目标导航插件)
11. [librealsense-2.50.0.zip -- LibRealSense SDK 源码包](#11-librealsense-2500zip----librealsense-sdk-源码包)

---

## 1. ranger_ros -- Ranger 系列底盘驱动 `[当前设备]`

**路径:** `ranger_ros/`
**来源:** https://github.com/agilexrobotics/ranger_ros.git
**协议:** BSD 3-Clause

为 AgileX Ranger 系列移动机器人底盘提供 ROS 驱动，通过 CAN 总线与底盘通信。**当前项目使用的底盘为 Ranger 全尺寸版。**

### 支持的硬件

| 型号 | 轮距 | 轴距 | 最大速度 |
|------|------|------|----------|
| Ranger | 0.56m | 0.90m | 2.7 m/s |
| Ranger Mini V1.0 | 0.36m | 0.36m | 1.5 m/s |
| Ranger Mini V2.0 | 0.364m | 0.494m | 5.4 m/s |

### 包含的 ROS 包

| 包名 | 说明 |
|------|------|
| `ranger_base` | 核心驱动节点（C++），发布里程计、系统状态、执行器状态、电池状态 |
| `ranger_msgs` | 自定义消息定义（SystemState、MotionState、ActuatorState 等） |
| `ranger_bringup` | 启动文件、地图、导航参数、CAN 配置脚本 |
| `sensor_msgs` | 本地打包的 sensor_msgs 包（确保版本兼容） |

### 运动模式（自动切换）

- **DUAL_ACKERMAN** -- 标准阿克曼转向（默认）
- **PARALLEL** -- 全轮平行转向（横向移动）
- **SPINNING** -- 原地旋转
- **SIDE_SLIP** -- 侧向滑移（仅 Ranger Mini V1）

节点根据 `/cmd_vel` 中 `linear.x`、`linear.y`、`angular.z` 的值自动选择运动模式。

### ROS 接口

**发布的话题:**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/system_state` | `ranger_msgs/SystemState` | 车辆状态、控制模式、错误码、电池电压 |
| `/motion_state` | `ranger_msgs/MotionState` | 当前运动模式 |
| `/actuator_state` | `ranger_msgs/ActuatorStateArray` | 8 个执行器状态 |
| `/odom` | `nav_msgs/Odometry` | 航迹推算里程计 |
| `/battery_state` | `sensor_msgs/BatteryState` | BMS 电池状态 |

**订阅的话题:**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制命令 |

### 使用方法

```bash
# 首次设置 CAN 适配器
sudo modprobe gs_usb
rosrun ranger_bringup setup_can2usb.bash

# 后续启动（每次开机后执行一次）
rosrun ranger_bringup bringup_can2usb.bash

# 启动底盘驱动（根据型号选择）
roslaunch ranger_bringup ranger.launch           # Ranger 全尺寸
roslaunch ranger_bringup ranger_mini_v1.launch    # Ranger Mini V1
roslaunch ranger_bringup ranger_mini_v2.launch    # Ranger Mini V2

# 启动完整导航栈（含地图、AMCL、move_base）
roslaunch ranger_bringup navigation_4wd.launch

# 启动 SLAM 建图
roslaunch ranger_bringup gmapping.launch
```

### 依赖

- `ugv_sdk`（本工作空间内）
- `libasio-dev`、`libboost-all-dev`、`eigen3`

---

## 2. ugv_sdk -- AgileX/Weston Robot 底盘通信 SDK

**路径:** `ugv_sdk/`
**来源:** https://github.com/westonrobot/ugv_sdk.git
**协议:** BSD

纯 C++ SDK 库（非 ROS 节点），通过 CAN 总线与 AgileX/Weston Robot 系列 UGV 底盘通信。可独立编译，也可在 catkin 工作空间中构建。

### 支持的机器人

| 机器人 | 协议 V1 | 协议 V2 |
|--------|---------|---------|
| Scout 1.0 / 2.0 | Yes | Yes |
| Scout Mini (Skid / Omni) | Yes | Yes |
| Hunter 1.0 / 2.0 | Yes | Yes |
| Bunker | Yes | Yes |
| Tracer | - | Yes |
| Ranger Mini 1.0 / 2.0 | - | Yes |
| Ranger | - | Yes |

### 主要 API 类（命名空间 `westonrobot`）

- `ScoutRobot` -- 差速驱动底盘
- `ScoutMiniOmniRobot` -- 全向底盘
- `HunterRobot` -- 阿克曼转向底盘（含刹车控制）
- `BunkerRobot` -- 履带底盘
- `TracerRobot` -- 轻型底盘
- `RangerRobot` -- 多模式转向底盘

### 使用示例

```cpp
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
using namespace westonrobot;

ScoutRobot scout(AGX_V2, false);
scout.Connect("can0");
scout.EnableCommandedMode();

// 以 50Hz 以上频率调用
scout.SetMotionCommand(1.0, 0.0);  // linear_vel, angular_vel

// 读取状态
auto state = scout.GetRobotState();
```

### CAN 适配器设置脚本

- `scripts/setup_can2usb.bash` -- 首次设置（加载 gs_usb 模块、启动 can0）
- `scripts/bringup_can2usb_500k.bash` -- 启动 can0（500kbps）
- `scripts/bringup_can2usb_1m.bash` -- 启动 can0（1Mbps）

---

## 3. TCP-IP-ROS-6AXis -- Dobot 六轴机械臂 SDK `[当前设备]`

**路径:** `TCP-IP-ROS-6AXis/`
**来源:** https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
**协议:** MIT

Dobot（越疆科技）六轴协作机械臂的官方 ROS SDK，通过 TCP/IP 协议与控制器通信，提供 ROS 服务、话题、Action 和 MoveIt 接口。**当前项目使用的机械臂为 CR10。**

### 支持的机械臂型号

| 型号 | MoveIt 包 | 型号 | MoveIt 包 |
|------|-----------|------|-----------|
| CR3 | `cr3_moveit` | CR12 | `cr12_moveit` |
| CR5 | `cr5_moveit` | CR16 | `cr16_moveit` |
| CR7 | `cr7_moveit` | ME6 | `me6_moveit` |
| CR10 | `cr10_moveit` | Nova2 | `nova2_moveit` |
|      |            | Nova5 | `nova5_moveit` |

通过环境变量选择型号：`export DOBOT_TYPE=cr5`

### 通信架构

```
用户代码 / MoveIt / RViz
        | (ROS 话题、服务、ActionLib)
        v
dobot_bringup (V3) 或 dobot_v4_bringup (V4)
        | (TCP Socket)
        v
Dobot 控制器
  端口 29999: Dashboard 命令（请求/响应）
  端口 30004: 实时反馈（二进制流，8ms 周期）
```

### 包含的 ROS 包

| 包名 | 说明 |
|------|------|
| `dobot_bringup` | V3 控制器通信节点（80+ 个 ROS 服务） |
| `dobot_v4_bringup` | V4 控制器通信节点（90+ 个 ROS 服务，支持多机器人） |
| `dobot_moveit` | 统一 MoveIt 启动器（根据 DOBOT_TYPE 分发） |
| `dobot_description` | URDF 模型（9 种机械臂） |
| `dobot_gazebo` | Gazebo 仿真 |
| `rviz_dobot_control` | RViz 面板插件（使能/禁用按钮） |
| `rosdemo_v3` | V3 演示程序 |
| `rosdemo_v4` | V4 演示程序 |
| `cr3_moveit` ~ `nova5_moveit` | 各型号 MoveIt 配置 |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 6 个关节位置（10Hz） |
| `/<type>_robot/msg/RobotStatus` | 自定义 | 使能/连接状态 |
| `/<type>_robot/msg/ToolVectorActual` | 自定义 | 末端执行器笛卡尔位姿 |
| `/<type>_robot/msg/FeedInfo` | `std_msgs/String` | V4 实时反馈 JSON（100Hz） |

### 使用方法

```bash
# 仿真（无需实物）
roslaunch dobot_description display.launch        # RViz 关节滑块
roslaunch dobot_moveit demo.launch                # MoveIt 仿真
roslaunch dobot_gazebo gazebo.launch              # Gazebo 仿真

# 真实机械臂控制（当前设备为 CR10）
export DOBOT_TYPE=cr10
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1
roslaunch dobot_moveit moveit.launch              # 另一个终端

# 多机器人
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.10 robotName:=robotA
roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.20 robotName:=robotB
```

### 关键服务（部分）

- `EnableRobot` / `DisableRobot` -- 使能/禁用
- `MovJ` / `MovL` / `JointMovJ` -- 关节/笛卡尔运动
- `ServoJ` / `ServoP` -- 实时伺服
- `EmergencyStop` / `ClearError` -- 急停/清除错误
- `DO` / `DI` / `AO` / `AI` -- 数字/模拟 IO
- `PositiveSolution` / `InverseSolution` -- 正/逆运动学

---

## 4. gripper -- DH-Robotics 夹爪驱动 `[当前设备]`

**路径:** `gripper/`
**作者:** 深圳大寰机器人科技有限公司

DH-Robotics 系列电动夹爪的 ROS 驱动，支持串口（UART）和 TCP/IP 两种通信方式。**当前项目使用的夹爪为 AG95（Modbus 协议）。**

### 包含的 ROS 包

| 包名 | 说明 |
|------|------|
| `dh_gripper_msgs` | 自定义消息（GripperCtrl、GripperState、GripperRotCtrl、GripperRotState） |
| `dh_gripper_driver` | 驱动节点、测试节点、关节状态发布节点 |

### 支持的夹爪型号

| 型号匹配 | 通信协议 | 轴数 |
|----------|----------|------|
| `AG95_CAN` | 自定义 CAN 协议 | 1（夹爪） |
| `DH3_CAN` | 自定义 CAN 协议 | 1（夹爪 + 旋转） |
| `AG95_MB`、`PGE`、`PGC`、`CGC` | Modbus RTU | 1（夹爪） |
| `RGI` | Modbus RTU | 2（夹爪 + 旋转） |

### ROS 接口

**订阅:**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gripper/ctrl` | `GripperCtrl` | 控制夹爪（位置、力、速度、初始化） |
| `/gripper/rot_ctrl` | `GripperRotCtrl` | 控制旋转轴（仅 RGI/DH3） |

**发布:**

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/gripper/states` | `GripperState` | 50Hz | 夹爪状态 |
| `/gripper/rot_states` | `GripperRotState` | 50Hz | 旋转状态 |
| `/gripper/joint_states` | `sensor_msgs/JointState` | 50Hz | 关节状态（0~0.637 rad） |

### 使用方法

```bash
# 默认 AG95 Modbus 夹爪
roslaunch dh_gripper_driver dh_gripper.launch

# 指定型号和端口
roslaunch dh_gripper_driver dh_gripper.launch GripperModel:=RGI Connectport:=/dev/ttyUSB1

# 启用测试节点（自动开合循环）
roslaunch dh_gripper_driver dh_gripper.launch test_run:=true

# RViz 可视化（需 dh_robotics_ag95_description 包）
roslaunch dh_gripper_driver dh_gripper_display.launch
```

### 编程控制示例

```python
pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=50)
msg = GripperCtrl()
msg.initialize = False
msg.position = 500   # Modbus 型号: 0-1000, AG95_CAN: 0-100
msg.force = 100      # 百分比
msg.speed = 100      # 百分比
pub.publish(msg)
```

---

## 5. lifting_ctrl -- 电动升降柱控制

**路径:** `lifting_ctrl/`
**语言:** Python

通过 RS-485 串口（Modbus RTU 协议）控制电动升降柱（线性执行器），用于升降机器人上的机械臂或工具平台。

### 支持的电机型号

| 型号 | 节点 | 编码器类型 |
|------|------|------------|
| 830ABS | `lifting_ctrl_service_node.py` | 绝对值编码器 |
| 850pro | `lifting_ctrl_service_node_850pro.py` | 增量编码器 |

### ROS 接口

**服务:**

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `LiftingMotorService` | `LiftMotorSrv` | 直接电机控制 |
| `LiftingMotorService`（接口层） | `LiftInterfaceSrv` | 多电机路由/复用 |

**话题:**

| 话题 | 类型 | 说明 |
|------|------|------|
| `LiftMotorStatePub` | `LiftMotorMsg` | 电机状态（高度、速度、电流、限位等） |

### 服务模式（mode 参数）

| mode | 功能 | val 参数 |
|------|------|----------|
| 0 | 位置控制 | 目标高度（mm） |
| 1 | 初始化/归零 | 无 |
| -2 | 急停 | 无 |
| -3 | 清除错误 | 无 |
| -4 | 速度控制 | 目标速度（mm/s） |
| -5 | 设置最大速度 | 转速（RPM） |
| -6 | 编码器清零 | 无 |
| -7 | 速度控制（RPM） | 目标转速 |

### 配置文件

每个电机有独立的 JSON 配置文件，位于 `scripts/config/` 目录：

- `1_lifting_motor_config.json` -- 电机 ID 1
- `2_lifting_motor_config.json` -- 电机 ID 2
- `4_lifting_motor_config.json` -- 电机 ID 4

关键参数：`portName`（串口）、`baudRate`（波特率）、`reductionRatio`（脉冲/mm 比）、`motorSpd`（最大转速）、`upLimitVal`/`downLimitVal`（行程限位）、`initMode`（上电初始化方式）。

### 使用方法

```bash
# 单电机启动
roslaunch lifting_ctrl start_motor.launch                    # 830ABS
roslaunch lifting_ctrl start_850pro_motor.launch             # 850pro

# 指定电机 ID 和命名空间
roslaunch lifting_ctrl start_motor.launch motor_id_:=2 lifter_ns:=lifter_2

# 发送控制命令
rosservice call /lifter_1/LiftingMotorService "val: 300
mode: 0"     # 移动到 300mm
rosservice call /lifter_1/LiftingMotorService "val: 0
mode: -2"    # 急停

# 监控状态
rostopic echo /lifter_1/LiftMotorStatePub
```

### 安全特性

- 过流保护（可配置阈值）
- 限位开关保护（上下限位）
- 高度软件限位
- 通信超时自动急停
- 离线检测（串口错误计数 > 4 则判定离线）
- 归零过程堵转检测

### 依赖

- `pyserial`（`pip3 install pyserial`）
- `bt_task_msgs`（本工作空间内，提供消息/服务定义）

---

## 6. bt_task_msgs -- 自定义消息/服务定义

**路径:** `bt_task_msgs/`
**协议:** BSD

纯消息/服务定义包（无可执行代码），为升降机构控制提供 ROS 消息和服务类型。

### 定义的消息

| 消息 | 字段数 | 说明 |
|------|--------|------|
| `LiftMotorMsg` | 28 | 升降电机完整状态（ID、模式、电压、电流、速度、位置、高度、限位、错误码等） |

### 定义的服务

| 服务 | 说明 |
|------|------|
| `LiftMotorSrv` | 底层电机控制（val + mode -> state） |
| `LiftInterfaceSrv` | 上层接口（val + mode + id -> message + success + code） |

### 依赖

- `std_msgs`、`message_generation`、`message_runtime`

---

## 7. rslidar_sdk -- RoboSense 激光雷达驱动

**路径:** `rslidar_sdk/`
**来源:** https://github.com/RoboSense-LiDAR/rslidar_sdk.git
**版本:** 1.5.0
**协议:** BSD 3-Clause

RoboSense 激光雷达官方 ROS 驱动，接收 UDP 数据包并解码为 3D 点云发布到 ROS 话题。

### 支持的雷达型号

RS-LiDAR-16、RS-LiDAR-32、RS-Bpearl、RS-Helios、RS-Helios-16P、RS-Ruby-128/80/48、RS-Ruby-Plus-128/80/48、RS-LiDAR-M1、RS-LiDAR-M2、RS-LiDAR-E1。

### 编译模式

在 `CMakeLists.txt` 中设置 `COMPILE_METHOD`：

| 值 | 说明 |
|----|------|
| `CATKIN` | ROS1 catkin 工作空间编译（当前设置） |
| `COLCON` | ROS2 colcon 工作空间编译 |
| `ORIGINAL` | 独立 CMake 编译 |

### 配置文件

`config/config.yaml` 中配置雷达参数：

```yaml
common:
  msg_source: 1            # 1=在线雷达, 2=ROS 包回放, 3=PCAP 文件
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSHELIOS_16P
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /rslidar_points
```

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 解码后的点云 |

### 使用方法

```bash
# ROS1 catkin 编译
# 1. 设置 COMPILE_METHOD 为 CATKIN
# 2. 将 package_ros1.xml 重命名为 package.xml
# 3. 编译
catkin_make

# 启动
roslaunch rslidar_sdk start.launch    # 同时打开 RViz
```

### 依赖

- `yaml-cpp`（>= 0.5.2）
- `libpcap`（>= 1.7.4）

---

## 8. realsense-ros -- Intel RealSense 深度相机驱动

**路径:** `realsense-ros/`
**版本:** 2.3.2
**协议:** Apache 2.0

Intel RealSense 深度相机的官方 ROS 封装，支持 D400 系列、SR300、L515 深度相机和 T265 追踪模块。

### 包含的 ROS 包

| 包名 | 说明 |
|------|------|
| `realsense2_camera` | 主驱动节点（nodelet），发布深度/彩色/红外/IMU/位姿数据 |
| `realsense2_description` | URDF 模型和 3D 网格文件 |

### 支持的设备

- D400 系列：D415、D435、D435i、D455、D465
- SR300、SR305
- L515、L535 LiDAR 相机
- T265 追踪模块

### 发布的话题（以 D435i 为例）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图像 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图（16位） |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 左红外 |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 右红外 |
| `/camera/gyro/sample` | `sensor_msgs/Imu` | 陀螺仪（D435i） |
| `/camera/accel/sample` | `sensor_msgs/Imu` | 加速度计（D435i） |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云（需启用 filter） |

### 使用方法

```bash
# 基本启动
roslaunch realsense2_camera rs_camera.launch

# 启用点云
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# 深度对齐到彩色
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 多相机
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=<sn1> serial_no_camera2:=<sn2>

# T265 追踪相机
roslaunch realsense2_camera rs_t265.launch

# 相机 3D 模型可视化
roslaunch realsense2_description view_d435_model.launch

# 运行时调参
rosrun rqt_reconfigure rqt_reconfigure
```

### 依赖

- `librealsense2`（>= 2.50.0，SDK 源码见 `librealsense-2.50.0.zip`）
- `cv_bridge`、`image_transport`、`ddynamic_reconfigure`、`diagnostic_updater`

---

## 9. SLAM -- 建图与激光里程计

**路径:** `SLAM/`

激光 SLAM（同时定位与建图）相关算法包的集合，提供 2D 建图、激光扫描匹配和激光里程计功能。

### 包含的 ROS 包

| 包名 | 说明 |
|------|------|
| `openslam_gmapping` | GMapping 核心算法库（粒子滤波 + 栅格地图） |
| `gmapping` | GMapping 的 ROS 封装节点 |
| `slam_gmapping` | 元包，整合上述两个包 |
| `laser_scan_matcher` | 基于 CSM 的增量激光扫描匹配（ICP） |
| `rf2o_laser_odometry` | 基于 Range Flow 的轻量级激光里程计 |

### 节点

| 节点 | 包 | 功能 |
|------|-----|------|
| `slam_gmapping` | gmapping | 在线 2D SLAM（订阅激光扫描 + 里程计 TF） |
| `slam_gmapping_replay` | gmapping | 离线回放 SLAM（从 rosbag 文件） |
| `laser_scan_matcher_node` | laser_scan_matcher | 逐帧扫描匹配，估计增量运动 |
| `rf2o_laser_odometry_node` | rf2o_laser_odometry | 密集激光里程计（0.9ms/帧，极低计算开销） |

### GMapping 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| `/tf` | `tf/tfMessage` | map -> odom 变换 |
| `~entropy` | `std_msgs/Float64` | 位姿熵 |

### GMapping 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~base_frame` | `base_link` | 机器人基座坐标系 |
| `~map_frame` | `map` | 地图坐标系 |
| `~odom_frame` | `odom` | 里程计坐标系 |
| `~particles` | 30 | 粒子数 |
| `~xmin/ymin/xmax/ymax` | -100/100 | 地图范围（米） |
| `~delta` | 0.05 | 地图分辨率（米/像素） |
| `~linearUpdate` | 1.0 | 触发更新的最小线性位移（米） |
| `~angularUpdate` | 0.5 | 触发更新的最小角度位移（弧度） |

### RF2O 激光里程计

- 输入：`/laser_scan`（`sensor_msgs/LaserScan`）
- 输出：`/odom_rf2o`（`nav_msgs/Odometry`）+ TF
- 频率：10Hz
- 特点：不依赖轮式里程计，基于 Range Flow 的密集对齐方法

---

## 10. plugin -- RViz 多目标导航插件

**路径:** `plugin/rviz_navi_multi_goals_pub_plugin/`
**来源:** https://github.com/autolaborcenter/rviz_navi_multi_goals_pub_plugin.git
**包名:** `navi_multi_goals_pub_rviz_plugin`

RViz 面板插件，扩展标准导航功能，支持多目标点顺序巡航和循环导航。

### 功能

1. **多目标点队列** -- 在地图上依次设置多个导航目标，机器人按顺序逐一到达
2. **暂停/取消** -- 取消当前目标后可从下一个目标继续
3. **循环巡航** -- 到达最后一个目标后自动回到第一个目标，无限循环

### 工作原理

- 订阅 `/move_base_simple/goal_temp`（拦截 RViz 的 2D Nav Goal）
- 按队列依次发布到 `/move_base_simple/goal`（move_base 监听的 topic）
- 监听 `/move_base/status` 判断到达状态，自动推进到下一个目标
- 通过 `/visualization_marker` 在地图上绘制箭头、文字和球体标记

### 使用方法

1. 启动导航栈
2. 在 RViz 中加载插件：`Panels -> Add New Panel -> MultiNaviGoalsPanel`
3. 添加 Marker 显示：`Display -> add -> Marker`
4. 重配置 2D Nav Goal 工具：`Tool Property` 中将 topic 改为 `/move_base_simple/goal_temp`
5. 在地图上点击设置目标点，点击 "Start navigation" 开始巡航

### 面板功能

- **建图 Tab** -- 启停 SLAM 建图、保存地图、选择已有地图
- **导航 Tab** -- 设置最大目标数、循环模式、目标点表格、开始/重置/取消按钮

---

## 11. librealsense-2.50.0.zip -- LibRealSense SDK 源码包

**路径:** `librealsense-2.50.0.zip`

Intel RealSense SDK v2.50.0 的源码压缩包。`realsense-ros` 包运行时需要此 SDK（`librealsense2`）作为系统依赖。解压后按其 README 进行编译安装。

---

## 整体架构示意

> 标注 `[当前设备]` 的为本项目实际使用的硬件

```
+------------------+     +------------------+     +------------------+
|   ranger_ros     |     |  TCP-IP-ROS-6AXis|     |   realsense-ros  |
| (Ranger 底盘驱动) |     | (Dobot 机械臂SDK) |     | (RealSense 相机)  |
| [当前设备]        |     | [当前设备: CR10]  |     +--------+---------+
+--------+---------+     +--------+---------+              |
         |                         |                         | USB
         | CAN Bus                 | TCP/IP                  v
         v                         v                +--------+---------+
+--------+---------+     +--------+---------+       |  RealSense 硬件  |
|     ugv_sdk      |     |  Dobot 控制器    |       +------------------+
| (底盘通信SDK库)    |     +------------------+
+------------------+

+------------------+     +------------------+     +------------------+
|    gripper       |     |  lifting_ctrl    |     |    bt_task_msgs  |
| (DH夹爪驱动)      |     | (升降柱控制)      |     | (消息/服务定义)   |
| [当前设备: AG95]  |     +--------+---------+     +------------------+
+--------+---------+              |
         |                         | RS-485/Modbus
         | Serial/TCP              v
         v                +--------+---------+
+--------+---------+      |  电动升降柱电机   |
|  DH-Robotics 夹爪 |      +------------------+
+------------------+

+------------------+     +------------------+     +------------------+
|    rslidar_sdk   |     |      SLAM        |     |     plugin       |
| (RoboSense 雷达)  |     | (gmapping/RF2O)  |     | (RViz多目标插件)  |
+--------+---------+     +--------+---------+     +------------------+
         |                         |
         | UDP                     | 激光扫描 + TF
         v                         v
+--------+---------+     +--------+---------+
|  RoboSense 雷达   |     |   move_base     |
+------------------+     | (Navigation)    |
                         +------------------+
```

---

## 依赖关系

```
ranger_ros
  +-- ugv_sdk
  +-- ranger_msgs

lifting_ctrl
  +-- bt_task_msgs

gripper
  +-- dh_gripper_msgs（内部包）

TCP-IP-ROS-6AXis
  （无外部工作空间依赖）

rslidar_sdk
  （无外部工作空间依赖）

realsense-ros
  +-- librealsense2（系统库）

SLAM
  （无外部工作空间依赖）

plugin
  （无外部工作空间依赖，依赖 rviz、move_base 等系统包）
```

---

## 注意事项

1. **CAN 适配器**：使用 Ranger 底盘前，每次开机需执行 `bringup_can2usb.bash` 启动 CAN 接口
2. **环境变量**：使用 Dobot 机械臂前需设置 `export DOBOT_TYPE=cr5`（或其他型号）
3. **串口权限**：使用夹爪和升降柱时，确保当前用户有串口访问权限（通常需加入 `dialout` 组）
4. **编译顺序**：建议先编译 `bt_task_msgs`，再编译 `lifting_ctrl`（因为后者依赖前者的消息定义）
5. **雷达标定**：使用 `rslidar_sdk` 前，需根据实际雷达型号修改 `config/config.yaml`
6. **RealSense SDK**：`realsense-ros` 需要先安装 `librealsense2 >= 2.50.0`，源码见 `librealsense-2.50.0.zip`
