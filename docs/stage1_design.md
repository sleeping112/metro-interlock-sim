# 单线地铁站台—列车—屏蔽门联锁仿真：阶段 1 设计基线

> 本文件是阶段 1 的交付物，也是后续所有实现的唯一接口基线。
> 目标栈固定为 **ROS 2 Humble + Gazebo Harmonic (gz-sim8) + ros_gz_bridge/ros_gz_sim + colcon/ament_cmake + C++17**。

## 1. 工作空间目录树

```text
togazebo/
├── README.md
├── docs/
│   ├── stage1_design.md              # 本文件（阶段 1）
│   ├── implementation_notes.md       # 实现约定（模型/关节/参数名）
│   └── acceptance.md                 # 端到端验收说明（阶段 5）
├── scripts/
│   ├── build.sh
│   ├── run_demo.sh
│   ├── run_headless.sh
│   └── verify_workspace.py
└── ws_metro/
    └── src/
        ├── metro_description/
        │   ├── package.xml
        │   ├── CMakeLists.txt
        │   ├── urdf/
        │   │   └── metro_train.urdf.xacro       # ROS 侧列车描述（TF/几何）
        │   ├── models/
        │   │   └── metro_train/
        │   │       ├── model.config
        │   │       └── model.sdf.xacro          # Gazebo 侧列车 SDF（插件、关节、碰撞）
        │   ├── meshes/
        │   │   ├── unit_box.stl                 # 单位盒网格，SDF 中 scale 使用
        │   │   └── unit_cylinder.stl
        │   └── config/
        │       └── train_geometry.yaml          # 几何/质量参数单一真源（供文档与校验）
        ├── metro_gazebo/
        │   ├── package.xml
        │   ├── CMakeLists.txt
        │   ├── include/metro_gazebo/
        │   │   ├── door_controller.hpp
        │   │   └── train_longitudinal.hpp
        │   └── src/
        │       ├── door_controller.cpp
        │       └── train_longitudinal.cpp
        ├── metro_application/
        │   ├── package.xml
        │   ├── CMakeLists.txt
        │   ├── include/metro_application/
        │   │   ├── interlock_node.hpp
        │   │   ├── fault_manager.hpp
        │   │   ├── train_speed_controller.hpp
        │   │   └── train_speed_controller_node.hpp
        │   └── src/
        │       ├── interlock_node.cpp
        │       ├── fault_manager.cpp
        │       ├── train_speed_controller.cpp
        │       ├── train_speed_controller_node.cpp
        │       └── speed_controller_main.cpp
        └── metro_bringup/
            ├── package.xml
            ├── CMakeLists.txt
            ├── worlds/
            │   └── metro_station.world          # SDF 1.8，站台/轨道/屏蔽门/门扇/插件
            ├── launch/
            │   ├── metro_sim.launch.py
            │   ├── metro_demo.launch.py
            │   └── acceptance.launch.py
            ├── config/
            │   ├── interlock_params.yaml
            │   ├── train_speed_params.yaml
            │   ├── bringup_params.yaml
            │   └── bridge.yaml
            ├── rviz/
            │   └── metro.rviz
            └── scripts/
                ├── acceptance_test.py
                └── inspect_interfaces.py
```

## 2. 各包 package.xml 摘要与 CMakeLists.txt 摘要

### 2.1 metro_description

`package.xml` 关键内容：

| 类别 | 依赖 |
|---|---|
| buildtool | `ament_cmake` |
| build/exec | `ament_cmake`, `xacro` |
| 运行时 | `ros_gz_sim`（模型资源安装/复用） |

`CMakeLists.txt` 关键内容：

```cmake
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf models meshes config
        DESTINATION share/${PROJECT_NAME})
ament_package()
```

职责：只提供模型/几何参数，不包含插件代码，不发布话题。

### 2.2 metro_gazebo

`package.xml` 关键内容：

| 类别 | 依赖 |
|---|---|
| buildtool | `ament_cmake` |
| build | `rclcpp`, `std_msgs`, `std_srvs`, `nav_msgs`, `geometry_msgs`, `gz-sim8`, `gz-transport12`, `gz-msgs9`, `gz-plugin2` |
| exec | `ros_gz_sim`, `rclcpp`, `std_msgs`, `std_srvs`, `nav_msgs`, `geometry_msgs` |

`CMakeLists.txt` 关键内容：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)          # 本地开发时允许 gz-sim9/10 回退
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(metro_door_controller SHARED src/door_controller.cpp)
add_library(metro_train_plugin SHARED src/train_longitudinal.cpp)
# 输出文件名固定为 libmetro_door_controller.so / libmetro_train_plugin.so
# 安装到 lib/，launch 中把该目录加入 GZ_SIM_SYSTEM_PLUGIN_PATH
```

职责：屏蔽门执行机构控制、列车纵向一维运动、Gazebo 真值里程计。

### 2.3 metro_application

`package.xml` 关键内容：

| 类别 | 依赖 |
|---|---|
| buildtool | `ament_cmake` |
| build/exec | `rclcpp`, `std_msgs`, `std_srvs`, `geometry_msgs`, `nav_msgs`, `rosgraph_msgs` |

`CMakeLists.txt` 关键内容：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(metro_application_core SHARED
  src/fault_manager.cpp src/train_speed_controller.cpp)
target_include_directories(...)
add_executable(interlock_node src/interlock_node.cpp)
add_executable(train_speed_controller_node src/speed_controller_main.cpp)
```

职责：安全联锁、故障管理、列车速度曲线；不包含 Gazebo 直接依赖。

### 2.4 metro_bringup

`package.xml` 关键内容：

| 类别 | 依赖 |
|---|---|
| buildtool | `ament_cmake` |
| exec | `launch`, `launch_ros`, `metro_description`, `metro_gazebo`, `metro_application`, `ros_gz_sim`, `ros_gz_bridge`, `rviz2`, `xacro`, `rclpy` |

`CMakeLists.txt` 关键内容：

```cmake
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/acceptance_test.py scripts/inspect_interfaces.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
```

职责：装配世界、启动 Gazebo、桥接、参数、RViz 与验收脚本；不含业务算法。

## 3. 接口契约表

### 3.1 话题

| 话题 | 类型 | 方向（相对联锁） | QoS | 频率 | 说明 |
|---|---|---|---|---|---|
| `/metro/train/command` | `geometry_msgs/msg/Twist` | 联锁 → 列车插件 | reliable, KeepLast(10) | 10 Hz | 只使用 `linear.x`，`angular` 忽略；联锁是唯一发布者 |
| `/metro/train/odom` | `nav_msgs/msg/Odometry` | Gazebo 真值 → 联锁/监视 | **sensor_data / best_effort**, KeepLast(10) | 50 Hz | 站台坐标系；`frame_id=station` |
| `/metro/psd/state` | `std_msgs/msg/Bool` | 屏蔽门插件 → 联锁 | reliable, KeepLast(10) | 10 Hz | `true` = 全部门扇已到关闭位并锁紧 |
| `/metro/interlock/fault` | `std_msgs/msg/String` | 联锁 → 监视 | reliable, KeepLast(10) | 事件触发 | 故障码保持 1 s 后发空串清除；同码 5 s 去重 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → 全节点 | reliable, KeepLast(1) | 仿真时钟 | `ros_gz_bridge` 桥接；所有节点 `use_sim_time=true` |

### 3.2 服务

| 服务 | 类型 | 服务端 | 语义 |
|---|---|---|---|
| `/metro/psd/command` | `std_srvs/srv/SetBool` | 联锁节点 | 总开关；`true`=请求开门，`false`=请求关门。开门请求先过 R1/R2 |
| `/metro/psd/unit_01/command` … `/metro/psd/unit_04/command` | `std_srvs/srv/SetBool` | DoorController 插件 | 单门单元执行机构；`true`=开，`false`=关。每单元独立 service，禁止合并 |
| `/metro/interlock/enable` | `std_srvs/srv/SetBool` | 联锁节点 | 联锁使能；`true`=投入，`false`=退出 |

服务 QoS：默认 reliable（`std_srvs` 服务不单独声明 QoS，ROS 2 服务默认 reliable）。

### 3.3 命名与编号

- 屏蔽门单元数 `N=4`，编号 `01..04`，两位十进制。
- 单元中心 x 坐标：`-15.0, -5.0, +5.0, +15.0` m；门扇中心线 `y=5.00` m。
- 服务/模型/关节名严格使用两位编号，例如 `psd_door_03`、`psd_left_joint_03`。

## 4. 联锁状态转移表

### 4.1 规则定义（优先级从高到低）

| 规则 | 条件 | 动作 | 故障 |
|---|---|---|---|
| R1 | `enable == false` | `/metro/psd/command` 保持关门；列车指令透传期望速度；不产生故障 | 无 |
| R2 | 收到开门请求且 `|x - x_stop| > 0.5 m` | 拒绝开门请求（service 返回 `success=false`）；屏蔽门保持关闭 | `TRAIN_NOT_IN_STOP_WINDOW` |
| R3 | `/metro/psd/state == false` | 列车指令强制 0，牵引封锁 | `PSD_NOT_LOCKED` |
| R4 | 开门指令发出后 3 s 内 `/metro/psd/state` 仍为 `true` | 列车指令强制 0，保持牵引封锁 | `PSD_OPEN_TIMEOUT` |
| R5 | 列车速度 `> 0.1 m/s` 且 `/metro/psd/state == false` | 紧急制动：列车指令强制 0 | `EMERGENCY_BRAKE_PSD_OPEN` |

**R3/R5 冲突处理（安全优先）：** 两者可同时成立时，控制动作一致（强制 0）；故障码上报更具体的 R5 `EMERGENCY_BRAKE_PSD_OPEN`。R5 是紧急安全分支，不改变 R3 的牵引封锁动作，只覆盖故障码优先级。该处理在 `docs/implementation_notes.md` 与代码注释中显式说明。

### 4.2 决策顺序（每个 10 Hz 控制周期）

```text
1. 若 !enable：关门、期望速度透传、清除故障 → 结束
2. 处理待处理开门请求：若 |x-x_stop|>0.5，拒绝 + R2 故障 → 结束
3. 若 !psd_state：列车指令=0；若 |v|>0.1 → R5 故障，否则 R3 故障
4. 若开门计时器超时且 psd_state 仍为 true：列车指令=0 + R4 故障
5. 否则：按速度曲线输出期望速度
```

### 4.3 状态转移表

| 当前状态 | 事件 | 条件 | 下一状态 | 输出 |
|---|---|---|---|---|
| `DISABLED` | `enable=true` | — | `IDLE_CLOSED` | 无故障；门保持关 |
| `IDLE_CLOSED` | 开门请求 | 在停车窗内 | `OPENING` | 向各单元发 open；启动 3 s 计时 |
| `IDLE_CLOSED` | 开门请求 | 不在停车窗内 | `IDLE_CLOSED` | 拒绝 + `TRAIN_NOT_IN_STOP_WINDOW` |
| `OPENING` | `psd_state=false` | — | `OPEN` | 取消/满足 R4 计时；牵引封锁（R3） |
| `OPENING` | 3 s 超时且 `psd_state=true` | — | `OPENING_FAULT` | 保持牵引封锁 + `PSD_OPEN_TIMEOUT` |
| `OPEN` | 关门请求 | — | `CLOSING` | 向各单元发 close |
| `CLOSING` | `psd_state=true` | — | `IDLE_CLOSED` | 解除 R3/R4；允许牵引 |
| `任何使能状态` | `|v|>0.1 && psd_state=false` | R5 | `EMERGENCY` | 列车指令 0 + `EMERGENCY_BRAKE_PSD_OPEN` |
| `EMERGENCY` | `psd_state=true` 且 `|v|<=0.1` | — | `IDLE_CLOSED` | 故障保持 1 s 后清除 |
| 任何状态 | `enable=false` | R1 | `DISABLED` | 关门、透传期望速度、无故障 |

### 4.4 故障码生命周期

1. 上升沿发布一次故障码，并在 1 s 内以 10 Hz 保持；1 s 后发布空串清除。
2. 同一故障码 5 s 内不重复发布（`last_publish + 5 s` 去重）。
3. 故障动作（牵引封锁）由**当前条件**决定，不依赖故障字符串是否仍在保持期。
4. 故障码集合固定为：`TRAIN_NOT_IN_STOP_WINDOW`、`PSD_NOT_LOCKED`、`PSD_OPEN_TIMEOUT`、`EMERGENCY_BRAKE_PSD_OPEN`。

## 5. 模型、场景与参数基线

### 5.1 列车

| 项 | 值 |
|---|---|
| 模型名 | `metro_train` |
| 车体 | 长 120 m × 宽 3 m × 高 3.8 m，质量 30 t |
| 运动 | 一维纵向 prismatic joint（`train_longitudinal`，axis=x） |
| 初始位置 | `x=-80 m`（由 launch spawn 指定），停车点 `x_stop=0.0 m` |
| 碰撞 | `train_body_collision`（120×3×3.8）+ 4 个车门碰撞盒（1.4×0.06×2.2） |
| 插件 | `libmetro_train_plugin.so` / `metro::gazebo::TrainLongitudinal` |
| 速度范围 | `[-20, +20] m/s`，默认巡航 8 m/s |

### 5.2 站台世界

| 项 | 值 |
|---|---|
| 世界 | `metro_station.world`，SDF 1.8 |
| 站台地面 | 50 m × 10 m × 0.3 m，`<static>true</static>`，中心 `(0,0,0.15)` |
| 两侧轨道床 | 各 50 m × 2 m × 0.2 m，`<static>true</static>`，中心 `(0,±6.65,0.10)` |
| 安全线 | 仅 visual（黄色条带），无 collision |
| 屏蔽门框架 | `psd_unit_01..04` 独立模型，`<static>true</static>` |
| 滑动门扇 | `psd_door_01..04` 模型内 `psd_left_XX`/`psd_right_XX`，prismatic，行程 1.8 m |
| 物理步长 | 0.001 s |
| 实时更新率 | 100 Hz |
| 碰撞 | 静态物体用简化凸包/盒体 collision；视觉可用网格 |

### 5.3 可调参数（默认值写入 `metro_bringup/config/*.yaml`）

| 参数 | 默认值 | 所属 |
|---|---|---|
| `stop_window_m` | 0.5 | 联锁 |
| `psd_open_timeout_s` | 3.0 | 联锁 |
| `emergency_speed_threshold_mps` | 0.1 | 联锁 |
| `fault_hold_s` | 1.0 | 联锁 |
| `fault_repeat_suppression_s` | 5.0 | 联锁 |
| `control_rate_hz` | 10.0 | 联锁 |
| `auto_run` | true | 联锁 |
| `manual_speed_mps` | 0.0 | 联锁（手动测试） |
| `x_stop_m` | 0.0 | 联锁 |
| `cruise_speed_mps` | 8.0 | 速度控制 |
| `brake_decel_mps2` | 1.0 | 速度控制 |
| `approach_margin_m` | 1.5 | 速度控制 |
| `max_accel_mps2` | 1.0 | 列车插件 |
| `max_decel_mps2` | 2.0 | 列车插件 |
| `psd_unit_count` | 4 | 联锁 + 门插件 |

## 6. 实现约定（供阶段 2–5 使用）

1. **单一指令源**：`interlock_node` 是 `/metro/train/command` 的唯一发布者。`train_speed_controller_node` 是台架/手动工具，默认 `publish_command=false`，不参与主 launch，避免双发布。主 launch 中联锁内置同一 `TrainSpeedController` 库完成自动停车曲线。
2. **服务归属**：总开关 `/metro/psd/command` 在联锁节点（安全前置条件 R1/R2）；单门 service 在 DoorController 插件（执行机构）。联锁对总开关命令扇出到各单元 service。
3. **里程计桥接**：列车插件只发 gz `gz.msgs.Odometry`；`bridge.yaml` 以 GZ→ROS 桥到 `/metro/train/odom`。命令以 ROS→GZ 方向桥到列车插件。
4. **仿真时间**：所有节点 `use_sim_time=true`；联锁/速度控制用 `get_clock()->now()` + `rclcpp::Duration`；插件周期用 `UpdateInfo.simTime`，不得用系统时钟。
5. **插件加载**：`Load()`（内部由 `Configure()` 调用）中创建 ROS publisher/service 与 gz transport 端点；构造/`Configure` 不阻塞、不 sleep。
6. **安装路径**：插件安装到 `lib/`；launch 将 `install/metro_gazebo/lib` 注入 `GZ_SIM_SYSTEM_PLUGIN_PATH`，将 `install/metro_description/share` 注入 `GZ_SIM_RESOURCE_PATH`。
7. **本地开发回退**：当前容器实际版本可能高于目标栈（例如 Lyrical/gz-sim10）。CMake 先查找目标版本 `gz-sim8`，找不到时才回退到本机已安装的 `gz-sim9/10`，仅用于编译/冒烟验证；所有源码与 SDF 仍以 Humble/gz-sim8 为目标。

## 7. 阶段划分与验收

| 阶段 | 交付 | 验收 |
|---|---|---|
| 1 | 本文件：目录、清单、接口契约、状态转移表 | 基线冻结，无实现代码 |
| 2 | `metro_description` + `metro_station.world` | `gz sdf -k` 通过；模型/关节/碰撞/质量可查 |
| 3 | `metro_gazebo` 两个插件 | colcon 编译通过；`ros2 service list` 出现 5 个 PSD 服务；`/metro/psd/state` 10 Hz |
| 4 | `metro_application` 两节点 | 规则 R1–R5 单测/集成通过；故障码与保持/去重正确 |
| 5 | `metro_bringup` | 一条 launch 启动全链路；验收场景脚本通过 |
