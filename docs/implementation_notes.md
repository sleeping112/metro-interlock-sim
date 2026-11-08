# 实现约定（阶段 2–5 共用，必须遵守）

本文件把 `stage1_design.md` 的接口落到**可执行的命名/坐标/参数**上。实现时若发现冲突，以本文件为准并回报 Lead。

## 0. 目标与本地环境

- 目标运行环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic（`gz-sim8`）+ `ros_gz`（Humble 对应版本）。
- 当前容器可能是更高版本（例如 ROS 2 Lyrical / gz-sim10）。CMake 允许：
  1. 先 `find_package(gz-sim8 QUIET)`，找到则使用版本化 target；
  2. 找不到时回退 `find_package(gz-sim QUIET)`（本机非版本化 vendor target），仅用于编译/冒烟；
  3. 源码、SDF、launch 的默认值仍以 Humble/Harmonic 为准。
- 不要新增第三方包。不要新增公共话题/服务/消息类型。

## 1. 坐标与几何（世界坐标，单位 m / kg / s）

- 站台：中心 `(0, 0, 0.15)`，尺寸 `50 × 10 × 0.3`，顶面 `z=0.3`，`y ∈ [-5, 5]`。
- 两侧轨道床：中心 `(0, ±6.65, 0.10)`，尺寸 `50 × 2 × 0.20`，顶面 `z=0.20`。
- 列车运行轨：`+y` 侧，中心线 `y=6.65`；车体宽 3 m，因此车体侧面靠站台一侧为 `y=5.15`。
- 屏蔽门门扇平面：`y=5.05`；门框主体平面：`y=5.30`（避免开门时门扇与门柱碰撞重叠）。
- 停车点 `x_stop=0.0`；停车窗 `±0.5 m`。
- 屏蔽门单元中心 `x = -15, -5, +5, +15`，共 4 个，编号 `01..04`。
- 列车初始 spawn 位置 `x=-80.0`（launch 的 `-x -80`）。列车模型内部 `train_body` 的局部原点在 `(0, 6.65, 2.3)`，长宽高 `120 × 3 × 3.8`，底面 `z=0.4`。

## 2. 列车模型（`metro_description/models/metro_train/model.sdf.xacro`）

必须包含：

- `model name="metro_train"`，`<static>false</static>`。
- `link name="train_base"`：无 collision/visual，质量 1 kg；通过 `joint name="train_base_fixed" type="fixed"` 固定到 `world`。该 link 是纵向滑动的固定基座。
- `link name="train_body"`：
  - `<pose>0 6.65 2.3 0 0 0</pose>`；
  - inertial：`mass=30000`，盒体惯量（Ixx=m/12*(3²+3.8²)，Iyy=m/12*(120²+3.8²)，Izz=m/12*(120²+3²)）；
  - collision `train_body_collision`：`<box><size>120 3 3.8</size></box>`；
  - 视觉：`<mesh><uri>model://metro_train/meshes/unit_box.stl</uri><scale>120 3 3.8</scale></mesh>`（mesh 是 1×1×1 单位盒）；
  - 4 个车门碰撞盒 `train_door_collision_01..04`：`1.4 × 0.06 × 2.2`，局部中心
    `(unit_x, 5.11-6.65, 1.5-2.3) = (unit_x, -1.54, -0.80)`，即世界 `y=5.11`。
- `joint name="train_longitudinal" type="prismatic"`：
  - `parent=train_base`, `child=train_body`；
  - axis `1 0 0`，`limit lower=-200 upper=200 effort=200000 velocity=20`；
  - dynamics `damping=0.2`, `friction=0.05`。
- 插件：
  ```xml
  <plugin filename="libmetro_train_plugin.so" name="metro::gazebo::TrainLongitudinal">
    <odom_topic>/metro/train/odom</odom_topic>
    <command_topic>/metro/train/command</command_topic>
    <odom_rate_hz>50.0</odom_rate_hz>
    <max_speed_mps>20.0</max_speed_mps>
    <max_accel_mps2>1.0</max_accel_mps2>
    <max_decel_mps2>2.0</max_decel_mps2>
    <rail_y_m>6.65</rail_y_m>
  </plugin>
  ```
- `model.config` 可省略（launch 直接用 xacro 生成 SDF）；但 `meshes/unit_box.stl` 必须安装。
- 另提供 ROS 侧 `urdf/metro_train.urdf.xacro`：`base_link` + `train_body`（固定/或 prismatic 关节仅用于可视化），尺寸一致；不要求被 launch 使用。

## 3. 世界与屏蔽门（`metro_bringup/worlds/metro_station.world`，SDF 1.8）

### 3.1 世界设置

- `<physics name="metro_physics" type="ignored">`：`max_step_size=0.001`，`real_time_update_rate=100`，`real_time_factor=1.0`。
- 系统插件：Physics、UserCommands、SceneBroadcaster、Sensors（可选 Contact）。
- 光照：1 个 directional light + 环境光。
- 站台、轨道床、安全线、PSD 门框、门扇全部**内联**在 world 中（不要用 `<include>`，因为需要每单元唯一 link/joint 名）。
- 安全线：黄色薄盒 `50 × 0.1 × 0.01`，在 `y=4.85, z=0.305`，**仅 visual，无 collision**。
- 静态物体 collision 用简化盒/凸包，visual 可用 box/mesh。

### 3.2 门框（每个单元一个独立 static model）

`model name="psd_unit_XX"`，`<static>true</static>`，`link name="psd_frame_XX"`：

- 左右门柱：`0.20 (x) × 0.35 (y) × 2.20 (z)`，中心 `(±1.90, 5.30, 1.10)`；
- 门楣：`4.00 × 0.35 × 0.30`，中心 `(0, 5.30, 2.70)`；
- 门槛：`4.00 × 0.35 × 0.05`，中心 `(0, 5.30, 0.325)`；
- 两侧固定玻璃/面板视觉：`1.0 × 0.05 × 2.2` 放在门柱外侧，可省略 collision；
- 所有 collision 用 box；材质颜色可用 SDF `<material><ambient>...`。
- **禁止**在门框内添加会挡住 3.6 m 开口的 collision。

### 3.3 门扇（每个单元一个运动 model）

`model name="psd_door_XX"`，`<static>false</static>`，`allow_auto_disable=false`：

- `link name="psd_base_XX"`：无 visual/collision，质量 1 kg；通过 fixed joint 固定到 world。
- `link name="psd_left_XX"`：
  - pose `(-0.90, 5.05, 1.45)`；
  - collision/visual：`1.80 × 0.06 × 2.20` 的玻璃盒（青色透明）；
  - inertial 质量 120 kg。
- `link name="psd_right_XX"`：pose `(+0.90, 5.05, 1.45)`，其余同上。
- `joint name="psd_left_joint_XX"`：prismatic，parent=`psd_base_XX`，child=`psd_left_XX`，axis x，limit `[-1.8, 0]`；
- `joint name="psd_right_joint_XX"`：prismatic，parent=`psd_base_XX`，child=`psd_right_XX`，axis x，limit `[0, 1.8]`；
- 关节 dynamics：`damping=2.0`，`friction=0.2`；effort 5000，velocity 1.0。
- 初始位置 0（全闭）；左开到位 -1.8，右开到位 +1.8。
- 门扇平面 `y=5.05`，与门框 `y=5.30` 留 0.25 m 层差，确保开门行程不碰门柱。
- 每单元的门扇 link/joint 名必须带两位编号 `_XX`。

### 3.4 DoorController 插件（world 级）

在世界末尾：
```xml
<plugin filename="libmetro_door_controller.so" name="metro::gazebo::DoorController">
  <unit_count>4</unit_count>
  <unit_model_prefix>psd_door_</unit_model_prefix>
  <left_joint_prefix>psd_left_joint_</left_joint_prefix>
  <right_joint_prefix>psd_right_joint_</right_joint_prefix>
  <door_speed_mps>0.8</door_speed_mps>
  <door_travel_m>1.8</door_travel_m>
  <state_rate_hz>10.0</state_rate_hz>
  <lock_delay_s>0.3</lock_delay_s>
  <open_position_tolerance_m>0.01</open_position_tolerance_m>
</plugin>
```
服务名从 `unit_count` 自动生成：`/metro/psd/unit_01/command` 等。

## 4. 插件实现约定

### 4.1 DoorController（`door_controller.cpp`）

- 继承 `gz::sim::System`、`ISystemConfigure`、`ISystemPreUpdate`、`ISystemPostUpdate`。
- `Configure()` 中调用私有 `Load()`：
  - 创建 `rclcpp::Node("metro_psd_door_controller")`；
  - 创建 `/metro/psd/state` publisher（`std_msgs::msg::Bool`，reliable，KeepLast(10)）；
  - 为每个单元创建 `/metro/psd/unit_XX/command` 服务（`std_srvs::srv::SetBool`），**每个单元独立 service**，回调设置该单元目标开/关；
  - 缓存 model/joint 实体（按名字），调用 `Joint::EnablePositionCheck` / `EnableVelocityCheck`；
  - 不阻塞、不 sleep。
- 每个 `PreUpdate` 用 `dt`（`UpdateInfo.dt`）驱动：
  - 目标开：左关节速度 `-door_speed`，右 `+door_speed`；目标关：左 `+door_speed`，右 `-door_speed`；
  - 用 `Joint::Position` 判断到达限位；到位/越界时 `SetVelocity({0.0})`；
  - 服务回调只改目标，不直接改位置。
- `PostUpdate` 以 `state_rate_hz`（默认 10 Hz）发布：
  - `true`：所有门扇位置绝对值 ≤ `open_position_tolerance`，且全部目标为关，且到位后经过 `lock_delay_s`；
  - `false`：任一目标为开或任一位置偏离关闭位。
- 每个日志/回调中标注对应规则（如 R3/R4 的输入源）。
- 析构时释放 ROS node；若启动 executor，必须 join/stop。

### 4.2 TrainLongitudinal（`train_longitudinal.cpp`）

- 继承 `gz::sim::System`、`ISystemConfigure`、`ISystemPreUpdate`、`ISystemPostUpdate`。
- `Load()` 创建 `gz::transport::Node`，advertise `gz.msgs.Odometry` 到 `odom_topic`（默认 `/metro/train/odom`），subscribe `gz.msgs.Twist` 到 `command_topic`（默认 `/metro/train/command`）。
- 只使用 `Twist.linear().x()`；`angular` 忽略。
- `PreUpdate`：
  - 读 `train_longitudinal` 关节当前速度（`JointVelocity` 组件或 `Joint::Velocity`）；
  - 目标速度 = clamp(cmd, ±max_speed)；
  - 按 `max_accel` / `max_decel` 做斜坡；
  - `Joint::SetVelocity({target})`；
  - 命令 0 且速度小于 1e-3 时置 0，避免蠕动。
- `PostUpdate`：以 `odom_rate_hz`（50 Hz）发布 `gz.msgs.Odometry`：
  - `header.frame_id="station"`，`child_frame_id="metro_train/train_body"`；
  - `pose.position = (initial_x + joint_position, rail_y, 0)`，姿态单位四元数；
  - `twist.linear.x = actual joint velocity`；
  - 时间戳用 `_info.simTime`，不用系统时钟。
- `Configure` 时由 `worldPose(_entity, _ecm)` 记录 `initial_x`（spawn 的 x），不要硬编码 -80。
- 不要创建 ROS 节点；通过 ros_gz_bridge 与 ROS 交互。

## 5. 应用节点约定

### 5.1 `interlock_node`

- 节点名 `interlock_node`，`use_sim_time=true`。
- 声明参数并打印生效值：
  - `stop_window_m` 0.5；`psd_open_timeout_s` 3.0；`emergency_speed_threshold_mps` 0.1；
  - `fault_hold_s` 1.0；`fault_repeat_suppression_s` 5.0；
  - `control_rate_hz` 10.0；`auto_run` true；`manual_speed_mps` 0.0；`x_stop_m` 0.0；
  - `psd_unit_count` 4；`cruise_speed_mps` 8.0；`brake_decel_mps2` 1.0；`approach_margin_m` 0.5；
  - `command_topic` 等话题名参数可固定，不强制。
- 订阅：
  - `/metro/train/odom`，`rclcpp::SensorDataQoS()`（best_effort），KeepLast(10)；
  - `/metro/psd/state`，reliable KeepLast(10)。
- 服务：
  - `/metro/interlock/enable` (`std_srvs/SetBool`)：切换 enable；false 时立即向所有单元发关门并清故障状态（R1）；
  - `/metro/psd/command` (`std_srvs/SetBool`)：开门前检查 enable + 停车窗；不在窗内返回 success=false、message 含 `TRAIN_NOT_IN_STOP_WINDOW` 并上报故障；通过则向全部单元发 true 并记录开门时刻。
- 发布：
  - `/metro/train/command` (`geometry_msgs/Twist`) 10 Hz；`linear.x` 为安全后速度，`angular=0`；
  - `/metro/interlock/fault` (`std_msgs/String`) 10 Hz 控制循环内由 `FaultManager` 管理。
- 控制循环 10 Hz（`create_wall_timer` 用 `get_clock()`）：按 R1→R2（请求时）→R3→R4→R5 顺序计算；R5 与 R3 同时成立时故障码用 `EMERGENCY_BRAKE_PSD_OPEN`。
- 速度：自动模式下用 `TrainSpeedController` 库，输入 odom.x/v；手动模式 `auto_run=false` 时用 `manual_speed_mps`（参数可在运行时 `ros2 param set`）。
- 向单元 service 扇出：为每个单元建 `rclcpp::Client<std_srvs::srv::SetBool>`，服务不可用时缓存待发命令，下一周期重试；不要阻塞 spin。
- 所有超时/周期判断用 `get_clock()->now()` 和 `rclcpp::Duration`。

### 5.2 `train_speed_controller_node`

- 节点名 `train_speed_controller_node`，独立可执行文件；默认 `publish_command=false`，避免与联锁双发布。
- 同名参数 `x_stop_m, cruise_speed_mps, brake_decel_mps2, approach_margin_m, max_speed_mps, auto_run, manual_speed_mps`。
- 订阅 `/metro/train/odom`（SensorDataQoS），以 10 Hz 发布 `/metro/train/command`（仅当 `publish_command=true`）。
- 打印警告：主 launch 不启动本节点；联锁节点内置同一速度控制律。
- 速度算法与联锁共用 `TrainSpeedController` 库。

### 5.3 FaultManager

- 输入 `rclcpp::Clock::SharedPtr`、publisher、`fault_hold_s`、`fault_repeat_suppression_s`。
- `raise(code)`：
  - 若该 code 在 `fault_repeat_suppression_s` 内发布过，跳过发布；
  - 否则发布 code，记录 `last_publish[code]`，激活保持到 now+hold。
- `tick()`：保持期内每 10 Hz 重发同一 code；到期发布 `""` 清空。
- 故障码固定集合；空串表示无活动故障。
- 同一时刻只保持一个活动故障码；更高优先级故障可替换。

## 6. bringup 约定

- 包 `metro_bringup` 依赖 `launch`, `launch_ros`, `xacro`, `ros_gz_sim`, `ros_gz_bridge`, `rviz2`, `rclpy`, 以及三个本地包。
- launch 中设置：
  - `GZ_SIM_RESOURCE_PATH` 追加 `<install>/metro_description/share/models`（让 `model://metro_train/meshes/...` 可解析）；
  - `GZ_SIM_SYSTEM_PLUGIN_PATH` 追加 `<install>/metro_gazebo/lib`。
- `bridge.yaml` 条目：
  ```yaml
  - ros_topic_name: "/clock"
    gz_topic_name: "/clock"
    ros_type_name: "rosgraph_msgs/msg/Clock"
    gz_type_name: "gz.msgs.Clock"
    direction: GZ_TO_ROS
  - ros_topic_name: "/metro/train/odom"
    gz_topic_name: "/metro/train/odom"
    ros_type_name: "nav_msgs/msg/Odometry"
    gz_type_name: "gz.msgs.Odometry"
    direction: GZ_TO_ROS
    qos_profile: sensor_data
  - ros_topic_name: "/metro/train/command"
    gz_topic_name: "/metro/train/command"
    ros_type_name: "geometry_msgs/msg/Twist"
    gz_type_name: "gz.msgs.Twist"
    direction: ROS_TO_GZ
  ```
- launch 读取 YAML 生成 `parameter_bridge` 的 CLI 参数（Humble 兼容）；`qos_profile` 字段在支持 YAML 配置的版本可用，CLI 模式下至少保证话题/类型/方向正确。
- `metro_sim.launch.py`：Gazebo world + spawn train（xacro→SDF string）+ bridge + 可选 RViz。
- `metro_demo.launch.py`：include sim + interlock + 参数 YAML + RViz。
- `acceptance.launch.py`：include demo，`auto_run=false`，启动 `acceptance_test.py`（延迟 3 s）。
- RViz fixed frame `station`，显示 Odometry `/metro/train/odom`、Grid、TF。

## 7. 文件所有权（避免并行写冲突）

| 所有者 | 写范围 |
|---|---|
| desc-builder | `ws_metro/src/metro_description/**`、`ws_metro/src/metro_bringup/worlds/**` |
| gazebo-builder | `ws_metro/src/metro_gazebo/**` |
| app-builder | `ws_metro/src/metro_application/**` |
| lead | `docs/**`、`scripts/**`、`README.md`、`ws_metro/src/metro_bringup/{package.xml,CMakeLists.txt,launch/**,config/**,rviz/**,scripts/**}` |

`package.xml` / `CMakeLists.txt` 由 Lead 先创建；构建源文件可在范围内按需调整，但不要改公共话题/服务/名字。
