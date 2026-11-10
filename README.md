# 单线地铁站台—列车—屏蔽门联锁仿真

ROS 2 + Gazebo 的地铁站台屏蔽门（PSD）—列车联锁仿真工作空间。目标技术栈为
**ROS 2 Humble + Gazebo Harmonic（gz-sim8）+ ros_gz_bridge/ros_gz_sim +
colcon/ament_cmake + C++17**，源码位于 `ws_metro/src`。

本仓库按需求分五个阶段实现，阶段 1 的接口基线见
[docs/stage1_design.md](docs/stage1_design.md)，实现约定见
[docs/implementation_notes.md](docs/implementation_notes.md)，验收说明见
[docs/acceptance.md](docs/acceptance.md)。阶段 5 之后完成了 A/B/C 场景丰富化，
详见 [docs/scene_enhancement.md](docs/scene_enhancement.md)。

## 1. 快速开始

```bash
# 1) 构建（脚本自动优先使用 humble；本机验证环境为 lyrical）
bash scripts/build.sh
# 也可显式指定：ROS_DISTRO=lyrical bash scripts/build.sh

# 2) 图形化完整场景：Gazebo + RViz + 乘客/到发/相机
bash scripts/run_demo.sh

# 3) 无界面完整场景（CI / 远程）
bash scripts/run_headless.sh

# 4) 完整场景：130 m 车站 + 12 PSD + 乘客 + 到发场景 + 相机/监控
bash scripts/run_scene.sh

# 5) 自动验收（无界面，启动全链路并运行 21 项检查）
bash scripts/run_acceptance.sh
```

也可以手动执行：

```bash
source /opt/ros/humble/setup.bash
cd ws_metro
colcon build --symlink-install
source install/setup.bash
ros2 launch metro_bringup metro_demo.launch.py
ros2 launch metro_bringup acceptance.launch.py
```

验收脚本只使用契约中的话题/服务：先验证 R1/R2，再让列车自动停车，验证
R3/R4/R5，最后关门清场。通过时打印：

```text
acceptance summary: 21/21 checks passed
ACCEPTANCE_RESULT=PASS
```

## 2. 工作空间结构

```text
togazebo/
├── README.md
├── docs/
│   ├── stage1_design.md          # 阶段 1：目录/清单/接口契约/状态转移表
│   ├── implementation_notes.md   # 阶段 2–5 实现约定
│   ├── scene_enhancement.md      # A/B/C 场景丰富化说明
│   └── acceptance.md             # 端到端验收场景与结果
├── scripts/
│   ├── build.sh
│   ├── run_demo.sh
│   ├── run_scene.sh
│   ├── run_headless.sh
│   ├── run_acceptance.sh
│   ├── ros_env.sh                # ROS 发行版自动探测
│   ├── generate_primitives.py    # 生成 1 m 单位 STL
│   ├── generate_world.py         # 生成 12 PSD/车站/乘客/相机的完整世界
│   ├── generate_station_textures.py # 程序化站名/线路图/出口标识
│   └── verify_workspace.py       # 静态自检
└── ws_metro/
    └── src/
        ├── metro_description/    # 列车 Xacro/URDF、PSD SDF、网格、几何参数
        ├── metro_gazebo/         # DoorController + TrainLongitudinal 插件
        ├── metro_application/    # 联锁节点、故障管理、速度控制节点
        └── metro_bringup/        # launch、bridge/参数 YAML、RViz、验收脚本
```

## 3. 接口契约

### 3.1 话题

| 话题 | 类型 | QoS | 频率 | 说明 |
|---|---|---|---|---|
| `/metro/train/command` | `geometry_msgs/msg/Twist` | reliable, depth 10 | 10 Hz | 只使用 `linear.x`；联锁是唯一发布者 |
| `/metro/train/odom` | `nav_msgs/msg/Odometry` | **sensor_data / best_effort** | 50 Hz | Gazebo 真值，`frame_id=station` |
| `/metro/psd/state` | `std_msgs/msg/Bool` | reliable, depth 10 | 10 Hz | `true` = 全部门扇关闭并锁紧 |
| `/metro/interlock/fault` | `std_msgs/msg/String` | reliable, depth 10 | 事件触发 | 保持 1 s；同码 5 s 去重；空串清除 |
| `/clock` | `rosgraph_msgs/msg/Clock` | reliable | 仿真时钟 | 由 `ros_gz_bridge` 从 Gazebo 桥接 |
| `/metro/platform/intrusion` | `std_msgs/msg/Bool` | reliable, depth 10 | 10 Hz | C 阶段站台边缘入侵检测 |
| `/metro/monitor/markers` | `visualization_msgs/msg/MarkerArray` | reliable, depth 10 | 10 Hz | C 阶段 RViz 监控显示 |
| `/metro/camera/platform/image` | `sensor_msgs/msg/Image` | sensor data | ~10 Hz | C 阶段站台相机 |
| `/metro/camera/platform/camera_info` | `sensor_msgs/msg/CameraInfo` | sensor data | ~10 Hz | 相机内参 |

### 3.2 服务

| 服务 | 类型 | 服务端 | 语义 |
|---|---|---|---|
| `/metro/psd/command` | `std_srvs/srv/SetBool` | 联锁节点 | 总开关；`true` 开门先过 R1/R2 |
| `/metro/psd/unit_01/command` … `unit_12/command` | `std_srvs/srv/SetBool` | DoorController 插件 | 每个单元独立服务，低层执行机构 |
| `/metro/interlock/enable` | `std_srvs/srv/SetBool` | 联锁节点 | 联锁使能 |

`/metro/train/odom` 的桥接发布端已通过 bridge 的
`qos_overrides./metro/train/odom.publisher.reliability=best_effort` 参数覆盖为
best effort；`ros2 topic info -v` 可检查。

## 4. 联锁规则

| 规则 | 条件 | 动作 | 故障码 |
|---|---|---|---|
| R1 | 联锁未使能 | 门保持关闭、列车指令透传、不报故障 | — |
| R2 | 不在停车窗 `|x−x_stop|≤0.5 m` 时请求开门 | 拒绝开门、门保持关闭 | `TRAIN_NOT_IN_STOP_WINDOW` |
| R3 | 门未全闭锁紧 (`state=false`) | 列车指令强制 0 | `PSD_NOT_LOCKED` |
| R4 | 开门后 3 s 状态仍为 `true` | 牵引封锁、列车指令 0 | `PSD_OPEN_TIMEOUT` |
| R5 | 速度 > 0.1 m/s 且门未全闭 | 紧急制动、列车指令 0 | `EMERGENCY_BRAKE_PSD_OPEN` |

故障发布：上升沿发布；保持期内 10 Hz 重发；1 s 后发空串清除；同一故障码
5 s 内不重复发布。R3 与 R5 同时成立时控制动作均为 0，故障码取更具体的 R5；
R5 解除后若门仍未关闭，再上报 R3（故障保持期不会被打断）。

## 5. 模型与场景

- 列车：长 120 m、宽 3 m、高 3.8 m、质量 30 t；视觉上 6 节 × 20 m 车厢，
  `train_base` 固定于世界，`train_body` 由 `train_longitudinal` prismatic
  关节约束在 x 轴；包含车体和 12 个车门碰撞盒，车体中心线 y=8.65 m。
  由 `libmetro_train_plugin.so` 驱动并以 50 Hz 发布 Gazebo 里程计。
- 站台：**130 × 14 × 0.3 m 岛式站台**，顶面 z=0.3；两侧轨道床各
  **180 × 2 × 0.2 m**，轨道中心 y=±8.65 m，全部 static；安全线/盲道仅视觉。
- 屏蔽门：**12 个单元**，中心 x = −55…+55 m，间距 10 m；`psd_unit_XX` 为独立
  static 框架模型；`psd_door_XX` 中 `psd_left_XX`/`psd_right_XX` 为 prismatic
  滑动门扇，行程 1.8 m。
- 场景资产：吊顶/柱网/灯带、闸机、座椅、售票机、站名/线路图/出口标识、
  隧道、道砟/轨枕/接触网、12 个乘客、对向背景列车、站台相机。
- 世界物理：`max_step_size=0.001 s`，`real_time_update_rate=100 Hz`，SDF 1.8。

## 6. 参数

所有应用层可调参数通过 ROS 2 parameter 暴露，默认值在
`ws_metro/src/metro_bringup/config/*.yaml`：

| 参数 | 默认值 | 位置 |
|---|---|---|
| `stop_window_m` | 0.5 | interlock_params.yaml |
| `psd_open_timeout_s` | 3.0 | interlock_params.yaml |
| `emergency_speed_threshold_mps` | 0.1 | interlock_params.yaml |
| `fault_hold_s` | 1.0 | interlock_params.yaml |
| `fault_repeat_suppression_s` | 5.0 | interlock_params.yaml |
| `control_rate_hz` | 10.0 | interlock_params.yaml |
| `auto_run` | true | interlock_params.yaml |
| `manual_speed_mps` | 0.0 | interlock_params.yaml |
| `x_stop_m` | 0.0 | interlock_params.yaml |
| `cruise_speed_mps` | 8.0 | interlock_params.yaml |
| `brake_decel_mps2` | 1.0 | interlock_params.yaml |
| `approach_margin_m` | 0.5 | interlock_params.yaml |
| `psd_unit_count` | 12 | interlock_params.yaml |

运行时手动测试：

```bash
ros2 param set /interlock_node auto_run false
ros2 param set /interlock_node manual_speed_mps 2.0
```

## 7. 单指令源设计（重要）

`interlock_node` 是 `/metro/train/command` 的唯一发布者：它内置
`TrainSpeedController` 用于自动停车曲线，并在同一节点内执行 R3/R4/R5 的
牵引封锁。`train_speed_controller_node` 作为独立的台架/手动工具提供，默认
`publish_command=false`，避免两个节点同时写安全指令；需要单独使用时：

```bash
ros2 run metro_application train_speed_controller_node --ros-args \
  -p publish_command:=true -p use_sim_time:=true
```

这与“联锁输出 `/metro/train/command`”的契约一致，也满足“R1 透传”的语义：
联锁关闭时把期望速度直接透传，不追加故障。

## 8. 实现与验证状态

| 阶段 | 交付 | 状态 |
|---|---|---|
| 1 | 目录/清单/接口契约/状态转移表 | ✅ `docs/stage1_design.md` |
| 2 | 列车 Xacro/URDF、PSD SDF、`metro_station.world` | ✅ `gz sdf -k` 通过 |
| 3 | DoorController、TrainLongitudinal 插件 | ✅ colcon 编译通过；12 个单元服务 |
| 4 | 联锁节点、故障管理、速度控制节点 | ✅ 规则 R1–R5 与故障生命周期 |
| 5 | launch/bridge/参数/RViz/验收脚本 | ✅ 21/21 验收通过 |
| A | 130 m 站台、12 PSD、6 车厢、车站资产/灯光/标识 | ✅ `gz sdf -k` 通过 |
| B | 到站—停靠—发车场景、12 乘客、对向列车 | ✅ 场景链路实测 |
| C | 站台入侵检测、MarkerArray 监控、站台相机 | ✅ 图像/入侵话题实测 |

本机验证环境的说明：仓库目标是最新的 ROS 2 Humble + Gazebo Harmonic
(gz-sim8)，但当前开发容器实际提供 ROS 2 Lyrical + gz-sim10。CMake 先查找
`gz-sim8`，找不到时才回退到本机无版本 vendor target（源码/SDF/launch 仍以
Humble/Harmonic 为目标）；端到端验收在本机 Lyrical/gz10 上完成。详见
[docs/acceptance.md](docs/acceptance.md)。

## 9. 常见问题

- **复制/重命名工作空间后构建报 CMakeCache 路径不一致**：colcon/CMake 缓存中
  保存了绝对路径。删除 `ws_metro/build`、`ws_metro/install`、`ws_metro/log`
  后重跑 `scripts/build.sh`；当前构建脚本会自动检测并清理这种陈旧缓存。
- **Gazebo 找不到插件**：确认是 `source ws_metro/install/setup.bash` 后通过
  launch 启动；launch 会把 `install/metro_gazebo/lib` 加入
  `GZ_SIM_SYSTEM_PLUGIN_PATH`。
- **里程计没有数据**：确认 `ros_gz_bridge` 已启动，`/clock` 有发布；节点必须
  `use_sim_time=true`。
- **`/metro/interlock/fault` 偶发空串**：这是故障保持 1 s 结束后按契约清除，
  不是丢失消息。
- **想改门数量/位置**：优先修改 `scripts/generate_world.py` 中的
  `UNIT_COUNT`/`UNIT_X`，重新生成世界；同时更新 `interlock_params.yaml`
  的 `psd_unit_count`、`metro_scene.launch.py` 的 monitor 参数。服务名自动
  按两位编号生成。
- **相机/渲染报错**：确认 headless 启动使用 `--headless-rendering`（
  `metro_sim.launch.py` 已自动加入）；无 GPU 环境可先把 `<sensors-system>`
  与 `platform_camera` 从场景中移除，再仅运行核心联锁验收。
