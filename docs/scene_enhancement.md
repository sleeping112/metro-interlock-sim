# 场景丰富化（A/B/C 阶段）实施说明

本文件记录在原有联锁仿真基础上完成的场景升级。核心联锁契约
（`/metro/train/command`、`/metro/train/odom`、`/metro/psd/state`、
`/metro/interlock/fault`、PSD 服务、R1–R5）保持不变。

## 1. A 阶段：场景资产与比例

### A1 几何修正

| 项 | 原值 | 升级后 |
|---|---|---|
| 站台 | 50 × 10 × 0.3 m | **130 × 14 × 0.3 m**，岛式站台 |
| 轨道床 | 50 × 2 × 0.2 m | **180 × 2 × 0.2 m**，轨道中心 y=±8.65 m |
| 列车 | 120 m 单盒体 | 120 m，6 × 20 m 车厢视觉，车门碰撞 12 个 |
| PSD | 4 单元 | **12 单元**，x=−55…+55，间距 10 m |
| 初始 spawn | x=−80 | **x=−160**（站外） |
| 安全线/盲道 | 有 | 保留，按 14 m 站台重新定位 |

### A2 建筑与设备

生成在 `metro_station.world` 中的静态/视觉资产：

- 站台吊顶、横梁、26 根柱、5 条灯带
- 站名墙、线路图、出口导向、PSD 编号、广告灯箱
- 座椅、垃圾桶、闸机、售票机、服务台、消防柜
- 隧道壳体（两端）、道砟、轨枕、钢轨、接触网/架空线
- 站台边缘安全线、盲道、排水/电缆槽视觉
- 站台指示灯、隧道弱光、车头/尾灯

### A3 列车视觉

`model.sdf.xacro` 现在包含：

- 6 节 20 m 车厢，车体、车窗带、腰线、车缝
- 12 个车门视觉面板（与 12 个 PSD 对齐）
- 空调机组、受电弓、头灯/尾灯
- 物理仍为单 `train_body` + `train_longitudinal` prismatic 关节

### A4 标识纹理

`models/metro_station/textures/` 下由 `scripts/generate_station_textures.py`
程序化生成（无外部下载）：

- `station_name.png`
- `line_map.png`
- `exit_sign.png`
- `mind_the_gap.png`

### A5 相机

`platform_camera` 模型带 Gazebo `camera` sensor，发布：

- `/metro/camera/platform/image` (`sensor_msgs/msg/Image`)
- `/metro/camera/platform/camera_info` (`sensor_msgs/msg/CameraInfo`)

headless 模式 launch 使用 `--headless-rendering` 启动 Gazebo。

---

## 2. B 阶段：运营动态

### B1 到站—停站—发车场景

`metro_scene.launch.py` 启动：

- `metro_demo`：世界、列车、bridge、联锁
- `scenario_orchestrator`：自动联锁使能 → 等停车 → 开门 → 20 s 停留 →
  关门 → 等锁紧 → 请求发车
- `platform_monitor`：RViz MarkerArray

发车通过标准 ROS 参数服务 `/interlock_node/set_parameters` 设置
`auto_run=false`、`manual_speed_mps=6.0`，不新增业务话题，联锁仍是
`/metro/train/command` 的唯一发布者。

### B2 乘客流

- 12 个低多边形乘客模型 `passenger_01..12`
- `libmetro_passenger_flow.so` 世界插件，订阅 `/metro/psd/state`
- 门打开时启动登乘波次，每人走向不同 PSD 单元，跨过门槛后隐藏
- 不新增控制话题，不影响物理联锁
- 开门/关门期间可由 `gz model -m passenger_XX -p` 观察位姿变化

### B3 对向背景列车

- `ambient_train` 在 −y 轨道循环通过
- `libmetro_ambient_train.so` 仅修改模型位姿，无碰撞/无 ROS 接口

---

## 3. C 阶段：感知与监控

### C1 站台边缘入侵检测

`libmetro_intrusion_detector.so` 监控 `passenger_*` 与站台危险带
（y=6.55…7.15 m），发布：

- `/metro/platform/intrusion` (`std_msgs/msg/Bool`, 10 Hz)

这是模型级感知替身，便于无相机/无 GPU 环境做确定性测试；后续可替换为
视觉/深度检测，而不改变话题。

### C2 监控 MarkerArray

`platform_monitor` 订阅 odom、PSD 状态、入侵状态，发布：

- `/metro/monitor/markers` (`visualization_msgs/msg/MarkerArray`, 10 Hz)

RViz 中显示列车体积、12 个 PSD 状态块、`PLATFORM CLEAR/INTRUSION` 文本和
站名。RViz 配置同时包含 `/metro/camera/platform/image` 的 Image 显示。

### C3 相机数据

见 A5。图像由 `ros_gz_bridge` 桥接为 ROS 图像，可在 RViz 或
`rqt_image_view` 中查看：

```bash
ros2 run rqt_image_view rqt_image_view /metro/camera/platform/image
```

---

## 4. 运行

```bash
cd /home/huang/workspace/subwaygazebo
bash scripts/build.sh

# 完整场景：到站—开门—登乘—关门—发车 + RViz + 相机
ros2 launch metro_bringup metro_scene.launch.py

# 无界面
ros2 launch metro_bringup metro_scene.launch.py headless:=true rviz:=false

# 仅核心联锁演示
ros2 launch metro_bringup metro_demo.launch.py

# 核心联锁回归验收
bash scripts/run_acceptance.sh
```

## 5. 验证记录

- 世界 SDF：`gz sdf -k` 通过（1299 行，12 单元、6 车厢、乘客、环境、相机）
- 插件：`colcon build` 通过（DoorController、TrainLongitudinal、
  PassengerFlow、AmbientTrain、PlatformIntrusionDetector）
- 12 个 `/metro/psd/unit_XX/command` 服务均出现
- 场景实测：
  - 列车从 x=−160 自动停车到 x=−0.48
  - PSD 开门 → 乘客波次启动 → 入侵检测 true/false → 20 s 停留
  - 关门锁紧 → 参数触发发车到 x>65
  - 相机图像约 9–10 Hz
  - MarkerArray 15 个 marker
- 核心验收：**21/21 PASS**（12 单元、130 m 站台、相机启用时）
