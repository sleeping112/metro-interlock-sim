# 端到端验收场景（阶段 5）

## 1. 场景目标

一条 launch 启动 Gazebo 世界、列车、ros_gz_bridge、联锁节点，然后由
`acceptance_test.py` 只通过公共话题/服务验证：

| 编号 | 场景步骤 | 期望 |
|---|---|---|
| 0 | 启动完成 | `/metro/psd/state=true`，服务齐全 |
| R1 | 关闭联锁后请求开门 | 拒绝；门保持关闭；无故障 |
| R2 | 使能联锁、列车在 x=−80 处请求开门 | 拒绝；`TRAIN_NOT_IN_STOP_WINDOW` |
| 自动停车 | 打开 `auto_run` | 列车在 ±0.5 m 停车窗内停住，指令为 0 |
| R3 | 在窗内开门 | `state=false`；`PSD_NOT_LOCKED`；列车指令 0 |
| 关门 | 关门 | 门扇回 0，`state=true` |
| R4 | 故障注入（负超时）后开门 | `PSD_OPEN_TIMEOUT`；牵引封锁 |
| R5 | 列车低速经过窗内时开门 | `EMERGENCY_BRAKE_PSD_OPEN`；列车指令 0 |
| 清场 | 关门、停车 | 门回到锁紧状态 |

`R4` 的正向路径（3 s 内 `state` 变 false）在 R3 场景中验证；超时分支通过把
`psd_open_timeout_s` 临时设为负值触发，这是验收脚本的故障注入手段，正常参数
仍为 3.0 s。

## 2. 运行方式

```bash
bash scripts/run_acceptance.sh
# 或
source /opt/ros/humble/setup.bash
source ws_metro/install/setup.bash
ros2 launch metro_bringup acceptance.launch.py
```

`acceptance.launch.py` 以 `headless:=true rviz:=false auto_run:=false` 启动
`metro_demo.launch.py`，4 s 后启动验收节点；验收结束后关闭所有进程。

## 3. 实际验收结果

本机验证环境（ROS 2 Lyrical / gz-sim10，目标栈 Humble/gz-sim8）：

```text
[PASS] PSD reports locked/closed at startup
[PASS] interlock disable service accepted
[PASS] R1: open refused while disabled
[PASS] R1: PSD stayed closed
[PASS] R1: no fault raised
[PASS] interlock enable service accepted
[PASS] R2: open refused outside window
[PASS] R2: fault TRAIN_NOT_IN_STOP_WINDOW published
[PASS] R2: PSD stayed closed
[PASS] speed curve stopped the train inside +/-0.5 m
[PASS] train command is zero at standstill
[PASS] open accepted in window
[PASS] R4 nominal: PSD state became false well within 3 s
[PASS] R3: fault PSD_NOT_LOCKED published
[PASS] R3: train command forced to zero while doors open
[PASS] close command accepted
[PASS] PSD locked after close
[PASS] R4: fault PSD_OPEN_TIMEOUT published on timeout
[PASS] R5: open accepted while moving in window
[PASS] R5: fault EMERGENCY_BRAKE_PSD_OPEN published
[PASS] R5: train command forced to zero by emergency brake
acceptance summary: 21/21 checks passed
ACCEPTANCE_RESULT=PASS
```

## 4. 接口抽查命令

```bash
# 话题/QoS/频率
ros2 topic list -t | grep metro
ros2 topic info -v /metro/train/odom       # PUBLISHER reliability=BEST_EFFORT
ros2 topic hz /metro/train/odom            # ~50 Hz (受实时因子影响)
ros2 topic hz /metro/psd/state             # ~10 Hz

# 服务齐全（必须 5 个 PSD 服务）
ros2 service list -t | grep '/metro/psd'

# 手动开门/关门（总开关）
ros2 service call /metro/psd/command std_srvs/srv/SetBool "{data: true}"
ros2 service call /metro/psd/command std_srvs/srv/SetBool "{data: false}"

# 单门独立控制
ros2 service call /metro/psd/unit_02/command std_srvs/srv/SetBool "{data: true}"

# 故障/状态
ros2 topic echo /metro/psd/state
ros2 topic echo /metro/interlock/fault
```

## 5. 验收中可观察到的物理行为

- 开门后 `gz model -m psd_door_01 -l psd_left_01` 的 x 由 −0.90 m 变为约
  −2.70 m，右门扇由 +0.90 m 变为约 +2.70 m，行程 1.80 m。
- 列车里程计 `x` 从 −80 m 平滑加速、按制动曲线停车，最终落在 ±0.5 m 窗内。
- 列车门碰撞盒与 PSD 框架之间保留 0.01–0.08 m 间隙，避免初始穿模；门扇
  与门框在 y 方向分层，开门行程不触发互相碰撞。

## 6. 故障注入边界

- R4 的正向路径由正常开门覆盖；超时分支用负 `psd_open_timeout_s` 注入。
- R5 通过“低速经过停车窗时请求开门”触发；在真实运营中，R2 通常会先拒绝
  列车未停稳时的开门请求，R5 是屏蔽门被意外打开时的最后一道防护。
- 验收脚本结束后会关闭门并停车；实际运营策略不在此仿真范围。
