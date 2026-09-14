# Tron2 双臂协议测试使用手册

本手册说明如何运行 `Tron2_arm_tests`。离线 pytest 只检查解析、校验、轨迹和报告逻辑，不连接 WebSocket；live 脚本默认且唯一支持的目标是本机仿真 `127.0.0.1:5000`，运行后会连接 WebSocket 并发送动作指令。

## 1. 环境准备

安装 Python 依赖：

```bash
cd Tron2_arm_tests
python3 -m pip install -r requirements.txt
```

本地仿真建议使用三个终端。启动控制器前必须先设置机器人 SN：

```bash
cd Tron2_controller
source install/local_setup.bash
setRobotSN DACH_TRON2A_001
```

`setRobotSN` 位于 `Tron2_controller/install/bin/`，source 后即可直接调用。本机仿真 SN 为 `DACH_TRON2A_001`，程序会写入控制器运行包默认的 SN 配置位置。未设置或 SN 为空时，控制器可能无法按正确型号启动。同一台机器上 SN 未变更时不必每次重复设置。

如果是首次 clone 本仓库，或顶层 `robot_description` 为空，请先在仓库根目录初始化机器人模型 submodule：

```bash
git submodule update --init --recursive
```

然后按下面顺序启动 C++ 仿真器和控制器：

```bash
cd Tron2_simulation_mujoco
./scripts/run_gui.sh
```

```bash
cd Tron2_controller
source install/local_setup.bash
install/run_manipulation_sim.sh
```

离线 pytest 可从仓库根目录运行，用于本地检查和 GitHub Actions：

```bash
cd ..
python3 -m pytest -q
```

离线 pytest 不会启动仿真器、连接控制器或发送运动命令。后续 `run_all.sh` 和专项脚本均属于 live 仿真联调步骤，需按上面的仿真器、控制器和高级开发者模式流程准备。

确认 WebSocket 端口可用：

```bash
python3 -c "import websocket; ws=websocket.create_connection('ws://127.0.0.1:5000', timeout=3); ws.close(); print('ok')"
```

## 2. 高级开发者模式

MoveJ、MoveH、MoveP、ServoJ、ServoP 等高层双臂接口只在高级开发者模式下可用。测试程序不会自动切换模式；live 运行前会先做 preflight：

- 仅当客户端已经收到 `notify_robot_info` 时，校验通知中的 IMU、电机和运行状态。
- 探测 `request_get_joint_state` 是否可用。
- 如果服务端返回 `fail_is_not_develop_mode`，程序会在发送任何动作前中止。
- 如果关节状态少于 16 维、包含非有限数值或速度未稳定，程序也会中止。
- 默认静止阈值为 `stable_velocity_rad_s: 0.05`，用于兼容仿真静止时的轻微速度抖动；报告会写出 `max_abs_dq` 和 `joint_dq`。

如果未收到 `notify_robot_info`，当前实现不会据此判定 preflight 失败，也不能由此证明目标安全。每次 live 前仍须人工确认连接的是预期仿真、仿真状态正常、控制器已接管且运动空间安全。

WebSocket 控制协议沿用官方 `tron2_env` 的明文 `ws://` 方式，不包含认证或 TLS。本开源测试套件只保留本机回环仿真能力，目标地址应为 `127.0.0.1`，不提供真机或现场内网连接配置。

本地仿真中请使用仓库根目录下的 `joystick_sim` 切换模式。没有遥控器时直接发布已抓取的 `/joystick` 组合键：

```bash
cd joystick_sim
./pub.sh switch
```

`switch` 先发布 `idle`（`L1 + X`），等待默认约 2 秒后再发布 `develop`（`R1 + Right`）。也可以分步执行 `./pub.sh idle` 和 `./pub.sh develop`；`idle` 的别名是 `l1x`，`develop` 的别名是 `dev`、`r1right` 和 `r1-right`。

`pub.sh` 支持 `--topic`、`--rate`、`--hold`、`--release`、`--gap` 和 `--wait-sub` 等参数，详见 [`../joystick_sim/README.md`](../joystick_sim/README.md)。未检测到订阅者时脚本会打印 warning，但仍会发布并退出 0。查看 `Tron2_controller` 终端，确认打印出 `L1+X damping`、`manipulation_damping controller starting` 或 `R1+RIGHT` 等模式切换日志。

本工作区只提供模拟手柄指令（上文的 `./pub.sh`），本机仿真链路不包含真实遥控器接入流程。切换过程如下：

1. 执行 `./pub.sh idle`（`L1 + X`），切换到 `idle` 模式。
2. 查看 `Tron2_controller` 控制器终端，确认打印出 `manipulation_damping controller starting`。
3. 执行 `./pub.sh develop`（`R1 + 方向右键`），进入高级开发者模式。

完成以上步骤后，再运行 Move 或 Servo 的 SDK live 控制。遇到非开发者模式时，请先重新切到高级开发者模式，再重新运行测试。

## 3. 推荐运行顺序

本地联调顺序必须是：先设置机器人 SN（`setRobotSN DACH_TRON2A_001`），再启动 MuJoCo 仿真器，然后启动双臂控制器，确认 WebSocket 和高级开发者模式正常，最后启动测试。控制器依赖仿真器发布的 `/motor/state`，不要颠倒仿真器和控制器的启动顺序。

先运行短时 smoke：

```bash
./scripts/run_all.sh
```

`run_all.sh` 继续使用 `config/test_cases.yaml`，只运行短时 smoke。

三个专项脚本加载的是仿真专用长时套件，建议逐个运行：

```bash
./scripts/run_move.sh
./scripts/run_servo.sh
./scripts/run_vla_servoj.sh
```

这些套件的全部用例都标记为 `simulation_only`，live 只接受本机仿真目标 `127.0.0.1`。远程 IP 或真机 IP 会在建立连接前被拒绝，即使传入 `--yes` 也不会绕过该限制。`--run-live` 和 `--long-run-sim` 仍可传入，但不再改变行为。

## 4. 套件规模与脚本

`run_all.sh` 运行默认 smoke 用例，适合快速检查 WebSocket 协议通路。

- `run_move.sh`：MoveJ、MoveH、MoveP 各 2 组，共 6 组。
- `run_servo.sh`：ServoJ、ServoP 各 2 组，共 4 组。
- `run_vla_servoj.sh`：VLA ServoJ 原时序回放 1 组。

专项总计 11 组。Move 和 Servo 各有 2 个时间档（1 秒和 2 秒）。Servo 每类保留 2 种确定性轨迹模式。Move 不再使用 named pattern，而是在 `config/move_cases.yaml` 里为每组直接写出多个不同目标点；时间档仍用于快慢指令调度。

MoveJ、MoveH、MoveP 中的时间档写入协议 `time`，表示计划到达时间；ServoJ、ServoP 中写入 `segment_duration`，表示从当前状态到目标点、以及回位阶段的插值时长。Move 每组按配置中的目标点序列循环，有效运动至少 65 秒；到达 65 秒阈值后会完成当前段，因此实际 `active_elapsed_sec` 可能略大于 65 秒。Servo 每组只插值到一个目标点，然后安全回位，不做周期性往返。

Move 每组包含至少 3 个互不相同的目标点，组内所有受控轴都赋予目标并在相邻点之间变化：MoveJ 为 14 个臂关节，MoveH 为头部 pitch/yaw，MoveP 为双臂位置和姿态。Servo 的 2 种模式覆盖双肩抬肘和腕-头协调，以及左右平移。

专项 ServoJ/ServoP 和 VLA 专项按 300 Hz 连续流式发送，不是单条命令；`run_all.sh` 中现有 ServoJ/ServoP smoke 用例则显式配置为 100 Hz。收到 `notify_servoJ` 或 `notify_servop` 会立即中止当前用例。VLA 不进入默认 smoke，它从当前保留的 `examples/vla_height/states_controller_safe.txt` 加载安全轨迹，保留 104.5667 秒时间线，完整回放一次。每组开始时先发送当前位置做 1.5 秒预热，再从当前位置插值到 VLA 轨迹的初始帧；预热和衔接时间不计入 104.5667 秒轨迹时序。

## 5. 耗时估算

Move 的最低有效运动时间为 `6 × 65 s = 390 s`；VLA 轨迹时间为 `1 × 104.5667 s = 104.5667 s`。Servo 专项每组只执行一次目标插值和一次回位，实际时长取决于 1～2 秒时间档。因此预计纯运动：

```text
390 s + 104.5667 s + Servo 插值与回位 ≈ 9 min+
```

这还不包括 preflight、当前位置衔接、每组安全回位、最终稳定等待、进程启动和报告写入。为便于监护、失败恢复和报告归档，建议按 Move、Servo、VLA 三个专项分开运行。

## 6. 常用参数

```bash
--robot-ip 127.0.0.1
--port 5000
--accid DACH_TRON2A_001
--output-dir results/sim-001
```

指定连接目标、软件序列号和报告目录。默认且唯一支持的 IP 是本机仿真 `127.0.0.1`，默认 ACCID 是 `DACH_TRON2A_001`。

```bash
--yes
```

兼容旧命令的空操作。当前实现不支持连接远程 IP 或真机。

```bash
--run-live
--long-run-sim
```

兼容旧命令的空操作。当前实现始终连接并发送动作指令；专项仿真套件默认即可运行，只需目标是回环地址。

## 7. VLA 轨迹校验与限位夹紧

VLA 加载会先检查列格式、时间单调、数值有限、相邻帧跳变、源速度和 XML 物理硬限位，再检查 XML 与控制器臂限位的交集。`hard_limit_tolerance_rad` 只容忍不超过 `0.001 rad` 的 XML 录制误差，不能绕过控制器限位；当前 VLA 专项使用的 `examples/vla_height/states_controller_safe.txt` 已满足限位，因此配置为 `0`。

当前精简版只保留运行时需要的安全轨迹文件，不再包含原始录制文件、离线生成脚本或生成报告。每组在轨迹前读取当前关节状态，先发送当前位置预热，再生成从当前位置到首帧的限速衔接，轨迹结束后再安全回位；这些预热和衔接时间不计入 104.5667 秒轨迹时序。

## 8. 报告判读

每次 live 会先在输出目录生成：

- `preflight_summary.json`

只有 preflight 通过并开始执行 cases 后，才会生成：

- `cases_summary.csv`
- `cases_summary.json`

关键字段：

- `ok`：该用例是否通过。
- `result`：失败类型，如 `response_fail`、`timeout`、`notify_or_accuracy_fail`。
- `response_status`：服务端 ACK 中的 `data.result`。
- `failure_stage`：失败发生的位置，如 `waypoint_N`、`active_stream`、`source`、`safe_return` 或 `final_stability`。
- `pattern` / `time_index` / `repeat_index`：Servo 轨迹模式、时间档和 VLA 重复编号。Move 长时组没有 `pattern`，目标点来自配置中的 `waypoints`。
- `requested_arrival_sec`：Move 的协议 `time`。
- `requested_segment_sec` / `segment_duration_sec`：Servo 单段时长。
- `requested_active_duration_sec` / `active_elapsed_sec`：目标有效运动时长和实际有效运动时长。
- `completed_cycles` / `completed_segments`：完成的完整轨迹循环数和 waypoint 段数。
- `requested_send_rate_hz` / `actual_send_rate_hz`：目标发送频率和实测发送频率。
- `planned_send_count` / `actual_send_count`：计划和实际发送样本数。
- `joint_max_abs_rad` / `joint_rms_rad`：关节误差。
- `head_max_abs_rad` / `head_rms_rad`：头部误差。
- `pose_position_m` / `pose_orientation_rad`：末端位姿误差。
- `arrive_elapsed_sec`：等待到位耗时。
- `max_abs_dq`：preflight 中观测到的最大关节速度。
- `clamped_value_count` / `clamped_frame_count` / `max_clamp_correction_rad` / `hard_limit_tolerance_rad`：VLA 运行时 XML 硬限位夹紧统计和允许容差；派生安全文件应全部为零。
- `messages`：协议失败、异步通知或逐命令状态的原始摘要。

注意：`response_status=success` 只表示 signaling 已接收并发布命令，不等于机器人已经到位。最终判定以到位、稳定、误差和异步通知为准。

## 9. 常见问题

连接超时：确认仿真控制器或机器人 WebSocket 服务已启动，并检查 `--robot-ip`。仿真默认应使用 `127.0.0.1`。

控制器启动异常或 ACCID 错误：确认已经执行 `setRobotSN DACH_TRON2A_001`，默认 `--accid DACH_TRON2A_001` 与控制器 SN 一致，或用 `--accid` 覆盖。

非高级开发者模式：preflight 可能返回 `developer_mode_required`。先切到高级开发者模式再运行。

诊断失败：如果 IMU 或电机状态不是 `OK`，程序会中止。先处理机器人诊断问题。

状态不稳定：关节速度超过 `stable_velocity_rad_s` 时会中止。等待机器人静止后重试。

到位超时：可能是目标不可达、控制器未接管、阈值过严或运动时间不足。先查看 `cases_summary.json` 中的最终状态和误差。

Servo 通知失败：查看 `messages` 中的 `notify_servoJ` 或 `notify_servop` 原始内容，常见原因是非法指令、电机错误或跳变过大。

发送频率不足：降低系统负载，或缩短其它高频任务。长时 Servo/VLA 专项目标频率为 300 Hz；`run_all.sh` 中现有 Servo smoke 为 100 Hz。

live 被拒绝：确认目标是本机仿真 `127.0.0.1`。`--yes` 不能解除回环限制。

## 10. 中断、失败与恢复

默认配置为 `safety.stop_on_failure: true`：某组失败后，Runner 不再执行后续组。正常完成有效运动后会尝试安全回位并做最终稳定/误差检查；但活动段响应失败、Servo/VLA 流式通知失败或 `safe_return` 本身失败时，不应假定测试进程已经把机器人恢复到 home。使用 `failure_stage`、`result`、`messages` 和最终状态判断停在何处。

本测试套件未实现软件急停。按 `Ctrl+C` 中断测试进程、断开 WebSocket、preflight 中止或失败回位都不等同于急停。本开源测试套件不提供真机连接流程；异常后建议：

1. 停止当前测试进程。
2. 确认仿真器、控制器、关节状态和周围空间；必要时重启仿真器，再重启控制器。
3. 确认 WebSocket 恢复并重新进入高级开发者模式。
4. 先运行短时 `./scripts/run_all.sh`。
5. 检查上一次 `cases_summary.json` 的 `failure_stage`，人工决定是否从对应专项重新开始。

## 11. 添加新用例

MoveJ/ServoJ 关节目标应来自 `robot_grasper.xml` 的 16 维协议映射：左臂 7、右臂 7、`head_pitch`、`head_yaw`。夹爪联动关节不属于 MoveJ/ServoJ 的 16 维协议。

新增 Move 长时组时，在 `config/move_cases.yaml` 的对应 `groups` 里直接写多个目标点，不要再依赖 named pattern。MoveJ 每个目标点应给出全部 14 个臂关节，MoveH 同时给出 pitch/yaw，MoveP 同时给出双臂位置和姿态；相邻目标点之间这些轴都要变化。优先围绕 `home_position` 或默认参考位姿做小幅增量，不要直接贴近物理硬限位。

新增 ServoJ 轨迹时，使用插值生成连续轨迹，并保证相邻指令跳变远小于 `command_jump_threshold`。

新增 VLA 文件时，保持与当前安全轨迹相同的列格式，并先在项目外完成控制器限位处理，再把可直接回放的安全轨迹放入项目并在 `config/vla_servoj_cases.yaml` 中引用。运行时程序会检查列数、时间单调、数值有限、XML/控制器有效关节限位和相邻帧跳变。
