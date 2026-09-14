# Tron2_arm_tests

`Tron2_arm_tests` 是 DACH_TRON2A 双臂 WebSocket JSON 协议测试套件，覆盖 MoveJ、MoveH、MoveP、ServoJ、ServoP，以及独立的 VLA ServoJ 回放。

完整说明见 [`USAGE.md`](USAGE.md)。

## 快速开始

安装依赖：

```bash
cd Tron2_arm_tests
python3 -m pip install -r requirements.txt
```

运行离线 pytest（不连接 WebSocket，不发送动作指令）：

```bash
cd ..
python3 -m pytest -q
```

启动仿真器和控制器并进入高级开发者模式后，先运行短时 live smoke。无遥控器时可在仓库根目录的 `joystick_sim` 中执行 `./pub.sh switch`；该命令会按默认 2 秒间隔依次发布 `idle`（`L1 + X`）和 `develop`（`R1 + Right`），更多别名和参数见 [`../joystick_sim/README.md`](../joystick_sim/README.md)。

```bash
./scripts/run_all.sh
```

脚本会直接连接 WebSocket 并发送动作指令。三个专项脚本是仿真专用长时套件：

```bash
./scripts/run_move.sh
./scripts/run_servo.sh
./scripts/run_vla_servoj.sh
```

live 仅允许本机仿真目标 `127.0.0.1`。远程 IP 或真机 IP 会在连接前被拒绝；`--yes` 仅作为兼容参数保留，不能授权远程连接。`--run-live` 和 `--long-run-sim` 仍可传入，但不再改变行为。

## 专项规模与耗时

- Move：MoveJ、MoveH、MoveP 各 2 组，共 6 组。
- Servo：ServoJ、ServoP 各 2 组，共 4 组。
- VLA ServoJ：1 组。
- 合计 11 组。

Move 和 Servo 各使用 2 个时间档（1 秒和 2 秒）。Servo 保留 2 种确定性安全轨迹模式。Move 在 `config/move_cases.yaml` 里为每组直接配置多个不同目标点，组内所有受控轴都会运动，并按该序列循环至少 65 秒；达到阈值后会完成当前段再退出。Move 的时间值是协议 `time`，Servo 的时间值是单目标插值时长 `segment_duration`。Servo 每组只插值到一个目标点，然后安全回位，不做周期性往返。

VLA 使用当前保留的 `examples/vla_height/states_controller_safe.txt`，保持安全轨迹文件中的时间戳和 104.5667 秒原时序不变，完整回放 1 次。每组开始时先读取当前关节状态，再从当前位置插值到 VLA 轨迹的初始帧，然后按源时序回放。

按最低有效运动时间估算，纯运动约为：

```text
6 × 65 s + 1 × 104.5667 s + Servo 插值与回位 ≈ 9 min+
```

加上 preflight、每组回位、稳定等待和进程开销，完整 live 仍建议按 Move、Servo、VLA 三个专项分开人工运行和保存报告。

## 安全行为

- 运行 live 脚本即会连接机器人并发送动作指令；离线 pytest 不连接 WebSocket。
- 默认 IP 是本机仿真 `127.0.0.1`。
- WebSocket 控制协议沿用官方 `tron2_env` 的明文 `ws://` 方式，不包含认证或 TLS；本测试套件只保留本机回环仿真能力，目标地址应为 `127.0.0.1`。
- 专项长时用例均标记为 `simulation_only`，仅允许本机仿真目标。
- live 前会执行 preflight，探测高级开发者服务，并检查关节状态维数、有限值和速度稳定性。只有已经收到 `notify_robot_info` 时，才会额外校验其中的 IMU、电机和运行状态；通知缺失不会自动证明目标安全，live 前仍须人工确认连接的是预期仿真、仿真状态正常且运动空间安全。
- 如果返回 `fail_is_not_develop_mode`，程序会在发送任何动作前中止。
- Servo/VLA 专项按 300 Hz 发送；专项 Servo 每组只做单目标插值和回位，`run_all.sh` 中现有 Servo smoke 用例按 100 Hz 发送。收到 `notify_servoJ` 或 `notify_servop` 会立即中止当前用例。
- 默认 `stop_on_failure: true`，首个失败用例后不再执行后续组。正常完成会执行安全回位；活动段、流式发送或回位阶段失败时应按 `failure_stage` 处理，不要假定自动恢复成功。
- 本测试套件未实现软件急停。停止测试进程、断开 WebSocket、preflight 中止或失败回位都不等同于急停；本开源测试套件不提供真机连接流程。

本地联调应先设置机器人 SN（在 `Tron2_controller` 下 `source install/local_setup.bash` 后执行 `setRobotSN DACH_TRON2A_001`），再启动 MuJoCo 仿真器，然后启动双臂控制器，确认 WebSocket 和高级开发者模式正常后，最后运行测试。中断或失败后先停止测试、确认仿真/机器人状态和空间安全，重新进入高级开发者模式，从短时 smoke 恢复，再继续未完成专项。

## 报告

live 会先写入：

- `preflight_summary.json`

只有 preflight 通过并实际进入 cases 执行后，才会继续生成：

- `cases_summary.csv`
- `cases_summary.json`

重点查看：

- 用例维度：`pattern`（Servo/VLA）、`time_index`、`repeat_index`、`requested_arrival_sec`、`requested_segment_sec`、`requested_active_duration_sec`。Move 目标点来自配置 `waypoints`。
- 循环维度：`active_elapsed_sec`、`completed_cycles`、`completed_segments`。
- 流式发送：`requested_send_rate_hz`、`planned_send_count`、`actual_send_count`、`actual_send_rate_hz`。
- ACK 与错误：`response_status`、`result`、`messages`、`failure_stage`。
- VLA 运行时硬限位夹紧：`clamped_value_count`、`clamped_frame_count`、`max_clamp_correction_rad`、`hard_limit_tolerance_rad`。

当前精简版只保留运行时需要的安全轨迹 `examples/vla_height/states_controller_safe.txt`；原始录制文件、离线生成脚本和生成报告已不再随项目保留。VLA 专项设置 `hard_limit_tolerance_rad: 0`，加载时仍会检查列格式、时间单调、数值有限、相邻帧跳变和关节限位；正常情况下不应报告运行时夹紧。回放开始时会先发送当前位置预热，再平滑过渡到轨迹首帧。

`response_status=success` 只表示 signaling 已接收命令，不等于机器人到位。最终结论以到位、稳定、误差和异步通知为准。
