# DACH_SIM 用户使用手册

本手册面向需要在本机运行 `DACH_TRON2A` 双臂仿真并通过 SDK 发送 Move/Servo 控制指令的用户。标准流程是：先设置机器人 SN 型号，再启动 C++ MuJoCo 仿真器，然后启动 `Tron2_controller` 控制器，再用 `joystick_sim` 发布手柄切模式指令（或使用真实遥控器），切换到 `idle` 和高级开发者模式，最后运行 SDK 控制或测试脚本。

## 1. 使用前准备

首次运行前，请先确认宿主机已安装预编译产物所需的系统运行库（以 Ubuntu 20.04 为准）：

```bash
sudo apt update
sudo apt install libglfw3 libglew2.1 libgl1 libwebsockets15
```

其中 `libglfw3`、`libglew2.1`、`libgl1` 供 MuJoCo 仿真器使用，`libwebsockets15` 供控制器侧的 `signaling_node` 使用。缺少这些库时，对应程序会在启动瞬间以 `error while loading shared libraries` 退出，详见第 7 节。完整依赖说明见仓库根目录 README 的「System dependencies」章节。

建议准备四个终端，分别用于：

1. 运行 `Tron2_simulation_mujoco` C++ 仿真器。
2. 运行 `Tron2_controller` 控制器。
3. 运行 `joystick_sim` 切模式脚本（无遥控器时发布 `/joystick` 指令）。
4. 运行 `Tron2_arm_tests` SDK 控制脚本。

启动顺序不能颠倒。启动控制器前必须先设置机器人 SN。控制器依赖仿真器发布的 `/motor/state`，SDK 控制又依赖控制器已经接管并进入高级开发者模式。

启动控制器前，在任意终端执行：

```bash
cd Tron2_controller
source install/local_setup.bash
setRobotSN DACH_TRON2A_001
```

`setRobotSN` 位于 `Tron2_controller/install/bin/`，source 后即可直接调用。本机仿真 SN 为 `DACH_TRON2A_001`，程序会写入控制器运行包默认的 SN 配置位置。未设置或 SN 为空时，控制器可能无法按正确型号启动。同一台机器上 SN 未变更时不必每次重复设置。

如首次运行 SDK 测试，请先安装 Python 依赖：

```bash
cd Tron2_arm_tests
python3 -m pip install -r requirements.txt
```

如果是首次 clone 本仓库，或顶层 `robot_description` 为空，请先初始化机器人模型 submodule：

```bash
git submodule update --init --recursive
```

## 2. 启动 MuJoCo 仿真器

在第一个终端执行：

```bash
cd Tron2_simulation_mujoco
./scripts/run_gui.sh
```

`run_gui.sh` 无参数启动时会默认读取：

```text
../robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml
```

启动前也可以先只检查默认模型是否能加载：

```bash
cd Tron2_simulation_mujoco
./scripts/run_headless.sh --check-model
```

如果当前环境没有图形界面，改用无窗口模式：

```bash
cd Tron2_simulation_mujoco
./scripts/run_headless.sh
```

如需指定其他模型，可通过 `--model` 或 `--config --robot` 传给 C++ 仿真器：

```bash
cd Tron2_simulation_mujoco
./scripts/run_headless.sh --model ../robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml
./scripts/run_headless.sh --config config/models.yaml --robot DACH_TRON2B
```

本仿真运行包固定使用随包提供的 MuJoCo 2.1 动态库，不需要安装 Python `mujoco`、
`numpy` 或 MROS Python 包。

## 3. 启动双臂控制器

确认已经执行过 `setRobotSN DACH_TRON2A_001` 后，在第二个终端执行：

```bash
cd Tron2_controller
source install/local_setup.bash
install/run_manipulation_sim.sh
```

也可以使用等价的底层启动命令：

```bash
cd Tron2_controller
source install/local_setup.bash
mroslaunch tron2_controllers/tron2a_manipulation_controllers_sim.launch
```

注意：`source install/local_setup.bash` 后，`MROS_ETC_PATH` 已经指向 `install/etc`，因此 `mroslaunch` 参数应写成 `tron2_controllers/...launch`，不要带 `install/etc/` 前缀。

## 4. 切换控制器模式

没有真实遥控器时，在第三个终端直接发布已抓取的 `/joystick` 组合键：

```bash
cd joystick_sim
./pub.sh switch
```

`switch` 会先发布 `idle`（`L1 + X`，进入 damping / idle），等待默认约 2 秒后再发布 `develop`（`R1 + Right`，工作模式轮巡到 developer）。也可以分步执行：

```bash
./pub.sh idle      # 等价于 L1 + X
./pub.sh develop   # 等价于 R1 + 方向右键
```

`idle` 的别名是 `l1x`；`develop` 的别名是 `dev`、`r1right` 和 `r1-right`。每段都会先按住再松开，便于控制器侧做边沿检测。

常用参数：

```text
--topic /joystick     话题名
--rate 20             发布频率 Hz
--hold 0.5            按住秒数
--release 0.2         松开秒数
--gap 2.0             switch 两段间隔
--wait-sub 3.0        等待订阅者秒数，超时仍发布
```

未检测到订阅者时脚本会打印 warning，但仍会发布并退出 0。发布后查看第二个终端的控制器日志；成功时应出现 `L1+X damping`、`manipulation_damping controller starting` 或 `R1+RIGHT` 等模式切换日志。

本工作区只提供模拟手柄指令（上文的 `./pub.sh`），本机仿真链路不包含真实遥控器接入流程。切换过程如下：

1. 执行 `./pub.sh idle`（`L1 + X`），将控制器切换到 `idle` 模式。
2. 确认控制器日志出现 `manipulation_damping controller starting`。
3. 执行 `./pub.sh develop`（`R1 + 方向右键`），进入高级开发者模式。

完成以上步骤后，控制器的 Move 和 Servo SDK 服务才可用。如果后续 SDK 运行提示 `developer_mode_required` 或 `fail_is_not_develop_mode`，通常表示尚未进入高级开发者模式，需要回到本步骤重新切换。

## 5. 运行 SDK 控制

在第四个终端进入 SDK 测试目录：

```bash
cd Tron2_arm_tests
```

确认仿真器、控制器和高级开发者模式都正常后，再运行短时 live smoke：

```bash
./scripts/run_all.sh
```

更完整的 Move、Servo 和 VLA 专项控制可以分开运行：

```bash
./scripts/run_move.sh
./scripts/run_servo.sh
./scripts/run_vla_servoj.sh
```

专项 live 是长时仿真用例，只允许连接本机仿真目标 `127.0.0.1`。`--yes` 不能绕过该限制。运行这些 live 脚本即会连接控制器并发送动作指令。

## 6. 成功标志

按顺序完成启动后，应满足以下状态：

1. `Tron2_simulation_mujoco` C++ 仿真器正在运行，并持续发布 `/motor/state`。
2. `Tron2_controller` 已启动，WebSocket 服务可通过 `127.0.0.1:5000` 访问。
3. 发布或按下 `L1 + X` 后，控制器终端出现 `manipulation_damping controller starting`。
4. 发布或按下 `R1 + 方向右键` 后，SDK live preflight 不再返回 `developer_mode_required` 或 `fail_is_not_develop_mode`。
5. `Tron2_arm_tests` 的 live 命令能够生成 `preflight_summary.json`；进入用例执行后会继续生成 `cases_summary.csv` 和 `cases_summary.json`。

可用下面命令快速检查 WebSocket 端口：

```bash
cd Tron2_arm_tests
python3 -c "import websocket; ws=websocket.create_connection('ws://127.0.0.1:5000', timeout=3); ws.close(); print('ok')"
```

## 7. 常见问题

启动瞬间报 `error while loading shared libraries`：宿主机缺少系统运行库。`tron2_mujoco_sim` 报 `libglfw.so.3` 或 `libGLEW.so.2.1` → 执行 `sudo apt install libglfw3 libglew2.1`；`signaling_node` 报 `libwebsockets.so.15` → 执行 `sudo apt install libwebsockets15`。注意动态链接器一次只报第一个缺失的库，补上一个后可能还会报下一个；无窗口模式（`run_headless.sh`）同样需要 glfw / GLEW。

控制器启动异常或型号不对：确认已经执行 `setRobotSN DACH_TRON2A_001`，并确认控制器读取到的 SN 为 `DACH_TRON2A_001`。

连接超时：确认 `Tron2_controller` 已启动，并检查 WebSocket 目标是否为 `127.0.0.1:5000`。

没有进入高级开发者模式：确认已经先执行 `./pub.sh idle`（别名 `l1x`）或按下 `L1 + X` 进入 `idle`，看到 `manipulation_damping controller starting` 后，再执行 `./pub.sh develop`（别名 `dev`、`r1right`、`r1-right`）或按下 `R1 + 方向右键`。无遥控器时也可以一次执行 `./pub.sh switch`，默认会在两段之间等待约 2 秒。

SDK live 被拒绝：如果提示 `developer_mode_required` 或 `fail_is_not_develop_mode`，重新执行手柄切模式流程后再运行。

专项 live 被拒绝：确认目标是本机回环地址。

状态不稳定：等待仿真器和控制器稳定后重试。live preflight 会检查关节状态维数、有限值和速度稳定性。

## 8. 安全注意事项

WebSocket 控制协议沿用官方 `tron2_env` 的明文 `ws://` 方式，不包含认证或 TLS。本开源工作区只保留本机回环仿真能力，目标地址应为 `127.0.0.1`，不提供真机或现场内网连接配置。

`Tron2_simulation_mujoco` 只用于本机 MuJoCo 仿真，机器人 XML 和 mesh 来自顶层 `robot_description`。

live SDK 命令会发送动作指令。即使在仿真环境中，也建议有人监护控制器终端和 SDK 终端；本开源工作区不提供真机连接流程。

本项目未实现软件急停。停止 SDK 进程、断开 WebSocket 或测试失败处理都不等同于急停；出现异常动作时，应立即使用物理急停或机器人侧安全机制。

如果执行过程中出现异常动作、控制器报错或 SDK 报告失败，应先停止 SDK 进程，再检查仿真器、控制器和机器人状态。必要时重启仿真器和控制器，重新进入高级开发者模式，然后从短时 live smoke 恢复。
