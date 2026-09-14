# Tron2_simulation_mujoco

这是 DACH_SIM 的 C++ MuJoCo-MROS 电机级仿真运行包。它加载顶层
`robot_description` 中的 MuJoCo XML 模型，订阅 `/motor/cmd`，发布
`/motor/state`，用于在本机和 `Tron2_controller` 联调双臂控制链路。

本目录不包含 High-Level 控制器，也不提供 MoveJ、MoveP、ServoJ 或 ServoP SDK
接口；这些接口由 `Tron2_controller` 通过 WebSocket 提供。

## 目录结构

```text
Tron2_simulation_mujoco/
├── README.md
├── tron2_mujoco_sim
├── config/
│   └── models.yaml
├── lib/
│   ├── libmroslib.so
│   └── libmujoco210.so
└── scripts/
    ├── run_gui.sh
    └── run_headless.sh
```

机器人模型位于仓库顶层 `robot_description/`。首次 clone 后如果模型目录为空，请在
仓库根目录初始化 submodule：

```bash
git submodule update --init --recursive
```

## 模型检查

```bash
cd Tron2_simulation_mujoco
./scripts/run_headless.sh --check-model
```

启动脚本会设置 C++ 运行所需的动态库路径和 MROS 仿真环境变量。传入
`--check-model` 时，脚本只加载默认 DACH_TRON2A 模型并检查 16 轴映射，不启动持续
仿真。

本运行包固定使用 MuJoCo 2.1，动态库随本目录 `lib/libmujoco210.so` 提供，不再需要
安装 Python `mujoco`、`numpy` 或 MROS Python 包。

## 系统依赖

本目录已随包提供 `lib/libmujoco210.so` 和 `lib/libmroslib.so`，但 C++ 仿真器仍需要系统提供 OpenGL/GLFW/GLEW 运行库。以 Ubuntu 20.04 / Debian 系发行版为例：

```bash
sudo apt update
sudo apt install libglfw3 libglew2.1 libgl1
```

`tron2_mujoco_sim` 直接链接 `libglfw.so.3`、`libGLEW.so.2.1` 和 `libGL.so.1`。`run_headless.sh` 仍会加载同一个二进制，因此无窗口模式也需要这些动态库。完整本机联调还需要控制器运行包侧的 `libwebsockets.so.15`（该库自身依赖 OpenSSL 1.1 运行库），见仓库根目录 README 的系统依赖说明。

## 启动带窗口仿真

```bash
cd Tron2_simulation_mujoco
./scripts/run_gui.sh
```

## 启动无窗口仿真

```bash
cd Tron2_simulation_mujoco
./scripts/run_headless.sh
```

无参数启动时，脚本默认使用：

```text
../robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml
```

脚本会先将该路径解析为绝对路径，再传给 C++ 仿真器。

## 指定模型或参数

可以通过 `--model` 显式指定 XML，或通过 `--config` 和 `--robot` 从模型配置中选择：

```bash
./scripts/run_gui.sh --model ../robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml
./scripts/run_headless.sh --config config/models.yaml --robot DACH_TRON2B
```

常用参数：

```text
--model PATH              MuJoCo XML 路径，优先级最高
--config PATH             模型配置文件
--robot NAME              从配置文件选择机器人
--rate HZ                 仿真状态发布频率，默认 300
--render-rate HZ          GUI 渲染频率，默认 30
--cmd-timeout SEC         命令超时，<=0 表示保持最后命令，默认 0
--realtime-factor VALUE   实时倍率，默认 1.0
--motor-cmd-topic TOPIC   默认 /motor/cmd
--motor-state-topic TOPIC 默认 /motor/state
--check-model             只加载模型并检查 16 轴映射
```

`run_gui.sh` 和 `run_headless.sh` 会在正式启动前自动执行一次 `--check-model`。
如果命令本身传入 `--check-model`，脚本只执行检查并退出。

## MROS 话题

仿真桥创建 MROS 节点 `tron2_mujoco_sim`。

它订阅：

```text
/motor/cmd
```

它发布：

```text
/motor/state
```

`/motor/cmd` 的消息类型是 `controller_msgs/JointCmd`，`/motor/state` 的消息类型是
`controller_msgs/JointState`。wire 顺序固定为：

```text
左臂 7 轴 + 右臂 7 轴 + 头部 pitch/yaw 2 轴
```

## 联调

启动仿真器后，再启动 `Tron2_controller`。通过 `joystick_sim` 切换控制器模式：

```bash
cd ../joystick_sim
./pub.sh switch
```

完整流程见仓库根目录 `USER_MANUAL.md`。

## 安全说明

本项目只用于本机 MuJoCo 仿真。live SDK 命令会发送动作指令，即使在仿真环境中，也应
有人监护控制器终端和 SDK 终端。
