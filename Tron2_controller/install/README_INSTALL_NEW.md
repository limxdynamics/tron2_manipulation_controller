# Ctrler 运行包

本运行包是预编译产物，启动前需确保宿主机已安装系统运行库（其中 `signaling_node` 依赖 `libwebsockets.so.15`）。安装命令与平台要求见仓库根目录 [README](../../README.md) 的「System dependencies」章节。

启动控制器前必须先设置机器人 SN 型号。在 `Tron2_controller` 目录执行：

```bash
source install/local_setup.bash
setRobotSN DACH_TRON2A_001
```

`setRobotSN` 位于 `install/bin/`，source 后即可直接调用。本机仿真 SN 为 `DACH_TRON2A_001`，程序会写入控制器运行包默认的 SN 配置位置。未设置或 SN 为空时，控制器可能无法按正确型号启动。同一台机器上 SN 未变更时不必每次重复设置。

然后启动控制器：

```bash
source install/local_setup.bash
mroslaunch tron2_controllers/tron2a_manipulation_controllers_sim.launch
```

更简单的启动方式：

```bash
install/run_manipulation_sim.sh
```

控制器运行包可能随附第三方动态库。版权归因和 MROS/Fast DDS 修改声明见仓库根目录
[`NOTICE`](../../NOTICE)，完整许可证文本见
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md)。

注意：`mroslaunch` 会按 `MROS_ETC_PATH` 解析相对路径。source
`install/local_setup.bash` 后，`MROS_ETC_PATH` 已经指向 `install/etc`，
所以 launch 参数应写成 `tron2_controllers/...launch`，不要再带
`install/etc/` 前缀。