# joystick_sim

向 `/joystick` 发布两种手柄组合键，用于在没有实体遥控器时切换控制器模式。

| 命令 | 组合键 | 按钮 | 作用 |
| --- | --- | --- | --- |
| `idle` | L1+X | `buttons[2]=1`, `buttons[4]=1` | 进入 damping / idle |
| `develop` | R1+Right | `buttons[7]=1`, `buttons[15]=1` | 工作模式轮巡（developer） |
| `switch` | 先 idle，再 develop | 同上 | 两段之间默认间隔 2s |

别名：`l1x` → idle；`dev` / `r1right` / `r1-right` → develop。

每段都会先按住再松开，方便 `EventTriggler` 做边沿检测。

## 运行

控制器已启动、且与本机 MROS localhost bus 相通时：

```bash
./pub.sh idle
./pub.sh develop
./pub.sh switch
```

常用参数：

```text
--topic /joystick     话题名
--rate 20             发布频率 Hz
--hold 0.5            按住秒数
--release 0.2         松开秒数
--gap 2.0             switch 两段间隔
--wait-sub 3.0        等待订阅者秒数，超时仍发布
```

示例：

```bash
./pub.sh idle --hold 0.8 --wait-sub 1
./pub.sh develop --topic /joystick --rate 20
```

未检测到订阅者时会打印 warning，仍会发布并退出 0。控制器侧成功时，日志应出现 `L1+X damping` 或 `R1+RIGHT`。

## 运行依赖

- 产物目录内的 `joystick_sim` 与 `lib/libmroslib.so`
- `pub.sh` 会设置 `LD_LIBRARY_PATH`、`MROS_LOCALHOST_ONLY=1`、`MROS_SIM_TIME=0`、`IS_SIM=1`
- 需要与控制器在同一 MROS 总线（默认 127.0.0.1）
