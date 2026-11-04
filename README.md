<p align="center">
  <img src="docs/banner.png" alt="tron2_manipulation_controller: the DACH_TRON2A dual-arm robot holding a box with both grippers" width="100%">
</p>

<p align="center">
  <b>English</b> | <a href="README_CN.md">中文</a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-2f6feb" alt="License: Apache-2.0"></a>
  <img src="https://img.shields.io/badge/Ubuntu-20.04-2f6feb" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/MuJoCo-2.1-2f6feb" alt="MuJoCo 2.1">
  <img src="https://img.shields.io/badge/runtime-prebuilt-2f6feb" alt="Prebuilt runtime">
</p>

# tron2_manipulation_controller

> **Part of the TRON2 open-source ecosystem.** Where to start and the other TRON2 repositories: [TRON2 quick start](https://github.com/limxdynamics/limxdynamics/blob/main/doc/quickstart.md#tron2).

This repository (workspace name `DACH_SIM`) is a local simulation and SDK validation workspace for the `DACH_TRON2A` dual-arm robot. The MuJoCo simulator, the dual-arm controller, the joystick teleoperator and the WebSocket SDK all run on one machine over `127.0.0.1`, so a motion command can be traced end to end without any hardware.

> **Scope:** loopback simulation only. Both runtime packages ship prebuilt, and the SDK path is validated against `127.0.0.1`; real hardware and on-site networks are out of scope (see [Notes](#notes)). The complete procedure — setup, mode switching, success checks and troubleshooting — is in [`USER_MANUAL.md`](USER_MANUAL.md).

## Demo

<p align="center">
  <img src="docs/demo.gif" alt="DACH_TRON2A in MuJoCo clamps a box between its grippers, lifts it, carries it 10 cm to the left and sets it down, driven by MoveJ and MoveP requests over the WebSocket SDK" width="720">
</p>

A pick-and-place run in real time. After a MoveJ to the prepared pose (not shown), nine MoveP requests approach the box, lower, clamp, lift, carry it 10 cm, place it, release, raise the grippers and pull the arms back. All 10 requests succeeded, with MoveP end positions within 1.7 cm of their targets.

The arm motion is the controller's real output: joint angles were read back over the same WebSocket API (`request_get_joint_state`, 25 Hz) during the run and replayed in MuJoCo. The table and box are added when rendering, and the box follows the grippers between the clamp and release steps: the simulator in this repository models the 16 arm and head joints only, with no objects or gripper fingers. The box is sized so that, checked every 0.1 s, no part of the arms passes into the box or the table.

## Highlights

- **The whole loop on one machine.** MuJoCo simulator, dual-arm controller, joystick and WebSocket SDK all talk over `127.0.0.1`; no robot needed.
- **Five motion interfaces.** `MoveJ`, `MoveP`, `MoveH` (head), `ServoJ` and `ServoP` as JSON requests over `ws://127.0.0.1:5000`.
- **QP motion planning with PD control** in the prebuilt `Tron2_controller` runtime.
- **The real model.** `DACH_TRON2A` with grippers from [tron2-robot-description](https://github.com/limxdynamics/tron2-robot-description): 2 × 7 arm joints and 2 head joints.
- **Tests in two layers.** Offline pytest in CI; live suites (smoke, MoveJ/H/P, ServoJ/P and a 104.6 s VLA trajectory replay) against the running simulator.

## How it works

<p align="center">
  <img src="docs/how_it_works_en.png" alt="Python code sends JSON MoveJ, MoveP, ServoJ and ServoP requests over ws://127.0.0.1:5000 to Tron2_controller, which exchanges /motor/cmd and /motor/state with the robot in MuJoCo" width="100%">
</p>

```text
Tron2_arm_tests -> ws://127.0.0.1:5000 -> Tron2_controller
Tron2_controller -> /motor/cmd -> Tron2_simulation_mujoco
Tron2_simulation_mujoco -> /motor/state -> Tron2_controller
```

`Tron2_simulation_mujoco` only handles low-level motor states and motor commands. SDK interfaces such as MoveJ, MoveP, ServoJ, and ServoP are provided by `Tron2_controller` over WebSocket.

## Quick start

Install the [system dependencies](#system-dependencies) first, and initialise the robot model submodule if `robot_description/` is empty:

```bash
git submodule update --init --recursive
```

Then use four terminals. The start order is not optional: the controller subscribes to what the simulator publishes, and the SDK only answers once the controller is in developer mode.

```bash
# once, in any terminal — without it the controller may not come up with the right model
cd Tron2_controller
source install/local_setup.bash
setRobotSN DACH_TRON2A_001

# terminal 1 — C++ MuJoCo simulator, publishes /motor/state
cd Tron2_simulation_mujoco
./scripts/run_gui.sh

# terminal 2 — controller, subscribes /motor/state and publishes /motor/cmd
cd Tron2_controller
source install/local_setup.bash
install/run_manipulation_sim.sh

# terminal 3 — no physical remote in this setup: publish L1 + X, then R1 + Right
cd joystick_sim
./pub.sh switch

# terminal 4 — SDK smoke test over ws://127.0.0.1:5000
cd Tron2_arm_tests
python3 -m pip install -r requirements.txt
./scripts/run_all.sh
```

`./pub.sh switch` publishes `idle` (`L1 + X`) and then `develop` (`R1 + Right`) with the default 2 s gap. The Move/Servo SDK services are available only after the controller logs its switch into developer mode; until then the SDK reports `developer_mode_required` or `fail_is_not_develop_mode`.

`./scripts/run_all.sh` is the short live smoke. The longer suites are `./scripts/run_move.sh`, `./scripts/run_servo.sh` and `./scripts/run_vla_servoj.sh`; all of them connect to the controller and send motion commands. On a host without a display, use `./scripts/run_headless.sh` instead of `run_gui.sh`; adding `--check-model` only loads the model and checks the 16-axis mapping.

## System dependencies

This repository includes some prebuilt runtime packages. Libraries such as `libmujoco210.so` and `libmroslib.so` are bundled with the repository, but the following runtime libraries are commonly absent from a minimal Linux installation and must come from the system package manager. Package names below use Ubuntu 20.04 / Debian-family distributions as the reference:

```bash
sudo apt update
sudo apt install libglfw3 libglew2.1 libgl1 libwebsockets15 libssl1.1
```

- `Tron2_simulation_mujoco/tron2_mujoco_sim` directly links `libglfw.so.3`, `libGLEW.so.2.1`, and `libGL.so.1`. The same binary is loaded by `run_headless.sh`, so these libraries are still required for headless startup.
- `Tron2_controller/install/bin/signaling_node` directly links `libwebsockets.so.15`.
- `libssl1.1` is not a direct dependency of any binary in this repository. It is a transitive dependency of the `libwebsockets15` package itself (`Depends: libssl1.1`), and is pulled in automatically by `apt` together with `libwebsockets15`.
- `libev4` is pulled in the same way by `libwebsockets15`; if you provide the 20.04 libraries by hand on a newer release, `libev.so.4` is needed too.
- `signaling_node` opens its WebSocket server on an IPv6 dual-stack socket. On a host or container with IPv6 disabled in the kernel the controller log shows `E: ERROR opening socket` and nothing listens on port 5000; enable IPv6 (for Docker, `--sysctl net.ipv6.conf.all.disable_ipv6=0`).
- Ubuntu 22.04 and newer releases usually do not ship `libssl1.1` in the default repositories. To run the current prebuilt controller directly, use an Ubuntu 20.04-compatible environment, or provide an OpenSSL 1.1 runtime (`libssl.so.1.1` / `libcrypto.so.1.1`) that `libwebsockets.so.15` can load.

## Project structure

```text
DACH_SIM/
├── USER_MANUAL.md             # Complete user manual
├── robot_description/         # TRON2 robot model submodule
├── Tron2_simulation_mujoco/   # C++ motor-level MuJoCo simulator
├── Tron2_controller/          # TRON2 dual-arm controller runtime package
├── joystick_sim/              # Joystick teleoperator simulator / mode-switch scripts
└── Tron2_arm_tests/           # WebSocket SDK test suite
```

## Documentation

- [`USER_MANUAL.md`](USER_MANUAL.md): robot startup, mode switching, and SDK control workflow.
- [`Tron2_simulation_mujoco/README.md`](Tron2_simulation_mujoco/README.md): C++ simulator environment and startup instructions.
- [`robot_description`](https://github.com/limxdynamics/tron2-robot-description): robot model, URDF/xacro, and mesh documentation. It is a submodule, so its README lives in its own repository.
- [`Tron2_controller/install/README_INSTALL_NEW.md`](Tron2_controller/install/README_INSTALL_NEW.md): instructions for launching the controller runtime package.
- [`joystick_sim/README.md`](joystick_sim/README.md): simulated joystick mode-switch commands, aliases, and runtime arguments.
- [`Tron2_arm_tests/USAGE.md`](Tron2_arm_tests/USAGE.md): SDK test parameters, reports, and dedicated test cases.
- [`NOTICE`](NOTICE): third-party attribution and the MROS / Fast DDS modification notice.
- [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md): full license texts of the third-party libraries.

## CI and offline tests

The repository ships the GitHub Actions workflow [`.github/workflows/ci.yml`](.github/workflows/ci.yml). On every push or pull request it installs the Python dependencies required by the SDK tests and runs:

```bash
python3 -m pytest -q
```

This command only runs offline pytest tests that neither connect to the robot nor start the simulator. `Tron2_arm_tests/scripts/run_all.sh` and the dedicated scripts are live simulation bring-up tests: they connect to the local `127.0.0.1:5000` endpoint and send motion commands, so they are not executed as part of the default CI.

## Verification

Run on 2026-11-03 in a cloud container (Ubuntu 24.04 host with the Ubuntu 20.04 `libglew2.1`, `libwebsockets15`, `libev4` and `libssl1.1` libraries), following the quick start with `run_headless.sh`:

- The simulator loads `robot_grasper.xml` (16 joints, 1 ms step), the controller starts and reports 14 arm joints, and `./pub.sh switch` puts it in developer mode.
- `./scripts/run_all.sh`: preflight passes and 4 of 5 cases pass (`H_SAFE`, `J_PREP`, `P_REACHABLE` 14.4 mm, `SJ_TRACK`). `SP_TRACK` (ServoP, 3 s to a target 20 cm away) failed in every run: the arm does not move. A ServoP to the pose that `P_REACHABLE` reaches with MoveP fails the same way.
- `./scripts/run_servo.sh` (run after `run_all.sh`): 4 of 4 pass, including both ServoP cases. Started on a fresh simulator instead, its first case is rejected because the arms are not yet in the prepared pose, so run `run_all.sh` first.
- `./scripts/run_move.sh`: the 5 cases that finished inside a 400 s limit passed (MoveJ ×2, MoveH ×2, MoveP ×1).
- The container has no IPv6, so `signaling_node` was run with a test-only shim that maps its IPv6 socket to IPv4 (see [system dependencies](#system-dependencies)).
- `run_gui.sh`, `run_vla_servoj.sh` and real hardware were not run.

## Notes

- The WebSocket control protocol follows the plain `ws://` scheme used by the official `tron2_env`; it has no authentication and no TLS. This open-source workspace only keeps loopback simulation capability, and target addresses should be `127.0.0.1`.
- The live SDK sends motion commands; this open-source workspace does not provide configurations for connecting to real hardware or to on-site internal networks.
- This project does not implement a software emergency stop. Stopping the SDK process, closing the WebSocket, or handling a failed test is not equivalent to an emergency stop; abnormal motions must be handled by the physical emergency stop, the joystick, or robot-side safety mechanisms.
- `Tron2_simulation_mujoco` is a C++ MuJoCo 2.1 runtime package for local simulation; robot XML and meshes come from the top-level `robot_description`.
- `Tron2_controller` is a prebuilt runtime package, not a full source project.

## License

Unless otherwise noted, this project is distributed under the Apache License 2.0 in [`LICENSE`](LICENSE). SPDX identifier: `Apache-2.0`. Attribution for the third-party components included in the prebuilt controller runtime is recorded in [`NOTICE`](NOTICE) and [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
