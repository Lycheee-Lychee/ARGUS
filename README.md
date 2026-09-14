# ARGUS

Bimanual wheeled robot stack for Jetson Thor: remote teleop, language primitives, vision/VLA slots.

Hardware: NVIDIA Thor · four-steer chassis (STM32) · dual [SO-ARM100/101](https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper) · Robonine parallel gripper.

This repo is the **bridge and agent**. It does not clone ABot-Claw or Harness VLA until camera + `vla_act` are wired.

## On Thor

```bash
cd ~/bimanual_stack   # or clone this repo
bash scripts/install_deps.sh   # python venv + tmux
bash scripts/bringup.sh
```

UI: `http://<thor-ip>:8080/` — language toggle **繁中 / EN** (top right).

Language without hardware:

```bash
bash scripts/run_tests.sh
.venv/bin/python agent/loop.py --plan-only "前進 1 秒"
```

Free-form phrases need `DEEPSEEK_API_KEY` in `.env` (DeepSeek cloud, JSON steps only). OpenClaw skill: `bash scripts/install_openclaw_skill.sh`.

## Layout

| Path | Role |
|------|------|
| `robot_bridge/` | HTTP API, teleop page, chassis + arm drivers |
| `agent/` | Language loop + OpenClaw skill (HTTP only) |
| `config/ports.yaml` | Feetech serial map |
| `docs/` | Architecture, e-stop, task list |
| `scripts/` | Bring-up, port detect, LeRobot install/calibrate |
| `sim/` | Optional SO-ARM MuJoCo (not the full vehicle) |

## Safety

`POST /stop` zeros the base and disconnects arm buses. `cmd_vel` times out in 500 ms. Do not expose port 8080 to the public internet.

## License

Code in this repository follows the licenses of included vendor sources where applicable (chassis firmware/ROS2, gripper hardware docs).
