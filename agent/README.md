# Phase 2 first half — language agent

This folder is the ABot-Claw *slot*: an agent that may only call `robot_bridge`.

```
you --language--> agent/loop.py --HTTP--> robot_bridge --serial--> hardware
```

## Run (on Thor)

```bash
cd ~/bimanual_stack
.venv/bin/python agent/loop.py --plan-only "前進 1 秒"
.venv/bin/python agent/loop.py "前進 1 秒 然後 關閉夾爪"
```

`--plan-only` stops after JSON. Without it, steps POST to `:8080` (mock chassis if no USB).

## Cloud planner (DeepSeek)

Known phrases (`前進 1 秒`, `關閉夾爪`, …) never call the cloud.

Copy `.env.example` to `.env` and put a key from [DeepSeek API](https://api-docs.deepseek.com/):

```
DEEPSEEK_API_KEY=sk-...
LLM_MODEL=deepseek-v4-flash
```

Then free-form sentences that are still *drive/gripper/stop* will be compiled to JSON. Visual skills (open door, tidy) are refused until `vla_act` is wired. No camera images are sent.

## Tests

```bash
bash scripts/run_tests.sh
```

## OpenClaw

```bash
bash scripts/install_openclaw_skill.sh   # copies skill files
bash scripts/install_openclaw.sh         # CLI + skill; gateway optional
```

The gateway is optional. `loop.py` is enough. If you do run OpenClaw, keep it on loopback and point it at DeepSeek during `openclaw onboard`. Skill path: `~/.openclaw/workspace/skills/robot-bridge/`.
