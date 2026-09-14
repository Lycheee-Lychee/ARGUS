# Phase 2 first half — language agent

This folder is the ABot-Claw *slot*: an agent that may only call `robot_bridge`.

```
you --language--> agent/loop.py --HTTP--> robot_bridge --serial--> hardware
```

## Run (on Thor)

```bash
cd ~/bimanual_stack
.venv/bin/python agent/loop.py "前進 1 秒 然後 關閉夾爪"
.venv/bin/python agent/loop.py --plan-only "左轉 2 秒"
```

OpenClaw: copy `SKILL.md` and `openclaw/*.md` into `~/.openclaw/workspace/skills/` after you install OpenClaw. The gateway is optional; `loop.py` is enough to test the architecture.

Ollama is optional. If it is not running, only the built-in phrases work.
