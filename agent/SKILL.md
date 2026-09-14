# Robot skill for OpenClaw / ABot-Claw

You are the high-level agent for one robot: a four-wheel-steer chassis plus dual SO-ARM arms.

## Hard rules

- Control the robot ONLY through HTTP to `http://127.0.0.1:8080`.
- Never open `/dev/ttyUSB*`, never SSH to motors, never call LeRobot directly.
- If `/health` says `"mock": true`, the body will not move. Do not claim the task succeeded in the real world.
- GET `/capabilities` before any visual or contact skill. If `vla_act.wired` is false, refuse open-door / pick / search — those need camera + VLA.
- POST `/stop` is e-stop: chassis disable + arm bus disconnect.
- `/arm/left` and `/arm/right` talk to LeRobot only if `config/ports.yaml` ports exist. Otherwise mock.

## Do now / don't

Do: chassis teleop primitives when the user explicitly drives (move, wait, stop).
Don't: invent open-door / tidy / search-and-rescue sequences (see `docs/TASKS.md`).
Don't: POST `/vla_act` until `/capabilities` shows it wired.

## Tools

```bash
curl -s http://127.0.0.1:8080/capabilities
curl -s http://127.0.0.1:8080/camera/latest
curl -s -X POST http://127.0.0.1:8080/vla_act \
  -H 'Content-Type: application/json' \
  -d '{"instruction":"grasp the handle"}'
curl -s -X POST http://127.0.0.1:8080/stop
curl -s -X POST http://127.0.0.1:8080/command \
  -H 'Content-Type: application/json' \
  -d '{"text":"前進 1 秒 然後 關閉夾爪"}'
```

For free-form tasks, prefer:

```bash
cd ~/bimanual_stack && .venv/bin/python agent/loop.py "把夾爪合上"
```

`agent/loop.py` uses the regex planner first, then Ollama if `OLLAMA_HOST` is up.

## Do now / don't

Do: chassis teleop primitives when the user explicitly drives (move, wait, stop, gripper mock).
Don't: invent open-door / tidy / search-and-rescue sequences.
Don't: POST `/vla_act` until `/capabilities` shows it wired. The slot exists so a policy can be plugged in later; it must not return fake grasps.
