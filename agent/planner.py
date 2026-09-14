"""Optional LLM planner. Falls back to the regex parser. Never talks to serial."""

from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot_bridge"))
from commands import parse_command

SYSTEM = """You control a bimanual wheeled robot through JSON steps only.
Allowed actions:
- move: vx, vy, wz, duration (seconds). +x forward, +y left, +wz CCW. |vx|,|vy|<=0.5, |wz|<=1.2
- wait: duration
- stop
- motor: enabled bool
- reset_odom
- gripper: side left|right|both, open 0..1 (1=open)
- arm_preset: side left|right|both, preset home|pick|place
- arm_joints: side, joints [6 floats]
If the user wants a visual skill you cannot do (find object, navigate by camera), say so in error.
Return ONLY JSON: {"steps":[...]} or {"error":"..."}.
"""


def _ollama_chat(text: str) -> dict[str, Any]:
    host = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434").rstrip("/")
    model = os.environ.get("OLLAMA_MODEL", "qwen2.5:7b")
    body = json.dumps(
        {
            "model": model,
            "stream": False,
            "format": "json",
            "messages": [
                {"role": "system", "content": SYSTEM},
                {"role": "user", "content": text},
            ],
        }
    ).encode()
    req = urllib.request.Request(
        host + "/api/chat",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        payload = json.loads(resp.read().decode())
    content = payload.get("message", {}).get("content", "")
    return json.loads(content)


def plan(text: str) -> dict[str, Any]:
    try:
        return parse_command(text)
    except ValueError:
        pass
    try:
        planned = _ollama_chat(text)
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
        raise ValueError(
            "unrecognized command and no local LLM. "
            "Use phrases like '前進 1 秒 然後 關閉夾爪', or start Ollama."
        )
    if planned.get("error"):
        raise ValueError(str(planned["error"]))
    if not planned.get("steps"):
        raise ValueError("LLM returned no steps")
    planned["text"] = text
    planned["source"] = "ollama"
    return planned
