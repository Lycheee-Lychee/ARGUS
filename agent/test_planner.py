from __future__ import annotations

import json
import urllib.error
from unittest import mock

import planner


def test_known_phrase_skips_llm():
    out = planner.plan("前進 1 秒 然後 關閉夾爪")
    assert out["source"] == "parser"
    assert out["steps"][0]["action"] == "move"
    assert out["steps"][1]["action"] == "gripper"


def test_unknown_without_key_raises():
    with mock.patch.dict("os.environ", {"DEEPSEEK_API_KEY": "", "LLM_API_KEY": ""}, clear=False):
        try:
            planner.plan("把左邊夾爪輕輕合上然後停")
        except ValueError as exc:
            assert "DEEPSEEK_API_KEY" in str(exc) or "unrecognized" in str(exc)
        else:
            raise AssertionError("expected ValueError")


def test_deepseek_fallback():
    fake = {"steps": [{"action": "gripper", "side": "left", "open": 0.0}, {"action": "stop"}]}
    with mock.patch.dict("os.environ", {"DEEPSEEK_API_KEY": "sk-test"}, clear=False):
        with mock.patch.object(planner, "_deepseek_chat", return_value=fake):
            out = planner.plan("輕輕合上左夾爪然後停下來")
    assert out["source"] == "deepseek"
    assert out["steps"][0]["side"] == "left"


def test_rejects_unknown_action():
    try:
        planner._validate({"steps": [{"action": "vla_act"}]}, "x", "deepseek")
    except ValueError:
        return
    raise AssertionError("expected invalid action")


def test_extract_fenced_json():
    raw = 'sure\n```json\n{"steps":[{"action":"stop"}]}\n```\n'
    assert planner._extract_json(raw)["steps"][0]["action"] == "stop"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(name, "ok")
    print("planner tests ok")
