# Robot control skill (OpenClaw / ABot-Claw later)

Call the Thor robot_bridge. Do not talk to serial ports directly.

Base URL: `http://127.0.0.1:8080` (on Thor) or `http://10.68.40.36:8080` (from laptop).

## Tools

- `POST /command` `{"text": "..."}` — natural language, sequenced
- `POST /task` `{"steps":[...]}` — explicit primitives
- `POST /stop` — e-stop
- `GET /state` — chassis + arms + current task

## Allowed phrases

前進/後退/左移/右移/左轉/右轉 + 時長  
打開夾爪 / 關閉夾爪  
左臂回home  
停

Example: `前進 1 秒 然後 關閉夾爪`

If hardware is unplugged the bridge stays in mock mode. Still call the same APIs.
