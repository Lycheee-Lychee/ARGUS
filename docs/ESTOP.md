# E-stop and access

## What `/stop` does

1. Cancels the task runner
2. Chassis velocity = 0, motors disabled
3. Arm LeRobot buses disconnect (`disable_torque_on_disconnect`)
4. cmd_vel timeout 500 ms if the web client dies

Use Space or the red STOP on http://10.68.40.36:8080/

Physical power cut is still required if software hangs.

## Who can call the robot

Bridge listens on `0.0.0.0:8080` with **no password**. Anyone on the same campus net can teleop.

Do not port-forward 8080 to the internet.

Optional later: `ROBOT_TOKEN` header. Not enabled yet so you cannot lock yourself out remotely.

## Agent rules

- If `mock: true` or inventory `disconnected`, do not claim a real-world success.
- Refuse door / pick / SAR until `vla_act.wired`.
