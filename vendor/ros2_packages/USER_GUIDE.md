# 户外重载型四轮舵轮底盘 — ROS2 用户使用说明书

## 1. 系统概述

本系统由两个 ROS2 功能包组成，用于控制四轮独立舵轮底盘：

| 功能包 | 作用 |
|--------|------|
| `chassis_driver` | 串口驱动节点，负责与 STM32 底盘通讯，发布里程计/IMU/TF |
| `keyboard_control` | 键盘遥控节点，通过按键控制底盘运动，发布速度指令 |

## 2. 硬件连接

```
底盘 STM32 (USART1) ── CH340 USB ── 上位机 /dev/ttyUSB0
                          波特率 115200
```

**上电顺序**：先给底盘上电（STM32 启动），等待 3 秒舵机初始化完成，再插 USB 到上位机。

## 3. 编译

### 3.1 前提条件

上位机需已安装 ROS2 Jazzy 及编译工具：

```bash
# 确认 ROS2 已安装
source /opt/ros/jazzy/setup.sh
ros2 --version

# 确认 colcon 可用
which colcon
```

### 3.2 获取功能包

将 `chassis_driver` 和 `keyboard_control` 功能包放入 ROS2 workspace 的 `src/` 目录：

```bash
cp -r chassis_driver ~/ros2_ws/src/
cp -r keyboard_control ~/ros2_ws/src/
```

### 3.3 编译

```bash
cd ~/ros2_ws
rm -rf build/ install/          # 首次编译可跳过
source /opt/ros/jazzy/setup.sh
colcon build --packages-select chassis_driver keyboard_control
```

编译成功输出 `Summary: 2 packages finished`。

## 4. 启动

### 4.1 启动底盘驱动

```bash
# 终端1: 先 source 环境，再启动
source ~/ros2_ws/install/setup.sh
ros2 run chassis_driver chassis_node
```

启动后看到 `[OK] 串口: /dev/ttyUSB0 @ 115200` 表示连接成功。

> **如果提示"无法打开串口"**：检查 STM32 是否上电、USB 线是否插好，用 `ls /dev/ttyUSB*` 确认设备存在。

### 4.2 启动键盘遥控

```bash
# 终端2:
source ~/ros2_ws/install/setup.sh
ros2 run keyboard_control keyboard_control
```

启动后屏幕显示按键说明面板，即可开始控制。

### 4.3 一键启动（Launch 方式）

```bash
source ~/ros2_ws/install/setup.sh
ros2 launch chassis_driver chassis_driver.launch.py
```

> Launch 文件只启动 `chassis_driver`，键盘遥控需手动启动。

## 5. 键盘遥控操作

### 5.1 操作流程

```
1. 确认底盘上电、USB 已连接
2. 启动 chassis_driver
3. 启动 keyboard_control
4. 按 h 键启动电机
5. 使用方向键控制底盘
6. 使用完毕后按 g 键停止电机
7. Ctrl+C 退出
```

### 5.2 按键说明

```
+========================================+
|       底盘键盘遥控 - 操作说明          |
+========================================+
|  方向控制                             |
|  u    i    o   左前 / 前进 / 右前     |
|  j    k    l   左移 / 全停 / 右移     |
|  m    ,    .   左后 / 后退 / 右后     |
|  t         b   逆时针 / 顺时针自转    |
+----------------------------------------+
|  系统控制                             |
|  h   启动电机    g   停止电机         |
|  f   里程计复位                       |
+----------------------------------------+
|  速度档位 (预设)                      |
|  1   低速 0.25m/s   2   中速 0.35m/s  |
|  3   快速 0.50m/s   4   高速 0.70m/s  |
+----------------------------------------+
|  微调                                 |
|  q/z X+/-  w/x Y+/-  e/c Z+/-  r 重置|
|            Ctrl+C 退出                |
+----------------------------------------+
```

### 5.3 按键行为说明

- **方向键**：按下持续运动，松开约 0.3 秒后自动停车（安全保护）
- **微调键** (q/w/e/z/x/c)：每次调整对应方向的速度步进值，实时生效
- **r 键**：重置步进为默认值 (x=0.3, y=0.3, z=0.5)
- **档位键** (1-4)：一键切换到预设速度，无需手动微调
- **k 键**：紧急全停（仅停止运动，不关闭电机）
- **g 键**：关闭电机驱动，底盘进入安全停机状态
- **h 键**：重新启动电机驱动

## 6. 参数配置

### 6.1 chassis_driver 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 串口设备路径 |
| `serial_baud` | `115200` | 波特率 |
| `control_mode` | `1` | 控制模式：0=独立轮控，1=XYZ速度 |
| `start_flag` | `1` | 启动时是否使能电机 |
| `wheelbase` | `0.39` | 前后轴距 (m)，仅 Mode 0 使用 |
| `track_width` | `0.44` | 左右轮距 (m)，仅 Mode 0 使用 |
| `publish_rate` | `20.0` | odom/imu 发布频率 (Hz) |
| `cmd_timeout_ms` | `500` | cmd_vel 超时自动停车时间 (ms) |
| `odom_frame` | `odom` | 里程计坐标系名称 |
| `base_frame` | `base_link` | 机器人基座坐标系名称 |

### 6.2 keyboard_control 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `step_x` | `0.30` | X方向步进 (m/s)，对应前进速度 |
| `step_y` | `0.30` | Y方向步进 (m/s)，对应横移速度 |
| `step_z` | `0.50` | Z方向步进 (rad/s)，对应自转角速度 |
| `publish_rate` | `30.0` | 指令发布频率 (Hz) |

### 6.3 命令行传参

```bash
# 修改串口设备
ros2 run chassis_driver chassis_node --ros-args -p serial_port:=/dev/ttyUSB1

# 修改默认速度
ros2 run keyboard_control keyboard_control --ros-args -p step_x:=0.5 -p step_z:=1.0

# 启动时不使能电机（安全模式）
ros2 run chassis_driver chassis_node --ros-args -p start_flag:=0
```

## 7. 控制模式说明

### 7.1 Mode 1 — XYZ速度模式（默认，推荐）

- **工作原理**：上位机发送 vx/vx/vth（前向/横向/旋转速度）给 STM32，STM32 自己完成四轮舵角+轮速解算
- **优点**：舵角限幅、180°翻转优化、角度过渡全部由固件处理，上位机无需关心底层
- **适用场景**：键盘遥控、ROS2 导航（nav2）、自主移动

```bash
ros2 run chassis_driver chassis_node --ros-args -p control_mode:=1
```

### 7.2 Mode 0 — 独立轮控模式

- **工作原理**：上位机计算四个轮的目标舵角和线速度，直接下发
- **优点**：可调试单个轮子、实现自定义舵角策略
- **适用场景**：底盘调试、轮子标定

```bash
ros2 run chassis_driver chassis_node --ros-args -p control_mode:=0
```

> **注意**：Mode 0 时键盘遥控仍正常工作，只是运动学解算从 STM32 转移到了上位机。

### 7.3 运行时切换

```bash
# 在另一个终端
ros2 param set /chassis_node control_mode 0   # 切换到独立轮控
ros2 param set /chassis_node control_mode 1   # 切换回 XYZ 模式
```

## 8. 话题一览

### 8.1 chassis_driver 话题

| 话题 | 类型 | 方向 | 频率 | 说明 |
|------|------|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 订阅 | — | 速度控制指令 |
| `motor_enable` | `std_msgs/Bool` | 订阅 | — | True=启动电机，False=停止 |
| `reset_odom` | `std_msgs/Empty` | 订阅 | — | 收到即复位里程计 |
| `odom` | `nav_msgs/Odometry` | 发布 | 20Hz | 里程计（位姿+速度） |
| `imu` | `sensor_msgs/Imu` | 发布 | 20Hz | IMU（姿态+角速度+线加速度） |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | 20Hz | odom→base_link 坐标变换 |

### 8.2 keyboard_control 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 发布 | 键盘速度指令 |
| `motor_enable` | `std_msgs/Bool` | 发布 | 电机启停 |
| `reset_odom` | `std_msgs/Empty` | 发布 | 里程计复位 |

## 9. 实用命令

### 9.1 查看底盘状态

```bash
# 实时查看里程计
ros2 topic echo /odom

# 实时查看 IMU
ros2 topic echo /imu

# 查看坐标变换
ros2 run tf2_ros tf2_echo odom base_link

# 查看所有话题
ros2 topic list

# 查看话题发布频率
ros2 topic hz /odom
```

### 9.2 手动控制

```bash
# 手动发送速度指令（前进 0.3 m/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10

# 手动停止电机
ros2 topic pub /motor_enable std_msgs/msg/Bool "{data: false}" -1

# 手动启动电机
ros2 topic pub /motor_enable std_msgs/msg/Bool "{data: true}" -1

# 手动复位里程计
ros2 topic pub /reset_odom std_msgs/msg/Empty "{}" -1
```

### 9.3 对接 ROS2 导航 (nav2)

```bash
# chassis_driver 已发布标准 /odom 和 /tf，可直接对接 nav2
# 在 nav2 配置中将 cmd_vel 话题名设为 "cmd_vel"（与 chassis_driver 一致）

ros2 launch nav2_bringup navigation_launch.py
```

## 10. 开机自启

如果希望底盘驱动随系统自动启动：

```bash
# 创建 systemd 服务
sudo tee /etc/systemd/system/chassis-driver.service << 'EOF'
[Unit]
Description=Chassis Driver ROS2 Node
After=network.target

[Service]
Type=simple
User=umeko
Environment="PATH=/home/umeko/.local/bin:/usr/bin:/bin"
ExecStart=/bin/bash -c 'source /home/umeko/ros2_ws/install/setup.sh && ros2 run chassis_driver chassis_node --ros-args -p start_flag:=1'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 启用
sudo systemctl enable chassis-driver
sudo systemctl start chassis-driver

# 查看状态
sudo systemctl status chassis-driver
```

> **注意**：开机自启时 `start_flag` 设为 1（直接使能电机），请确保底盘已上电且 USB 已连接。如需安全模式改为 `start_flag:=0`。

## 11. 故障排查

| 现象 | 原因 | 解决方法 |
|------|------|----------|
| `无法打开串口` | STM32 未上电或 USB 未插 | 检查底盘供电，`ls /dev/ttyUSB*` 确认设备存在 |
| 键盘按键无反应 | 终端焦点不在键盘窗口 | 点击键盘控制终端窗口，确保在输入状态 |
| 底盘不动 | 电机未使能 | 按 `h` 键启动电机 |
| odom 数据为 0 | 串口刚连接，数据未稳定 | 等待 2-3 秒再看 |
| 底盘朝反方向移动 | 电机线序或轮子安装方向问题 | 联系技术人员检查硬件接线 |
| 编译失败 | 缺少 ROS2 依赖 | 检查 `package.xml` 中的依赖是否已安装 |
| 命令未找到 `ros2` | 未 source 环境 | 执行 `source /opt/ros/jazzy/setup.sh` |

## 12. 坐标系说明

```
    前方 (+X)
      ↑
      │   B(前左)    A(前右)
      │      ○        ○
      │
      │      ○        ○
      │   C(后左)    D(后右)
      └──────────────→ 右方 (+Y)
```

- **前进/后退**：沿 X 轴方向
- **左移/右移**：沿 Y 轴方向
- **逆时针/顺时针**：绕 Z 轴旋转（Z 轴垂直地面向上）

## 13. 安全注意事项

1. **启动前确认**：底盘周围 1 米内无障碍物和人员
2. **紧急停车**：按 `g` 键关闭电机，或按 `k` 键发送零速指令
3. **超时保护**：上位机与底盘 USB 断开后，cmd_vel 超时 500ms 自动停车
4. **电压监控**：底盘 STM32 固件在电压低于 18.5V 时自动停机保护
5. **调试模式**：`start_flag:=0` 启动可防止上电即走，适合调试场景
