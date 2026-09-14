# chassis_driver — 底盘串口驱动

通过 USB 转串口和 STM32 通讯，订阅 cmd_vel 发布 odom/imu。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | cmd_vel | geometry_msgs/Twist | 速度指令 |
| 订阅 | motor_enable | std_msgs/Bool | 启停电机 |
| 订阅 | reset_odom | std_msgs/Empty | 复位里程计 |
| 发布 | odom | nav_msgs/Odometry | 里程计 |
| 发布 | imu | sensor_msgs/Imu | IMU数据 |
| 发布 | /tf | TFMessage | odom→base_link |

## 串口协议

- 波特率: 115200, 8N1
- 下行 (PC→STM32): 53字节, 帧头 0xAA 0xAA
- 上行 (STM32→PC): 125字节, 帧头 0xAA 0xAA 0xF1
- 字节序: 大端 (Big-Endian)
- 校验: 前52字节求和 mod 256

### 下行帧结构 (53字节)

| 偏移 | 长度 | 内容 |
|------|------|------|
| 0-1 | 2 | 帧头 0xAA 0xAA |
| 4-7 | 4 | 启停标志 (1.0f=启动, 0.0f=停止) |
| 8-11 | 4 | 控制模式 (1.0f=XYZ速度, 0.0f=独立轮控) |
| 12-27 | 16 | Mode0: 4轮舵角(deg) / Mode1: Velocity_X(m/s) |
| 28-43 | 16 | Mode0: 4轮速度(m/s) / Mode1: Velocity_Y,Velocity_Z |
| 44-47 | 4 | 里程计复位 (0.0f=复位, 1.0f=保持) |
| 52 | 1 | 校验和 |

### 上行帧结构 (125字节)

| 偏移 | 长度 | 内容 |
|------|------|------|
| 0-2 | 3 | 帧头 0xAA 0xAA 0xF1 |
| 4-19 | 16 | 启停标志 + 4轮舵角(deg) + 4轮速度(m/s) |
| 40-51 | 12 | 陀螺仪(deg/s) + 加速度(raw) + 姿态角(deg) |
| 76-83 | 8 | 电压(10mV) + 里程计x(m),y(m),θ(rad) |
| 124 | 1 | 校验和 |

## 控制模式

- **Mode 1 (默认)**: 发送 vx/vy/vth，STM32 做运动学解算
- **Mode 0**: 上位机做逆运动学，发送4轮独立角度+速度

两种模式的逆运动学公式已和 STM32 固件对齐，行为一致。

## 参数

见 config/params.yaml。关键参数:
- `serial_port`: 串口设备路径
- `control_mode`: 1=XYZ速度, 0=独立轮控
- `wheelbase` / `track_width`: 底盘几何参数 (Mode 0 使用)
- `cmd_timeout_ms`: cmd_vel 超时自动停车

## 使用

```bash
# 编译
colcon build --packages-select chassis_driver

# 运行 (使用默认参数)
ros2 run chassis_driver chassis_node

# 运行 (加载参数文件)
ros2 launch chassis_driver chassis_driver.launch.py
```
