/* chassis_config.h - 底盘参数集中定义
 * 换底盘时只改本文件
 * 当前: 户外重载型四轮舵轮底盘(大功率版)
 * 单位: 长度mm, 角度度, 电压mV
 */

#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

/*---- 轮子 & 编码器 ----*/
#define WHEEL_DIAMETER_MM      250.0f
#define WHEEL_CIRCUMFERENCE_MM (3.14159265358979f * WHEEL_DIAMETER_MM)
#define WHEEL_CIRCUMFERENCE_M  (WHEEL_CIRCUMFERENCE_MM / 1000.0f)
#define MOTOR_GEAR_RATIO       19.0f
#define ENCODER_PPR            4096
#define ENCODER_CPR            (ENCODER_PPR * 4)   /* 4096*4=16384 */
#define CONTROL_FREQ_HZ        200.0f
#define VEL_TO_ENCODER  (MOTOR_GEAR_RATIO * (float)ENCODER_CPR / WHEEL_CIRCUMFERENCE_M / (float)CONTROL_FREQ_HZ)

/*---- 底盘几何 ----*/
#define WHEELBASE_MM           390
#define TRACK_WIDTH_MM         440

/*---- 舵机角度校准 ----*/
/* B/D镜像安装: 横移时A/C=+90, B/D=-90 */
#define SERVO_OFFSET_A  1.01f
#define SERVO_OFFSET_B  0.97f
#define SERVO_OFFSET_C  1.03f
#define SERVO_OFFSET_D  0.99f
#define SERVO_MIRROR_BD         1

/* 舵角硬限幅(度) - A/C轮和B/D轮不对称 */
#define SERVO_LIMIT_A_MAX     +92
#define SERVO_LIMIT_A_MIN     -38
#define SERVO_LIMIT_B_MAX     +38
#define SERVO_LIMIT_B_MIN     -92
#define SERVO_LIMIT_C_MAX     +92
#define SERVO_LIMIT_C_MIN     -38
#define SERVO_LIMIT_D_MAX     +38
#define SERVO_LIMIT_D_MIN     -92

/*---- PID参数 ----*/
#define VELOCITY_KP     0.6f
#define VELOCITY_KI     2.1f
#define GYRO_KP         1.5f
#define GYRO_KI         0.001f
#define GYRO_KD         0.1f
#define LINE_KP         500.0f
#define LINE_KI         3.0f
#define LINE_KD         500.0f
#define DEFAULT_SPEED   300

/* PS2手柄/串口 4档速度(编码器计数单位)
 * 换算: 1单位≈0.0005m/s
 * 300→0.15  500→0.25  700→0.35  900→0.45 m/s */
#define SPEED_GEAR_1    300
#define SPEED_GEAR_2    500
#define SPEED_GEAR_3    700
#define SPEED_GEAR_4    900

/*---- 阿克曼转向 ----*/
#define MOVE_ANGLE_TRANSLATION  90
#define MOVE_ANGLE_DIAGONAL     35
#define MOVE_ANGLE_SPIN         54
#define ACKERMANN_4W_MAX_ANGLE      36
#define ACKERMANN_2W_MAX_ANGLE      36
#define ACKERMANN_4W_TURN_RADIUS    1500
#define ACKERMANN_2W_TURN_RADIUS    2600
#define ANGLE_SLEW_RATE     0.5f
#define ANGLE_SLEW_RATE_XYZ 0.5f

/* XYZ模式轮速限制 */
#define MAX_WHEEL_SPEED     1.1f
#define MIN_LINEAR_SPEED    0.01f
#define MAX_ANGULAR_SPEED   5.2f
#define MIN_ANGULAR_SPEED   0.048f

/*---- 电机驱动限制 ----*/
#define PWM_ARR             7199
#define PWM_MAX             7199
#define PWM_SLEW_LIMIT      500

/*---- 电池电压阈值(mV) ----*/
/* 6S锂电: 标称22.2V, 满电25.2V */
#define BATT_WARN            2000    /* 20.0V 闪烁提醒 */
#define BATT_STOP            1850    /* 18.5V 停机保护 */
#define BATT_USB_ONLY        700     /* 7.0V  仅USB供电 */
#define BATT_LED_WARN        2000
#define BATT_LED_CRITICAL    1850

/*---- 异常检测阈值 ----*/
#define FAULT_FEEDBACK_TIMEOUT_MS   1500
#define FAULT_STALL_TIMEOUT_MS      5000
#define FAULT_VOLTAGE_TIMEOUT_MS    20000
#define COMM_SERIAL_TIMEOUT         600
#define COMM_CAN_TIMEOUT            100
#define FAULT_SINGLE_ENCODER_MS     1500

/*---- CAN总线 ----*/
/* 500Kbps @ 36MHz APB1 */
#define CAN_TSJW    1
#define CAN_TBS2    2
#define CAN_TBS1    3
#define CAN_BRP     6
#define CAN_ID_A    0x121
#define CAN_ID_B    0x122
#define CAN_ID_C    0x123
#define CAN_ID_D    0x124

/*---- OLED显示 ----*/
#define OLED_PAGE_SWITCH_TICKS_1    100
#define OLED_PAGE_SWITCH_TICKS_2    200

#endif
