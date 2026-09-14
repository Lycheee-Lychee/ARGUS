/* sys.h - 系统头文件
 * 替代原Keil依赖的sys.h
 * 硬件驱动头文件和全局变量声明完整保留
 */
#ifndef __SYS_H
#define __SYS_H

#include <stm32f10x.h>

//底盘参数配置
#include "chassis_config.h"

//常用宏定义
#define to_rad  0.017453f   // 角度转弧度
#define to_deg  57.29578f   // 弧度转角度
#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

//位带操作宏
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr    (GPIOA_BASE+12)
#define GPIOB_ODR_Addr    (GPIOB_BASE+12)
#define GPIOC_ODR_Addr    (GPIOC_BASE+12)
#define GPIOD_ODR_Addr    (GPIOD_BASE+12)
#define GPIOE_ODR_Addr    (GPIOE_BASE+12)
#define GPIOF_ODR_Addr    (GPIOF_BASE+12)
#define GPIOG_ODR_Addr    (GPIOG_BASE+12)

#define GPIOA_IDR_Addr    (GPIOA_BASE+8)
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)

//外部中断GPIO选择
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6

#define FTIR   1
#define RTIR   2

//JTAG模式
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00

#define SYSTEM_SUPPORT_UCOS  0
#define Len 30  // 22基础+8 PID参数(Velocity_KP/KI, Gyro_KP/KI/KD, line_KP/KI/KD)

//硬件驱动头文件
#include "delay.h"
#include "led_drv.h"
#include "key.h"
#include "oled.h"
#include "usart.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "exti.h"
#include "pstwo.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "DMA.h"
#include "CAN.h"

//全局变量声明
extern int anjian_app, huakuai_app, yaogan_app;
extern volatile u8 Start_Flag, Start_Flag_one;
extern u8 PS2_KEY, PS2_LX, PS2_LY, PS2_RX, PS2_RY;
extern volatile u8 flag_move, MOVE_mode;
extern int STEP;
extern float Velocity_center;
extern float Encoder_A, Encoder_B, Encoder_C, Encoder_D;
extern volatile int Motor_A, Motor_B, Motor_C, Motor_D;
extern volatile int Voltage;
extern volatile float Roll, Pitch, Yaw, gyro_Roll, gyro_Pitch, gyro_Yaw, accel_x, accel_y, accel_z;
extern u8 send_buf[130];
extern u8 FLAG_USART, data_len;
extern u8 USART1_func;
extern u8 USART1_data[54];
extern float data_u[30];
extern u16 FLAG_USART_ON;
extern volatile float Angle_target_A, Angle_current_A;
extern volatile float Angle_target_B, Angle_current_B;
extern volatile float Angle_target_C, Angle_current_C;
extern volatile float Angle_target_D, Angle_current_D;
extern u8 txbuf[8], rxbuf[8], Rxbuf_1[8], Rxbuf_2[8], Rxbuf_3[8], Rxbuf_4[8];
extern u8 FLAG_CAN;
extern int Velocity_target_A, Velocity_target_B, Velocity_target_C, Velocity_target_D;

//扩展全局变量
extern u16 CAN_ID1, CAN_ID2, CAN_ID3, CAN_ID4;
extern u16 FLAG_CAN_ON;
extern u8 CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D, can_ser;
extern u8 abnormal, fault;
extern u8 Start_Flag_STOP;
extern volatile u8 Flag_STOP;
extern u8 show_flag, show_on_flag;
extern volatile u8 Flag_Mode;
extern u16 Flag_Z1, Flag_Z2, Flag_Z3, Flag_Z4;
extern int Turn_R;
extern float DIS_speed_A, DIS_speed_B, DIS_speed_C, DIS_speed_D, DIS_speed;
extern int Last_Motor_A, Last_Motor_B, Last_Motor_C, Last_Motor_D;
extern float Velocity_X, Velocity_Y, Turn_Z;
extern float velocity_x, velocity_y, velocity_z;
extern float Angle_DIS_A, Angle_DIS_B, Angle_DIS_C, Angle_DIS_D;
extern float Gyro_KP, Gyro_KI, Gyro_KD;
extern float line_KP, line_KI, line_KD;
extern float Velocity_KP, Velocity_KI;
extern float Angle_MID_A, Angle_MID_B, Angle_MID_C, Angle_MID_D;
extern float Velocity_MID_A, Velocity_MID_B, Velocity_MID_C, Velocity_MID_D;
extern float Velocity_MID_X, Velocity_MID_Y, Velocity_MID_Z, Velocity_MID_H;
extern float Angle_error_A, Angle_error_B, Angle_error_C, Angle_error_D;
extern float Current_angle_A, Current_angle_B, Current_angle_C, Current_angle_D;
extern float offset_a, offset_b, offset_c, offset_d;
extern float Yaw_target;
extern float Flag_init;
extern double Delta_x, Delta_y, Delta_th;
extern double vx, vy, vth;

//函数声明
void Stm32_Clock_Init(u8 PLL);
void Sys_Soft_Reset(void);
void Sys_Standby(void);
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);
void JTAG_Set(u8 mode);

/* WFI / interrupt control — GCC inline assembly wrappers */
void WFI_SET(void);
void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(u32 addr);

//标准C头文件
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#endif /* __SYS_H */
