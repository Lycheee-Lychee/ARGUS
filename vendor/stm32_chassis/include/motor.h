#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>

/* PWM 输出通道 — TIM8 四路 PWM
 * 户外重载型：大功率驱动板使用双引脚方向控制（H桥）
 * PWM映射: A=CCR2(PC7), B=CCR1(PC6), C=CCR4(PC9), D=CCR3(PC8)
 */
#define PWMA   TIM8->CCR2
#define PWMB   TIM8->CCR1
#define PWMC   TIM8->CCR4
#define PWMD   TIM8->CCR3

/* 方向控制引脚 — 双引脚 H 桥模式
 * (1,0=正转 / 0,1=反转 / 0,0=刹车)
 * 户外重载型引脚分配:
 *  电机A: INA1=PC3, INA2=PC4
 *  电机B: INB1=PC1, INB2=PC2
 *  电机C: INC1=PB0, INC2=PB15
 *  电机D: IND1=PA4, IND2=PA5
 */
#define INA1   PCout(3)
#define INB1   PCout(1)
#define INC1   PBout(0)
#define IND1   PAout(4)

#define INA2   PCout(4)
#define INB2   PCout(2)
#define INC2   PBout(15)
#define IND2   PAout(5)

void Motor_PWM_Init(u16 arr, u16 psc);
void Steering_engine_PWM_Init(u16 arr, u16 psc);
#endif
