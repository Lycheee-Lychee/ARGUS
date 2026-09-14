#include "motor.h"

/* 电机PWM初始化
 * 引脚:
 *   PWM: PC6(TIM8_CH1=B), PC7(TIM8_CH2=A), PC8(TIM8_CH3=D), PC9(TIM8_CH4=C)
 *   方向: PA4(IND1), PA5(IND2), PB0(INC1), PB15(INC2), PC1(INB1), PC2(INB2), PC3(INA1), PC4(INA2)
 * 频率: 72MHz / (arr+1) / (psc+1) = 72MHz / 7200 / 1 = 10kHz (psc=0时)
 *       实际使用 40kHz: 72MHz / 7200 = 10kHz...
 *        原代码 arr=7199, psc=0 → 72MHz/7200 = 10kHz
 */
void Motor_PWM_Init(u16 arr, u16 psc)
{
	RCC->APB2ENR|=1<<4;       // PORTC时钟使能
	RCC->APB2ENR|=1<<3;       // PORTB时钟使能
	RCC->APB2ENR|=1<<2;       // PORTA时钟使能

	/* 方向控制引脚初始化 — 全部推挽输出 */
	GPIOA->CRL&=0XFF00FFFF;   // PA4 PA5 清除原设置
	GPIOA->CRL|=0X00220000;   // PA4 PA5 推挽输出 50MHz

	GPIOC->CRL&=0XFFF0000F;   // PC1 PC2 PC3 PC4 清除原设置
	GPIOC->CRL|=0X00022220;   // PC1 PC2 PC3 PC4 推挽输出 50MHz

	GPIOB->CRL&=0XFFFFFFF0;   // PB0 清除原设置
	GPIOB->CRL|=0X00000002;   // PB0 推挽输出 50MHz

	GPIOB->CRH&=0X0FFFFFFF;   // PB15 清除原设置
	GPIOB->CRH|=0X20000000;   // PB15 推挽输出 50MHz

	//默认方向=刹车
	INA1=0; INA2=0;
	INB1=0; INB2=0;
	INC1=0; INC2=0;
	IND1=0; IND2=0;

	//PWM输出引脚初始化
	RCC->APB2ENR|=1<<13;       // 使能TIM8时钟
	RCC->APB2ENR|=1<<4;        // PORTC时钟使能

	GPIOC->CRL&=0X00FFFFFF;    // PORTC 6 7 复用输出
	GPIOC->CRL|=0XBB000000;
	GPIOC->CRH&=0XFFFFFF00;    // PORTC 8 9 复用输出
	GPIOC->CRH|=0X000000BB;

	TIM8->ARR=arr;             // 自动重装值
	TIM8->PSC=psc;             // 预分频

	TIM8->CCMR1|=6<<4;         // CH1 PWM1模式
	TIM8->CCMR1|=6<<12;        // CH2 PWM1模式
	TIM8->CCMR2|=6<<4;         // CH3 PWM1模式
	TIM8->CCMR2|=6<<12;        // CH4 PWM1模式

	TIM8->CCMR1|=1<<3;         // CH1 预装载使能
	TIM8->CCMR1|=1<<11;        // CH2 预装载使能
	TIM8->CCMR2|=1<<3;         // CH3 预装载使能
	TIM8->CCMR2|=1<<11;        // CH4 预装载使能

	TIM8->CCER|=1<<0;          // CH1 输出使能
	TIM8->CCER|=1<<4;          // CH2 输出使能
	TIM8->CCER|=1<<8;          // CH3 输出使能
	TIM8->CCER|=1<<12;         // CH4 输出使能

	TIM8->CCR1=0;
	TIM8->CCR2=0;
	TIM8->CCR3=0;
	TIM8->CCR4=0;
	TIM8->BDTR |= 1<<15;       // 刹车死区使能（PWM输出前提）
	TIM8->CR1=0x0080;          // ARPE 使能
	TIM8->CR1|=0x01;           // 使能定时器
}

void Steering_engine_PWM_Init(u16 arr, u16 psc)
{
	RCC->APB2ENR|=1<<11;       // 使能TIM1时钟
	RCC->APB2ENR|=1<<2;        // PORTA时钟使能
	GPIOA->CRH&=0XFFFF0FF0;    // PORTA8 11 复用输出
	GPIOA->CRH|=0X0000B00B;

	TIM1->ARR=arr;
	TIM1->PSC=psc;

	TIM1->CCMR1|=6<<4;         // CH1 PWM2模式
	TIM1->CCMR2|=6<<12;        // CH4 PWM2模式

	TIM1->CCMR1|=1<<3;         // CH1 预装载使能
	TIM1->CCMR2|=1<<11;        // CH4 预装载使能

	TIM1->CCER|=1<<0;          // CH1 输出使能
	TIM1->CCER|=1<<12;         // CH4 输出使能
	TIM1->CCR1=0;
	TIM1->CCR4=0;
	TIM1->BDTR |= 1<<15;
	TIM1->CR1=0x0080;
	TIM1->CR1|=0x01;
}

/* 双引脚H桥方向控制
 *   motor>0: (方向1,方向2) = (0,1) → 电机正转
 *   motor<0: (方向1,方向2) = (1,0) → 电机反转
 *   motor=0: (方向1,方向2) = (0,0) → 刹车
 * PWM占空比 = |motor| / (ARR+1)
 */
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
	if(motor_a<0)     INA1=1, INA2=0, PWMA=-motor_a;
	else              INA1=0, INA2=1, PWMA=+motor_a;

	if(motor_b<0)     INB1=1, INB2=0, PWMB=-motor_b;
	else              INB1=0, INB2=1, PWMB=+motor_b;

	if(motor_c<0)     INC1=1, INC2=0, PWMC=-motor_c;
	else              INC1=0, INC2=1, PWMC=+motor_c;

	if(motor_d<0)     IND1=1, IND2=0, PWMD=-motor_d;
	else              IND1=0, IND2=1, PWMD=+motor_d;
}
