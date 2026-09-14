#include "exti.h"

extern void Usart_rt_data(void);
extern void Key(void);
extern void APP_Control(void);
extern void PS2_Control(void);
extern void CAN1_Receive_data(void);
extern void Motion_control(void);
extern void Abnormal_state_handling(void);
extern void CAN1_SEND_data(u8, u8, u8, u8, float, float, float, float);
extern void Set_Pwm(int, int, int, int);
extern int velocity_Control(int idx, int encoder, int target);
extern int Read_Encoder(u8);
extern void Read_DMP(void);
extern void Led_Flash(u16);
extern int Get_battery_voltage(void);

extern volatile u8 delay_30, delay_flag;
extern volatile u8 Start_Flag, Start_Flag_one;
extern u8 Start_Flag_STOP;
extern volatile u8 Flag_STOP;
extern volatile int Motor_A, Motor_B, Motor_C, Motor_D;
extern float Encoder_A, Encoder_B, Encoder_C, Encoder_D;
extern volatile float Roll, Pitch, Yaw;
extern volatile float Angle_target_A, Angle_target_B, Angle_target_C, Angle_target_D;
extern volatile float Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D;
extern float Angle_error_A, Angle_error_B, Angle_error_C, Angle_error_D;
extern float offset_a, offset_b, offset_c, offset_d;
extern int Velocity_target_A, Velocity_target_B, Velocity_target_C, Velocity_target_D;
extern volatile int Voltage;
extern int count_size, count_sum;
extern u8 CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D;
extern float Yaw_target;

/**************************************************************************
外部中断初始化
**************************************************************************/
void EXTI_Init(void)
{
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟
	GPIOB->CRH&=0XFFF0FFFF;
	GPIOB->CRH|=0X00080000;//PB12浮空输入
  GPIOB->ODR|=1<<12;     //PB12上拉
	Ex_NVIC_Config(GPIO_B,12,FTIR);		//下降沿触发
	MY_NVIC_Init(2,1,EXTI15_10_IRQn,2);  	//抢占2，响应优先级1，组2
}

/**************************************************************************
MPU6050 INT引脚5毫秒中断
**************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	 if(INT==0)
	{
		EXTI->PR=1<<12; //清除LINE12上的中断标志位

		 //****************传感器数据获取****************************//
		 //编码器IIR低通滤波
			 { static float enc_a=0,enc_b=0,enc_c=0,enc_d=0;
			   enc_a=enc_a*0.85f+(float)Read_Encoder(5)*0.15f;
			   enc_b=enc_b*0.85f+(float)Read_Encoder(3)*0.15f;
			   enc_c=enc_c*0.85f+(float)Read_Encoder(2)*0.15f;
			   enc_d=enc_d*0.85f+(float)Read_Encoder(4)*0.15f;
			   Encoder_A=enc_a;Encoder_B=enc_b;Encoder_C=enc_c;Encoder_D=enc_d;}
     Read_DMP();//获取角度信息

		 Usart_rt_data();	//串口数据收发处理

		//*******************指令控制*****************//
		 Key();//按键检测
		 APP_Control();//APP蓝牙指令
		 PS2_Control();//PS2指令
		 CAN1_Receive_data();//can 处理接收的数据	获取当前角度
		 Motion_control();//运动姿态控制

 ////异常状态关闭电机////
		 Abnormal_state_handling();
 ////异常状态关闭电机////

		 if(Start_Flag==1 && Start_Flag_one==1)
		   {
				CAN_EN_A=1,CAN_EN_B=1,CAN_EN_C=1,CAN_EN_D=1;
				if(Flag_STOP==0){
			    Motor_A = +velocity_Control(0, +Encoder_A, +Velocity_target_A);
			    Motor_B = -velocity_Control(1, -Encoder_B, +Velocity_target_B);
			    Motor_C = -velocity_Control(2, -Encoder_C, +Velocity_target_C);
			    Motor_D = +velocity_Control(3, +Encoder_D, +Velocity_target_D);}
			}
		 if(Start_Flag==1 && Start_Flag_one==0)
		   {
				Velocity_target_A=0;Velocity_target_B=0;Velocity_target_C=0;Velocity_target_D=0;
				CAN_EN_A=0,CAN_EN_B=0,CAN_EN_C=0,CAN_EN_D=0;
				for(int i=0;i<4;i++) velocity_Control(i, 0, 0);
			}
		 if(Start_Flag==0)
			 {
	        CAN_EN_A=0,CAN_EN_B=0,CAN_EN_C=0,CAN_EN_D=0;
	        for(int i=0;i<4;i++) velocity_Control(i, 0, 0);
			  }

	//舵角硬限幅
	if(Angle_target_A > SERVO_LIMIT_A_MAX)Angle_target_A=SERVO_LIMIT_A_MAX;
	if(Angle_target_B > SERVO_LIMIT_B_MAX)Angle_target_B=SERVO_LIMIT_B_MAX;
	if(Angle_target_C > SERVO_LIMIT_C_MAX)Angle_target_C=SERVO_LIMIT_C_MAX;
	if(Angle_target_D > SERVO_LIMIT_D_MAX)Angle_target_D=SERVO_LIMIT_D_MAX;
	if(Angle_target_A < SERVO_LIMIT_A_MIN)Angle_target_A=SERVO_LIMIT_A_MIN;
	if(Angle_target_B < SERVO_LIMIT_B_MIN)Angle_target_B=SERVO_LIMIT_B_MIN;
	if(Angle_target_C < SERVO_LIMIT_C_MIN)Angle_target_C=SERVO_LIMIT_C_MIN;
	if(Angle_target_D < SERVO_LIMIT_D_MIN)Angle_target_D=SERVO_LIMIT_D_MIN;

		  CAN1_SEND_data(CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D,
		                  -(Angle_target_A - Angle_error_A)*offset_a,
		                  -(Angle_target_B - Angle_error_B)*offset_b,
		                  -(Angle_target_C - Angle_error_C)*offset_c,
		                  -(Angle_target_D - Angle_error_D)*offset_d);//can 发送目标角度

    //****************电机控制****************************//
		 if(Start_Flag==1&& Start_Flag_one==1&& Start_Flag_STOP==0&& Flag_STOP==0)
		   {
			  Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);	//PWM赋值
		   }
		 else //关闭电机，清除部分参数状态
			 {
				Motor_A=0;Motor_B=0;Motor_C=0;Motor_D=0;
		    Set_Pwm(0,0,0,0);
				Yaw_target= Yaw;
			 }

//************************************其它**************************************************//
    if(Voltage<BATT_LED_CRITICAL && Voltage>700)Led_Flash(20);//电量严重不足
		else if(Voltage<BATT_LED_WARN)Led_Flash(40);//电量不足
	  else Led_Flash(100);//led闪烁

		count_sum+=Get_battery_voltage();//电压采样累计
	  count_size++;
		if(count_size==200) Voltage=count_sum/count_size,count_sum=0,count_size=0;//求平均电压

		if(delay_flag==1)
		  {
			 if(++delay_30==2)delay_30=0,delay_flag=0;  //给主函数提供10ms的精准延时
		  }
 }

}