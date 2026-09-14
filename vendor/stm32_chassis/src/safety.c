#include "sys.h"

extern void Abnormal_state_handling(void);
extern void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d);
extern void Led_Flash(u16 time);
u8 enc_fault=0;
u8 enc_fault_locked=0;
u8 estop_pressed=0;
extern u16 Flag_Z5, Flag_Z6;

void Abnormal_state_handling(void){//异常状态处理

			    //****************编码器异常优先检测****************************//
			    enc_fault=0;
			    if(
			 		   (fabs(Encoder_A)<6&&fabs(Encoder_B)>60&&fabs(Encoder_C)>60&&fabs(Encoder_D)>60)
				    ||
					  (fabs(Encoder_B)<6&&fabs(Encoder_C)>60&&fabs(Encoder_D)>60&&fabs(Encoder_A)>60)
				    ||
				    (fabs(Encoder_C)<6&&fabs(Encoder_D)>60&&fabs(Encoder_A)>60&&fabs(Encoder_B)>60)
				    ||
				    (fabs(Encoder_D)<6&&fabs(Encoder_A)>60&&fabs(Encoder_B)>60&&fabs(Encoder_C)>60)
					){ Flag_Z6++;
			        if(fabs(Encoder_A)<6)enc_fault|=1; if(fabs(Encoder_B)<6)enc_fault|=2;
			        if(fabs(Encoder_C)<6)enc_fault|=4; if(fabs(Encoder_D)<6)enc_fault|=8; }
			    else {Flag_Z6=0;}
			    if(Flag_Z6>300){Start_Flag_one=0;abnormal|=8;enc_fault_locked|=enc_fault;}
			    if(abnormal&8)enc_fault=enc_fault_locked;

			    //****************电机异常(跳过编码器故障轮)****************************//

			    if(
					  (!(enc_fault&1)&&((Motor_A>=+PWM_MAX && Encoder_A<-100)||(Motor_A<=-PWM_MAX && Encoder_A>+100)))
				    ||
			  		  (!(enc_fault&2)&&((Motor_B>=+PWM_MAX && Encoder_B<-100)||(Motor_B<=-PWM_MAX && Encoder_B>+100)))
				    ||
				    (!(enc_fault&4)&&((Motor_C>=+PWM_MAX && Encoder_C<-100)||(Motor_C<=-PWM_MAX && Encoder_C>+100)))
				    ||
				    (!(enc_fault&8)&&((Motor_D>=+PWM_MAX && Encoder_D<-100)||(Motor_D<=-PWM_MAX && Encoder_D>+100)))
					  )Flag_Z1++;
					else Flag_Z1=0;
					if(Flag_Z1>300)Start_Flag_one=0,abnormal|=1;//反馈异常

				 	if(
			 			  (!(enc_fault&1)&&abs(Motor_A)>=+PWM_MAX &&fabs(Encoder_A)<10)
					    ||
						  (!(enc_fault&2)&&abs(Motor_B)>=+PWM_MAX &&fabs(Encoder_B)<10)
					    ||
					    (!(enc_fault&4)&&abs(Motor_C)>=+PWM_MAX &&fabs(Encoder_C)<10)
					    ||
					    (!(enc_fault&8)&&abs(Motor_D)>=+PWM_MAX &&fabs(Encoder_D)<10)
						)Flag_Z2++;
					else Flag_Z2=0;
					if(Flag_Z2>1000)Start_Flag_one=0,abnormal|=2;//堵转

	    //****************电压异常处理****************************//
	        if(Voltage < BATT_STOP) Flag_Z3++;
	        else Flag_Z3=0;
	        if(Flag_Z3 > 4000) Start_Flag_one=0;

		    if(Voltage<700)Start_Flag_STOP=1;//仅USB供电,禁止启动
		    else Start_Flag_STOP=0;

		    //****************通讯异常处理****************************//
		  	if(FLAG_USART_ON<600)FLAG_USART_ON++;//串口超时检测
		  	if(FLAG_CAN_ON<100)FLAG_CAN_ON++;//CAN超时检测

////PWM限制，防止烧坏电机↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓//

				     if((Motor_A - Last_Motor_A) > +PWM_SLEW_LIMIT) Motor_A = Last_Motor_A + PWM_SLEW_LIMIT;
				else if((Motor_A - Last_Motor_A) < -PWM_SLEW_LIMIT) Motor_A = Last_Motor_A - PWM_SLEW_LIMIT;
				   Last_Motor_A = Motor_A;
				     if((Motor_B - Last_Motor_B) > +PWM_SLEW_LIMIT) Motor_B = Last_Motor_B + PWM_SLEW_LIMIT;
				else if((Motor_B - Last_Motor_B) < -PWM_SLEW_LIMIT) Motor_B = Last_Motor_B - PWM_SLEW_LIMIT;
				   Last_Motor_B = Motor_B;
				     if((Motor_C - Last_Motor_C) > +PWM_SLEW_LIMIT) Motor_C = Last_Motor_C + PWM_SLEW_LIMIT;
				else if((Motor_C - Last_Motor_C) < -PWM_SLEW_LIMIT) Motor_C = Last_Motor_C - PWM_SLEW_LIMIT;
				   Last_Motor_C = Motor_C;
				     if((Motor_D - Last_Motor_D) > +PWM_SLEW_LIMIT) Motor_D = Last_Motor_D + PWM_SLEW_LIMIT;
				else if((Motor_D - Last_Motor_D) < -PWM_SLEW_LIMIT) Motor_D = Last_Motor_D - PWM_SLEW_LIMIT;
				   Last_Motor_D = Motor_D;

				if(Motor_A>+PWM_MAX)Motor_A=+PWM_MAX;
			  if(Motor_A<-PWM_MAX)Motor_A=-PWM_MAX;
				if(Motor_B>+PWM_MAX)Motor_B=+PWM_MAX;
			  if(Motor_B<-PWM_MAX)Motor_B=-PWM_MAX;
				if(Motor_C>+PWM_MAX)Motor_C=+PWM_MAX;
			  if(Motor_C<-PWM_MAX)Motor_C=-PWM_MAX;
				if(Motor_D>+PWM_MAX)Motor_D=+PWM_MAX;
			  if(Motor_D<-PWM_MAX)Motor_D=-PWM_MAX;

				if(RED_KEY==0){Start_Flag=0;estop_pressed=1;}
				if(RED_KEY==1){estop_pressed=0;}

			    //****************CAN通讯异常检测****************************//
			    {   extern volatile u8 can_online; static u16 can_fault_cnt;
			        if(can_online != 0x0F) can_fault_cnt++; else can_fault_cnt=0;
			        if(can_fault_cnt > 100){ abnormal |= 0x10; Start_Flag_one=0; }
			        else abnormal &= ~0x10; }

		    if(  (fault&1)==0
					 ||(fault&2)==0
					 ||(fault&4)==0
					 ||(fault&8)==0
				  )Flag_Z5++;
				else Flag_Z5=0;
				if(Flag_Z5>20)Start_Flag=0;	//舵机没有角度回传则停止
}