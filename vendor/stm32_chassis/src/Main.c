#include "sys.h"

/*********************************************
作者：星洛智能
淘宝店铺：http://shop180997663.taobao.com/
底盘类型：户外重载型四轮舵轮底盘
*********************************************/

/*******************
以笛卡尔坐标系为参考
Roll为绕X轴旋转角，
Pitch为绕Y轴旋转角，
Yaw为绕Z轴旋转角。
*******************/

/* 
 * 全局变量定义（按功能分9组，调哪个模块找哪个组）
 *  */


volatile u8 Start_Flag=0;
volatile u8 Start_Flag_one=1;
u8  Start_Flag_STOP=0;
volatile u8  Flag_STOP=0;
u8 show_flag=0;
u8  show_on_flag=0;
volatile u8 delay_30,delay_flag;
volatile u8 flag_move=0;
volatile u8 MOVE_mode=0;
u8 flag_action=0;
volatile u8 Flag_Mode=0;


u8 PS2_KEY,PS2_LX,PS2_LY,PS2_RX,PS2_RY;


u16 Flag_Z1,Flag_Z2,Flag_Z3,Flag_Z4,Flag_Z5,Flag_Z6;
u8 abnormal=0;
u8 fault=0;


u16 FLAG_USART_ON=600;
u16 FLAG_CAN_ON;
u16 CAN_ID1,CAN_ID2,CAN_ID3,CAN_ID4;
u8  CAN_EN_A=0,CAN_EN_B=0,CAN_EN_C=0,CAN_EN_D=0;
u8  can_ser=15;


volatile int Voltage;
int count_size,count_sum;


volatile int Motor_A,Motor_B,Motor_C,Motor_D;
int Last_Motor_A,Last_Motor_B,Last_Motor_C,Last_Motor_D;
int Velocity_target_A,Velocity_target_B,Velocity_target_C,Velocity_target_D;


float Encoder_A,Encoder_B,Encoder_C,Encoder_D;
volatile float Roll,Pitch,Yaw;
volatile float gyro_Roll,gyro_Pitch,gyro_Yaw;
volatile float accel_x,accel_y,accel_z;


volatile float Angle_target_A,Angle_target_B,Angle_target_C,Angle_target_D;
volatile float Angle_current_A,Angle_current_B,Angle_current_C,Angle_current_D;
float Angle_error_A=0,Angle_error_B=0,Angle_error_C=0,Angle_error_D=0;
float Current_angle_A,Current_angle_B,Current_angle_C,Current_angle_D;


/* PID */
float Gyro_KP,Gyro_KI,Gyro_KD;
float line_KP,line_KI,line_KD;
float Velocity_KP,Velocity_KI;
float Velocity_center;
/* 运动控制 */
float Steering_speed=1.0;
int Turn_R,STEP;
float Yaw_target;
float DIS_speed_A,DIS_speed_B,DIS_speed_C,DIS_speed_D;
float DIS_speed;
/* ROS串口命令值 */
float Angle_MID_A,Angle_MID_B,Angle_MID_C,Angle_MID_D;
float Velocity_MID_A,Velocity_MID_B,Velocity_MID_C,Velocity_MID_D;
float Velocity_MID_X,Velocity_MID_Y,Velocity_MID_Z;
float Velocity_MID_H;
float Velocity_X,Turn_Z,Velocity_Y;  // 车体速度命令
float velocity_x,velocity_y,velocity_z; // 计算速度
float Angle_DIS_A,Angle_DIS_B,Angle_DIS_C,Angle_DIS_D;
/* 舵机校准 */
float offset_a=1.f,offset_b=1.f,offset_c=1.f,offset_d=1.f;
/* 里程计 */
double Delta_x=0.0,Delta_y=0.0,Delta_th=0.0;
double vx=0.0,vy=0.0,vth=0.0;
float Flag_init=1;

int main(void)
{
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试

	Motor_PWM_Init(7199,0);         //初始化电机PWM 40KHZ

	OLED_Init();                    //OLED初始化
	LED_Init();                     //LED初始化
	KEY_Init();                     //按键初始化
	usart1_init(72,115200);       //串口1初始化
	usart2_init(36,9600);           //串口2初始化

  delay_ms(100);
	Dma_Init(1,DMA1_Channel4,(u32)&USART1->DR,(u32)send_buf,Len*4+5);//初始化DMA1:通道4,数据源send_buf[],传输目标：外设地址 &USART1->DR
	USART1->CR3|=1<<7;                 //串口1DMA中断使能
 	Dma_Enable(DMA1_Channel4,Len*4+5);      //DMA搬运使能
	delay_ms(100);                   //延时

	Encoder_Init_TIM2();            //初始化编码器
	Encoder_Init_TIM3();            //初始化编码器
	Encoder_Init_TIM4();            //初始化编码器
	Encoder_Init_TIM5();            //初始化编码器

	Adc_Init();                     //adc初始化
	delay_ms(100);                  //延时

	IIC_Init();                     //IIC初始化
  delay_ms(100);                   //延时
  MPU6050_initialize();           //MPU6050初始化
  delay_ms(10);                   //延时
  DMP_Init();                     //初始化DMP
  delay_ms(500);                  //延时

	PS2_Init();                     //PS2手柄初始化
	PS2_SetInit();                  //PS2手柄初始化

   offset_a=SERVO_OFFSET_A, offset_b=SERVO_OFFSET_B, offset_c=SERVO_OFFSET_C, offset_d=SERVO_OFFSET_D;//舵机角度补偿

  Gyro_KP=GYRO_KP, Gyro_KI=GYRO_KI, Gyro_KD=GYRO_KD;//车体角速度调整PID参数
	line_KP=LINE_KP, line_KI=LINE_KI, line_KD=LINE_KD;       //车体角度调整PD参数
  Velocity_KP=VELOCITY_KP, Velocity_KI=VELOCITY_KI;         //电机速度控制PI参数
  Velocity_center=DEFAULT_SPEED;                    //设定车子速度，默认速度为300

	CAN1_Mode_Init(CAN_TSJW, CAN_TBS2, CAN_TBS1, CAN_BRP, 0);      //CAN初始化
  CAN_ID1=CAN_ID_A; CAN_ID2=CAN_ID_B; CAN_ID3=CAN_ID_C; CAN_ID4=CAN_ID_D;

  delay_ms(500);                  //延时足够时间等待舵机初始化

	for(u8 i=0;i<50;i++){
		  CAN1_Receive_data();//can 处理接收的数据	获取当前角度
		  CAN1_SEND_data(0, 0, 0, 0,
				                  Angle_target_A,
				                  Angle_target_B,
				                  Angle_target_C,
				                  Angle_target_D);//can 发送目标角度
		  delay_ms(5);
		}

	static u8 going=0;
		if(can_ser!=15)going=1;

	/* 
	 * 角度归算 — 户外重载型绝对值舵机（无就近原则，不可无限旋转）
	 *
	 * 舵机绝对编码器返回多圈累计值（可能为±360/±720范围内），
	 * 需归算到机械实际范围。每轮归算阈值不同（机械安装差异）。
	 *  */
		if(Current_angle_A > +100&&Current_angle_A <= +360)Angle_error_A =-360;
		if(Current_angle_A > +360&&Current_angle_A <= +720)Angle_error_A =-720;
		if(Current_angle_A < -50 &&Current_angle_A >= -360)Angle_error_A =+360;
		if(Current_angle_A < -360&&Current_angle_A >= -720)Angle_error_A =+720;

		if(Current_angle_B > +50 &&Current_angle_B <= +360)Angle_error_B =-360;
		if(Current_angle_B > +360&&Current_angle_B <= +720)Angle_error_B =-720;
		if(Current_angle_B < -100&&Current_angle_B >= -360)Angle_error_B =+360;
		if(Current_angle_B < -360&&Current_angle_B >= -720)Angle_error_B =+720;

		if(Current_angle_C > +100&&Current_angle_C <= +360)Angle_error_C =-360;
		if(Current_angle_C > +360&&Current_angle_C <= +720)Angle_error_C =-720;
		if(Current_angle_C < -50 &&Current_angle_C >= -360)Angle_error_C =+360;
		if(Current_angle_C < -360&&Current_angle_C >= -720)Angle_error_C =+720;

		if(Current_angle_D > +50 &&Current_angle_D <= +360)Angle_error_D =-360;
		if(Current_angle_D > +360&&Current_angle_D <= +720)Angle_error_D =-720;
		if(Current_angle_D < -100&&Current_angle_D >= -360)Angle_error_D =+360;
		if(Current_angle_D < -360&&Current_angle_D >= -720)Angle_error_D =+720;

         Angle_current_A = 	(Current_angle_A + Angle_error_A)/offset_a;
         Angle_current_B = 	(Current_angle_B + Angle_error_B)/offset_b;
         Angle_current_C = 	(Current_angle_C + Angle_error_C)/offset_c;
         Angle_current_D = 	(Current_angle_D + Angle_error_D)/offset_d;

		     Angle_target_A = Angle_current_A;
		     Angle_target_B = Angle_current_B;
		     Angle_target_C = Angle_current_C;
		     Angle_target_D = Angle_current_D;

	while(going){}	//出现小概率事件，舵机的初始角度错误，此时不给控制，需要断电重启

	delay_ms(100);                  //延时
  EXTI_Init();                    //MPU6050 5ms定时中断初始化

	while(1)
		{
			  show_flag++;                  // 计数器累加，控制OLED显示轮次
				if(show_flag==1){static u8 bt_cnt=0;if(++bt_cnt>=10){send_data_to_blue();bt_cnt=0;}}// 每40循环发一次蓝牙，避免阻塞
				if(show_flag>=3) // oled显示
					{
           if(show_on_flag==0){
				   	  if(Voltage<2000&&Voltage>700)
							  {
								static u8 uu;
								uu++;
								if(uu>=15)OLED_Display_Off();
								else OLED_Display_On(),oled_show();// oled显示
								if(uu>=20)uu=0;
		            }
						  else OLED_Display_On(),oled_show();// oled显示
					 }
					else OLED_Display_Off();
					}
         if(show_flag==4)show_flag=0;
			  delay_flag=1;	               // 主循环标志位，由5毫秒中断清零使能下一轮
				while(delay_flag);           // 等待中断
		}
}

/****************************************************************************************
增量式PI控制
****************************************************************************************/
