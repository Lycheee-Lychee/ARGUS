#include "usart.h"	  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{

	while((USART2->SR&0X40)==0);
	USART2->DR = (u8) ch;
  return ch;
}
// _write for GCC: newlib printf calls _write, not fputc
int _write(int file, char *ptr, int len)
{
	int i;
	for(i=0; i<len; i++) {
		while((USART2->SR&0X40)==0);
		USART2->DR = (u8)ptr[i];
	}
	return len;
}
#endif 

//******************************����1******************************************//
/////////////////////////////////////////////////////////////////////////////////
//****************usart1����һ���ֽ�************************************//
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
//****************����1��ʼ��************************************//
void usart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬����
	GPIOA->ODR|=1<<9;	  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
	USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART1_IRQn,2);//��ռ���ȼ�2,��Ӧ���ȼ�2,��2
}

//******************************����1�����ж�*************************************//

u8 USART1_data[54];
u8 data_len=0;
u8 FLAG_USART=0;	
u8 Sum;	
u8 USART1_func=0;	
int break_count=0;
int USART1_IRQHandler(void)
{
	if(USART1->SR&(1<<5))
	{
		u8 ch = USART1->DR;
		if(data_len == 0) {
			if(ch == 0xAA) { USART1_data[0] = ch; data_len = 1; }
		}
		else if(data_len == 1) {
			if(ch == 0xAA) { USART1_data[1] = ch; data_len = 2; }
			else { data_len = 0; }
		}
		else if(data_len == 2) {
			// Store 3rd byte: function code discriminator
			USART1_data[2] = ch; data_len = 3;
			USART1_func = (ch == 0xF3) ? 0xF3 : 0;
		}
		else {
			USART1_data[data_len] = ch; data_len++;
			if(USART1_func == 0xF3) {
				// 0xF3 PID frame: 17 bytes (2 header + 1 func + 1 id + 12 data + 1 checksum)
				if(data_len >= 17) {
					u8 sum_chk = 0;
					for(u8 j=0;j<16;j++) sum_chk += USART1_data[j];
					if(sum_chk == USART1_data[16]) {
						u8 loop = USART1_data[3];
						float kp = b2f(USART1_data[4], USART1_data[5], USART1_data[6], USART1_data[7]);
						float ki = b2f(USART1_data[8], USART1_data[9], USART1_data[10], USART1_data[11]);
						float kd = b2f(USART1_data[12], USART1_data[13], USART1_data[14], USART1_data[15]);
						switch(loop) {
							case 0: Velocity_KP=kp; Velocity_KI=ki; break;
							case 1: Gyro_KP=kp; Gyro_KI=ki; Gyro_KD=kd; break;
							case 2: line_KP=kp; line_KI=ki; line_KD=kd; break;
						}
					}
					data_len = 0;
					USART1_func = 0;
				}
			} else {
				// Legacy 0xF2 frame: 53 bytes (2 header + 50 data + 1 checksum)
				if(data_len > 52) {
					FLAG_USART = 1;
					data_len = 0;
				}
			}
		}
	}
	return 0;
}

float data_u[30];

void Send_data_ROS(void)
{

    u8 tbuf[Len*4];
    unsigned char *p;
				
    for(u8 i=0;i<Len;i++){
	            p=(unsigned char *)&data_u[i];
        tbuf[4*i+0]=(unsigned char)(*(p+3));
        tbuf[4*i+1]=(unsigned char)(*(p+2));
        tbuf[4*i+2]=(unsigned char)(*(p+1));
        tbuf[4*i+3]=(unsigned char)(*(p+0));
    }		
		
    usart1_sent(0XF1,tbuf,Len*4);//�Զ���֡,������0XF1
}
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���48�ֽ�!!
//len:data����Ч���ݸ���

u8 send_buf[130];  // Len*4+6 = 126, 向上取整
void usart1_sent(u8 fun,u8*data,u8 len)
{
    
    
    if(len>124)return;   //���88�ֽ�����
    send_buf[len+4]=0;  //У��������
    send_buf[0]=0XAA;   //֡ͷ
	  send_buf[1]=0XAA;   //֡ͷ
    send_buf[2]=fun;    //������
    send_buf[3]=len;    //���ݳ���
//    send_buf[len+5]= 0X0D;    //֡β	
    for(u8 i=0;i<len;i++)send_buf[4+i]=data[i];         //��������
    for(u8 i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];//����У���

}
//4��byte����ת��Ϊһ��float��ֵ
float b2f(byte m0, byte m1, byte m2, byte m3)
{
//�����λ
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//�����
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//��β�� 
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

//******************************����2********************************//
///////////////////////////////////////////////////////////////////////
//****************************usart2����һ���ֽ�*********************//
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
//******************************����2��ʼ��**************************//
void usart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO״̬����
  
	RCC->APB1RSTR|=1<<17;   //��λ����1
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    //PE�ж�ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//��2��������ȼ� 
}

//***************************��ҡAPPͨѶ*******************//
/*********************************************
���ߣ���������
�Ա����̣�http://shop180997663.taobao.com/
*********************************************/
void LANYAO_APP(int data)//�������ݵ���ҡAPP
{
  static int SEND_DATA[10];	
  int i,j,k,data_c;
if(data==0)usart2_send(0x30);
if(data>0)
	{	
	 i=data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }	
			for(k=0;k<j;k++){SEND_DATA[k]=data%10;data=data/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	if(data<0)
	{
   data_c=-data; 		
	 i=-data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }
   usart2_send(0x2D);				
			for(k=0;k<j;k++){SEND_DATA[k]=data_c%10;data_c=data_c/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	usart2_send(0x0A),
	usart2_send(0x0D);
  memset(SEND_DATA, 0, sizeof(int)*10);	
} 
//**************************����2�����ж�***********************//
int Usart2_Receive;
int data_app[10];
u8 flag_mode_app=0;
int anjian_app,huakuai_app,yaogan_app=510;
/*********************************************
���ߣ���������
�Ա����̣�http://shop180997663.taobao.com/
*********************************************/
int USART2_IRQHandler(void)
{	
 static u8 i=0;
	if(USART2->SR&(1<<5))//���յ�����
	{	   
		      Usart2_Receive=USART2->DR;		
					data_app[i] = USART2->DR;//��ȡ����		                              
			switch(i)
			{
				case 0:
					if( data_app[0]==0X79)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
					}
					break;
				case 1:
					if( data_app[1]==0X62)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=0;//��������
					}
			   	else	if( data_app[1]==0X76)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=1;//��������
					}
					else	if( data_app[1]==0X64)//����֡ͷ����
					{
						i++;//��һ���ֽ�����
						flag_mode_app=2;//ҡ������
					}
					break;				
				case 2:
						i++;//��һ���ֽ�����
					break;	
				case 3:            
            i++;//��һ���ֽ�����	
					break;
				case 4:
					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 anjian_app=data_app[2],i=0;//һ֡�������
					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
						 huakuai_app=data_app[2],i=0;//һ֡�������
					 if(flag_mode_app==2)i++;
					break;
				case 5:
					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
					 yaogan_app= data_app[2]+data_app[3],i=0;//һ֡�������				
					break;
				default:
					break;				
			}
   }
return 0;	

}
//*******************����3��ʼ��,PCLK2 ʱ��Ƶ��(Mhz),bound:������************************//
void usart3_init(u32 pclk2,u32 bound)
{  	 
 float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	

	RCC->APB2ENR|=1<<0;    //��������ʱ��
	RCC->APB2ENR|=1<<4;   //ʹ��PORTC��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOC->CRH&=0XFFFF00FF; //IO״̬����
	GPIOC->CRH|=0X00008B00;//PC10��� PC11����
	GPIOC->ODR|=1<<10;	 
  AFIO->MAPR|=1<<4;      //������ӳ��

	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//��2��������ȼ� 
}

//****************************�������ܣ�����3�����ж�***************************************//

int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
	{	      	
   	u8 data = USART3->DR;//��ȡ����
	
   }
return 0;	
}
void Usart_rt_data(void) //**********************************串口通讯收发

{

/**********************************串口1通讯收发,上位机控制********************************************************************************/

			 float Velocity_line_A, Velocity_line_B, Velocity_line_C, Velocity_line_D;

			 Velocity_line_A = +( Encoder_A / VEL_TO_ENCODER);// 单位m/s
			 Velocity_line_B = -( (Encoder_B/(float)ENCODER_CPR)/MOTOR_GEAR_RATIO)*(float)CONTROL_FREQ_HZ * WHEEL_CIRCUMFERENCE_M;// 单位m/s
			 Velocity_line_C = -( (Encoder_C/(float)ENCODER_CPR)/MOTOR_GEAR_RATIO)*(float)CONTROL_FREQ_HZ * WHEEL_CIRCUMFERENCE_M;// 单位m/s
			 Velocity_line_D = +( (Encoder_D/(float)ENCODER_CPR)/MOTOR_GEAR_RATIO)*(float)CONTROL_FREQ_HZ * WHEEL_CIRCUMFERENCE_M;// 单位m/s	

			 

       Odom_xyz(Velocity_line_A,Velocity_line_B,Velocity_line_C,Velocity_line_D,Angle_current_A,Angle_current_B,Angle_current_C,Angle_current_D);//底盘计算的里程计数据

////***********************X轴移动0.8米后Y轴移动0.8米***********************//	

// static int count_TIME = 0;

							          /*<01>*/data_u[0]  = Start_Flag;//电机启动开关，1启动 0停止

							          /*<02>*/data_u[1]  = Angle_current_A; 

							          /*<03>*/data_u[2]  = Angle_current_B; 

							          /*<04>*/data_u[3]  = Angle_current_C; 

							          /*<05>*/data_u[4]  = Angle_current_D; //ABCD四轮的当前转角 deg

							          /*<06>*/data_u[5]  = Velocity_line_A ;

							          /*<07>*/data_u[6]  = Velocity_line_B ;    

							          /*<08>*/data_u[7]  = Velocity_line_C ;    

							          /*<09>*/data_u[8]  = Velocity_line_D ; //ABCD四轮的当前线速度 m/s

							          /*<10>*/data_u[9]  = gyro_Roll ; 

							          /*<11>*/data_u[10] = gyro_Pitch ; 

							          /*<12>*/data_u[11] = gyro_Yaw ;//XYZ三轴角速度原始数值

							          /*<13>*/data_u[12] = accel_x ; 

							          /*<14>*/data_u[13] = accel_y ; 

							          /*<15>*/data_u[14] = accel_z ;//XYZ三轴加速度原始数值

                        /*<16>*/data_u[15] = Roll ;			  

                        /*<17>*/data_u[16] = Pitch ;

                        /*<18>*/data_u[17] = Yaw ;	//XYZ三轴角度							 

							          /*<19>*/data_u[18] = Voltage ; //电池电压

							          /*<20>*/data_u[19] = Delta_x ;//X轴位移

							          /*<21>*/data_u[20] = Delta_y ;//Y轴位移

							          /*<22>*/data_u[21] = Delta_th ;//Z轴累积角度	

									          /*<23>*/data_u[22] = Velocity_KP ;//速度环P
									          /*<24>*/data_u[23] = Velocity_KI ;//速度环I
									          /*<25>*/data_u[24] = Gyro_KP ;//陀螺环P
									          /*<26>*/data_u[25] = Gyro_KI ;//陀螺环I
									          /*<27>*/data_u[26] = Gyro_KD ;//陀螺环D
									          /*<28>*/data_u[27] = line_KP ;//直线环P
									          /*<29>*/data_u[28] = line_KI ;//直线环I
									          /*<30>*/data_u[29] = line_KD ;//直线环D
//Z轴累积角度	

												

			 if(DMA1->ISR&(1<<13))//通道4传输完成标志(（通道号-1）*4+1)

			       {				 

			        Dma_Disenable(DMA1_Channel4);//DMA搬运除能，通道2 	

              Send_data_ROS();//发送22个数据

								//发送数据				

			        DMA1->IFCR|=1<<13;//清空通道4传输完成标志

			        Dma_Enable(DMA1_Channel4,Len*4+5);//DMA搬运使能，通道1                    

			       }	

						 

       if(FLAG_USART==1)//如果串口1标志位置1，表示已完成一组数据传输

	        { 				 	

						static u8 sum;  

	          for(u8 j=0;j<52;j++)sum+=USART1_data[j];    //计算校验和	

            if(sum==USART1_data[52])

									{					

						                   Start_Flag     =   b2f(USART1_data[4],  USART1_data[5],  USART1_data[6],  USART1_data[7]); //获取启动停止标志

														   Flag_Mode      =   b2f(USART1_data[8],  USART1_data[9],  USART1_data[10], USART1_data[11]);//获取控制模式

										if(Flag_Mode==0){

															 Angle_MID_A    =   b2f(USART1_data[12], USART1_data[13], USART1_data[14], USART1_data[15]);//获取A轮转向角

															 Angle_MID_B    =   b2f(USART1_data[16], USART1_data[17], USART1_data[18], USART1_data[19]);//获取B轮转向角

															 Angle_MID_C    =   b2f(USART1_data[20], USART1_data[21], USART1_data[22], USART1_data[23]);//获取C轮转向角

															 Angle_MID_D    =   b2f(USART1_data[24], USART1_data[25], USART1_data[26], USART1_data[27]);//获取D轮转向角											

						                   Velocity_MID_A =   b2f(USART1_data[28], USART1_data[29], USART1_data[30], USART1_data[31]);//获取A轮滚动速度

														   Velocity_MID_B =   b2f(USART1_data[32], USART1_data[33], USART1_data[34], USART1_data[35]);//获取B轮滚动速度

						                   Velocity_MID_C =   b2f(USART1_data[36], USART1_data[37], USART1_data[38], USART1_data[39]);//获取C轮滚动速度

														   Velocity_MID_D =   b2f(USART1_data[40], USART1_data[41], USART1_data[42], USART1_data[43]);//获取D轮滚动速度											

														   Flag_init      =   b2f(USART1_data[44], USART1_data[45], USART1_data[46], USART1_data[47]);//里程计初始化为0  1/1 不变/初始化													         

										}

										else if(Flag_Mode==1){

															 Velocity_MID_X =   b2f(USART1_data[12], USART1_data[13], USART1_data[14], USART1_data[15]);//获取X方向速度

															 Velocity_MID_Y =   b2f(USART1_data[16], USART1_data[17], USART1_data[18], USART1_data[19]);//获取Y方向速度	

						                   Velocity_MID_Z =   b2f(USART1_data[20], USART1_data[21], USART1_data[22], USART1_data[23]);//获取Z方向角速度（RAD/S）

														   Flag_init      =   b2f(USART1_data[44], USART1_data[45], USART1_data[46], USART1_data[47]);//里程计初始化为0  1/0 不变/初始化													

										}

									

									}	

						sum=0;	

						USART1_data[52]=0XFF;//清除和校验位									

			 																								

				    FLAG_USART_ON=0;

					  FLAG_USART=0;//串口标志位置0							

			   }

				 					 

/**********************************串口1通讯收发,END********************************************************************************/

}

//发送数据帧

/*

	      txbuf[0]=0XAA;//帧头

				txbuf[1]=0; //数据位 使能位 0/1 停止/启动

        txbuf[2]=0;	//数据位 目标角度低8位 （角度设置为short型分步发送）

				txbuf[3]=0;	//数据位 目标角度高8位 

			  txbuf[4]=0; //数据位 当前角度小数点后2位 

				txbuf[5]=0;	//数据位 预留 	

	  		txbuf[6]=0; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]

				txbuf[7]=0xAF;//帧尾

*/

void  APP_Control(void)

{

     if(anjian_app==1)Start_Flag=1,anjian_app=0;//按键1启动

	   if(anjian_app==2)Start_Flag=0,anjian_app=0;//按键2关闭

     if(anjian_app==3)MOVE_mode=1,anjian_app=0;//平移模式

	   if(anjian_app==4)MOVE_mode=0,anjian_app=0;//阿克曼转弯和自转模式

	   if(anjian_app==5)Velocity_center=SPEED_GEAR_1,anjian_app=0;

	   if(anjian_app==6)Velocity_center=SPEED_GEAR_2,anjian_app=0;

	   if(anjian_app==7)Velocity_center=SPEED_GEAR_3,anjian_app=0;

	   if(anjian_app==8)Velocity_center=SPEED_GEAR_4,anjian_app=0;

	

	   //按区间来，避免临界抖动

	if(MOVE_mode==1)//平移模式

	  {

	   if(yaogan_app >=250 && yaogan_app <290)flag_move = 1;//前进

	   else if(yaogan_app >=70 && yaogan_app <110)flag_move = 2;//后退

	

		 else if(yaogan_app >=160 && yaogan_app <200)flag_move = 3;//左横移

		 else if( (yaogan_app >=0 && yaogan_app <20) || (yaogan_app >=340 && yaogan_app <=360) )flag_move = 4;//右横移

	

	   else if(yaogan_app >=205 && yaogan_app <245)flag_move = 5;//左斜上45°

	   else if(yaogan_app >=295 && yaogan_app <335)flag_move = 6;//右斜上45°

	

	   else if(yaogan_app >=115 && yaogan_app <155)flag_move = 7;//左斜下45°

	   else if(yaogan_app >=25 && yaogan_app <65)flag_move = 8;//右斜下45°

		 else flag_move = 0;	

		

	  }

	else if(MOVE_mode==0)//阿克曼转弯和自转模式

	  {

	        if(yaogan_app >=250 && yaogan_app <290)flag_move = 1,STEP-=10;//前进

	   else if(yaogan_app >=70  && yaogan_app <110)flag_move = 2,STEP-=10;//后退

	

		 else if(yaogan_app >=160 && yaogan_app <200)flag_move = 9,STEP-=10;//左自转

		 else if( (yaogan_app >=0 && yaogan_app <20) || 

			   (yaogan_app >=340 && yaogan_app <=360) )flag_move= 10,STEP-=10;//右自转

	

	   else if(yaogan_app >=205 && yaogan_app <245)flag_move = 11,STEP+=10;//左前转弯		

	   else if(yaogan_app >=295 && yaogan_app <335)flag_move = 12,STEP+=10;//右前转弯

			

	   else if(yaogan_app >=115 && yaogan_app <155)flag_move = 13,STEP+=10;//左后转弯		

 

	   else if(yaogan_app >=25  && yaogan_app <65 )flag_move = 14,STEP+=10;//右后转弯

		 else flag_move = 0;	

	  }

}

void  send_data_to_blue(void)

{

          printf("  A:");					

          printf("%.1f",Yaw);

					

          printf("B:");						

          printf("%.2fV",((float)Voltage)/100);

					

          printf("C:");							

          printf("%d",(int)Encoder_A);

					

          printf("D:");						

          printf("%d",(int)Encoder_B);

					

          printf("E:");						

          printf("%d",(int)Encoder_C);

					

          printf("F:");						

          printf("%d",(int)Encoder_D);//发送当前参数值到手机APP 然后在usart.c 文件里面串口2中断函数里接收来自上位机的参数修改

          printf("G");						

          printf("%d",(short)Velocity_KP);

          printf("H");						

          printf("%d",(short)Velocity_KI);

          printf("I");						

          printf("%d",(short)line_KP);

          printf("J");						

          printf("%d",(short)line_KI);

          printf("K");						

          printf("%d",(short)line_KD);				 

		 

          printf("; ");	

	

}

/****************************************************************************************

运动控制

****************************************************************************************/

