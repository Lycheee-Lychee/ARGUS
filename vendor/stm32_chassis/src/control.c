#include "sys.h"

/* ---- 运动控制模块 ----
 * Motion_control, 4路PID, Z轴陀螺稳像, 直线PD, 位置PD
 */

extern void Motion_control(void);
extern int velocityA_Control(int encoder, int target);
extern int velocityB_Control(int encoder, int target);
extern int velocityC_Control(int encoder, int target);
extern int velocityD_Control(int encoder, int target);
extern float V_Z_Control(float Gyro, float Gyro_control);
extern int straight_line_Control(float Angle, float Target);
extern float Position_x(float Encoder, float Target);
extern float Position_y(float Encoder, float Target);

void Motion_control(void)

{

static float flag_LA;

static float flag_LB;

static float flag_LC;

static float flag_LD;

static float angle_A;

static float angle_B;

static float angle_C;

static float angle_D;		

	

static float L1,L2,LA,LB,LC,LD;	

static float angle_in,angle_out;

	

static u8	marker_1=0;

static u8	marker_2=0;

static u8	marker_3=0;

static u8	marker_4=0;

static u8	marker_5=0;

static u8	marker_6=0;		

static u16	counter1=0,counter2=0;

static u8 last_flag_move=0;

static u8 first_move=0;	

static u8 flag_chanone=0;	



static float last_angle_A, last_angle_B, last_angle_C, last_angle_D;

static short Length_l=WHEELBASE_MM, Length_s=TRACK_WIDTH_MM;/*前后轴距340mm，左右轮距240mm*/





if(FLAG_USART_ON>300)

	{	if(flag_chanone==1){marker_1=0;marker_2=0;marker_3=0;counter1=0;counter2=0;flag_chanone=0;}



		      if(flag_move>=15&&flag_move<=18){//4轮转角阿克曼，转弯半径逐渐减小到550mm 车子转弯半径 Turn_R最好450以上 

		          if(STEP>1000)STEP=1000;

						  if(STEP<0)STEP=0;

		          Turn_R=ACKERMANN_4W_TURN_RADIUS -STEP;

						

					    L1=	Turn_R + (Length_s/2);//转弯圆心到车辆外侧轮中心的距离

					 	  L2= Turn_R - (Length_s/2);//转弯圆心到车辆内侧轮中心的距离

	            LA= sqrt((Length_l/2)*(Length_l/2) + L2*L2);//内侧轮子转弯半径

	            LB= sqrt((Length_l/2)*(Length_l/2) + L1*L1);//外侧轮子转弯半径



	            angle_out= atan((Length_l/2)/L1)* to_deg;//外侧前轮子转的角度

	            angle_in = atan((Length_l/2)/L2)* to_deg;//内侧前轮子转的角度

              if(angle_in > +ACKERMANN_4W_MAX_ANGLE)angle_in=+ACKERMANN_4W_MAX_ANGLE;

              if(angle_in < -ACKERMANN_4W_MAX_ANGLE)angle_in=-ACKERMANN_4W_MAX_ANGLE;

              if(angle_out> +ACKERMANN_4W_MAX_ANGLE)angle_out=+ACKERMANN_4W_MAX_ANGLE;

              if(angle_out< -ACKERMANN_4W_MAX_ANGLE)angle_out=-ACKERMANN_4W_MAX_ANGLE;

					}

					else { //

		          if(STEP>1800)STEP=1800;

						  if(STEP<0)STEP=0;

		          Turn_R=ACKERMANN_2W_TURN_RADIUS -STEP;//2轮转角阿克曼，初始转弯半径逐渐减小到500mm,Turn_R最好400以上 

		

					    L1=	sqrt(Turn_R*Turn_R - (Length_l/2)*(Length_l/2));//转弯中心到车辆两后轮中间的距离	

					 	  L2= Turn_R;//车子中心转弯行程

	            LA= sqrt(Length_l*Length_l +(L1+(Length_s/2))*(L1+(Length_s/2)));//外侧前轮子转弯半径

	            LB= sqrt(Length_l*Length_l +(L1-(Length_s/2))*(L1-(Length_s/2)));//内侧前轮子转弯半径

	            LC= L1 - (Length_s/2);//内侧后轮转弯半径

	            LD= L1 + (Length_s/2);//外侧后轮转弯半径

	            angle_out= atan(Length_l/(L1+(Length_s/2)))* to_deg;//外侧前轮子转的角度

	            angle_in = atan(Length_l/(L1-(Length_s/2)))* to_deg;//内侧前轮子转的角度

              if(angle_in > +ACKERMANN_2W_MAX_ANGLE)angle_in=+ACKERMANN_2W_MAX_ANGLE;

              if(angle_in < -ACKERMANN_2W_MAX_ANGLE)angle_in=-ACKERMANN_2W_MAX_ANGLE;

              if(angle_out> +ACKERMANN_2W_MAX_ANGLE)angle_out=+ACKERMANN_2W_MAX_ANGLE;

              if(angle_out< -ACKERMANN_2W_MAX_ANGLE)angle_out=-ACKERMANN_2W_MAX_ANGLE;						

						}

		



						

							/*确定轮子要转的角度*/



											  if(flag_move ==  1||flag_move ==  2)angle_A=0,   angle_B=0,   angle_C=0,   angle_D=0;

									 else if(flag_move ==  3||flag_move ==  4)angle_A=+90, angle_B=-90, angle_C=+90, angle_D=-90;//B/D镜像安装

									 else if(flag_move ==  5||flag_move ==  8)angle_A=+MOVE_ANGLE_DIAGONAL, angle_B=+MOVE_ANGLE_DIAGONAL, angle_C=+MOVE_ANGLE_DIAGONAL, angle_D=+MOVE_ANGLE_DIAGONAL;					 

									 else if(flag_move ==  6||flag_move ==  7)angle_A=-MOVE_ANGLE_DIAGONAL, angle_B=-MOVE_ANGLE_DIAGONAL, angle_C=-MOVE_ANGLE_DIAGONAL, angle_D=-MOVE_ANGLE_DIAGONAL;				

									 else if(flag_move ==  9||flag_move == 10)angle_A=+MOVE_ANGLE_SPIN, angle_B=-MOVE_ANGLE_SPIN, angle_C=+MOVE_ANGLE_SPIN, angle_D=-MOVE_ANGLE_SPIN;

									 else if(flag_move == 11||flag_move == 13)angle_A=+angle_out, angle_B=+angle_in, angle_C=0, angle_D=0;

									 else if(flag_move == 12||flag_move == 14)angle_A=-angle_in, angle_B=-angle_out, angle_C=0, angle_D=0;

									 else if(flag_move == 15||flag_move == 17)angle_A=+angle_out, angle_B=+angle_in, angle_C=-angle_in, angle_D=-angle_out;

									 else if(flag_move == 16||flag_move == 18)angle_A=-angle_in, angle_B=-angle_out, angle_C=+angle_out, angle_D=+angle_in;													

								 

		          /*改变指令时的轮子转角和滚速的衔接优化*/

							/*前进后退左斜右斜左转右转停止 ,这些指令，互相转换时轮子滚速不停止*/

              /*除以上情况，指令转换皆等轮子滚速为0后再进行转角，转角完成后再给轮子滚速*/

						if(first_move==0&&Start_Flag==1&&flag_move!=0)marker_5=1,first_move=1;//第一次启动

             if(marker_5==0){

							   marker_1=0, marker_2=0,marker_3=0,marker_4=0;

							   if((last_flag_move!=flag_move)&&

									  (last_flag_move==3||last_flag_move==4||

								     last_flag_move==9||last_flag_move==10||

								     flag_move==3||flag_move==4||

								     flag_move==9||flag_move==10)

								   )marker_5=1;	//前后两次指令不同并且其中一个指令包含横移或自转

							   if((last_flag_move!=flag_move)&&

									  (last_flag_move==15||last_flag_move==16||

								     last_flag_move==17||last_flag_move==18||

								     flag_move==15||flag_move==16||

								     flag_move==17||flag_move==18)

								   )marker_5=1;	//前后两次指令不同并且其中一个指令包含横移或自转								 

							}											

             if(marker_5==1)

							   {									  

										if(marker_3==0)marker_1=1,marker_2=1,marker_3=1,marker_4=0;//停止转角和滚动

	                  if(marker_3==1&&marker_4==0){										   

										     if(fabs(Encoder_A)<20&&fabs(Encoder_B)<20&&fabs(Encoder_C)<20&&fabs(Encoder_D)<20)counter1++;

											   else counter1=0;

												 if(counter1>20){counter1=0;marker_1=0;marker_4=1;Flag_STOP=1; }//滚动接近停止后可以转角

										}	

	                  else if(marker_4==1){//转角完成后可以滚动											  

                         if((fabs(Angle_current_A -angle_A)<6)&&(fabs(Angle_current_B -angle_B)<6)

													&&(fabs(Angle_current_C -angle_C)<6)&&(fabs(Angle_current_D -angle_D)<6))counter2++;

												 else counter2=0;

												 if(counter2>50){counter2=0; Flag_STOP=0; marker_1=0;marker_2=0;marker_3=0;marker_4=0;marker_5=0;if(flag_move!=0)last_flag_move=flag_move;}	

										}										

								 }		

								 																							 							

							

							/*确定轮子的滚向和速度比例*/					 

									 if(flag_move ==  1)flag_LA=+1,flag_LB=+1,flag_LC=+1,flag_LD=+1;//定义前进时轮子转向和速度比例

							else if(flag_move ==  2)flag_LA=-1,flag_LB=-1,flag_LC=-1,flag_LD=-1;//定义后退时轮子转向

					 

							else if(flag_move ==  3)flag_LA=+1,flag_LB=-1,flag_LC=+1,flag_LD=-1;//左横移 B/D镜像

							else if(flag_move ==  4)flag_LA=-1,flag_LB=+1,flag_LC=-1,flag_LD=+1;//右横移 B/D镜像

					 

							else if(flag_move ==  5)flag_LA=+1,flag_LB=+1,flag_LC=+1,flag_LD=+1;//左斜上

							else if(flag_move ==  8)flag_LA=-1,flag_LB=-1,flag_LC=-1,flag_LD=-1;//右斜下

					  

							else if(flag_move ==  6)flag_LA=+1,flag_LB=+1,flag_LC=+1,flag_LD=+1;//右斜上

							else if(flag_move ==  7)flag_LA=-1,flag_LB=-1,flag_LC=-1,flag_LD=-1;//左斜下

					 

							else if(flag_move ==  9)flag_LA=+0.5,flag_LB=-0.5,flag_LC=-0.5,flag_LD=+0.5;//左自转

							else if(flag_move == 10)flag_LA=-0.5,flag_LB=+0.5,flag_LC=+0.5,flag_LD=-0.5;//右自转

					 

							else if(flag_move == 11)flag_LA=+(LA/L2),flag_LB=+(LB/L2),flag_LC=+(LC/L2),flag_LD=+(LD/L2);//左前转

							else if(flag_move == 12)flag_LA=+(LB/L2),flag_LB=+(LA/L2),flag_LC=+(LD/L2),flag_LD=+(LC/L2);//右前转

					 

							else if(flag_move == 13)flag_LA=-(LA/L2),flag_LB=-(LB/L2),flag_LC=-(LC/L2),flag_LD=-(LD/L2);//左后转

							else if(flag_move == 14)flag_LA=-(LB/L2),flag_LB=-(LA/L2),flag_LC=-(LD/L2),flag_LD=-(LC/L2);//右后转					 



							else if(flag_move == 15)flag_LA=+(LB/Turn_R),flag_LB=+(LA/Turn_R),flag_LC=+(LA/Turn_R),flag_LD=+(LB/Turn_R);//左前转

							else if(flag_move == 16)flag_LA=+(LA/Turn_R),flag_LB=+(LB/Turn_R),flag_LC=+(LB/Turn_R),flag_LD=+(LA/Turn_R);//右前转

							else if(flag_move == 17)flag_LA=-(LB/Turn_R),flag_LB=-(LA/Turn_R),flag_LC=-(LA/Turn_R),flag_LD=-(LB/Turn_R);//左后转

							else if(flag_move == 18)flag_LA=-(LA/Turn_R),flag_LB=-(LB/Turn_R),flag_LC=-(LB/Turn_R),flag_LD=-(LA/Turn_R);//右后转							

																											

							/*差速方式进行车子角度调整*/

									 if(flag_move ==  1)DIS_speed_A=-DIS_speed, DIS_speed_B=+DIS_speed, DIS_speed_C=+DIS_speed, DIS_speed_D=-DIS_speed;

							else if(flag_move ==  2)DIS_speed_A=+DIS_speed, DIS_speed_B=-DIS_speed, DIS_speed_C=-DIS_speed, DIS_speed_D=+DIS_speed;

					 

							else if(flag_move ==  3)DIS_speed_A=-DIS_speed, DIS_speed_B=-DIS_speed, DIS_speed_C=+DIS_speed, DIS_speed_D=+DIS_speed;

							else if(flag_move ==  4)DIS_speed_A=+DIS_speed, DIS_speed_B=+DIS_speed, DIS_speed_C=-DIS_speed, DIS_speed_D=-DIS_speed;

					 

							else if(flag_move ==  5)DIS_speed_A=-DIS_speed, DIS_speed_B=0, DIS_speed_C=+DIS_speed, DIS_speed_D=0;

							else if(flag_move ==  8)DIS_speed_A=+DIS_speed, DIS_speed_B=0, DIS_speed_C=-DIS_speed, DIS_speed_D=0;

					  

							else if(flag_move ==  6)DIS_speed_A=0, DIS_speed_B=+DIS_speed, DIS_speed_C=0, DIS_speed_D=-DIS_speed;

							else if(flag_move ==  7)DIS_speed_A=0, DIS_speed_B=-DIS_speed, DIS_speed_C=0, DIS_speed_D=+DIS_speed;

					 

							else DIS_speed_A=0, DIS_speed_B=0, DIS_speed_C=0, DIS_speed_D=0;

														

							

																															

																

    if(flag_move != 0){		

							if(marker_1==0&&Start_Flag==1){

			                      if((angle_A - last_angle_A)>+0.6)angle_A = last_angle_A +0.6;//做单次角度改变限制，可以使得角度变化更柔和

			                      if((angle_B - last_angle_B)>+0.6)angle_B = last_angle_B +0.6;

			                      if((angle_C - last_angle_C)>+0.6)angle_C = last_angle_C +0.6;

			                      if((angle_D - last_angle_D)>+0.6)angle_D = last_angle_D +0.6;

			                      if((angle_A - last_angle_A)<-0.6)angle_A = last_angle_A -0.6;

			                      if((angle_B - last_angle_B)<-0.6)angle_B = last_angle_B -0.6;

			                      if((angle_C - last_angle_C)<-0.6)angle_C = last_angle_C -0.6;

			                      if((angle_D - last_angle_D)<-0.6)angle_D = last_angle_D -0.6;

			                      last_angle_A = angle_A;

			                      last_angle_B = angle_B;

			                      last_angle_C = angle_C;

			                      last_angle_D = angle_D;



                            Angle_target_A =  angle_A;//得到最终目标角度

                            Angle_target_B =  angle_B;	

                            Angle_target_C =  angle_C;	

                            Angle_target_D =  angle_D;

							           }



													  				 

												if(marker_2==0){

	                             Velocity_target_A = (DIS_speed_A +Velocity_center)*flag_LA ;//得到最终目标速度

	                             Velocity_target_B = (DIS_speed_B +Velocity_center)*flag_LB;

	                             Velocity_target_C = (DIS_speed_C +Velocity_center)*flag_LC;

	                             Velocity_target_D = (DIS_speed_D +Velocity_center)*flag_LD;

													



														   if(flag_move > 8) DIS_speed=0,marker_6=0;//非直线行走状态不做直线调整		

                               else {

																      if((fabs(Angle_current_A -Angle_target_A)<5)&&(fabs(Angle_current_B -Angle_target_B)<5)

														              &&(fabs(Angle_current_C -Angle_target_C)<5)&&(fabs(Angle_current_D -Angle_target_D)<5))

													              {	

                                          if(marker_6==0)Yaw_target = Yaw,marker_6=1;	//做直线调整之前进行一次角度设定																				

	              					                DIS_speed = straight_line_Control(Yaw,Yaw_target);//角度闭环，轮子差速的方式进行直线移动调整

 	              					                     if(DIS_speed > +65)DIS_speed= +65;

	               					                else if(DIS_speed < -65)DIS_speed= -65;																

													              }

															      }																	

														 }

												if(marker_2==1){

	                            Velocity_target_A = 0;

	                            Velocity_target_B = 0;

	                            Velocity_target_C = 0;

	                            Velocity_target_D = 0;	

														 }																



	

											

	                    }   					

		

    if(flag_move == 0){

			                 DIS_speed=0;counter1=0;counter2=0;Flag_STOP=0;	

                       marker_1=0;marker_2=0;marker_3=0;marker_4=0;marker_5=0;marker_6=0;				

	                     last_angle_A = Angle_current_A;

	                     last_angle_B = Angle_current_B;

	                     last_angle_C = Angle_current_C;

	                     last_angle_D = Angle_current_D;

	                            Velocity_target_A = 0;

	                            Velocity_target_B = 0;

	                            Velocity_target_C = 0;

	                            Velocity_target_D = 0;	

                       if((fabs(Angle_current_A -angle_A)>20)||(fabs(Angle_current_B -angle_B)>20)

												||(fabs(Angle_current_C -angle_C)>20)||(fabs(Angle_current_D -angle_D)>20))last_flag_move=10;				

	                    }	

			

								 

	}

  

	 //转换来自ROS的控制指令

	if(FLAG_USART_ON<300){/*前后轴距340mm，左右轮距240mm*/

		      if(flag_chanone==0){marker_1=0;marker_2=0;marker_3=0;counter1=0;counter2=0;flag_chanone=1;}

					//舵轮独立控制模式

          if(Flag_Mode==0){

     			  if(Start_Flag==1){

     		           if(Angle_target_A > Angle_MID_A){Angle_target_A -= 0.6;if(Angle_target_A<Angle_MID_A)Angle_target_A=Angle_MID_A;}//上位机的转向角平缓转化为控制角度	

     		      else if(Angle_target_A < Angle_MID_A){Angle_target_A += 0.6;if(Angle_target_A>Angle_MID_A)Angle_target_A=Angle_MID_A;}//上位机的转向角平缓转化为控制角度	



     		           if(Angle_target_B > Angle_MID_B){Angle_target_B -= 0.6;if(Angle_target_B<Angle_MID_B)Angle_target_B=Angle_MID_B;}//上位机的转向角平缓转化为控制角度	

     		      else if(Angle_target_B < Angle_MID_B){Angle_target_B += 0.6;if(Angle_target_B>Angle_MID_B)Angle_target_B=Angle_MID_B;}//上位机的转向角平缓转化为控制角度									



     		           if(Angle_target_C > Angle_MID_C){Angle_target_C -= 0.6;if(Angle_target_C<Angle_MID_C)Angle_target_C=Angle_MID_C;}//上位机的转向角平缓转化为控制角度	

     		      else if(Angle_target_C < Angle_MID_C){Angle_target_C += 0.6;if(Angle_target_C>Angle_MID_C)Angle_target_C=Angle_MID_C;}//上位机的转向角平缓转化为控制角度	

							

     		           if(Angle_target_D > Angle_MID_D){Angle_target_D -= 0.6;if(Angle_target_D<Angle_MID_D)Angle_target_D=Angle_MID_D;}//上位机的转向角平缓转化为控制角度	

     		      else if(Angle_target_D < Angle_MID_D){Angle_target_D += 0.6;if(Angle_target_D>Angle_MID_D)Angle_target_D=Angle_MID_D;}//上位机的转向角平缓转化为控制角度									



               Velocity_target_A = (int)(Velocity_MID_A * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

               Velocity_target_B = (int)(Velocity_MID_B * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

               Velocity_target_C = (int)(Velocity_MID_C * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

               Velocity_target_D = (int)(Velocity_MID_D * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值							

     				 }

     			 }

      //XYZ三轴速度控制模式					

     			 else{

               //底盘参数

               const float L1_half = WHEELBASE_MM / 2000.0f;

               const float W_half  = TRACK_WIDTH_MM / 2000.0f;

               const float L = sqrt(L1_half*L1_half + W_half*W_half);

               const float sin_theta = L1_half / L;  // ≈0.816

               const float cos_theta = W_half / L;  // ≈0.577

               

               

               

               



               //变量初始化

               float Get_angle_A = 0.0f, Get_angle_B = 0.0f, Get_angle_C = 0.0f, Get_angle_D = 0.0f;

						 

               float Set_angle_A = 0.0f, Set_angle_B = 0.0f, Set_angle_C = 0.0f, Set_angle_D = 0.0f;

               float Velocity_AX = 0.0f, Velocity_AY = 0.0f, Velocity_BX = 0.0f, Velocity_BY = 0.0f;

               float Velocity_CX = 0.0f, Velocity_CY = 0.0f, Velocity_DX = 0.0f, Velocity_DY = 0.0f;

               float Velocity_H_A = 0.0f, Velocity_H_B = 0.0f, Velocity_H_C = 0.0f, Velocity_H_D = 0.0f;

               short forword_A = 1, forword_B = 1, forword_C = 1, forword_D = 1;

				 

               float omega = 0.0f;



               //输入速度限幅

               float Velocity_X = Velocity_MID_X;

               float Velocity_Y = Velocity_MID_Y;

               float Velocity_Z = Velocity_MID_Z;

               if (fabs(Velocity_X) < MIN_LINEAR_SPEED) Velocity_X = 0.0f;

               if (fabs(Velocity_Y) < MIN_LINEAR_SPEED) Velocity_Y = 0.0f;

               if (Velocity_Z > MAX_ANGULAR_SPEED) Velocity_Z = MAX_ANGULAR_SPEED;

               if (Velocity_Z < -MAX_ANGULAR_SPEED) Velocity_Z = -MAX_ANGULAR_SPEED;

							 if (fabs(Velocity_Z) < MIN_ANGULAR_SPEED)Velocity_Z=0;

               omega = Velocity_Z;



               //无操作时清零

               if (Velocity_X == 0.0f && Velocity_Y == 0.0f && Velocity_Z == 0.0f) {

                   Velocity_AX = Velocity_AY = Velocity_BX = Velocity_BY = 0.0f;

                   Velocity_CX = Velocity_CY = Velocity_DX = Velocity_DY = 0.0f;

                   Velocity_H_A = Velocity_H_B = Velocity_H_C = Velocity_H_D = 0.0f;

                   forword_A = forword_B = forword_C = forword_D = 1;



               } else {

                   //速度分解

                   // A轮（右上角）：牵连速度 = -omega*L*sin_theta（X向） + omega*L*cos_theta（Y向）

                   Velocity_AX = Velocity_X + omega * L * cos_theta;

                   Velocity_AY = Velocity_Y + omega * L * sin_theta;

                   // B轮（左上角）：牵连速度 = +omega*L*sin_theta（X向） + omega*L*cos_theta（Y向）

                   Velocity_BX = Velocity_X - omega * L * cos_theta;

                   Velocity_BY = Velocity_Y + omega * L * sin_theta;

                   // C轮（左下角）：牵连速度 = +omega*L*sin_theta（X向） - omega*L*cos_theta（Y向）

                   Velocity_CX = Velocity_X - omega * L * cos_theta;

                   Velocity_CY = Velocity_Y - omega * L * sin_theta;

                   // D轮（右下角）：牵连速度 = -omega*L*sin_theta（X向） - omega*L*cos_theta（Y向）

                   Velocity_DX = Velocity_X + omega * L * cos_theta;

                   Velocity_DY = Velocity_Y - omega * L * sin_theta;



                   //计算轮速

                   Velocity_H_A = sqrt(Velocity_AX*Velocity_AX + Velocity_AY*Velocity_AY);

                   Velocity_H_B = sqrt(Velocity_BX*Velocity_BX + Velocity_BY*Velocity_BY);

                   Velocity_H_C = sqrt(Velocity_CX*Velocity_CX + Velocity_CY*Velocity_CY);

                   Velocity_H_D = sqrt(Velocity_DX*Velocity_DX + Velocity_DY*Velocity_DY);



                   //速度归一化

                   float max_speed = fmaxf(fmaxf(Velocity_H_A, Velocity_H_B), fmaxf(Velocity_H_C, Velocity_H_D));

                   if (max_speed > MAX_WHEEL_SPEED) {

                       float scale = MAX_WHEEL_SPEED / max_speed;

                       Velocity_H_A *= scale;

                       Velocity_H_B *= scale;

                       Velocity_H_C *= scale;

                       Velocity_H_D *= scale;

                       //同步缩放速度分量

                       Velocity_AX *= scale; Velocity_AY *= scale;

                       Velocity_BX *= scale; Velocity_BY *= scale;

                       Velocity_CX *= scale; Velocity_CY *= scale;

                       Velocity_DX *= scale; Velocity_DY *= scale;

                   }



                   //计算目标角度

                   Get_angle_A = atan2(Velocity_AY, Velocity_AX) * to_deg;  // 弧度→角度（180/π≈57.3）

                   Get_angle_B = atan2(Velocity_BY, Velocity_BX) * to_deg;

                   Get_angle_C = atan2(Velocity_CY, Velocity_CX) * to_deg;

                   Get_angle_D = atan2(Velocity_DY, Velocity_DX) * to_deg;

		//舵机限幅(有限旋转范围)
		forword_A = forword_B = forword_C = forword_D = 1;
		float target_A = Get_angle_A;
		float target_B = Get_angle_B;
		float target_C = Get_angle_C;
		float target_D = Get_angle_D;

		// A轮限幅检查
		if(target_A > SERVO_LIMIT_A_MAX) {
			float flipped = target_A - 180.0f;
			if(flipped >= SERVO_LIMIT_A_MIN) { target_A = flipped; forword_A = -1; }
			else { target_A = SERVO_LIMIT_A_MAX; }
		} else if(target_A < SERVO_LIMIT_A_MIN) {
			float flipped = target_A + 180.0f;
			if(flipped <= SERVO_LIMIT_A_MAX) { target_A = flipped; forword_A = -1; }
			else { target_A = SERVO_LIMIT_A_MIN; }
		}
		// B轮限幅检查
		if(target_B > SERVO_LIMIT_B_MAX) {
			float flipped = target_B - 180.0f;
			if(flipped >= SERVO_LIMIT_B_MIN) { target_B = flipped; forword_B = -1; }
			else { target_B = SERVO_LIMIT_B_MAX; }
		} else if(target_B < SERVO_LIMIT_B_MIN) {
			float flipped = target_B + 180.0f;
			if(flipped <= SERVO_LIMIT_B_MAX) { target_B = flipped; forword_B = -1; }
			else { target_B = SERVO_LIMIT_B_MIN; }
		}
		// C轮限幅检查
		if(target_C > SERVO_LIMIT_C_MAX) {
			float flipped = target_C - 180.0f;
			if(flipped >= SERVO_LIMIT_C_MIN) { target_C = flipped; forword_C = -1; }
			else { target_C = SERVO_LIMIT_C_MAX; }
		} else if(target_C < SERVO_LIMIT_C_MIN) {
			float flipped = target_C + 180.0f;
			if(flipped <= SERVO_LIMIT_C_MAX) { target_C = flipped; forword_C = -1; }
			else { target_C = SERVO_LIMIT_C_MIN; }
		}
		// D轮限幅检查
		if(target_D > SERVO_LIMIT_D_MAX) {
			float flipped = target_D - 180.0f;
			if(flipped >= SERVO_LIMIT_D_MIN) { target_D = flipped; forword_D = -1; }
			else { target_D = SERVO_LIMIT_D_MAX; }
		} else if(target_D < SERVO_LIMIT_D_MIN) {
			float flipped = target_D + 180.0f;
			if(flipped <= SERVO_LIMIT_D_MAX) { target_D = flipped; forword_D = -1; }
			else { target_D = SERVO_LIMIT_D_MIN; }
		}

		Set_angle_A = target_A;
		Set_angle_B = target_B;
		Set_angle_C = target_C;
		Set_angle_D = target_D;

									 





               									 }



              //角度平滑过渡

              if (Start_Flag == 1) {

                  // 角度目标值渐变（步长可根据舵机响应速度调整）

                  if(Angle_target_A > Set_angle_A){Angle_target_A -= ANGLE_SLEW_RATE_XYZ;if(Angle_target_A < Set_angle_A)Angle_target_A = Set_angle_A;}

                  else if(Angle_target_A < Set_angle_A){Angle_target_A += ANGLE_SLEW_RATE_XYZ;if(Angle_target_A > Set_angle_A)Angle_target_A = Set_angle_A;}



                  if(Angle_target_B > Set_angle_B){Angle_target_B -= ANGLE_SLEW_RATE_XYZ;if(Angle_target_B < Set_angle_B)Angle_target_B = Set_angle_B;}

                  else if(Angle_target_B < Set_angle_B){Angle_target_B += ANGLE_SLEW_RATE_XYZ;if(Angle_target_B > Set_angle_B)Angle_target_B = Set_angle_B;}



                  if(Angle_target_C > Set_angle_C){Angle_target_C -= ANGLE_SLEW_RATE_XYZ;if(Angle_target_C < Set_angle_C)Angle_target_C = Set_angle_C;}

                  else if(Angle_target_C < Set_angle_C){Angle_target_C += ANGLE_SLEW_RATE_XYZ;if(Angle_target_C > Set_angle_C)Angle_target_C = Set_angle_C;}



                  if(Angle_target_D > Set_angle_D){Angle_target_D -= ANGLE_SLEW_RATE_XYZ;if(Angle_target_D < Set_angle_D)Angle_target_D = Set_angle_D;}

                  else if(Angle_target_D < Set_angle_D){Angle_target_D += ANGLE_SLEW_RATE_XYZ;if(Angle_target_D > Set_angle_D)Angle_target_D = Set_angle_D;}



                  //  【】等到位再启动逻辑 

                  // 规则：

                  // 只要是角度变化 → 一律等到误差 <10° 再启动（彻底防滑）

                  // 小角度依然快，大角度绝对稳

                  // =====

                  int all_wheel_in_position = 0;



                  //4轮到位才允许走

                  if( fabsf(Angle_target_A - Angle_current_A) < 10.0f &&

                      fabsf(Angle_target_B - Angle_current_B) < 10.0f &&

                      fabsf(Angle_target_C - Angle_current_C) < 10.0f &&

                      fabsf(Angle_target_D - Angle_current_D) < 10.0f )

                  {

                      all_wheel_in_position = 1;

                  }

                  else

                  {

                      all_wheel_in_position = 0;

                  }



                  

                  if(all_wheel_in_position)

                  {

                      //到位允许运动

                      marker_1 = 0;

                      marker_2 = 0;

                      marker_3 = 1;

                  }

                  else

                  {

                      //未到位禁止运动

                      marker_1 = 1;

                      marker_2 = 1;

                      marker_3 = 0;

                  }







                   //电机速度计算

                   if (marker_2 == 1) {

                       Velocity_target_A = 0.0f;

                       Velocity_target_B = 0.0f;

                       Velocity_target_C = 0.0f;

                       Velocity_target_D = 0.0f;

                       if (fabs(Encoder_A) < 20 && fabs(Encoder_B) < 20 &&

                           fabs(Encoder_C) < 20 && fabs(Encoder_D) < 20) {

                           counter1++;

                       } else {

                           counter1 = 0;

                       }

                       if (counter1 > 5) {

                           marker_1 = 0;

                           marker_3 = 1;

                           counter1 = 0;

                       }

                   } else {







                       Velocity_target_A = (int)(forword_A * Velocity_H_A * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

                       Velocity_target_B = (int)(forword_B * Velocity_H_B * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

                       Velocity_target_C = (int)(forword_C * Velocity_H_C * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值

                       Velocity_target_D = (int)(forword_D * Velocity_H_D * VEL_TO_ENCODER);//线速度转化为电机编码器目标数值



                   }

               }

	         } 		

   }

}


/* 合并的速度PID — 取代4个重复函数
 * idx: 0=A, 1=B, 2=C, 3=D
 */
typedef struct { float velocity, bias, last_bias; } VelPID;
static VelPID pid[4];

int velocity_Control(int idx, int encoder, int target)
{
    VelPID *p = &pid[idx];
    p->bias = target - encoder;
    p->velocity += Velocity_KP*(p->bias - p->last_bias) + p->bias*Velocity_KI/100.0f;
    if(p->velocity > +PWM_MAX) p->velocity = +PWM_MAX;
    if(p->velocity < -PWM_MAX) p->velocity = -PWM_MAX;
    p->last_bias = p->bias;
    if(Start_Flag==0) { p->last_bias = 0; p->velocity = 0; }
    return (int)p->velocity;
}

/****************************************************************************************

函数功能：车体Z轴角速度闭环

入口参数：

        float Angle 当前角度

        float Target  目标角度

        int Output 最终数值

        int Last_bias  上一次偏差值*/

/****************************************************************************************/

float V_Z_Control(float Gyro,float Gyro_control)

{  

   static float PWM,error,Bias,Last_Bias,D_Bias,Last_Gyro;

	 Bias=Gyro - Gyro_control;                                       //获取偏差

	 error+=Bias;                                                    //偏差累积

	 if(error>+1000)error=+1000;                                     //积分限幅

	 if(error<-1000)error=-1000;                                     //积分限幅

	 D_Bias=Last_Gyro - Gyro;                                        // D项仅作用于测量值,消除目标突变时的微分冲击

	 PWM=Gyro_KP*Bias/1000 + Gyro_KI*error/1000 + D_Bias*Gyro_KD/1000;   //获取最终数值

	 Last_Bias=Bias;                                                 //本次偏差赋值作为上次偏差

	 Last_Gyro=Gyro;                                                 //本次测量值赋值作为上次

	 if(Start_Flag==0) Last_Bias=0,PWM=0,error=0,Last_Gyro=0;        //停止时参数清零

	 return PWM;

}

/****************************************************************************************

函数功能：车体角度闭环

入口参数：

        float Angle 当前角度

        float Target  目标角度

        int *Output 最终数值

        int *Last_bias  上一次偏差值*/

/****************************************************************************************/


int straight_line_Control(float Angle,float Target)

{  

	float Least,Differential,Output;

  static float error,bias,Last_bias;

  	      Least =Angle-Target;//获取偏差		

          bias *=0.8;		           //一阶低通滤波 

          bias += Least*0.2;	     //一阶低通滤波 

	        Differential=bias - Last_bias;  //获取偏差变化率

	        Last_bias=bias;               //保存上一次的偏差

	        error +=Least;

          if(error> +100)error=+100; //限幅

          if(error< -100)error=-100; //限幅	

		      Output=bias*line_KP/10 + error*line_KI/10 + Differential*line_KD/10; //得到最终数值

	  return Output;	

}



/**************************************************************************

函数功能：赋值给PWM寄存器

*************************************************************************/


//入口参数为轮子转速 m/s，舵轮角度 degree


float Position_x(float Encoder,float Target)

{  

   static float PWM,Last_Position,Position_Bias,Position_Differential;

	 static float Position_Least;

  	Position_Least =Encoder-Target;                       //获取位置偏差 



    Position_Bias *=0.8;		                              //一阶低通滤波    

    Position_Bias += Position_Least*0.2;                  //一阶低通滤波                    

	  Position_Differential=Position_Bias-Last_Position;    //求出偏差的微分                         

		PWM=Position_Bias*0.8f + Position_Differential*0.015f;//PD控制累加 

	  Last_Position=Position_Bias;                          //本次偏差保存

	  return PWM;

}

/**************************************************************************

函数功能：位置PD控制 

**************************************************************************/

float Position_y(float Encoder,float Target)

{  

   static float PWM,Last_Position,Position_Bias,Position_Differential;

	 static float Position_Least;

  	Position_Least =Encoder-Target;                       //获取位置偏差 

	

    Position_Bias *=0.8;		                              //一阶低通滤波    

    Position_Bias += Position_Least*0.2;                  //一阶低通滤波                    

	  Position_Differential=Position_Bias-Last_Position;    //求出偏差的微分                         

		PWM=Position_Bias*0.8f + Position_Differential*0.015f;//PD控制累加 

	  Last_Position=Position_Bias;                          //本次偏差保存

	  return PWM;

}



