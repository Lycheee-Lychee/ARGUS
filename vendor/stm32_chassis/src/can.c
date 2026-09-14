#include "CAN.h"	

//CAN1初始化
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN1_Normal_Init(1,2,3,12,1);
//则波特率为:   36/(1+3+2)/12=500K  
//返回值:0,初始化OK;
//其它,初始化失败;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1;//先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	RCC->APB2ENR|=1<<0;    //开启辅助时钟
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	 
	RCC->APB1ENR|=1<<25;//使能CAN1时钟 CAN1使用的是APB1的时钟(max:36M)
	
	GPIOB->CRH&=0XFFFFFF00; 
	GPIOB->CRH|=0X000000B8;//PB8 RX,PB9 TX推挽输出   	 
  GPIOB->ODR|=3<<8;
	AFIO->MAPR|=2<<13;     //CAN部分重映射

	CAN1->MCR=0x0000;	//退出睡眠模式(同时设置所有位为0)
	CAN1->MCR|=1<<0;		//请求CAN1进入初始化模式
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2;//进入初始化模式失败
	}
	CAN1->MCR|=0<<7;		//非时间触发通信模式
	CAN1->MCR|=0<<6;		//软件自动离线管理
	CAN1->MCR|=0<<5;		//睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)
	CAN1->MCR|=1<<4;		//禁止报文自动传送
	CAN1->MCR|=0<<3;		//报文不锁定,新的覆盖旧的
	CAN1->MCR|=0<<2;		//优先级由报文标识符决定
	CAN1->BTR=0x00000000;//清除原来的设置.
	CAN1->BTR|=mode<<30;	//模式设置 0,普通模式;1,回环模式;
	CAN1->BTR|=tsjw<<24; //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN1->BTR|=tbs2<<20; //Tbs2=tbs2+1个时间单位
	CAN1->BTR|=tbs1<<16;	//Tbs1=tbs1+1个时间单位
	CAN1->BTR|=brp<<0;  	//分频系数(Fdiv)为brp+1
						//波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->MCR&=~(1<<0);	//请求CAN1退出初始化模式
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3;//退出初始化模式失败
	}
	//过滤器初始化
	CAN1->FMR|=1<<0;			//过滤器组工作在初始化模式
	CAN1->FA1R&=~(1<<0);		//过滤器0不激活
	CAN1->FS1R|=1<<0; 		//过滤器位宽为32位.
	CAN1->FM1R|=0<<0;		//过滤器0工作在标识符屏蔽位模式
	CAN1->FFA1R|=0<<0;		//过滤器0关联到FIFO0
	CAN1->sFilterRegister[0].FR1=0X00000000;//32位ID
	CAN1->sFilterRegister[0].FR2=0X00000000;//32位MASK
	CAN1->FA1R|=1<<0;		//激活过滤器0
	CAN1->FMR&=0<<0;			//过滤器组进入正常模式

#if CAN1_RX0_INT_ENABLE
 	//使用中断接收
	CAN1->IER|=1<<1;			//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(1,0,USB_LP_CAN1_RX0_IRQn,2);//组2
#endif
	return 0;
}  
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//*dat:数据指针.
//返回值:0~3,邮箱编号.0XFF,无有效邮箱.
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;			//邮箱0为空
	else if(CAN1->TSR&(1<<27))mbox=1;	//邮箱1为空
	else if(CAN1->TSR&(1<<28))mbox=2;	//邮箱2为空
	else return 0XFF;					//无空邮箱,无法发送 
	CAN1->sTxMailBox[mbox].TIR=0;		//清除之前的设置
	if(ide==0)	//标准帧
	{
		id&=0x7ff;//取低11位stdid
		id<<=21;		  
	}else		//扩展帧
	{
		id&=0X1FFFFFFF;//取低32位extid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F;//得到低四位
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;		   //设置DLC.
	//待发送数据存入邮箱.
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //请求发送邮箱数据
	return mbox;
}
//获得发送状态.
//mbox:邮箱编号;
//返回值:发送状态. 0,挂起;0X05,发送失败;0X07,发送成功.
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			//RQCP0
			sta |= CAN1->TSR&(1<<1);			//TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24);	//TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		//RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		//TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25);	//TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	//RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	//TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26);	//TME2
			break;
		default:
			sta=0X05;//邮箱号不对,肯定失败.
		break;
	}
	return sta;
} 
//得到在FIFO0/FIFO1中接收到的报文个数.
//fifox:0/1.FIFO编号;
//返回值:FIFO0/FIFO1中的报文个数.
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN1->RF0R&0x03; 
	else if(fifox==1)return CAN1->RF1R&0x03; 
	else return 0;
}
//接收数据
//fifox:邮箱号
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//dat:数据缓存区
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//得到标识符选择位的值  
 	if(*ide==0)//标准标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	   //扩展标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//得到远程发送请求值.
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;//得到DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//得到FMI
	//接收数据
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  	if(fifox==0)CAN1->RF0R|=0X20;//释放FIFO0邮箱
	else if(fifox==1)CAN1->RF1R|=0X20;//释放FIFO1邮箱	 
}
//CAN1发送一组数据(固定格式:ID为0X601,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0XFFF)return 1;							//发送失败?
	return 0;										//发送成功;
}
//CAN1口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;			//没有接收到数据,直接退出 	 
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		//接收错误	   
	return len;	
}
u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0XFFF)return 1;							//发送失败?
	return 0;										//发送成功;
}

/* 
*  函数名称：CAN1_Send_Numm(u32 id,u8* msg)
*  入口参数：id  ID号，msg  被输送数组的名称
*  函数输出： 1，发送失败；   0，发送成功  
*  函数功能：给给定的id发送一个数组的命令
*/
u8 CAN1_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0XFFF)return 1;							//发送失败?
	return 0;
}


//数据发送函数
void CAN1_SEND(int id,u8* temp ) 
{
				CAN1_Send_Num(id,temp); 
}


extern u16 CAN_ID1,CAN_ID2,CAN_ID3,CAN_ID4;
u8 FLAG_CAN;
u8 txbuf[8],rxbuf[8],Rxbuf_1[8],Rxbuf_2[8],Rxbuf_3[8],Rxbuf_4[8];
volatile u8 can_online = 0x0F;   // bit0-3: 1=servo online, 0=offline
static u8 can_to[4] = {0};       // per-servo timeout counters

#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//接收中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  u8 i;
	u32 id;
	u8 ide,rtr,len;     
 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,rxbuf);
 if(id==CAN_ID1)//若ID匹配
	{	
		for(i=0;i<8;i++)
		{
		Rxbuf_1[i]=rxbuf[i];
		rxbuf[i]=0;
		}
    FLAG_CAN=1;
  }
  if(id==CAN_ID2)//若ID匹配
	{	
		for(i=0;i<8;i++)
		{
		Rxbuf_2[i]=rxbuf[i];
		rxbuf[i]=0;
	 	}
    FLAG_CAN=1;
  }
  if(id==CAN_ID3)//若ID匹配
	{	
		for(i=0;i<8;i++)
		{
		Rxbuf_3[i]=rxbuf[i];
		rxbuf[i]=0;
		}
    FLAG_CAN=1;
  }
  if(id==CAN_ID4)//若ID匹配
	{	
		for(i=0;i<8;i++)
		{
		Rxbuf_4[i]=rxbuf[i];
		rxbuf[i]=0;
		}
   FLAG_CAN=1;
  }	
}

#endif


void CAN1_SEND_data(u8 EN_A,u8 EN_B,u8 EN_C,u8 EN_D,float angle_A,float angle_B,float angle_C,float angle_D) 
{
	static u8 sum=0;
	static u8 Time_step;
	short data_A,data_B,data_C,data_D;
	
	data_A = (short)(angle_A*100)%100;
	data_B = (short)(angle_B*100)%100;	
	data_C = (short)(angle_C*100)%100;	
	data_D = (short)(angle_D*100)%100;	
	
    if(FLAG_CAN_ON<30)
			{		
	     Time_step++;
	     if(Time_step==1)
				  {sum=0;
	    			txbuf[0]=0XAA;//帧头
	    			txbuf[1]=EN_A; 
	    			txbuf[2]=(short)angle_A>>0;	
	    			txbuf[3]=(short)angle_A>>8;		
			      txbuf[4]=(short)data_A>>0;	  
				    txbuf[5]=(short)data_A>>8;	
            for(u8 i=1;i<6;i++)sum+=txbuf[i];	
	      		txbuf[6]=sum; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]
		    		txbuf[7]=0xAF;//帧尾						
		    		
	          CAN1_SEND(CAN_ID1,txbuf);//CAN发送 id can_id  发送数据长度 8 个字节
	          

	        }
	     if(Time_step==2)
				  {sum=0;
	    			txbuf[0]=0XAA;//帧头
	    			txbuf[1]=EN_B; 
	    			txbuf[2]=(short)angle_B>>0;	
	    			txbuf[3]=(short)angle_B>>8;		
			      txbuf[4]=(short)data_B>>0;	  
				    txbuf[5]=(short)data_B>>8;	
            for(u8 i=1;i<6;i++)sum+=txbuf[i];	
	      		txbuf[6]=sum; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]
		    		txbuf[7]=0xAF;//帧尾						
		    		
	          CAN1_SEND(CAN_ID2,txbuf);//CAN发送 id can_id  发送数据长度 8 个字节

	
	        } 
	     if(Time_step==3)
				  {sum=0;
	    			txbuf[0]=0XAA;//帧头
	    			txbuf[1]=EN_C; 
	    			txbuf[2]=(short)angle_C>>0;	
	    			txbuf[3]=(short)angle_C>>8;		
			      txbuf[4]=(short)data_C>>0;	  
				    txbuf[5]=(short)data_C>>8;	
            for(u8 i=1;i<6;i++)sum+=txbuf[i];	
	      		txbuf[6]=sum; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]
		    		txbuf[7]=0xAF;//帧尾						
		    		
	          CAN1_SEND(CAN_ID3,txbuf);//CAN发送 id can_id  发送数据长度 8 个字节
	          

	        } 
	     if(Time_step==4)
				  {sum=0;
	    			txbuf[0]=0XAA;//帧头
	    			txbuf[1]=EN_D; 
	    			txbuf[2]=(short)angle_D>>0;	
	    			txbuf[3]=(short)angle_D>>8;		
			      txbuf[4]=(short)data_D>>0;	  
				    txbuf[5]=(short)data_D>>8;	
            for(u8 i=1;i<6;i++)sum+=txbuf[i];	
	      		txbuf[6]=sum; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]
		    		txbuf[7]=0xAF;//帧尾						
		    		
	          CAN1_SEND(CAN_ID4,txbuf);//CAN发送 id can_id  发送数据长度 8 个字节
	          
            Time_step=0;	
	        } 					
		}  
}

//处理接收到的数据帧 
/*
	      Rxbuf[0]=0XAA;//帧头
				Rxbuf[1]=0; //数据位 使能位 0/1 停止/启动
				Rxbuf[2]=0;	//数据位 当前角度低8位 （角度设置为short型分步接收）
				Rxbuf[3]=0;	//数据位 当前角度高8位 
			  Rxbuf[4]=0; //数据位 当前角度小数点后2位 
				Rxbuf[5]=0;	//数据位 预留 	
	  		Rxbuf[6]=0; //校验位 txbuf[1]到txbuf[5]之和=txbuf[6]
				Rxbuf[7]=0xAF;//帧尾

*/	
void CAN1_Receive_data(void) //具体数据接收在can中断服务函数里，这里做数据处理
{
        if(FLAG_CAN==1)
					{
				    if(Rxbuf_1[0]==0XAA&&Rxbuf_1[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_1[i];//和校验确认
					      if(Rxbuf_1[6]==sum){fault|=1;
	                  can_to[0]=0; can_online|=1;
                  Current_angle_A =  -((short)(Rxbuf_1[2]|(Rxbuf_1[3]<<8)) + (float)((short)(Rxbuf_1[4]|(Rxbuf_1[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_1[0]=0XFF;						
				      }

				    if(Rxbuf_2[0]==0XAA&&Rxbuf_2[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_2[i];//和校验确认
					      if(Rxbuf_2[6]==sum){fault|=2;
	                  can_to[1]=0; can_online|=2;
                  Current_angle_B =  -((short)(Rxbuf_2[2]|(Rxbuf_2[3]<<8)) + (float)((short)(Rxbuf_2[4]|(Rxbuf_2[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_2[0]=0XFF;				
				      }

				    if(Rxbuf_3[0]==0XAA&&Rxbuf_3[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_3[i];//和校验确认
					      if(Rxbuf_3[6]==sum){fault|=4;
	                  can_to[2]=0; can_online|=4;
                  Current_angle_C =  -((short)(Rxbuf_3[2]|(Rxbuf_3[3]<<8)) + (float)((short)(Rxbuf_3[4]|(Rxbuf_3[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_3[0]=0XFF;				
				      }

				    if(Rxbuf_4[0]==0XAA&&Rxbuf_4[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_4[i];//和校验确认
					      if(Rxbuf_4[6]==sum){fault|=8;
	                  can_to[3]=0; can_online|=8;
                  Current_angle_D =  -((short)(Rxbuf_4[2]|(Rxbuf_4[3]<<8)) + (float)((short)(Rxbuf_4[4]|(Rxbuf_4[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_4[0]=0XFF;				
				      }
					  
					  FLAG_CAN_ON=0;
				    FLAG_CAN=0;
				  }
	         // Per-servo CAN timeout: 25*5=125ms offline detection
	         for(u8 i=0;i<4;i++){if(can_to[i]<255)can_to[i]++;if(can_to[i]>25)can_online&=~(1<<i);}
         Angle_current_A = 	(Current_angle_A + Angle_error_A)/offset_a;
         Angle_current_B = 	(Current_angle_B + Angle_error_B)/offset_b;
         Angle_current_C = 	(Current_angle_C + Angle_error_C)/offset_c;
         Angle_current_D = 	(Current_angle_D + Angle_error_D)/offset_d;						
}
