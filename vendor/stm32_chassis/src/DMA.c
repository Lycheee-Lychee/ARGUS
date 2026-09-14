#include "DMA.h" 	 

  
//DMA通道初始化函数
//参数说明:
//          DMA_CHx      :选择DMA控制通道,DMA1：1-7,DMA2：1-5
//          P_Adress     :外设地址
//          M_Adress     :存储器地址	 
void Dma_Init(u8 num,DMA_Channel_TypeDef * DMA_CHx,u32 P_Address ,u32 M_Address,u16 Number)
{
	    if(num==1)
	    RCC->AHBENR |= 1<<0;//开启DMA1时钟
      else if(num==2)
	    RCC->AHBENR |= 1<<1;//开启DMA2时钟
			
	    delay_ms(10);                
	    //必须配置好通道后配置地址
	    DMA_CHx -> CPAR = P_Address;    //设定外设寄存器地址
	    DMA_CHx -> CMAR = M_Address;    //设定数据寄存器地址
	    delay_ms(10);
	    DMA_CHx -> CNDTR = Number;       //数据传输量	
	    delay_ms(10);
			DMA_CHx -> CCR  &= 0x00000000;       //复位  
		  delay_ms(10);
	    DMA_CHx -> CCR  |= 0<<14;          //0:非存储器到存储器模式; 1:存储器到存储器模式 
	   	//设定为中等优先级
			DMA_CHx -> CCR  |= 0<<13;          //00:低 01:中 10:高 11:最高 
	    DMA_CHx -> CCR  |= 0<<12;          //通道优先级,由[13:12]两位控制
			//设定存储数据宽度
      DMA_CHx -> CCR  |= 0<<11;          //00:8位 01:16位 10:32位 11:保留 
	    DMA_CHx -> CCR  |= 0<<10;          //存储器数据宽度,由[11:10]控制

			//设定外设数据宽度
		  DMA_CHx -> CCR  |= 0<<9;           //00:8位 01:16位 10:32位 11:保留
	    DMA_CHx -> CCR  |= 0<<8;           //外设数据宽度,由[9:8]两位控制

			//设定地址增量
			DMA_CHx -> CCR  |= 1<<7;           //0:不执行存储器地址增量操作 1:执行存储器地址增量操作			
	    DMA_CHx -> CCR  |= 0<<6;           //0:不执行外设地址增量操作   1:执行外设地址增量操作

	   	//设定数据传输方向
			DMA_CHx -> CCR  |= 0<<5;           //0:不执行循环操作 1:执行循环操作 
	    DMA_CHx -> CCR  |= 1<<4;           //设定数据传输方向   0:从外设读 1:从存储器读
			
//      DMA_CHx -> CCR  |= 1<<3;           //允许传输错误中断，读写一个保留的地址区域，将会产生MDA传输错误
//      DMA_CHx -> CCR  |= 1<<2;           //允许半传输中断			
//	    DMA_CHx -> CCR  |= 1<<1;           //允许传输完成中断
	    
	    		
}
	 
	 	 
//DMA通道使能
//参数说明:
//          DMA_CHx      :选择DMA控制器通道,DMA1：1-7,DMA2：1-5
//          Number       :数据传输量
void Dma_Enable(DMA_Channel_TypeDef * DMA_CHx,u16 Number)
{
	    DMA_CHx -> CNDTR = Number;       //数据传输量
	    DMA_CHx -> CCR |= 1<<0;          //开始DMA传输
}
//DMA通道除能
//参数说明:
//          DMA_CHx      :选择DMA控制器通道,DMA1：1-7,DMA2：1-5
//          Number       :数据传输量
void Dma_Disenable(DMA_Channel_TypeDef * DMA_CHx)
{
	    DMA_CHx->CCR&=~(1<<0);       //关闭DMA传输
}
