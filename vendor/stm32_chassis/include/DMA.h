#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
void Dma_Enable(DMA_Channel_TypeDef * DMA_CHx,u16 Number);
void Dma_Disenable(DMA_Channel_TypeDef * DMA_CHx);
void Dma_Init(u8 num,DMA_Channel_TypeDef * DMA_CHx,u32 P_Address ,u32 M_Address,u16 Number);
#endif	
