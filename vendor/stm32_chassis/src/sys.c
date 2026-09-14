/* sys.c - 系统初始化
 * GCC内联汇编替代Keil __asm语法
 * 寄存器级逻辑完整保留原版
 */
#include "sys.h"

//NVIC向量表偏移
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
    SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80);
}

//NVIC优先级分组
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
    u32 temp, temp1;
    temp1 = (~NVIC_Group) & 0x07;
    temp1 <<= 8;
    temp = SCB->AIRCR;
    temp &= 0X0000F8FF;
    temp |= 0X05FA0000;
    temp |= temp1;
    SCB->AIRCR = temp;
}

//NVIC初始化
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority,
                  u8 NVIC_Channel, u8 NVIC_Group)
{
    u32 temp;
    MY_NVIC_PriorityGroupConfig(NVIC_Group);
    temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
    temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
    temp &= 0xf;
    NVIC->ISER[NVIC_Channel / 32] |= (1 << (NVIC_Channel % 32));
    NVIC->IP[NVIC_Channel] |= temp << 4;
}

/* ----- 外部中断配置 ----- */
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
    u8 EXTADDR;
    u8 EXTOFFSET;
    EXTADDR = BITx / 4;
    EXTOFFSET = (BITx % 4) * 4;
    RCC->APB2ENR |= 0x01;
    AFIO->EXTICR[EXTADDR] &= ~(0x000F << EXTOFFSET);
    AFIO->EXTICR[EXTADDR] |= GPIOx << EXTOFFSET;
    EXTI->IMR |= 1 << BITx;
    if (TRIM & 0x01) EXTI->FTSR |= 1 << BITx;
    if (TRIM & 0x02) EXTI->RTSR |= 1 << BITx;
}

/* ----- RCC复位到默认值 ----- */
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000;
    RCC->APB2RSTR = 0x00000000;
    RCC->AHBENR = 0x00000014;
    RCC->APB2ENR = 0x00000000;
    RCC->APB1ENR = 0x00000000;
    RCC->CR |= 0x00000001;
    RCC->CFGR &= 0xF8FF0000;
    RCC->CR &= 0xFEF6FFFF;
    RCC->CR &= 0xFFFBFFFF;
    RCC->CFGR &= 0xFF80FFFF;
    RCC->CIR = 0x00000000;
    MY_NVIC_SetVectorTable(0x08000000, 0x0);
}

/* ----- JTAG模式设置 ----- */
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp = mode;
    temp <<= 25;
    RCC->APB2ENR |= 1 << 0;
    AFIO->MAPR &= 0XF8FFFFFF;
    AFIO->MAPR |= temp;
}

/* ----- 系统时钟初始化 (PLL=HSE*PLL, PLL=9 → 72MHz) ----- */
void Stm32_Clock_Init(u8 PLL)
{
    unsigned char temp = 0;
    MYRCC_DeInit();
    RCC->CR |= 0x00010000;       /* HSEON */
    while (!(RCC->CR >> 17));     /* 等待HSE就绪 */
    RCC->CFGR = 0X00000400;       /* APB1=DIV2; APB2=DIV1; AHB=DIV1 */
    PLL -= 2;
    RCC->CFGR |= PLL << 18;       /* PLL倍频 */
    RCC->CFGR |= 1 << 16;         /* PLLSRC = HSE */
    FLASH->ACR |= 0x32;           /* 2个等待周期 */
    RCC->CR |= 0x01000000;        /* PLLON */
    while (!(RCC->CR >> 25));     /* 等待PLL就绪 */
    RCC->CFGR |= 0x00000002;      /* PLL作为SYSCLK */
    while (temp != 0x02) {
        temp = RCC->CFGR >> 2;
        temp &= 0x03;
    }
}

/* ----- GCC内联汇编包装 (替代Keil __asm) ----- */
__attribute__((naked)) void WFI_SET(void)
{
    __asm volatile ("wfi");
    __asm volatile ("bx lr");
}

__attribute__((naked)) void INTX_DISABLE(void)
{
    __asm volatile ("cpsid i");
    __asm volatile ("bx lr");
}

__attribute__((naked)) void INTX_ENABLE(void)
{
    __asm volatile ("cpsie i");
    __asm volatile ("bx lr");
}

__attribute__((naked)) void MSR_MSP(u32 addr)
{
    __asm volatile ("msr msp, r0");
    __asm volatile ("bx lr");
}

/* ----- 待机模式 ----- */
void Sys_Standby(void)
{
    SCB->SCR |= 1 << 2;
    RCC->APB1ENR |= 1 << 28;
    PWR->CSR |= 1 << 8;
    PWR->CR |= 1 << 2;
    PWR->CR |= 1 << 1;
    WFI_SET();
}

/* ----- 软件复位 ----- */
void Sys_Soft_Reset(void)
{
    SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}
