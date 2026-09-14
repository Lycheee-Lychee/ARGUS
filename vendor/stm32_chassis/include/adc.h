#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define voltage_Ch 10
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_voltage(void); 
#endif 















