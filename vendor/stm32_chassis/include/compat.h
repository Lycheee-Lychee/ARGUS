/* Keil类型别名兼容头文件(GCC) */
#ifndef __COMPAT_H
#define __COMPAT_H

#include <stdint.h>

/* Keil-style type aliases used throughout the codebase */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;

typedef volatile uint8_t  vu8;
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

#endif
