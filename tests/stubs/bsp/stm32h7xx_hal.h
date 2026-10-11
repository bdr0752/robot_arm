#pragma once
#include "usart.h"

#define __STATIC_FORCEINLINE static inline
typedef struct { uint32_t CYCCNT; } DWT_Type;
extern DWT_Type Test_Dwt;
#define DWT (&Test_Dwt)

extern "C" uint32_t HAL_GetTick(void);
