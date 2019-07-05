#ifndef __VCXO_H__
#define __VCXO_H__

#include "stm32f0xx_hal.h"

void vcxo_adjust(int16_t input);      //把buf_delay输入，这个函数会计算并调整晶振频率


#endif
