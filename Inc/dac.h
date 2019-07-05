#ifndef __DAC_H__
#define __DAC_H__

#include "stm32f0xx_hal.h"

void dac_init(void);
void dac_set_value(uint16_t dat);        //设置dac，0~4096设置0~3.3v

#endif
