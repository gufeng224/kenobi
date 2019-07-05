#ifndef __NAU85L04_H__
#define __NAU85L04_H__

#include "stm32f0xx_hal.h"

void nau85l04_init(I2C_HandleTypeDef *hi2c);
void nau85l04_poweroff(I2C_HandleTypeDef *hi2c);

#endif
