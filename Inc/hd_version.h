#ifndef __HD_VERSION_H__
#define __HD_VERSION_H__

#include "stm32f0xx_hal.h"

typedef enum {
    HD_VERSION_DIGITAL_MIC = 0,
    HD_VERSION_ANALOG_MIC, 
} hd_version_t;

void hd_version_init(ADC_HandleTypeDef*);
hd_version_t hd_version_getVersion(void);

#endif
