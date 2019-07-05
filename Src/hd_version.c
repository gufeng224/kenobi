#include "hd_version.h"
static hd_version_t hd_version;

void hd_version_init(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(hadc), HAL_ADC_STATE_REG_EOC)) {
        uint16_t value = HAL_ADC_GetValue(hadc);
        if(value < (4096*1/3.3)) {
            hd_version = HD_VERSION_DIGITAL_MIC;
        } else if(value < (4096*2/3.3)) {
            hd_version = HD_VERSION_ANALOG_MIC;
        } else {
            //hd_version = HD_VERSION_ANALOG_MIC;
					hd_version = HD_VERSION_DIGITAL_MIC;
        }
    }
}


hd_version_t hd_version_getVersion(void)
{
    return hd_version;
}

