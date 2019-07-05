#include "nau85l04.h"

static const  uint16_t  data[]={
    0x0 ,	0x0000,	  /*!(0x0000)*/
    0x3 ,	0x0000,	  /*!(0x0140)*/
    0x4 ,	0x0001,
    0x5 ,	0x3126,
    0x6 ,	0x0008,	
    0x7 ,	0x0010,
    0x8 ,	0xC000,
    0x9 ,	0x6000,
    0xA ,	0xF13C,
    0x10,	0x0002,		/*!I2S mode*/
    0x11,	0x0402,		/*!*/
    0x12,	0x0000,		/*!*/
    0x13,	0x0000,
    0x14,	0xC00F,		/*!*/

    0x20,	0x467A,
    0x21,       0x310A,      //    0x21,	0x320A,
    0x22,	0xF000,
    0x23,	0xD0D0,
    0x24,	0xD0D0,

    0x30,	0x0000,
    0x31,	0x0000,
    0x32,	0x0000,
    0x33,	0x0000,
    0x34,	0x0000,
    0x35,	0x0000,
    0x36,	0x0000,
    0x37,	0x0000,
    0x38,	0x1010,
    0x39,	0x1010,
    0x3A,	0x0003,		/*!*/
    0x40,	0x0400,
    0x41,	0x1400,
    0x42,	0x2400,
    0x43,	0x3400,
    0x44,	0x00E4,
    0x50,	0x0000,
    0x51,	0x0000,
    0x52,	0xEFFF,
    0x60,	0x0060,
    0x61,	0x0000,
    0x64,	0x0000,
    0x65,	0x0220,
    0x66,	0x000F,		/*!*/
    0x67,	0x0D38,		/*!*/
    0x68,	0x3000,
    0x69,	0x0011,
    0x6A,	0x0011,
    0x6B,	0x2525,		/*!*/
    0x6C,	0x2525,		/*!*/
    0x6D,	0xF000,		/*!*/
    0x1 ,	0x000F,		/*!*/
    0x2 , 0xC00A,		/*!*/
};
static const  uint16_t  data_poweroff[]={
    0x0 ,	0x0000,	  /*!(0x0000)*/
    0x3 ,	0x0000,	  /*!(0x0140)*/
    0x4 ,	0x0001,
    0x5 ,	0x3126,
    0x6 ,	0x0008,	
    0x7 ,	0x0010,
    0x8 ,	0xC000,
    0x9 ,	0x6000,
    0xA ,	0xF13C,
    0x10,	0x0002,		/*!I2S mode*/
    0x11,	0x8000,		/*!*/
    0x12,	0x8000,		/*!*/
    0x13,	0x0000,
    0x14,	0xC000,		/*!*/
    0x20,	0x407A,
    0x21,	0x0000,
    0x22,	0x0000,
    0x23,	0x1010,
    0x24,	0x1010,
    0x30,	0x0000,
    0x31,	0x0000,
    0x32,	0x0000,
    0x33,	0x0000,
    0x34,	0x0000,
    0x35,	0x0000,
    0x36,	0x0000,
    0x37,	0x0000,
    0x38,	0x1010,
    0x39,	0x1010,
    0x3A,	0x0003,		/*!*/
    0x40,	0x0400,
    0x41,	0x1400,
    0x42,	0x2400,
    0x43,	0x3400,
    0x44,	0x00E4,
    0x50,	0x0000,
    0x51,	0x0000,
    0x52,	0xEFFF,
    0x60,	0x0060,
    0x61,	0x0000,
    0x64,	0x0000,
    0x65,	0x0220,
    0x66,	0x0000,		/*!*/
    0x67,	0x0000,		/*!*/
    0x68,	0x3000,
    0x69,	0x0011,
    0x6A,	0x0011,
    0x6B,	0x2525,		/*!*/
    0x6C,	0x2525,		/*!*/
    0x6D,	0x0000,		/*!*/
    0x1 ,	0x0000,		/*!*/
    0x2 , 0xC000,		/*!*/
};
static void set_cs()     //设置cs引脚，这个引脚决定nau85l04的i2c地址最低位
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}
void nau85l04_init(I2C_HandleTypeDef *hi2c)
{
    static uint8_t write[4];
    int i;
    set_cs();
    HAL_Delay(50);
    for(i=0;i<sizeof(data)/2;){
        write[0] = (data[i]>>8)&0xFF;
        write[1] = data[i]&0xFF;
        write[2] = (data[i+1]>>8)&0xFF;
        write[3] = data[i+1]&0xFF;
        if(HAL_OK == HAL_I2C_Master_Transmit(hi2c, 0x38, write, 4, 1000)) {
            i += 2;
        }
    }
}
void nau85l04_poweroff(I2C_HandleTypeDef *hi2c)
{
    static uint8_t write[4];
    int i;
    set_cs();
    HAL_Delay(50);
    for(i=0;i<sizeof(data_poweroff)/2;){
        write[0] = (data_poweroff[i]>>8)&0xFF;
        write[1] = data_poweroff[i]&0xFF;
        write[2] = (data_poweroff[i+1]>>8)&0xFF;
        write[3] = data_poweroff[i+1]&0xFF;
        if(HAL_OK == HAL_I2C_Master_Transmit(hi2c, 0x38, write, 4, 1000)) {
            i += 2;
        }
    }
}

