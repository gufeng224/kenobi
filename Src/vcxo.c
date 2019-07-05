#include "vcxo.h"
#include "stm32f0xx_hal.h"
#include "usbd_audio.h"
#include "dac.h"

//#define POS_CTRL
#define CALC_COUNT      100     //每传来CALC_COUNT次才真正进行一次计算和校准
static void vcxo_adjust_count(long input)
{
    /****************** 二级平衡，让buf_delay处在AUDIO_OUT_BUF_SIZE/2处 ***************/
#ifdef POS_CTRL
    //目标量
    static long target_buf_delay = (AUDIO_MIC_BUF_SIZE/2)*CALC_COUNT;      //目标平衡位置
    //
    long buf_delay_diff=0;
    #define kp_2 0.2
#endif
    /****************** 一级平衡 ***************/
    //目标量
    static long target_v_buf_delay = 0;
    //
    long buf_delay, v_buf_delay;
    static long buf_delay_last=0;
    static int flag_first=1;
    //控制量
#define kp 3     //vcxo_ppm_diff = kp*(v_buf_delay-target_v_buf_delay)
    long vcxo_ppm_diff=0;     //vcxo_ppm的增量，如果buf_delay在变大，则应当调小vcxo_ppm_diff
    static long vcxo_ppm=2048;      //输出量

    /****************** 二级平衡控制 ****************/
#ifdef POS_CTRL
    buf_delay_diff = (long)input - target_buf_delay;
    target_v_buf_delay = kp_2 * buf_delay_diff;
#endif
    /****************** 一级平衡控制 ****************/
    buf_delay = input;
    if(flag_first) {
        flag_first = 0;
        buf_delay_last = input;
    }
    v_buf_delay = buf_delay_last - buf_delay;
    buf_delay_last = buf_delay;
    
    vcxo_ppm_diff = kp * (v_buf_delay - target_v_buf_delay);
    vcxo_ppm += vcxo_ppm_diff;
#define VCXO_ADJ_MIN			0
#define VCXO_ADJ_MAX			4095
    if(vcxo_ppm > VCXO_ADJ_MAX) vcxo_ppm = VCXO_ADJ_MAX;
    else if(vcxo_ppm < VCXO_ADJ_MIN) vcxo_ppm = VCXO_ADJ_MIN;

    //控制器输出是vcxo_ppm，下面进行设置
    dac_set_value((uint16_t)vcxo_ppm);
}

void vcxo_adjust(int16_t input)
{
    static long input_sum=0;
    static int count = 0;
    count++;
    input_sum += input;
    if(count == CALC_COUNT) {
        count = 0;
        vcxo_adjust_count(input_sum);
        input_sum = 0;
    }
}
