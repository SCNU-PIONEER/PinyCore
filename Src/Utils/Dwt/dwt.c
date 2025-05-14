#include "dwt.h"

void dwt_init()
{
    /* 使能DWT外设 */
    DEM_CR |= (uint32_t)DEM_CR_TRCENA;

    /* DWT CYCCNT寄存器计数清0 */
    DWT_CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */ DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

uint32_t dwt_read(void) { return ((uint32_t)DWT_CYCCNT); }

void dwt_delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;

    /* 在函数内部初始化时间戳寄存器， */
#if (CPU_TS_INIT_IN_DELAY_FUNCTION)
    /* 初始化时间戳并清零 */
    CPU_TS_TmrInit(5);
#endif

    ticks = us * (HAL_RCC_GetSysClockFreq() / 1000000); /* 需要的节拍数 */
    tcnt = 0;
    told = (uint32_t)dwt_read(); /* 刚进入时的计数器值 */

    while (1) {
        tnow = (uint32_t)dwt_read();
        if (tnow != told) {
            /* 32位计数器是递增计数器 */
            if (tnow > told) {
                tcnt += tnow - told;
            }
            /* 重新装载 */
            else {
                tcnt += UINT32_MAX - told + tnow;
            }

            told = tnow;

            /*时间超过/等于要延迟的时间,则退出 */
            if (tcnt >= ticks)
                break;
        }
    }
}

void dwt_delay_ms(uint32_t ms) { dwt_delay_us(ms * 1000); }

float dwt_get_dt(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) /
               ((float)(HAL_RCC_GetSysClockFreq()));
    *cnt_last = cnt_now;
    return dt;
}

float dwt_get_freq(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float freq = ((float)(HAL_RCC_GetSysClockFreq())) /
                 (uint32_t)(cnt_now - *cnt_last);
    *cnt_last = cnt_now;
    return freq;
}
