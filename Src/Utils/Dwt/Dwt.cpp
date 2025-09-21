#include "Dwt.hpp"

void dwtInit() { Dwt::instance(); }

uint32_t dwtRead() { return Dwt::instance().read(); }

Dwt::Dwt()
{
    /* 使能DWT外设 */
    DEM_CR |= (uint32_t)DEM_CR_TRCENA;

    /* DWT CYCCNT寄存器计数清0 */
    DWT_CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */ DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

uint32_t Dwt::read() { return ((uint32_t)DWT_CYCCNT); }

void Dwt::delayUs(uint32_t _us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;

    /* 在函数内部初始化时间戳寄存器， */
#if (CPU_TS_INIT_IN_DELAY_FUNCTION)
    /* 初始化时间戳并清零 */
    CPU_TS_TmrInit(5);
#endif

    ticks = _us * (HAL_RCC_GetSysClockFreq() / 1000000); /* 需要的节拍数 */
    tcnt = 0;
    told = read(); /* 刚进入时的计数器值 */

    while (true) {
        tnow = read();
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

void Dwt::delayMs(uint32_t _ms) { delayUs(_ms * 1000); }

float Dwt::getDt(uint32_t *_cnt_last)
{
    volatile uint32_t cntNow = DWT->CYCCNT;
    float dt = ((uint32_t)(cntNow - *_cnt_last)) /
               ((float)(HAL_RCC_GetSysClockFreq()));
    *_cnt_last = cntNow;
    return dt;
}

float Dwt::getFreq(uint32_t *_cnt_last)
{
    volatile uint32_t cntNow = DWT->CYCCNT;
    float freq = ((float)(HAL_RCC_GetSysClockFreq())) /
                 (uint32_t)(cntNow - *_cnt_last);
    *_cnt_last = cntNow;
    return freq;
}
