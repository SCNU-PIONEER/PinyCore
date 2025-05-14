#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "main.h"

#define DWT_CR           *(__IO uint32_t *)0xE0001000
#define DWT_CYCCNT       *(__IO uint32_t *)0xE0001004
#define DEM_CR           *(__IO uint32_t *)0xE000EDFC

#define DEM_CR_TRCENA    (1 << 24)
#define DWT_CR_CYCCNTENA (1 << 0)

/**
* @brief  初始化时间戳
* @param  无
* @retval 无
* @note   使用延时函数前，必须调用本函数
*/
void dwt_init();

/**
  * @brief  读取当前时间戳
  * @param  无
  * @retval 当前时间戳，即DWT_CYCCNT寄存器的值
  */
uint32_t dwt_read(void);

/**
* @brief  采用CPU的内部计数实现精确延时，32位计数器
* @param  us : 延迟长度，单位1 us
* @retval 无
* @note   使用本函数前必须先调用CPU_TS_TmrInit函数使能计数器，
            或使能宏CPU_TS_INIT_IN_DELAY_FUNCTION
            最大延时值为60秒，即60s=2的32次方/72000000
*/
void dwt_delay_us(uint32_t us);

/**
* @brief  精确延时ms
* @param  ms : 延迟长度，单位1 ms
* @retval 无
*/
void dwt_delay_ms(uint32_t ms);

/**
 * @brief 读取相差时间戳
 *
 * @param cnt_last
 * @return dt
 */
float dwt_get_dt(uint32_t *cnt_last);

/**
 * @brief 读取相差频率
 *
 * @param cnt_last
 * @return dt
 */
float dwt_get_freq(uint32_t *cnt_last);

#ifdef __cplusplus
}
#endif
