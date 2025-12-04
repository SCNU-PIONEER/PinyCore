/* Includes ------------------------------------------------------------------*/
// #include "memory.hpp"
#include "FreeRTOS.h"

extern "C" {

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide
 * an implementation of vApplicationGetIdleTaskMemory() to provide the memory
 * that is used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **_ppxIdleTaskTCBBuffer,
                                   StackType_t **_ppxIdleTaskStackBuffer,
                                   uint32_t *_pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
   * function then they must be declared static - otherwise they will be
   * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *_ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *_ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *_pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **_ppxTimerTaskTCBBuffer,
                                    StackType_t **_ppxTimerTaskStackBuffer,
                                    uint32_t *_pulTimerTaskStackSize)
{
    /* If the buffers to be provided to the Timer task are declared inside this
   * function then they must be declared static - otherwise they will be
   * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *_ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *_ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *_pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
}
