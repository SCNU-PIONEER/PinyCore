#include "StmLog.hpp"
#include "Bsp.hpp"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"
#include "FreeRTOS.h"
#include "task.h"

#if SOC_USE_CUSTOM
#include "Soc.hpp"
#endif

void PinyCore::bspInit()
{
#if SOC_USE_CUSTOM
    SOC_CUSTOM_INIT();
#endif

    // sysview init must be before segger RTT init (in log)
    LOG::Logger::instance().init();
    SEGGER_SYSVIEW_Conf();

#if BSP_USE_CAN
    Can::instance().init();
#endif
}

void PinyCore::coreInit()
{
    AppManager::instance().initApp();

    LOG::info("Piny", "kernal start");

    vTaskStartScheduler();
}

void PinyCore::init()
{
    /* Welcome to Piny */
    bspInit();
    coreInit();
}

void initPinyCore() { PinyCore::instance().init(); }
