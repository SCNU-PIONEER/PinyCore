#include "StmLog.hpp"
#include "Bsp.hpp"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"
#include "FreeRTOS.h"
#include "task.h"

void PinyCore::bspInit()
{
    SEGGER_SYSVIEW_Conf();

    Can::instance().init();
}

void PinyCore::coreInit()
{
    AppManager::instance()->initApp();

    LOG::info("Piny", "kernal start");

    vTaskStartScheduler();
}

void PinyCore::init()
{
    /* Welcome to Piny */
    bspInit();
    coreInit();
}

void initPinyCore() { PinyCore::instance()->init(); }
