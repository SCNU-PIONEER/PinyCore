#include "cmsis_os2.h"
#include "StmLog.hpp"
#include "Bsp_can.hpp"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"


void PinyCore::bspInit()
{
    SEGGER_SYSVIEW_Conf();

    Can::instance().init();
}

void PinyCore::coreInit()
{
    osKernelInitialize();

    AppManager::instance()->initApp();

    LOG::info("Piny", "kernal start");

    osKernelStart();
}

void PinyCore::init()
{
    /* Welcome to Piny */
    bspInit();
    coreInit();
}

void initPinyCore() { PinyCore::instance()->init(); }
