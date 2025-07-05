#include "Soc.hpp"
#include "cmsis_os2.h"
#include "StmLog.hpp"
#include "Bsp_can.hpp"
#include "Dwt.h"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"
#include "sdkconfig.h"

void PinyCore::bspInit()
{
    dwt_init();

    extern canHandle HCAN1;
    Can::instance().init(&HCAN1, RX_FIFO0);
    // Can::instance().init(&HCAN1, RX_FIFO1);
}

void PinyCore::coreInit()
{
    osKernelInitialize();

    SEGGER_SYSVIEW_Conf();

    AppManager::instance()->initApp();

    osKernelStart();
}

void PinyCore::init()
{
    /* Welcome to Piny */
    bspInit();
    coreInit();

    log.info(LOCATION, "Piny", "init");
}

void initPinyCore() { PinyCore::instance()->init(); }
