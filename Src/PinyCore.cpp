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

    SEGGER_SYSVIEW_Conf();

    extern canHandle HCAN1;
    Can::instance().init(&HCAN1, RX_FIFO0);
    extern canHandle HCAN2;
    Can::instance().init(&HCAN2, RX_FIFO1);
#ifdef HCAN3
    extern canHandle HCAN3;
    Can::instance().init(&HCAN3, RX_FIFO0);
#endif
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
