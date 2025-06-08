#include "Soc.hpp"
#include "cmsis_os2.h"
#include "stm_log.hpp"
#include "Bsp_can.hpp"
#include "dwt.h"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"

#include "bmi088.hpp"


// #include "../Lib/TinyMPC/src/tinympc/tiny_api.hpp"


void PinyCore::bspInit()
{
    dwt_init();

    extern canHandle hcan1;
    Can::instance().init(&hcan1, RX_FIFO0);
    // Can::instance().init(&hcan1, RX_FIFO1);
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
