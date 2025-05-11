#include "Soc.hpp"
#include "cmsis_os2.h"
#include "stm_log.hpp"
#include "Bsp_can.hpp"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"

#include "MotorManager.hpp"


// #include "../Lib/TinyMPC/src/tinympc/tiny_api.hpp"


void PinyCore::bspInit()
{
    extern canHandle hcan1;
    Can::instance().init(&hcan1, RX_FIFO0);
    // Can::instance().init(&hcan1, RX_FIFO1);
}

void PinyCore::osInit() { AppManager::instance()->createApp(); }

void PinyCore::coreInit() { SEGGER_SYSVIEW_Conf(); }

void PinyCore::init()
{
    bspInit();
    coreInit();
    osInit();

    log.info("Piny", "init.");
}

void initPinyCore() { PinyCore::instance()->init(); }
