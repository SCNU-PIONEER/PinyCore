#include "Soc.hpp"
#include "cmsis_os2.h"
#include "stm_log.hpp"
#include "Bsp_can.hpp"
#include "PinyCore.hpp"
#include "AppManager.hpp"
#include "SEGGER_SYSVIEW.h"

#include "MotorManager.hpp"


// #include "../Lib/TinyMPC/src/tinympc/tiny_api.hpp"

static void logCallback(const char *_type, const char *_color,
                        const char *_format, ...)
{
    va_list args;
    va_start(args, _format);
    logProtoVaList(_type, _color, _format, args);
    va_end(args);
}

void PinyCore::bspInit()
{
    extern canHandle hcan1;
    Can::instance().init(&hcan1, RX_FIFO0);
    // Can::instance().init(&hcan1, RX_FIFO1);
}

void PinyCore::osInit()
{
    AppManager::instance()->createApp();
}

void PinyCore::coreInit()
{
    SEGGER_SYSVIEW_Conf();

    this->registerLogger(logCallback);

    PINYMOTOR::MotorManager::instance()->registerLogger(logCallback);
}

void PinyCore::init()
{
    bspInit();
    coreInit();
    osInit();

    this->log("INFO", "green", "PinyCore init done.\n");
}

void initPinyCore() { PinyCore::instance()->init(); }
