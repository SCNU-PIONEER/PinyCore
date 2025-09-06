#include "Daemons.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "StmLog.hpp"

#include "Rc.hpp"

Daemons::Daemons()
{
    schedule([]() {
        if (!RC::Rc::instance().isOnline()) {
            LOG::error("Daemons", "RC Online");
        }
    });
}

Daemons &Daemons::instance()
{
    static Daemons instance;
    return instance;
}

void Daemons::schedule(std::function<void()> _func)
{
    cb.push_back(std::move(_func));
}

void Daemons::update()
{
    static uint32_t updateCnt = 0;
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();

        for (auto &func : cb) {
            func();
        }
    }
}
