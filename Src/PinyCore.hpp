#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

void initPinyCore();

#ifdef __cplusplus
}

#include "stm_log.hpp"


class PinyCore {
public:
    inline static PinyCore *instance()
    {
        static PinyCore instance_;
        return &instance_;
    }

    PinyCore(const PinyCore &) = delete;
    PinyCore &operator=(const PinyCore &) = delete;

    LOG::Logger &log = LOG::Logger::instance();

    void bspInit();  // TODO:
    void osInit();   // TODO:
    void coreInit(); // TODO:

    void init();

private:
    PinyCore() = default;
};

#endif
