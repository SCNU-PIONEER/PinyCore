#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void initPinyCore();

#ifdef __cplusplus
}

#include "StmLog.hpp"


class PinyCore {
public:
    static PinyCore *instance()
    {
        static PinyCore instance;
        return &instance;
    }

    PinyCore(const PinyCore &) = delete;
    PinyCore &operator=(const PinyCore &) = delete;

    LOG::Logger &log = LOG::Logger::instance();

    void bspInit();  // TODO:
    void coreInit(); // TODO:

    void init();

private:
    PinyCore() = default;
};

#endif
