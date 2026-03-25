#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void initPinyCore();

#ifdef __cplusplus
}

#include "Singleton.hpp"

class PinyCore : public Singleton<PinyCore> {
public:
    void bspInit();  // TODO:
    void coreInit(); // TODO:

    void init();

private:
    PinyCore() = default;
    friend class Singleton<PinyCore>;
};

#endif
