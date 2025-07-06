#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"

#include "DM4310.hpp"

#include "GM3510.hpp"

#include "GM6020.hpp"

#include "RcMsg.hpp"

class TestModule {
public:
    inline static TestModule *instance()
    {
        static TestModule instance_;
        return &instance_;
    }

    TestModule(const TestModule &) = delete;

    void init();

    void update();

    void task();

    RC::RcRawMsg_t rcMsg;

    StateFactory stateFactory_;

    LOG::Logger &log = LOG::Logger::instance();

    /*MOTOR*/
    PINYMOTOR::DM4310 *testDM4310Motor;
    PINYMOTOR::GM3510 *testGM3510Motor;
    PINYMOTOR::GM6020 *testGM6020Motor;

private:
    TestModule() = default;
};
