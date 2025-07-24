#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"
#include <memory>

#include "DM4310.hpp"

#include "GM3510.hpp"

#include "GM6020.hpp"

#include "M3508.hpp"

#include "RcMsg.hpp"

namespace TEST {
enum class FSMState_e : uint8_t {
    MOTOR_VEL_CTRL = 1,
    MOTOR_POS_CTRL = 2,
    MOTOR_STOP = 3,
};
}

class TestModule {
public:
    static TestModule *instance();

    TestModule(const TestModule &) = delete;

    void init();

    void update();

    void task();

    RC::RcRawMsg_t rcMsg;

    StateFactory stateFactory_;

    LOG::Logger &log = LOG::Logger::instance();

    /*MOTOR*/
    std::unique_ptr<PINYMOTOR::DMMOTOR::DM4310> testDM4310Motor;
    std::unique_ptr<PINYMOTOR::DJI_ODMOTOR::GM3510> testGM3510Motor;
    std::unique_ptr<PINYMOTOR::DJIMOTOR::GM6020> testGM6020Motor;
    std::unique_ptr<PINYMOTOR::DJIMOTOR::M3508> testM3508Motor;

private:
    TestModule() = default;
};
