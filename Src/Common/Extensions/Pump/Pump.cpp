#include "Pump.hpp"
#include "StmLog.hpp"

// 实现初始化函数
void PUMP::Controller::init()
{
    apply(PUMP::ALL_OFF);
    LOG::Logger::instance().info(LOCATION, "Pump", "initialized");
}
