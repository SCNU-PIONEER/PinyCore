#include "QuadricycleController.hpp"
#include <cmath>
#include <algorithm>

using namespace PINYMOTOR;

QuadricycleController::QuadricycleController(ChassisType_e chassisType)
        : PowerController(chassisType)
{
    energyPid_ = std::make_unique<positonalPid>(0.1f, 0, 0, 0.002f, 0, 0.f, 0);
    powerPid_ =
            std::make_unique<positonalPid>(300.f, 0, 0, 0.002f, 0, 400.f, 0);
}

void QuadricycleController::cmdPowerCalc(float *motorSpeed)
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().begin();
    float motorCmdRads[4] = { 0 };
    float powerSum = 0;
    for (int i = 0; i < motorNum_; i++) {
        motorCmdRads[i] = (motorSpeed[i] * it->second->RR()) / 60.f * 2 * M_PI;
        //TODO:elec的转换
        cmdPower[i] = (M3508.k0 * it->second->getCmdCurr() * motorCmdRads[i] +
                       M3508.MLC * motorCmdRads[i] * motorCmdRads[i] +
                       M3508.ESR * it->second->getCmdCurr() *
                               it->second->getCmdCurr() +
                       M3508.LeakagePower);
        powerSum += cmdPower[i];
        it++;
    }
    chassisRawPower = powerSum;
}

void QuadricycleController::relPowerCalc()
{
    MotorManager *motorManager = MotorManager::instance();
    CapData_s _capData = cap_.getCapData();
    auto it = motorManager->motors().begin();
    float motorRelRads[4] = { 0 };
    float powerSum = 0;
    for (int i = 0; i < motorNum_; i++) {
        Data_s motorData = it->second->data();
        motorRelRads[i] = motorData.spdRadps * it->second->RR();
        relPower[i] = (M3508.k0 * motorData.curr * motorRelRads[i] +
                       M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                       M3508.ESR * motorData.curr * motorData.curr +
                       M3508.LeakagePower);
        powerSum += relPower[i];
        it++;
    }
    chassisRealPower = powerSum;
    capFeedbackPower = _capData.inputVoltage * _capData.outputCurrent;
}

void QuadricycleController::currentCalc()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().begin();
    float motorRelRads[4] = { 0 };
    for (int i = 0; i < motorNum_; i++) {
        Data_s motorData = it->second->data();
        motorRelRads[i] = motorData.spdRadps * it->second->RR();

        float Discriminant =
                M3508.k0 * M3508.k0 * motorRelRads[i] * motorRelRads[i] -
                4 * M3508.ESR *
                        (M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                         M3508.LeakagePower - setPower[i]);
        if (Discriminant < 0) {
            Discriminant = 0;
        }

        float sign = (it->second->getCmdCurr() > 0) ? 1 : -1;
        setIq[i] = (-M3508.k0 * motorRelRads[i] + sign * sqrt(Discriminant)) /
                   (2 * M3508.ESR);
        it++;
    }
}

std::vector<float> QuadricycleController::powerCtrl(float *motorSpeed)
{
    //TODO: refereeData
    refereeDataUpdate();
    CapData_s _capData = cap_.getCapData();

    float bufferDP = std::clamp(energyPid_->calc(expPowerBuffer, powerBuffer),
                                -1.f, 1.f); //动态规划缓冲能量

    float voltageRange = powf(VCapMAX, 2.f) - powf(VCapMIN, 2.f);
    capRealRatio = (powf(_capData.capVoltage, 2.f) - powf(VCapMIN, 2.f)) /
                   voltageRange;

    float ratioErr = capRealRatio - capCmdRatio;
    float capExpRatio = std::clamp(capCmdRatio + ratioErr * bufferDP, 0.f, 1.f);

    offsetPower = powerPid_->calc(capExpRatio, capRealRatio);
    maxPower = std::clamp(limitPower - offsetPower, limitPower,
                          _capData.capVoltage * CAPCurrMax + limitPower);

    cmdPowerCalc(motorSpeed);
    if (chassisRawPower > maxPower) {
#if USELinearityRatio == 1
        powerRatio = maxPower / chassisRawPower;
#elif USEConfidenceLevel == 1
        //TODO:fix
#else
        powerRatio = 1.f;
#endif
    }

    float chassisSetPower = 0;
    static float lastSetPower = 0;
    for (int i = 0; i < motorNum_; i++) {
        setPower[i] = cmdPower[i] * powerRatio;
        chassisSetPower += setPower[i];
    }
    currentCalc();
    relPowerCalc();

    static uint32_t _taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) /
                        (xTaskGetTickCount() - _taskTick) * 1000.f;
    lastSetPower = chassisSetPower;
    _taskTick = xTaskGetTickCount();
    chargeCmdPower = std::clamp(limitPower - 0.01f * setPowerDot, 30.f, 120.f);
    cap_.capDataSend(chargeCmdPower, capEnable_, capFeedforwardEnable_,
                     static_cast<uint16_t>(chassisSetPower));

    return setIq;
}
