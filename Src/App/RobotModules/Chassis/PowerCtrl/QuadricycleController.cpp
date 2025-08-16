#include "QuadricycleController.hpp"
#include <cmath>
#include <algorithm>

using namespace PINYMOTOR;

QuadricycleController::QuadricycleController(ChassisType_e _chassisType)
        : PowerController(_chassisType)
{
    energyPid_ = std::make_unique<PositonalPid>(0.1f, 0, 0, 0.002f, 0, 0.f, 0);
    powerPid_ =
            std::make_unique<PositonalPid>(300.f, 0, 0, 0.002f, 0, 400.f, 0);
}

void QuadricycleController::cmdPowerCalc(float *_motorSpeed)
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().begin();
    float motorCmdRads[4] = { 0 };
    float powerSum = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        motorCmdRads[i] = (_motorSpeed[i] * it->second->rr()) / 60.f * 2 * PI;
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
    CapData_s capData = cap_.getCapData();
    auto it = motorManager->motors().begin();
    float motorRelRads[4] = { 0 };
    float powerSum = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = it->second->data();
        motorRelRads[i] = motorData.spdRadps * it->second->rr();

        relPower[i] = (M3508.k0 * motorData.curr * motorRelRads[i] +
                       M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                       M3508.ESR * motorData.curr * motorData.curr +
                       M3508.LeakagePower);
        powerSum += relPower[i];
        it++;
    }
    chassisRealPower = powerSum;
    capFeedbackPower = capData.inputVoltage * capData.outputCurrent;
}

void QuadricycleController::currentCalc()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().begin();
    float motorRelRads[4] = { 0 };
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = it->second->data();
        motorRelRads[i] = motorData.spdRadps * it->second->rr();

        float discriminant =
                (M3508.k0 * M3508.k0 * motorRelRads[i] * motorRelRads[i]) -
                (4 * M3508.ESR *
                 (M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                  M3508.LeakagePower - setPower[i]));

        discriminant = std::max(discriminant, 0.0f);

        float sign = (it->second->getCmdCurr() > 0) ? 1 : -1;
        setIq[i] = (-M3508.k0 * motorRelRads[i] + sign * sqrt(discriminant)) /
                   (2 * M3508.ESR);
        it++;
    }
}

void QuadricycleController::rlsUpdate()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().begin();
    float motorRelRads[4] = { 0 };
    float vectorValue[3] = { 0 };
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = it->second->data();
        motorRelRads[i] = motorData.spdRadps * it->second->rr();
        vectorValue[0] += motorData.curr * motorRelRads[i];
        vectorValue[1] += motorRelRads[i] * motorRelRads[i];
        vectorValue[2] += motorData.curr * motorData.curr;
        it++;
    }

    Matrix<3, 1> inputVector(vectorValue);
    wheelRLS_.update(inputVector,
                     capFeedbackPower - (M3508.LeakagePower * 4.f));

    Matrix<3, 1> params = wheelRLS_.getEstVector();
    M3508.k0 = params[0][0];
    M3508.MLC = params[1][0];
    M3508.ESR = params[2][0];
}


std::vector<float> QuadricycleController::powerCtrl(float *_motorSpeed)
{
    //TODO: refereeData
    refereeDataUpdate();
    CapData_s capData = cap_.getCapData();

    float bufferDP = std::clamp(energyPid_->calc(expPowerBuffer, powerBuffer),
                                -1.f, 1.f); //动态规划缓冲能量

    float voltageRange = powf(VCapMAX, 2.f) - powf(VCapMIN, 2.f);
    capRealRatio =
            (powf(capData.capVoltage, 2.f) - powf(VCapMIN, 2.f)) / voltageRange;

    float ratioErr = capRealRatio - capCmdRatio;
    float capExpRatio =
            std::clamp(capCmdRatio + (ratioErr * bufferDP), 0.f, 1.f);

    offsetPower = powerPid_->calc(capExpRatio, capRealRatio);
    maxPower = std::clamp(limitPower - offsetPower, limitPower,
                          (capData.capVoltage * CAPCurrMax) + limitPower);

    cmdPowerCalc(_motorSpeed);
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
    for (uint8_t i = 0; i < motorNum_; i++) {
        setPower[i] = cmdPower[i] * powerRatio;
        chassisSetPower += setPower[i];
    }

    currentCalc();
    relPowerCalc();
    rlsUpdate();

    static uint32_t taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) /
                        (xTaskGetTickCount() - taskTick) * 1000.f;
    lastSetPower = chassisSetPower;
    taskTick = xTaskGetTickCount();
    chargeCmdPower =
            std::clamp(limitPower - (0.01f * setPowerDot), 30.f, 120.f);
    cap_.capDataSend(chargeCmdPower, capEnable_, capFeedforwardEnable_,
                     static_cast<uint16_t>(chassisSetPower));

    return setIq;
}
