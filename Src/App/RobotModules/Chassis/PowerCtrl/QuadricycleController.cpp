#include "QuadricycleController.hpp"
#include "SuperCap.hpp"
#include <cmath>
#include <algorithm>

using namespace PINYMOTOR;

QuadricycleController::QuadricycleController(ChassisType_e _chassisType)
        : PowerController(_chassisType)
{
}

void QuadricycleController::cmdPowerCalc(const float *_motorSpeed,
                                         PINYMOTOR::IMotor **_motor,
                                         const float *_cmd)
{
    float motorCmdRads[4] = { 0, 0, 0, 0 };
    float powerSum = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        motorCmdRads[i] = (_motorSpeed[i] * _motor[i]->rr()) / 60.f * 2 * PI;
        //TODO:elec的转换
        cmdPower[i] = (M3508.KN * _cmd[i] * motorCmdRads[i] +
                       M3508.MLC * motorCmdRads[i] * motorCmdRads[i] +
                       M3508.ESR * _cmd[i] * _cmd[i] + M3508.LeakagePower);
        powerSum += cmdPower[i];
    }
    chassisRawPower = powerSum;
}

void QuadricycleController::relPowerCalc(PINYMOTOR::IMotor **_motor)
{
    CapData_s capData = cap_.getCapData();
    float motorRelRads[4] = { 0, 0, 0, 0 };
    float powerSum = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = _motor[i]->data();
        motorRelRads[i] = motorData.spdRadps * _motor[i]->rr();
        relPower[i] = (M3508.KN * motorData.curr * motorRelRads[i] +
                       M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                       M3508.ESR * motorData.curr * motorData.curr +
                       M3508.LeakagePower);
        powerSum += relPower[i];
    }
    chassisRealPower = powerSum;
    capFeedbackPower = capData.inputVoltage * capData.outputCurrent;
}

void QuadricycleController::currentCalc(PINYMOTOR::IMotor **_motor)
{
    float motorRelRads[4] = { 0, 0, 0, 0 };
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = _motor[i]->data();
        motorRelRads[i] = motorData.spdRadps * _motor[i]->rr();

        float discriminant =
                (M3508.KN * M3508.KN * motorRelRads[i] * motorRelRads[i]) -
                (4 * M3508.ESR *
                 (M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                  M3508.LeakagePower - setPower[i]));

        discriminant = std::max(discriminant, 0.0f);

        float sign = (_motor[i]->getCmdCurr() > 0) ? 1 : -1;
        setIq[i] = (-M3508.KN * motorRelRads[i] + sign * sqrt(discriminant)) /
                   (2 * M3508.ESR);
    }
}

void QuadricycleController::rlsUpdate(PINYMOTOR::IMotor **_motor)
{
    float motorRelRads[4] = { 0, 0, 0, 0 };
    float vectorValue[3] = { 0, 0, 0 };
    for (uint8_t i = 0; i < motorNum_; i++) {
        Data_s motorData = _motor[i]->data();
        motorRelRads[i] = motorData.spdRadps * _motor[i]->rr();
        vectorValue[0] += motorData.curr * motorRelRads[i];
        vectorValue[1] += motorRelRads[i] * motorRelRads[i];
        vectorValue[2] += motorData.curr * motorData.curr;
    }

    Matrix<3, 1> inputVector(vectorValue);
    wheelRLS_.update(inputVector,
                     capFeedbackPower - (M3508.LeakagePower * 4.f));

    Matrix<3, 1> params = wheelRLS_.getEstVector();
    M3508.KN = params[0][0];
    M3508.MLC = params[0][1];
    M3508.ESR = params[0][2];
}


std::vector<float> QuadricycleController::powerCtrl(const float *_motorSpeed,
                                                    PINYMOTOR::IMotor **_motor,
                                                    const float *_cmd)
{
    //TODO: refereeData
    refereeDataUpdate();

    CapData_s capData = cap_.getCapData();

    dynamicPower(capData.capVoltage);

    cmdPowerCalc(_motorSpeed, _motor, _cmd);
    if (chassisRawPower > maxPower) //限制最大输出功率
        powerRatio = maxPower / chassisRawPower;
    else
        powerRatio = 1.f;

    float chassisSetPower = 0;
    static float lastSetPower = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        setPower[i] = cmdPower[i] * powerRatio;
        chassisSetPower += setPower[i];
    }

    currentCalc(_motor);
    relPowerCalc(_motor);

    if constexpr (APP_USE_RLS) {
        rlsUpdate(_motor);
    }

    static uint32_t taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) /
                        static_cast<float>(xTaskGetTickCount() - taskTick) *
                        1000.f;
    lastSetPower = chassisSetPower;
    taskTick = xTaskGetTickCount();
    chargeCmdPower =
            std::clamp(limitPower - (0.01f * setPowerDot), 30.f, 120.f);
    cap_.capDataSend(chargeCmdPower, capEnable_, capFeedforwardEnable_,
                     static_cast<uint16_t>(chassisSetPower));

    return setIq;
}
