#include "QuadricycleController.hpp"

using namespace PINYMOTOR;

QuadricycleController::QuadricycleController(ChassisType_e _chassisType,
                                             CAP *_cap)
        : PowerController(_chassisType, _cap)
{
}

void QuadricycleController::cmdPowerCalc(const float *_motorSpeed,
                                         IMotor *_motor[4], const float *_cmd)
{
    float motorCmdRads[4] = { 0, 0, 0, 0 };
    float powerSum = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        motorCmdRads[i] = _motorSpeed[i] * _motor[i]->rr();
        cmdPower[i] = (M3508.KN * _cmd[i] * motorCmdRads[i] +
                       M3508.MLC * motorCmdRads[i] * motorCmdRads[i] +
                       M3508.ESR * _cmd[i] * _cmd[i] + M3508.LeakagePower);
        powerSum += cmdPower[i];
    }
    chassisRawPower = powerSum;
}

void QuadricycleController::relPowerCalc(IMotor *_motor[4])
{
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
}

void QuadricycleController::currentCalc(IMotor *_motor[4], const float *_cmd)
{
    float motorRelRads[4] = { 0, 0, 0, 0 };
    for (uint8_t i = 0; i < motorNum_; i++) {
        motorRelRads[i] = _motor[i]->data().spdRadps * _motor[i]->rr();

        float discriminant = std::max(
                (M3508.KN * M3508.KN * motorRelRads[i] * motorRelRads[i]) -
                        (4 * M3508.ESR *
                         (M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                          M3508.LeakagePower - setPower[i])),
                0.0f);
        float sign = (_cmd[i] == 0) ? 0.f : ((_cmd[i] > 0) ? 1.f : -1.f);
        setIq[i] = (sign >= 0) ? (std::clamp((-(M3508.KN * motorRelRads[i]) +
                                              sign * sqrtf(discriminant)) /
                                                     (2 * M3508.ESR),
                                             0.f, _cmd[i])) :
                                 std::clamp((-(M3508.KN * motorRelRads[i]) +
                                             sign * sqrtf(discriminant)) /
                                                    (2 * M3508.ESR),
                                            _cmd[i], 0.f);
    }
}

void QuadricycleController::rlsUpdate(IMotor *_motor[4])
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

    if (params[0][0] > 0 && params[1][0] > 0 && params[2][0] > 0) {
        M3508.KN = params[0][0];
        M3508.MLC = params[1][0];
        M3508.ESR = params[2][0];
    }
}

std::vector<float> QuadricycleController::powerCtrl(const float *_motorSpeed,
                                                    IMotor *_motor[4],
                                                    const float *_cmd,
                                                    RefereeMsg_s _msg)
{
    update(_msg);

    cmdPowerCalc(_motorSpeed, _motor, _cmd);
    if (chassisRawPower > maxPower) //限制最大输出功率
        powerRatio = maxPower / chassisRawPower;
    else
        powerRatio = 1.f;

    float powerSum = 0;
    static float lastSetPower = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        setPower[i] = cmdPower[i] * powerRatio;
        powerSum += setPower[i];
    }
    chassisSetPower = powerSum;

    currentCalc(_motor, _cmd);
    relPowerCalc(_motor);

#if POWERCTRL_USE_RLS
    rlsUpdate(_motor);
#endif
    static uint32_t taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) /
                        static_cast<float>(xTaskGetTickCount() - taskTick) *
                        1000.f;
    lastSetPower = chassisSetPower;
    taskTick = xTaskGetTickCount();
    if (cap_ != nullptr)
        cap_->chargeCmdPower =
                std::clamp(limitPower - (0.01f * setPowerDot), 30.f, 120.f);

    return setIq;
}
