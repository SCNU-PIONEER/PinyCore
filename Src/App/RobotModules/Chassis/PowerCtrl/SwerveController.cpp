#include "SwerveController.hpp"
#include "Projdefs.hpp"
#include <cmath>
#include <algorithm>

using namespace PINYMOTOR;

SwerveController::SwerveController(ChassisType_e _chassisType)
        : PowerController(_chassisType)
{
}

void SwerveController::cmdPowerCalc(const float *_motorSpeed,
                                    PINYMOTOR::IMotor **_motor,
                                    const float *_cmd)
{
    float motorCmdRads[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float steerPowerSum = 0;
    float wheelPowerSum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        motorCmdRads[i] = (_motorSpeed[i] * _motor[i]->rr()) / 60.f * 2 * PI;
        //TODO:elec的转换
        cmdPower[i] = (M3508.KN * _cmd[i] * motorCmdRads[i] +
                       M3508.MLC * motorCmdRads[i] * motorCmdRads[i] +
                       M3508.ESR * _cmd[i] * _cmd[i] + M3508.LeakagePower);
        cmdPower[i + 4] =
                (GM6020.KN * _cmd[i + 4] * _motor[i]->data().spdRadps +
                 GM6020.MLC * _motor[i]->data().spdRadps *
                         _motor[i]->data().spdRadps +
                 GM6020.ESR * _cmd[i + 4] * _cmd[i + 4] + GM6020.LeakagePower);
        wheelPowerSum += cmdPower[i];
        steerPowerSum += cmdPower[i + 4];
    }
    wheelCmdPower_ = wheelPowerSum;
    steerCmdPower_ = steerPowerSum;
    chassisRawPower = wheelPowerSum + steerPowerSum;
}

void SwerveController::relPowerCalc(PINYMOTOR::IMotor **_motor)
{
    CapData_s capData = cap_.getCapData();
    float motorRelRads[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float steerPowerSum = 0;
    float wheelPowerSum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        Data_s motorData = _motor[i]->data();
        motorRelRads[i] = motorData.spdRadps * _motor[i]->rr();
        relPower[i] = (M3508.KN * motorData.curr * motorRelRads[i] +
                       M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                       M3508.ESR * motorData.curr * motorData.curr +
                       M3508.LeakagePower);
        motorData = _motor[i + 4]->data();
        motorRelRads[i + 4] = motorData.spdRadps * _motor[i + 4]->rr();
        relPower[i + 4] =
                (GM6020.KN * motorData.curr * motorRelRads[i + 4] +
                 GM6020.MLC * motorRelRads[i + 4] * motorRelRads[i + 4] +
                 GM6020.ESR * motorData.curr * motorData.curr +
                 GM6020.LeakagePower);
        wheelPowerSum += relPower[i];
        steerPowerSum += relPower[i + 4];
    }
    steerRelPower_ = steerPowerSum;
    wheelRelPower_ = wheelPowerSum;
    chassisRealPower = wheelPowerSum + steerPowerSum;
    capFeedbackPower = capData.inputVoltage * capData.outputCurrent;
}

void SwerveController::currentCalc(PINYMOTOR::IMotor **_motor)
{
    float motorRelRads[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (uint8_t i = 0; i < 4; i++) {
        Data_s m3508MotorData = _motor[i]->data();
        Data_s gm6020MotorData = _motor[i + 4]->data();
        motorRelRads[i] = m3508MotorData.spdRadps * _motor[i]->rr();
        motorRelRads[i + 4] = gm6020MotorData.spdRadps * _motor[i + 4]->rr();

        float m3508Discriminant =
                (M3508.KN * M3508.KN * motorRelRads[i] * motorRelRads[i]) -
                (4 * M3508.ESR *
                 (M3508.MLC * motorRelRads[i] * motorRelRads[i] +
                  M3508.LeakagePower - setPower[i]));
        float gm6020Discriminant =
                (GM6020.KN * GM6020.KN * motorRelRads[i + 4] *
                 motorRelRads[i + 4]) -
                (4 * GM6020.ESR *
                 (GM6020.MLC * motorRelRads[i + 4] * motorRelRads[i + 4] +
                  GM6020.LeakagePower - setPower[i + 4]));

        m3508Discriminant = std::max(m3508Discriminant, 0.0f);
        gm6020Discriminant = std::max(gm6020Discriminant, 0.0f);

        float sign = (_motor[i]->getCmdCurr() > 0) ? 1 : -1;
        setIq[i] =
                (-M3508.KN * motorRelRads[i] + sign * sqrt(m3508Discriminant)) /
                (2 * M3508.ESR);
        sign = (_motor[i + 4]->getCmdCurr() > 0) ? 1 : -1;
        setIq[i + 4] = (-GM6020.KN * motorRelRads[i + 4] +
                        sign * sqrt(gm6020Discriminant)) /
                       (2 * GM6020.ESR);
    }
}

void SwerveController::rlsUpdate(PINYMOTOR::IMotor **_motor)
{
    float motorRelRads[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float m3508Vector[3] = { 0, 0, 0 };
    float gm6020Vector[3] = { 0, 0, 0 };
    for (uint8_t i = 0; i < 4; i++) {
        Data_s motorData = _motor[i]->data();
        motorRelRads[i] = motorData.spdRadps * _motor[i]->rr();
        m3508Vector[0] += motorData.curr * motorRelRads[i];
        m3508Vector[1] += motorRelRads[i] * motorRelRads[i];
        m3508Vector[2] += motorData.curr * motorData.curr;
        motorData = _motor[i + 4]->data();
        motorRelRads[i + 4] = motorData.spdRadps * _motor[i + 4]->rr();
        gm6020Vector[0] += motorData.curr * motorRelRads[i + 4];
        gm6020Vector[1] += motorRelRads[i + 4] * motorRelRads[i + 4];
        gm6020Vector[2] += motorData.curr * motorData.curr;
    }

    Matrix<3, 1> m3508InputVector(m3508Vector);
    wheelRLS_.update(m3508InputVector,
                     capFeedbackPower - (M3508.LeakagePower * 4.f));
    Matrix<3, 1> gm6020InputVector2(gm6020Vector);
    steerRLS_.update(gm6020InputVector2,
                     capFeedbackPower - (GM6020.LeakagePower * 4.f));

    Matrix<3, 1> params = wheelRLS_.getEstVector();
    M3508.KN = params[0][0];
    M3508.MLC = params[1][0];
    M3508.ESR = params[2][0];
}


std::vector<float> SwerveController::powerCtrl(const float *_motorSpeed,
                                               PINYMOTOR::IMotor **_motor,
                                               const float *_cmd)
{
    //TODO: refereeData
    refereeDataUpdate();

    CapData_s capData = cap_.getCapData();

    dynamicPower(capData.capVoltage);
    cmdPowerCalc(_motorSpeed, _motor, _cmd);

    float steerMaxPower = maxPower * 0.8f;
    float wheelMaxPower = std::max(maxPower * 0.2f, maxPower - steerCmdPower_);

    wheelPowerRatio_ = (wheelCmdPower_ > wheelMaxPower) ?
                               (wheelMaxPower / wheelCmdPower_) :
                               1.f;
    steerPowerRatio_ = (steerCmdPower_ > steerMaxPower) ?
                               (steerMaxPower / steerCmdPower_) :
                               1.f;
    float chassisSetPower = 0;
    static float lastSetPower = 0;
    for (uint8_t i = 0; i < 4; i++) {
        setPower[i] = cmdPower[i] * wheelPowerRatio_;
        setPower[i + 4] = cmdPower[i + 4] * steerPowerRatio_;
        chassisSetPower += setPower[i];
        chassisSetPower += setPower[i + 4];
    }

    currentCalc(_motor);
    relPowerCalc(_motor);

    if constexpr (APP_USE_UI) {
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
