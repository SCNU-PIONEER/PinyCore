#include "GM3510.hpp"

#include "DJI_ODMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;

GM3510::GM3510(const char _name[16], InitConfig_s _config)
        : DJI_ODMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-GM3510");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x204;

    this->model_.txBaseId = 0x1FF;

    this->stats_ = DJI_ODMotorStats_s(29000.f, // voltTxCodeSpan
                                      8192.f,  // torqRxCodeSpan TODO:

                                      1.3f,  // currRated
                                      0.14f, // torqRated

                                      24.f, // voltMax
                                      0.6f, // currMax
                                      0.2f, // torqMax

                                      0.16f // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();
    
    this->log(
            "INFO", "green",
            "Motor %s: An instance of DJI_ODMotor created, rxBaseId = 0x%03X, txBaseId = 0x%03X",
            this->name_, this->model_.rxBaseId, this->model_.txBaseId);
}

MotorTypeDef_e GM3510::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e::FDCAN &&
        this->comType_ != ComType_e::CAN) {
        rslt |= 1;
        this->log("ERROR", "red", "GM3510 %s: only support FDCAN or CAN comtype",
                  this->name_);
    }

    if (this->workMode_ != WorkMode_e::TRIP_VOLT) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "GM3510 %s: WorkMode is not supported", this->name_);
    }
    
    if (this->offsetId_ > 3) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "GM3510 %s: Max Offset ID is only 3!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "GM3510 %s: Max TxFreq is only 1000!",
                    this->name_);
    }

    return rslt;
}
