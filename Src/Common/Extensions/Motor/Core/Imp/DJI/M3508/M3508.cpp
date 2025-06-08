#include "M3508.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
M3508::M3508(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-M3508");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = (3591.f / 187.f);
    this->model_.rxBaseId = 0x200;
    
    if(_config.offsetId > 3)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;

    this->stats_ = DJIMotorStats_s(25000.f, // voltTxCodeSpan
                                   16384.f, // currTxCodeSpan
                                   8192.f,  // currRxCodeSpan

                                   10.f, // currRated
                                   3.f,  // torqRated

                                   25.2f, // voltMax
                                   2.5f,  // currMax
                                   4.5f,  // torqMax

                                   0.3f // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();

    this->log("INFO", "green",
                          "Motor %s: An instance of DJIMotor created", this->name_);
}

MotorTypeDef_e M3508::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ == ComType_e ::UART ||
        this->comType_ == ComType_e ::FDCAN) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M3508 %s: ComType is not supported", this->name_);
    }

    if (this->workMode_ != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M3508 %s: WorkMode is not supported", this->name_);
    }
    
    if (this->offsetId_ > 8) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M3508 %s: Max Offset ID is only 8!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "M3508 %s: Max TxFreq is only 1000!",
                    this->name_);
    }

    return rslt;
}
