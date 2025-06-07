#include "DM4340.hpp"

#include "DMMotor.hpp"
#include <cstring>

using namespace PINYMOTOR;
DM4340::DM4340(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, _config)
{
    strcpy(this->model_.name, "DM-DM4340");
    this->model_.measureMax = 16383;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x10;
    this->model_.txBaseId = 0x00;

    this->stats_ = DMMotorStats_s{
        12.5f, // PMax
        30.f,  // VMax
        10.f,  // TMax

        500.f, // MITKpMax
        5.f,   // MITKdMax

        10000.f, // currTxCodeSpan

        2.5f, // currRated
        9.f,  // torqRated

        8.f,  // currMax
        27.f, // torqMax

        0.9333f // torqConstant
    };

    this->registerMotor();
    this->registerRecvCallback();
    
    checkBaseConfig();
    
    this->log("INFO", "green",
                          "Motor %s: An instance of DM4310 created", this->name_);
    // TODO:
}

MotorTypeDef_e DM4340::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ == ComType_e ::UART) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "DM4340 %s: UART is not supported", this->name_);
    }

    if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "DM4340 %s: WorkMode is not supported", this->name_);
    }
    
    if (this->offsetId_ > 9) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "DM4340 %s: Max Offset ID is only 9!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "DM4340 %s: Max TxFreq is only 1000!",
                    this->name_);
    }

    return rslt;
}
