#include "GM6020.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
GM6020::GM6020(const char _name[16], InitConfig_s _config)
        : DJIMotor<GM6020>(_name, _config)
{
    strcpy(this->model_.name, "DJI-GM6020");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x204;
    
    if(_config.offsetId > 3)
        this->model_.txBaseId = 0x2FE;
    else
        this->model_.txBaseId = 0x1FE;

    this->log("WARNING", "", "Please check the WorkMode of %s", this->name_);

    this->stats_ = DJIMotorStats_s(
        25000.f, // voltTxCodeSpan
        16384.f, // currTxCodeSpan
        8192.f,  // currRxCodeSpan
                                        
        1.62f, // currRated
        1.2f,  // torqRated
         
        25.2f,  // voltMax
        0.9f,  // currMax
        0.86f, // torqMax
                              
        0.741f // torqConstant
    );
    this->log("INFO", "green",
                          "Motor %s: An instance of DJIMotor created", this->name_);
}

MotorTypeDef_e GM6020::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ == ComType_e ::UART ||
        this->comType_ == ComType_e ::FDCAN) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "GM6020 %s: ComType is not supported", this->name_);
    }

    if (!(this->workMode_ == WorkMode_e::QUAD_CURR ||
          this->workMode_ == WorkMode_e::QUAD_VOLT)) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "GM6020 %s: WorkMode is not supported", this->name_);
    }
    
    if (this->offsetId_ > 7) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "GM6020 %s: Max Offset ID is only 7!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "GM6020 %s: Max TxFreq is only 1000!",
                    this->name_);
    }

    return rslt;
}

