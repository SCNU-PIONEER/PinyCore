#include "M2006.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
M2006::M2006(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-M2006");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 36.f;
    this->model_.rxBaseId = 0x200;
    
    if(_config.offsetId > 3)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;

    this->stats_ = DJIMotorStats_s(
            25000.f, // voltTxCodeSpan
            16384.f, // currTxCodeSpan
            8192.f,  // currRxCodeSpan

            3.f, // currRated
            1.f, // torqRated

            25.2f, //voltmax
            //搭配c610无堵转电流和堵转扭矩数据，所以还是用额定数据
            3.f, // currMax
            1.f, // torqMax

            0.18f // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();

    checkBaseConfig();

    this->log("INFO", "green",
                          "Motor %s: An instance of DJIMotor created", this->name_);
}

MotorTypeDef_e M2006::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ == ComType_e ::UART ||
        this->comType_ == ComType_e ::FDCAN) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M2006 %s: ComType is not supported", this->name_);
    }

    if (this->workMode_ != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M2006 %s: WorkMode is not supported", this->name_);
    }
    
    if (this->offsetId_ > 8) {
        rslt |= 1;
        this->log("ERROR", "red",
                          "M2006 %s: Max Offset ID is only 8!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "M2006 %s: Max TxFreq is only 1000!",
                    this->name_);
    }

    return rslt;
}
