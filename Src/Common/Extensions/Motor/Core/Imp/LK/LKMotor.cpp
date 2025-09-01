#include "LKMotor.hpp"
#include "MotorCommonMacros.hpp"

using namespace PINYMOTOR;
using namespace LKMOTOR;

// 构造函数
LKMotor::LKMotor(const char name[16], InitConfig_s config, uint8_t index)
    : Base(name, std::move(config)), motorIndex_(index) {
    // 初始化接收队列
    this->rxQueue_ = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>::data));
    
    // 检查配置
    LOG::CHECK(checkBaseConfig());
    
    // 初始化模型参数
    initModelParams();
    
    // 初始发送ID设置为扭矩控制
    currentTxId_ = status_.txTorqueId;
    
    // 注册电机
    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();
    
    LOG::info("LKMotor", "%s: LKMotor instance created, index: %d", this->name_, motorIndex_);
}

// 析构函数
LKMotor::~LKMotor() {
    this->cancelRecvCallback();
    this->cancelMotor();
    this->removeMotorFromMap();
    LOG::info("LKMotor", "%s: LKMotor instance destroyed", this->name_);
}

// 覆盖电机状态参数
void LKMotor::overrideStats(const Status_s& newStats) {
    status_ = newStats;
    updateTxId(); // 更新发送ID
}

// 获取电机CAN ID
uint16_t LKMotor::canId() const {
    return status_.rxBaseId + motorIndex_;
}

// 获取电机主ID
uint16_t LKMotor::masterId() const {
    return status_.rxBaseId;
}

// 获取电机唯一标识
uint16_t LKMotor::uid() {
    return canId();
}

// 注册接收回调函数
void LKMotor::registerRecvCallback() {
    Can::instance().registerCallback(
        reinterpret_cast<canHandle*>(this->pComHandle_), 
        canId(),
        [this](const uint8_t* rxBuf) {
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(this->rxQueue_, rxBuf, &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    );
    LOG::info("LKMotor", "%s: Receive callback registered, CAN ID: 0x%hx", 
              this->name_, canId());
}

// 取消接收回调函数
void LKMotor::cancelRecvCallback() {
    Can::instance().unregisterCallback(
        reinterpret_cast<canHandle*>(this->pComHandle_), 
        canId()
    );
    LOG::info("LKMotor", "%s: Receive callback canceled, CAN ID: 0x%hx", 
              this->name_, canId());
}

// 更新发送ID
void LKMotor::updateTxId() {
    switch (this->workMode_) {
        case WorkMode_e::QUAD_CURR:
            currentTxId_ = status_.txTorqueId;
            break;
        case WorkMode_e::QUAD_VEL:
            currentTxId_ = status_.txSpeedId;
            break;
        case WorkMode_e::QUAD_POS:
            currentTxId_ = status_.txAngleId;
            break;
        case WorkMode_e::QUAD_MIX:
            currentTxId_ = status_.txMixId;
            break;
        default:
            LOG::error("LKMotor", "%s: Unsupported work mode", this->name_);
            currentTxId_ = status_.txTorqueId; // 默认使用扭矩控制
            break;
    }
}

// 发送数据
MotorTypeDef_e LKMotor::send(uint16_t sendId, uint8_t* txBuf, uint8_t len) {
    if (this->group_ == nullptr) {
        LOG::error("LKMotor", "%s: Group not found", this->name_);
        return MOTOR_ERROR;
    }
    
    if (this->checkGroupSend(this->group_)) {
        return static_cast<MotorTypeDef_e>(
            Can::instance().transmitData(
                reinterpret_cast<canHandle*>(this->pComHandle_),
                sendId,
                txBuf,
                len
            )
        );
    }
    return MOTOR_OK;
}

// 解析接收数据
MotorTypeDef_e LKMotor::parse(const RxBus_s::CANRxBuf_s<8>& rxBuf) {
    Feedback_s fb;
    // 从缓冲区解析数据
    const uint8_t* data = rxBuf.data;
    fb.cmd = data[0];
    fb.temperature = data[1];
    fb.current = static_cast<int16_t>((data[2] << 8) | data[3]);
    fb.speed = static_cast<int16_t>((data[4] << 8) | data[5]);
    fb.angle = static_cast<uint16_t>((data[6] << 8) | data[7]);
    
    // 转换为物理量
    // 电流转换 (A)
    float current = fb.current / status_.currentDataMax * status_.currentMax;
    this->data_.curr = this->isReverse_ ? -current : current;
    
    // 扭矩转换 
    this->data_.torq = this->data_.curr * status_.torqueConstant;
    
    // 速度转换 
    float speed = static_cast<float>(fb.speed);
    this->data_.spdRpm = this->isReverse_ ? -speed : speed;
    this->data_.spdRadps = speed * PI / 180.0f;
    
    // 角度转换 (编码器值 -> 弧度)
    float angle = static_cast<float>(fb.angle) / status_.encoderSpan * 2.0f * PI;
    this->data_.rawAng = this->isReverse_ ? (2.0f * PI - angle) : angle;
    
    // 计算相对零点的角度
    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + 2.0f * PI : del;
    
    // 温度
    this->data_.tempture = fb.temperature;
    
    // 处理多圈角度
    float angDiff = getMinorArc(this->data_.rawAng, this->data_.angLast, 2.0f * PI) 
                   / status_.reductionRatio;
    
    // 检查电机在线状态
    if ((this->globalState_ == GlobalState_e::OFFLINE || 
         this->globalState_ == GlobalState_e::UNREGISTER) &&
        this->data_.angLast != this->data_.rawAng) {
        this->globalState_ = GlobalState_e::ONLINE;
        angDiff = 0;
        this->data_.multipCirAng = this->data_.rawAng / status_.reductionRatio;
    }
    
    this->data_.angLast = this->data_.rawAng;
    this->data_.multipCirAng += angDiff;
    this->data_.cirNum = this->data_.multipCirAng / (2.0f * PI);
    this->data_.singleCirAng = rangeMap(this->data_.multipCirAng);
    
    return MOTOR_OK;
}

// 控制逻辑
MotorTypeDef_e LKMotor::ctrl() {
    MotorTypeDef_e result = MOTOR_OK;
    int16_t ctrlCmd = 0;
    
    // 根据工作模式和命令类型计算控制指令
    switch (this->workMode_) {
        case WorkMode_e::QUAD_CURR: // 电流/扭矩模式
            switch (this->cmd_.curCmdType) {
                case MotorCmdType_e::SET_ELEC:
                    ctrlCmd = static_cast<int16_t>(
                        this->cmd_.elec / status_.currentMax * status_.currentDataMax
                    );
                    break;
                case MotorCmdType_e::SET_TORQ:
                    // 扭矩转电流指令
                    ctrlCmd = static_cast<int16_t>(
                        (this->cmd_.torq / status_.torqueConstant) / 
                        status_.currentMax * status_.currentDataMax
                    );
                    break;
                case MotorCmdType_e::SET_VEL:
                    if (this->velPID_ != nullptr) {
                        this->cmd_.torq = this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
                        ctrlCmd = static_cast<int16_t>(
                            (this->cmd_.torq / status_.torqueConstant) / 
                            status_.currentMax * status_.currentDataMax
                        );
                    } else {
                        LOG::error("LKMotor", "%s: Velocity PID is null", this->name_);
                        result = MOTOR_ERROR;
                    }
                    break;
                default:
                    LOG::error("LKMotor", "%s: Unsupported command type", this->name_);
                    result = MOTOR_ERROR;
                    break;
            }
            break;
            
        case WorkMode_e::QUAD_VEL: // 速度模式
            // 速度指令直接使用
            ctrlCmd = static_cast<int16_t>(this->cmd_.vel * 180.0f / PI);
            break;
            
        case WorkMode_e::QUAD_POS: // 位置模式
            // 位置指令转换为编码器值
            ctrlCmd = static_cast<int16_t>(
                this->cmd_.pos / (2.0f * PI) * status_.encoderSpan
            );
            break;
            
        default:
            LOG::error("LKMotor", "%s: Unsupported work mode", this->name_);
            result = MOTOR_ERROR;
            break;
    }
    
    // 处理电机反转
    if (this->isReverse_) {
        ctrlCmd = -ctrlCmd;
    }
    
    // 限制指令范围
    if (this->workMode_ == WorkMode_e::QUAD_CURR) {
        ctrlCmd = std::clamp(ctrlCmd, 
                           static_cast<int16_t>(-status_.currentDataMax), 
                           static_cast<int16_t>(status_.currentDataMax));
    }
    
    // 存储控制指令，等待广播发送
    txBuf_.data[motorIndex_] = ctrlCmd;
    txBuf_.sendFlag = this->cmd_.SW;
    
    return result;
}

// 更新电机状态
MotorTypeDef_e LKMotor::update() {
    // 接收并解析数据
    if (xQueueReceive(this->rxQueue_, this->rxBuf_.data, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
    }
    
    // 处理命令队列
    if (xQueueReceive(this->cmdQueue_, &this->cmdBuf_, 0) == pdTRUE) {
        this->parseCmd();
        updateTxId(); // 根据新命令更新发送ID
    }
    
    // 执行控制逻辑
    return ctrl();
}

// 挂载电机数据到广播帧/
void LKMotor::mountToBroadcastFrame(Transmit_s& broadcastFrame) {
    if (txBuf_.sendFlag) {
        broadcastFrame.data[motorIndex_] = txBuf_.data[motorIndex_];
    } else {
        broadcastFrame.data[motorIndex_] = 0;
    }
    broadcastFrame.sendFlag = broadcastFrame.sendFlag || txBuf_.sendFlag;
}