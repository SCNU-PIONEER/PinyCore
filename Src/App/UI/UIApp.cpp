#include "./UIApp.hpp"
#include "Bsp_can.hpp"
#include "StmLog.hpp"
#include "UIApp.hpp"
#include "UIDesigner.hpp"
#include "sdkconfig.h"
#include "magic_enum/magic_enum.hpp"

using namespace UI;

extern UART_HandleTypeDef UI_UART;

APP::APP() : client_(UI_UART) {};

void APP::init()
{
    /* need to config */
    dynamicConfig(dynamicInfo_);
    staticConfig(constInfo_);

    // 初始化UI链表
    if (client_.initList(dynamicInfo_, DYNAIMIC_NUM, constInfo_, STATIC_NUM) == Status_e::ERROR) {
        LOG::error("UI", "List init failed");
    } else {
        LOG::info("UI", "List init success");
    }

    LOG::info("UI", "task init success");
}

auto copyString = []<typename T>(char *_text, T _id) {
    memset(_text, 0, 30);
    const auto &str = magic_enum::enum_name(_id);
    for (size_t i = 0; i < str.size(); i++) {
        _text[i] = str[i];
    }
};


void APP::updateChassis(const ChassisUIMsg_s &_msg)
{
    //收到数据后更新UI，然后ready对应的UI
    //如果dynamicInfo_[0]为CHAR，收到_msg.state改变了，则改变dynamicInfo_[0].text，然后client_.ready(&dynamicInfo_[0]);
    //注意动态字符UI不能一直 client_.ready，不然其他UI会发不出去
    //client_.ready(&dynamicInfo_[0]);
    using namespace UI::DYNAMIC_UI;
    copyString(dynamicInfo_[static_cast<uint8_t>(Name_e::CHASSIS_STATE)].config.text, _msg.state);
    switch (_msg.state) {
    case CHASSIS::FSMState_e::RUN:
        dynamicInfo_[static_cast<uint8_t>(Name_e::CHASSIS_STATE)].config.color = Color_e::GREEN;
        break;
    case CHASSIS::FSMState_e::SPIN:
        dynamicInfo_[static_cast<uint8_t>(Name_e::CHASSIS_STATE)].config.color = Color_e::CYAN_BLUE;
        break;
    case CHASSIS::FSMState_e::STOP:
        dynamicInfo_[static_cast<uint8_t>(Name_e::CHASSIS_STATE)].config.color = Color_e::YELLOW;
        break;
    }
    client_.ready(&dynamicInfo_[static_cast<uint8_t>(Name_e::CHASSIS_STATE)]);
}

void APP::updateGimbal(const GimbalUIMsg_s &_msg)
{
    using namespace UI::DYNAMIC_UI;
    copyString(dynamicInfo_[static_cast<uint8_t>(Name_e::GIMBAL_STATE)].config.text, _msg.state);
    switch (_msg.state) {
    case GIMBAL::FSMState_e::RUN:
        dynamicInfo_[static_cast<uint8_t>(Name_e::GIMBAL_STATE)].config.color = Color_e::GREEN;
        break;
    case GIMBAL::FSMState_e::STOP:
        dynamicInfo_[static_cast<uint8_t>(Name_e::GIMBAL_STATE)].config.color = Color_e::YELLOW;
        break;
    case GIMBAL::FSMState_e::LAUNCH:
        dynamicInfo_[static_cast<uint8_t>(Name_e::GIMBAL_STATE)].config.color = Color_e::CYAN_BLUE;
        break;
    }
    client_.ready(&dynamicInfo_[static_cast<uint8_t>(Name_e::GIMBAL_STATE)]);
}

void APP::updateArm(const ArmUIMsg_s &_msg)
{
    using namespace UI::DYNAMIC_UI;

    copyString(dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)].config.text, _msg.state);
    copyString(dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CLAMP_STATE)].config.text, _msg.clampState);
    copyString(dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CUSTOM_CTRL_STATE)].config.text, _msg.customCtrlisOnline);

    switch (_msg.state) {
    case ARM::FSMState_e::STOP:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)].config.color = Color_e::YELLOW;
        break;
    case ARM::FSMState_e::NORMAL:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)].config.color = Color_e::GREEN;
        break;
    case ARM::FSMState_e::PLAN:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)].config.color = Color_e::CYAN_BLUE;
        break;
    case ARM::FSMState_e::TEACH:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)].config.color = Color_e::ORANGE;
        break;
    }

    switch(_msg.clampState) {
    case ARM::ClampState_e::NONE:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CLAMP_STATE)].config.color = Color_e::YELLOW;
        break;
    case ARM::ClampState_e::RELEASE:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CLAMP_STATE)].config.color = Color_e::GREEN;
        break;
    case ARM::ClampState_e::CLAMP:
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CLAMP_STATE)].config.color = Color_e::CYAN_BLUE;
        break;
    }

    if (_msg.customCtrlisOnline == ARM::CustomCtrlState_e::ONLINE) {
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CUSTOM_CTRL_STATE)].config.color = Color_e::GREEN;
    } else {
        dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_CUSTOM_CTRL_STATE)].config.color = Color_e::YELLOW;
    }

    client_.ready(&dynamicInfo_[static_cast<uint8_t>(Name_e::ARM_STATE)]);
}

void ArmRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("ArmRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void GimbalRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("GimbalRxPacket", " Receive cb registered, uId:0x%hx \n", uid());
}

void APP::task()
{
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();
        if (!client_.isCharInit || !client_.isGraphicInit) {
            client_.sendInit();
        } else {
            GimbalUIMsg_s gimbalMsg;
            gimbalMsg.state = gimbalRxPacket_.data().gimbalState;
            updateGimbal(gimbalMsg);

            ArmUIMsg_s armMsg;
            armMsg.state = armRxPacket_.data().armState;
            armMsg.clampState = armRxPacket_.data().clampState;
            armMsg.customCtrlisOnline = armRxPacket_.data().customCtrlisOnline;
            updateArm(armMsg);

            client_.send();
        }
    }
}

void APP::uiReInit()
{
    client_.isCharInit = false;
    client_.isGraphicInit = false;
    client_.isIniting = true;
    client_.initTimes = 0;
    client_.nodeReset();
}
