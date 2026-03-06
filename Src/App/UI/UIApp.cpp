#include "./UIApp.hpp"
#include "StmLog.hpp"
#include "UIDesigner.hpp"
#include "sdkconfig.h"

using namespace UI;

extern UART_HandleTypeDef UI_UART;

UIAPP::UIAPP() : client_(UI_UART) {};

void UIAPP::init()
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
    rxQueue = xQueueCreate(10, sizeof(Msg_s));

    LOG::info("UI", "task init success");
}

void UIAPP::update(const Msg_s *_msg)
{
    // 更新UI信息
    switch (_msg->type) {
    case Event_e::CHASSIS:
        updateChassis((ChassisUIMsg_s *)_msg->pdata);
        break;
    case Event_e::GIMBAL:
        updateGimbal((GimbalUIMsg_s *)_msg->pdata);
        break;
    case Event_e::ARM:
        updateArm((ArmUIMsg_s *)_msg->pdata);
        break;
    case Event_e::ARMORBOOSTER:
        updateArmorBooster((ArmorBoosterUIMsg_s *)_msg->pdata);
        break;
    default:
        break;
    }
}

void UIAPP::updateChassis(const ChassisUIMsg_s *_msg)
{
    (void)_msg;
    //收到数据后更新UI，然后ready对应的UI
    //如果dynamicInfo_[0]为CHAR，收到_msg.state改变了，则dynamicInfo_[0].text = _msg.state
    //client_.ready(&dynamicInfo_[0]);
}

void UIAPP::updateGimbal(const GimbalUIMsg_s *_msg)
{
    (void)_msg;
    //client_.ready(&dynamicInfo_[0]);
}

void UIAPP::updateArm(const ArmUIMsg_s *_msg)
{
    (void)_msg;
    // client_.ready(&dynamicInfo_[0]);
}

void UIAPP::updateArmorBooster(const ArmorBoosterUIMsg_s *_msg)
{
    (void)_msg;
    // client_.ready(&dynamicInfo_[0]);
}

void UIAPP::task()
{
    Msg_s param = {};
    if (xQueueReceive(rxQueue, &param, 0) == pdTRUE) {
        update(&param);
    }
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();
        if (!client_.isCharInit || !client_.isGraphicInit)
            client_.sendInit();
        else
            client_.send();
    }
}

void UIAPP::uiReInit()
{
    client_.isCharInit = false;
    client_.isGraphicInit = false;
}
