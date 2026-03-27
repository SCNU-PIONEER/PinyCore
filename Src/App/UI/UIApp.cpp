#include "./UIApp.hpp"
#include "StmLog.hpp"
#include "UIDesigner.hpp"
#include "sdkconfig.h"

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

void APP::updateChassis(const ChassisUIMsg_s &_msg)
{
    (void)_msg;
    //收到数据后更新UI，然后ready对应的UI
    //如果dynamicInfo_[0]为CHAR，收到_msg.state改变了，则改变dynamicInfo_[0].text，然后client_.ready(&dynamicInfo_[0]);
    //注意动态字符UI不能一直 client_.ready，不然其他UI会发不出去
    //client_.ready(&dynamicInfo_[0]);
}

void APP::updateGimbal(const GimbalUIMsg_s &_msg)
{
    (void)_msg;
    //client_.ready(&dynamicInfo_[0]);
}

void APP::updateArm(const ArmUIMsg_s &_msg)
{
    (void)_msg;
    // client_.ready(&dynamicInfo_[0]);
}

void APP::updateArmorBooster(const ArmorBoosterUIMsg_s &_msg)
{
    (void)_msg;
    // client_.ready(&dynamicInfo_[0]);
}

void APP::task()
{
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();
        if (!client_.isCharInit || !client_.isGraphicInit) {
            client_.sendInit();
            resetCnt = 0;
        } else if (resetCnt >= pdMS_TO_TICKS(1000))
            client_.send();
        resetCnt++;
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
