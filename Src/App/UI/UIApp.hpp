#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "MsgImpl.hpp"
#include "./UIDesigner.hpp"

namespace UI {

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 用户配置区 ━━━━━━━━━━━━━━━━━━━━━━━━━━

// 发送间隔时间(ms) 默认40ms (25HZ), 裁判系统上限是30HZ
static constexpr uint8_t SEND_INTERVAL = 40;

// 队列接收事件
enum class Event_e : uint8_t { CHASSIS, GIMBAL, ARM, ARMORBOOSTER };

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

struct Msg_s {
    Event_e type;
    void *pdata;
};

class UIAPP {
public:
    static UIAPP *instance()
    {
        static UIAPP instance;
        return &instance;
    }

    UIAPP(const UIAPP &) = delete;
    UIAPP &operator=(const UIAPP &) = delete;

    void init();
    void update(const Msg_s *_param);
    void task();

    QueueHandle_t rxQueue;

    Client &client() { return client_; }
    void uiReInit();

protected:
    void updateChassis(const ChassisUIMsg_s *_msg);
    void updateGimbal(const GimbalUIMsg_s *_msg);
    void updateArm(const ArmUIMsg_s *_msg);
    void updateArmorBooster(const ArmorBoosterUIMsg_s *_msg);

private:
    UIAPP();
    Client client_;
    Info_s dynamicInfo_[DYNAIMIC_NUM] = {};
    Info_s constInfo_[STATIC_NUM] = {};
    uint32_t updateCnt = 0;
};

} // namespace UI
