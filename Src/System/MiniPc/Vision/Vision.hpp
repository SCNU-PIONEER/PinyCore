#pragma once

#include "MiniPc.hpp"

#include <cstdint>
#include <memory>

namespace MINIPC {

#pragma pack(push, 1)
#pragma pack(push, 1)
typedef struct {
    uint8_t header;
    uint8_t detectColor : 1;
    uint8_t taskMode : 2;
    bool resetTracker : 1;
    bool isPlay : 1;
    bool changeTarget : 1;
    uint8_t reserved : 2;
    float roll;
    float pitch;
    float yaw;
    float aimX;        // unuse
    float aimY;        // unuse
    float aimZ;        // unuse
    uint16_t gameTime; // s [0,450]
    uint32_t timestamp;
    uint16_t checksum;
} EcToVision_s;
//发来的原始数据
typedef struct {
    uint8_t header;
    float targetYaw;
    float targetPitch;
    float yawDiff;
    float pitchDiff;
    float distance;
    uint8_t fire;
    uint32_t capTimestamp;
    uint16_t tOffset;
    uint16_t checksum;
} VisionToEc_s;
#pragma pack(pop)
//处理后的数据
typedef struct {
    uint8_t control;
    uint8_t fire;
    float targetYaw;
    float targetPitch;
    float yaw;
    float yawVel;
    float yawAcc;
    float pitch;
    float pitchVel;
    float pitchAcc;
} V2EData_s;


class Vision : public MiniPc {
public:
    static constexpr uint8_t VISION_UART_REQ_SOF = 0x5A;
    static constexpr uint8_t VISION_UART_CTRL_SOF = 0xA5;
    static constexpr uint16_t VISION_DATA_LEN = (sizeof(VisionToEc_s));

    Vision();
    void init(const EventGroupHandle_t &_event, uint32_t _eventBit) override;
    void usbIdleCallback(uint8_t *_buf, uint16_t _len, const EventGroupHandle_t &_event, uint32_t _eventBit) override;
    void parseData() override;
    void sendData() override;

    void rxFreqCalc();
    float getRxFreq() { return rxFreq_; }

    EcToVision_s &eToVData() { return ecToVisionDatas_; };
    V2EData_s &v2eData() { return v2eDatas_; };

private:
    VisionToEc_s visionToEcDatas_;
    V2EData_s v2eDatas_;
    EcToVision_s ecToVisionDatas_;

    uint8_t rxBuffer_[VISION_DATA_LEN];
    float rxFreq_;
    uint16_t rxCnt_ = 0;
};
} // namespace MINIPC

inline std::unique_ptr<MINIPC::Vision> vision;
