#pragma once

#include "MiniPc.hpp"

#include <cstdint>
#include <memory>


#define VISION_READY_EVENT (1 << 3)

namespace MINIPC {

#pragma pack(push, 1)
typedef struct {
    uint8_t visionHeader; // 0x5A
    uint8_t localColor : 1;
    uint8_t taskMode : 2;
    bool resetTracker : 1;
    uint8_t isPlay : 1;
    bool targetChange : 1;
    uint8_t reserved : 2;
    float roll;
    float pitch;
    float yaw; // 主要需要两个轴的角度数据
    float aimX;
    float aimY;
    float aimZ; // aim的三个参数主要可以提供给视觉调试时使用，可不发送数据
    uint16_t gameTime;
    uint32_t sTimestamp;
    uint16_t checkSum;
} EcToVision_s;

typedef struct {
    uint8_t visionHeader; // 0xA5
    uint8_t target : 2;
    uint8_t idNum : 3;
    uint8_t armorNum : 3;
    float x, y,
            z; // 在世界坐标系（imu的坐标系中的位置），z的值为当前瞄准装甲板的高度
    float yaw; // 当前瞄准的装甲板朝向（在imu坐标系中的角度）
    float vx;
    float vy;
    float vz;
    float vYaw;          // 小陀螺速度
    float r1;            // 当前瞄准的装甲板与车身中心的水平距离
    float r2;            // 另一侧装甲板与车身中心的水平距离
    float dz;            // 另一侧装甲板的高度
    uint32_t rTimestamp; //(ms)frame capture time
    uint16_t firstPhase; //(ms) speed_t offset
    uint16_t checkSum;
} VisionToEc_s;
#pragma pack(pop)


class Vision : public MiniPc {
public:
    static constexpr uint8_t VISION_UART_REQ_SOF = 0x5A;
    static constexpr uint8_t VISION_UART_CTRL_SOF = 0xA5;
    static constexpr uint16_t VISION_DATA_LEN = (sizeof(VisionToEc_s));

    Vision();
    void init(EventGroupHandle_t _event) override;
    void parseData() override;
    void sendData() override;
    void usbIdleCallback(uint8_t *_buf, uint16_t _len) override;

    void rxFreqCalc();
    float getRxFreq() { return rxFreq_; }
    VisionToEc_s &getVisionToEcDataPtr() { return visionToEcDatas_; };

private:
    VisionToEc_s visionToEcDatas_;
    EcToVision_s ecToVisionDatas_;
    uint8_t rxBuffer_[VISION_DATA_LEN];
    float rxFreq_;
    uint16_t rxCnt_ = 0;
};
} // namespace MINIPC

inline std::unique_ptr<MINIPC::Vision> vision;
