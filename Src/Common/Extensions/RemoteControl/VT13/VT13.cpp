#include "VT13.hpp"
#include <cstdint>
#include <cstring>
#include "Crc.hpp"
#include "Dwt.hpp"
#include <cstdio>

using namespace REFEREE;

VT13::VT13(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, uint32_t _eventBit)
        : RemoteControl(_huart, FRAME_LENGTH, _event)
{
    // Should be executed after MX_USARTx_UART_Init()
    uart_.recvDmaInit(buf_, FRAME_LENGTH_MAX);
    uart_.registerCallback([this, _eventBit](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
        this->callBackFromISR(_huart, _dataLength, _eventBit);
    });
    rxLostCnt_ = RX_LOST_MAX;
}

VT13::~VT13()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

void VT13::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit)
{
    UNUSED(_huart);
    UNUSED(_pos);
    xEventGroupSetBitsFromISR(event_, _eventBit, nullptr);
}


// debug
uint8_t data[50] = { 0 };
uint8_t crc = 0;
uint32_t cnt = 0;
float freq = 0;
uint32_t cnt1 = 0;
float freq1 = 0;
volatile float ang[6];
void VT13::parse()
{
    memcpy(data, buf_, FRAME_LENGTH_MAX);

    // 遍历是因为每一次空闲中断时数据包可能不在缓冲区的开头，甚至可能存在多个数据包，需要逐字节检查帧头和校验来找到有效数据包的位置
    for (uint8_t i = 0; i < FRAME_LENGTH_MAX; i++) {
        if (buf_[i] == 0xA9) {
            if ((i + FRAME_LENGTH) > FRAME_LENGTH_MAX) {
                break;
            }
            if (buf_[i + 1] == 0x53 && Verify_CRC16_Check_Sum(&buf_[i], FRAME_LENGTH)) {
                raw_ = reinterpret_cast<RcRawData_s *>(&buf_[i]);
                data_.KBM.lastKey = data_.KBM.key;
                data_.gamepad.ch[0] = static_cast<int16_t>(raw_->ch0 - CH_VALUE_OFFSET);
                data_.gamepad.ch[1] = static_cast<int16_t>(raw_->ch1 - CH_VALUE_OFFSET);
                data_.gamepad.ch[2] = static_cast<int16_t>(raw_->ch2 - CH_VALUE_OFFSET);
                data_.gamepad.ch[3] = static_cast<int16_t>(raw_->ch3 - CH_VALUE_OFFSET);
                data_.gamepad.sw = static_cast<Sw_e>(raw_->sw);
                data_.gamepad.pause = raw_->pause;
                data_.gamepad.fn1 = raw_->fn1;
                data_.gamepad.fn2 = raw_->fn2;
                data_.gamepad.wheel = static_cast<int16_t>(raw_->wheel - CH_VALUE_OFFSET);
                data_.gamepad.trigger = raw_->trigger;
                data_.KBM.mouseX = raw_->mouseX;
                data_.KBM.mouseY = raw_->mouseY;
                data_.KBM.mouseZ = raw_->mouseZ;
                data_.KBM.mouseLeft = raw_->mouseLeft;
                data_.KBM.mouseRight = raw_->mouseRight;
                data_.KBM.mouseMiddle = raw_->mouseMiddle;
                data_.KBM.key = raw_->key;
                rxLostCnt_ = 0;
                freq1 = Dwt::instance().getFreq(&cnt1);
                i += FRAME_LENGTH;
            }
        }
#if EXTENSION_CUSTOM_CTRL
        else if (buf_[i] == 0xA5 && Verify_CRC16_Check_Sum(&buf_[i], sizeof(TxData_s))) {
            if ((i + sizeof(TxData_s)) > FRAME_LENGTH_MAX) {
                break;
            }
            // 从右边看
            // joint1 , joint2， joint4 , joint5 ，joint6 顺时针增长，
            // joint3 逆时针增长
            for (uint8_t j = 0; j < 6; j++) {
                ctrlData_.joint[j] = ((TxData_s *)&buf_[i])->ctrlData.joint[j];
                ang[j] = ((TxData_s *)&buf_[i])->ctrlData.joint[j];
                ctrlData_.push = ((TxData_s *)&buf_[i])->ctrlData.push;
                ctrlData_.direction = ((TxData_s *)&buf_[i])->ctrlData.direction;
            }
            freq = Dwt::instance().getFreq(&cnt);
            i += sizeof(TxData_s);
            printf("angle: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]);
        }
#endif
    }

    //     if (buf_[0] == 0xA9) {
    //         raw_ = reinterpret_cast<RcRawData_s *>(buf_);
    //         if (buf_[1] == 0x53 && Verify_CRC16_Check_Sum(buf_, FRAME_LENGTH)) {
    //             data_.gamepad.ch[0] = static_cast<int16_t>(raw_->ch0 - CH_VALUE_OFFSET);
    //             data_.gamepad.ch[1] = static_cast<int16_t>(raw_->ch1 - CH_VALUE_OFFSET);
    //             data_.gamepad.ch[2] = static_cast<int16_t>(raw_->ch2 - CH_VALUE_OFFSET);
    //             data_.gamepad.ch[3] = static_cast<int16_t>(raw_->ch3 - CH_VALUE_OFFSET);
    //             data_.gamepad.sw = static_cast<Sw_e>(raw_->sw);
    //             data_.gamepad.pause = raw_->pause;
    //             data_.gamepad.fn1 = raw_->fn1;
    //             data_.gamepad.fn2 = raw_->fn2;
    //             data_.gamepad.wheel = static_cast<int16_t>(raw_->wheel - CH_VALUE_OFFSET);
    //             data_.gamepad.trigger = raw_->trigger;
    //             data_.KBM.mouseX = raw_->mouseX;
    //             data_.KBM.mouseY = raw_->mouseY;
    //             data_.KBM.mouseZ = raw_->mouseZ;
    //             data_.KBM.mouseLeft = raw_->mouseLeft;
    //             data_.KBM.mouseRight = raw_->mouseRight;
    //             data_.KBM.mouseMiddle = raw_->mouseMiddle;
    //             data_.KBM.key = raw_->key;
    //             rxLostCnt_ = 0;
    //             freq1 = Dwt::instance().getFreq(&cnt1);
    //         }

    // #if EXTENSION_CUSTOM_CTRL
    //     } else if (buf_[0] == 0xA5 && Verify_CRC16_Check_Sum(buf_, sizeof(TxData_s))) {
    //         for (uint8_t i = 0; i < 6; i++) {
    //             ctrlData_.joint[i] = ((TxData_s *)buf_)->ctrlData.joint[i];
    //             ang[i] = ((TxData_s *)buf_)->ctrlData.joint[i];
    //             ctrlData_.push = ((TxData_s *)buf_)->ctrlData.push;
    //             ctrlData_.direction = ((TxData_s *)buf_)->ctrlData.direction;
    //         }
    //         freq = Dwt::instance().getFreq(&cnt);
    //         printf("angle: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]);
    //     }
    // #endif

    memset(buf_, 0, FRAME_LENGTH_MAX);
    uart_.receiveDma(buf_, FRAME_LENGTH_MAX);
}

bool VT13::isOnline()
{
    if (rxLostCnt_ < RX_LOST_MAX) {
        rxLostCnt_.fetch_add(1, std::memory_order_relaxed);
        return true;
    } else
        return false;
}

void *VT13::getData() { return &data_; }

#if EXTENSION_CUSTOM_CTRL
const CtrlData_s &VT13::getCtrlData() { return ctrlData_; }
#endif
