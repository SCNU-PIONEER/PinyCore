#include "DmImu.hpp"
#include "Soc.hpp"
#include <cstring>
#include <numbers>

constexpr float F_PI = std::numbers::pi_v<float>;

namespace DM_IMU {
#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN (-58.8f)
#define GYRO_CAN_MAX  (34.88f)
#define GYRO_CAN_MIN  (-34.88f)


#define PITCH_CAN_MAX (F_PI / 2.0f)
#define PITCH_CAN_MIN (-F_PI / 2.0f)
#define ROLL_CAN_MAX  (F_PI)
#define ROLL_CAN_MIN  (-F_PI)
#define YAW_CAN_MAX   (F_PI)
#define YAW_CAN_MIN   (-F_PI)

#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

/**
 * @brief Convert unsigned integer to float
 */
static float uint_to_float(int _x_int, float _x_min, float _x_max, int _bits)
{
    float span = _x_max - _x_min;
    float offset = _x_min;
    return ((float)_x_int) * span / ((float)((1 << _bits) - 1)) + offset;
}

DmImu::DmImu(CAN_HandleTypeDef *_hcan, uint16_t _id, uint16_t _feedbackId, uint16_t _sendId)
        : _hcan(_hcan), _id(_id), _feedbackId(_feedbackId), _sendId(_sendId)

{
    Can::instance().registerCallback(_hcan, _feedbackId,
                                     [this](const uint8_t *_rxData) { this->canCallback(_rxData); });
}
void DmImu::getData()
{
    const uint8_t *data = rxData;

    switch (data[0]) {
    case 0x01: { // Accelerometer data
        uint16_t accel[3];
        accel[0] = data[3] << 8 | data[2];
        accel[1] = data[5] << 8 | data[4];
        accel[2] = data[7] << 8 | data[6];

        _datas.body.ax = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
        _datas.body.ay = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
        _datas.body.az = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
        break;
    }
    case 0x02: { // Gyroscope data
        uint16_t gyro[3];
        gyro[0] = data[3] << 8 | data[2];
        gyro[1] = data[5] << 8 | data[4];
        gyro[2] = data[7] << 8 | data[6];

        _datas.body.gx = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        _datas.body.gy = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        _datas.body.gz = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        break;
    }
    case 0x03: { // Euler angle data
        int euler[3];
        euler[0] = data[3] << 8 | data[2];
        euler[1] = data[5] << 8 | data[4];
        euler[2] = data[7] << 8 | data[6];

        _datas.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
        _datas.yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
        _datas.roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
        break;
    }
    case 0x04: { // Quaternion data
        int w = data[1] << 6 | ((data[2] & 0xF8) >> 2);
        int x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
        int y = (data[4] & 0x0F) << 10 | (data[5] << 2) | (data[6] & 0xC0) >> 6;
        int z = (data[6] & 0x3F) << 8 | data[7];

        _datas.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
        _datas.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
        _datas.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
        _datas.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
    }
    default:
        break;
    }
}

void DmImu::sendRequest(uint8_t _reg)
{
    uint8_t txBuf[4] = { 0 };
    txBuf[0] = static_cast<uint8_t>(_id);        // CANID_L
    txBuf[1] = static_cast<uint8_t>((_id >> 8)); // CANID_H
    txBuf[2] = _reg;                             // RID
    txBuf[3] = 0xCC;
    Can::instance().transmitData(_hcan, _sendId, txBuf, 4);
}

void DmImu::canCallback(const uint8_t *_rxData) { memcpy(rxData, _rxData, 8); }
} // namespace DM_IMU
