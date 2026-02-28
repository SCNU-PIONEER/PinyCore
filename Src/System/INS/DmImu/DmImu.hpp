#pragma once

#include "Bsp_can.hpp"
#include <cstdint>

namespace DM_IMU {


typedef struct ImuData_s {
    // rpy
    float roll, yaw, pitch; // euler angle , unit:rad

    struct {
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    } body;

    // Quaternion: w, x, y, z
    float q[4];

} ImuData_s;


class DmImu {
public:
    /**
     * @param hcan        Pointer to the CAN handle
     * @param id          IMU's logical ID
     * @param feedbackId  CAN ID for receiving IMU data
     * @param sendId      CAN ID for sending requests to IMU
     */
    DmImu(CAN_HandleTypeDef *_hcan, uint16_t _id, uint16_t _feedbackId, uint16_t _sendId);

    void sendRequest(uint8_t _reg);
    void getData();
    void canCallback(const uint8_t *_rxData);

    ImuData_s _datas{};

private:
    CAN_HandleTypeDef *_hcan;
    uint8_t rxData[8];
    uint16_t _id;
    uint16_t _feedbackId;
    uint16_t _sendId;
};

} // namespace DM_IMU
