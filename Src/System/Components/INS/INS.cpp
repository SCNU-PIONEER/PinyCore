#include "INS.hpp"
#include "sdkconfig.h"
#include "cmsis_os2.h"
#include "TopicRouter.hpp"

extern SPI_HandleTypeDef IMU_SPI;

using namespace INS_SYS;

const AccCali_s accCali = {
    // default accelerometer calibration
    .accel_T = { { 1.010860f, 0.015129f, -0.001459f },
                 { 0.001142f, 1.009152f, 0.006399f },
                 { -0.005477f, 0.002071f, 1.013539f } },
    .accel_offs = { -34.944336f, -3.310059f, 107.792969f }
};
const GyroCali_s gyroCali = {
    // default gyroscope calibration
    .gx_bias = -1.93095636f, .gy_bias = -5.93262482f, .gz_bias = 0.222163752f,
    .gx_tco_k = 0.f,         .gx_tco_b0 = 0.f,        .gy_tco_k = 0.f,
    .gy_tco_b0 = 0.f,        .gz_tco_k = 0.f,         .gz_tco_b0 = 0.f
};

INS::INS()
        : insPub_(new Publisher<INSData_s>(&TopicRouter::instance().insTopic,
                                           &insDat_))
{
    xTaskCreate(INS::task, "ins_task", 256, this, osPriorityRealtime7, nullptr);

    LOG::info("INS", "task init success");
}

void INS::task(void *_param)
{
    auto instance = static_cast<INS *>(_param);
    auto &bmi088 = instance->bmi088_;
    auto &cali = instance->imuCali_;

    while (bmi088.init(&IMU_SPI))
        // wait for ACK from BMI088
        ;

    cali.init(accCali, gyroCali, bmi088.getAccelMappingVaule(),
              bmi088.getGyroMappingVaule());

    instance->DCM_.init();

    while (true) {
        // read BMI088 data
        bmi088.readRaw(); // read raw 6 axis data from device
        bmi088.read();    // serialize data to real format

        // update IMU calibration
        cali.updateTemperature(bmi088.getTemperature());
        cali.correctA(bmi088.getRawAccelX(), bmi088.getRawAccelY(),
                      bmi088.getRawAccelZ());
        cali.correctG(bmi088.getRawGyroX(), bmi088.getRawGyroY(),
                      bmi088.getRawGyroZ());

        // the order of axis is defined as:
        /*
                      Z
                      |
                      |
                      |
                      |     X: IN FRONT OF YOUR HEAD
                      |   /
                      | /
            Y <-------ROBOT 
        */
        // load raw INS needed data, you must transform the raw imu data to correct order
        IMUSensorData_s data = {
            .a = { .x = cali.getOutput().ax,
                   .y = cali.getOutput().ay,
                   .z = -cali.getOutput().az },
            .g = { .x = cali.getOutput().gx,
                   .y = cali.getOutput().gy,
                   .z = -cali.getOutput().gz },
        };

        if constexpr (CORRECT_IMU_DATA) {
            cali.steadyStateDetection();
        }

        // update INS
        instance->update(&data, instance->bmi088_.getTimestamp());

        vTaskDelay(1);
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void INS::update(IMUSensorData_s *_sensorDat, float _dt)
{
    this->dt_ = _dt; // update time interval

    float w = insDat_.q[0], x = insDat_.q[1], y = insDat_.q[2],
          z = insDat_.q[3];

    // Update DCM algorithm
    DCM_.update(_sensorDat->g.x, _sensorDat->g.y, _sensorDat->g.z,
                _sensorDat->a.x, _sensorDat->a.y, _sensorDat->a.z, this->dt_);

    // Quaternion data
    DCM_.getQuaternion(insDat_.q);

    // Get the Euler angles
    insDat_.roll = DCM_.getRoll();
    insDat_.pitch = DCM_.getPitch();
    insDat_.yaw = DCM_.getYaw();

    // Update the body axis system data
    insDat_.body.gx = _sensorDat->g.x;
    insDat_.body.gy = _sensorDat->g.y;
    insDat_.body.gz = _sensorDat->g.z;
    insDat_.body.ax = _sensorDat->a.x;
    insDat_.body.ay = _sensorDat->a.y;
    insDat_.body.az = _sensorDat->a.z;

    // Update Rotation Matrix
#if ROTATION_MATRIX_PITCH_ONLY
    R_[0][0] = cosf(insDat_.pitch);
    R_[0][1] = 0.0f;
    R_[0][2] = sinf(insDat_.pitch);
    R_[1][0] = 0.0f;
    R_[1][1] = 1.0f;
    R_[1][2] = 0.0f;
    R_[2][0] = -sinf(insDat_.pitch);
    R_[2][1] = 0.0f;
    R_[2][2] = cosf(insDat_.pitch);
#else
    R_[0][0] = 1.f - 2.f * y * y - 2.f * z * z; // 1-2y^2-2z^2
    R_[0][1] = 2.f * x * y - 2.f * w * z;       // 2xy - 2wz
    R_[0][2] = 2.f * x * z + 2.f * w * y;       // 2xz + 2wy
    R_[1][0] = 2.f * x * y + 2.f * w * z;       // 2xy + 2wz
    R_[1][1] = 1.f - 2.f * x * x - 2.f * z * z; // 1-2x^2-2z^2
    R_[1][2] = 2.f * y * z - 2.f * w * x;       // 2yz - 2wx
    R_[2][0] = 2.f * x * z - 2.f * w * y;       // 2xz - 2wy
    R_[2][1] = 2.f * y * z + 2.f * w * x;       // 2xy + 2wz
    R_[2][2] = 1.f - 2.f * x * x - 2.f * y * y; // 1-2x^2-2y^2
#endif

    // Transform body axis data to earth axis system using the rotation matrix
    bodyVectorT_[0][0] = insDat_.body.ax;
    bodyVectorT_[1][0] = insDat_.body.ay;
    bodyVectorT_[2][0] = insDat_.body.az;
    earthVectorT_ = R_ * bodyVectorT_;
    insDat_.earth.ax = earthVectorT_[0][0];
    insDat_.earth.ay = earthVectorT_[1][0];
    insDat_.earth.az = earthVectorT_[2][0];

    bodyVectorT_[0][0] = insDat_.body.gx;
    bodyVectorT_[1][0] = insDat_.body.gy;
    bodyVectorT_[2][0] = insDat_.body.gz;
    earthVectorT_ = R_ * bodyVectorT_;
    insDat_.earth.gx = earthVectorT_[0][0];
    insDat_.earth.gy = earthVectorT_[1][0];
    insDat_.earth.gz = earthVectorT_[2][0];

    // bodyVectorT_[0][0] = insDat_.body.mx;
    // bodyVectorT_[1][0] = insDat_.body.my;
    // bodyVectorT_[2][0] = insDat_.body.mz;
    // earthVectorT_ = R_ * bodyVectorT_;
    // insDat_.earth.mx = earthVectorT_[0][0];
    // insDat_.earth.my = earthVectorT_[1][0];
    // insDat_.earth.mz = earthVectorT_[2][0];

    // send queue
    insPub_->publish();
}
