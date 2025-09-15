#pragma once

#include "./Algorithm/DcmAHRS/DcmAHRS.hpp"
#include "./Algorithm/Calibrate/calibrate.hpp"

#include "Matrix.hpp"

#include "Topic.hpp"
#include "Bmi088.hpp"

#define ROTATION_MATRIX_PITCH_ONLY 0

namespace INS_SYS {
struct IMUSensorData_s {
    struct Accel_s {
        float x, y, z;
    } a; // accelerometer data , unit:m/s^2

    struct Gyro_s {
        float x, y, z;
    } g; // gyroscope data , unit:rad/s

    struct Mag_s {
        float x, y, z;
    } m; // magnetometer data , unit:gauss

    float temperature; // temperature data , uint:degC
};

struct INSData_s {
    // rpy
    float roll, yaw, pitch; // euler angle , unit:rad

    // Quaternion: w, x, y, z
    float q[4];

    // body axis system
    struct Body_s {
        float ax, ay, az; // accelerometer , unit:m/s^2
        float gx, gy, gz; // gyroscope , unit:rad/s
        float mx, my, mz; // magnetometer , unit:gauss
    } body;

    // earth axis system (ENU)
    struct Earth_s {
        float ax, ay, az; // accelerometer , unit:m/s^2
        float gx, gy, gz; // gyroscope , unit:rad/s
        float mx, my, mz; // magnetometer , unit:gauss
    } earth;
};

class INS {
public:
    INS();

    static void task(void *_param);

    void init();

    void update(float _dt);
    float roll() const { return insDat_.roll; }
    float yaw() const { return insDat_.yaw; }
    float pitch() const { return insDat_.pitch; }

private:
    float dt_ = 0.001f; // default time interval in seconds

    // 3x3 rotation matrix data
    Matrix<3, 3> R_;
    // 3x1 vector
    Matrix<3, 1> bodyVectorT_;
    Matrix<3, 1> earthVectorT_;

    // IMU calibration
    IMUCalibration imuCali_; // IMU calibration object

    // IMU AHRS algorithm
    IMU_DCM_AHRS::DcmAhrs DCM_{ dt_ }; // DCM algorithm object

    // INS data
    IMUSensorData_s rawDat_; // raw data from IMU, body axis system

    INSData_s insDat_; // data after INS algorithm, body and earth axis system

    Publisher<INSData_s> *insPub_;

    BMI088 bmi088_;
};
} // namespace INS_SYS
