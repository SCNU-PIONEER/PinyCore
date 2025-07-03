#pragma once

#include "./Algorithm/DCM_AHRS/DCM_AHRS.hpp"
#include "./Algorithm/Calibrate/Calibrate.hpp"

#define ROTATION_MATRIX_PITCH_ONLY 0

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

    float temperature; // temperature data , unit:degC
};

class INS {
// public:
//     INS(const INS &) = delete;
//     INS &operator=(const INS &) = delete;
//     inline static INS* instance() {
//         static INS instance_;
//         return &instance_;
//     }

// private:
//     INS() = default;
public:
    void init(const AccCali_s &accCali_, const GyroCali_s &gyroCali_);
    void update(IMUSensorData_s *_sensorDat, float _dt);
    inline float roll() const { return insDat_.roll; }
    inline float yaw() const { return insDat_.yaw; }
    inline float pitch() const { return insDat_.pitch; }

private:
    float dt_ = 0.001f; // default time interval in seconds

    // 3x3 rotation matrix data
    float R_data_[9] = { 0.0f };
    // arm_matrix_instance_f32 R_ = { 3, 3, R_data_ };
    // 3x1 vector
    float bodyV_data_[3] = { 0.0f };
    float earthV_data_[3] = { 0.0f };
    // arm_matrix_instance_f32 body_ = { 3, 1, bodyV_data_ };
    // arm_matrix_instance_f32 earth_ = { 3, 1, earthV_data_ };
    
    // IMU calibration
    // IMU Algorithm
    ImuCalibration imuCali_; // IMU calibration object

    IMU_DCM_AHRS::DCM_AHRS DCM_ = IMU_DCM_AHRS::DCM_AHRS(dt_); // DCM algorithm object

    // INS data
    IMUSensorData_s rawDat_; // raw data from IMU, body axis system

    INSData_s insDat_; // data after INS algorithm, body and earth axis system
};
