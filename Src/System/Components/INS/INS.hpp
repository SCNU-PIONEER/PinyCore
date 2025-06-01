#pragma once

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

    // body axis system
    struct Body_s {
        float ax, ay, az; // accelerometer , unit:m/s^2
        float gx, gy, gz; // gyroscope , unit:rad/s
        float mx, my, mz; // magnetometer , unit:gauss
    } body;

    // earth axis system
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
    void init();
    void update(IMUSensorData_s *_sensorDat, float _dt);
    inline float roll() const { return insDat.roll; }
    inline float yaw() const { return insDat.yaw; }
    inline float pitch() const { return insDat.pitch; }

private:
    IMUSensorData_s rawDat; // raw data from IMU, body axis system
    INSData_s insDat; // data after INS algorithm, body and earth axis system
};
