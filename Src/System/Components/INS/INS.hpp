#pragma once

struct IMUSensorData_s {
    float ax, ay, az; // accelerometer data , uint:m/s^2
    float gx, gy, gz; // gyroscope data , uint:rad/s
    float mx, my, mz; // magnetometer data , uint:gauss
    
    float temperature; // temperature data , uint:degC
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

private:
    IMUSensorData_s rawDat; // raw data from IMU, body axis system
    
};

