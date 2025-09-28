#pragma once

namespace IMU_MAHONY_AHRS {

struct EData_s {
    float roll;
    float pitch;
    float yaw;
};

class Mahony {
public:
    void regiter(float _sampleFrequency, float _twoKp, float _twoKi);
    void init(float _ax, float _ay, float _az);

    void update(float _gx, float _gy, float _gz, float _ax, float _ay,
                float _az, float _dt);

    EData_s getEdata();

    float getRoll();
    float getPitch();
    float getYaw();

protected:
    float invSqrt(float _x);
    void updateIMU(float _gx, float _gy, float _gz, float _ax, float _ay,
                   float _az, float _dt);
    void computeAngles();

private:
    float twoKp;          // 2 * proportional gain (Kp)
    float twoKi;          // 2 * integral gain (Ki)
    float q0, q1, q2, q3; // quaternion of sensor frame
    float integralFBx, integralFBy,
            integralFBz; // integral error terms scaled by Ki
    float invSampleFreq;
    char anglesComputed;

    EData_s edata_;
};

} // namespace IMU_MAHONY_AHRS
