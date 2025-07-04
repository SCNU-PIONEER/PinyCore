#pragma once

namespace IMU_MahonyAHRS {

struct edata_s {
    float roll;
    float pitch;
    float yaw;
};

class Mahony {
public:
    void regiter(float sampleFrequency, float _twoKp, float _twoKi);
    void init(float _ax, float _ay, float _az);

    void update(float _gx, float _gy, float _gz, float _ax, float _ay,
                float _az, float _dt);

    edata_s getEdata(void);

    float getRoll(void);
    float getPitch(void);
    float getYaw(void);

protected:
    float invSqrt(float x);
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

    edata_s edata_;
};

}
