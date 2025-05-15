#pragma once

typedef struct {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} imu_data_fp_t;

typedef struct {
    signed short gx, gy, gz;
    signed short ax, ay, az;
    float g_fullscale;
    float a_fullscale;
    float temperature;
} imu_data_raw_t;

typedef struct {
    float accel_T[3][3];
    float accel_offs[3];
} AccCali_s;

typedef struct {
    float gx_bias, gy_bias, gz_bias;
    float gx_tco_k, gx_tco_b0;
    float gy_tco_k, gy_tco_b0;
    float gz_tco_k, gz_tco_b0;
} GyroCali_s;

typedef enum {
    ACC_PX = 0,
    ACC_NX = 1,
    ACC_PX_O = 6,
    ACC_NX_O = 7,
    ACC_PY = 2,
    ACC_NY = 3,
    ACC_PY_O = 8,
    ACC_NY_O = 9,
    ACC_PZ = 4,
    ACC_NZ = 5,
    ACC_PZ_O = 10,
    ACC_NZ_O = 11,
    ACC_CAL_FINAL = 12
} acc_cali_state_t;


class ImuCalibration {
public:
    ImuCalibration(const AccCali_s &acc_cali, const GyroCali_s &gyro_cali,
                   const float temp = 0.0f)
            : acc_cali_(acc_cali), gyro_cali_(gyro_cali)
    {
    }

    imu_data_fp_t Correct(const imu_data_raw_t &imu_data);

private:
    AccCali_s acc_cali_;
    GyroCali_s gyro_cali_;
};
